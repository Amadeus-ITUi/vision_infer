#include "driver/config/smartcar_config.h"

#include "app/vision_thread/vision_thread.h"
#include "driver/vision/vision_config.h"
#include "driver/vision/vision_image_processor.h"
#include "driver/vision/vision_transport.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <limits.h>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>

namespace
{
using RawMap = std::unordered_map<std::string, std::string>;

struct StringStorage
{
    std::string udp_web_server_ip;
    std::array<std::string, VISION_NCNN_CONFIG_MAX_LABELS> ncnn_labels;
};

struct ConfigSnapshot
{
    vision_runtime_config_t vision_runtime{};
    vision_processor_config_t vision_processor{};
    StringStorage strings{};
    std::string loaded_path;
};

StringStorage g_string_storage;
std::mutex g_config_mutex;
std::string g_loaded_config_path;

std::string executable_dir_config_path()
{
    char exe_path[PATH_MAX] = {0};
    const ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len <= 0)
    {
        return "";
    }

    exe_path[len] = '\0';
    std::string full_path(exe_path);
    const size_t slash_pos = full_path.find_last_of('/');
    if (slash_pos == std::string::npos)
    {
        return "";
    }
    return full_path.substr(0, slash_pos + 1) + "smartcar_config.toml";
}

std::string trim(const std::string &value)
{
    size_t begin = 0;
    while (begin < value.size() && std::isspace(static_cast<unsigned char>(value[begin])))
    {
        ++begin;
    }

    size_t end = value.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1])))
    {
        --end;
    }
    return value.substr(begin, end - begin);
}

std::string strip_comment(const std::string &line)
{
    bool in_quotes = false;
    bool escaped = false;
    for (size_t i = 0; i < line.size(); ++i)
    {
        const char ch = line[i];
        if (escaped)
        {
            escaped = false;
            continue;
        }
        if (ch == '\\')
        {
            escaped = true;
            continue;
        }
        if (ch == '"')
        {
            in_quotes = !in_quotes;
            continue;
        }
        if (!in_quotes && ch == '#')
        {
            return line.substr(0, i);
        }
    }
    return line;
}

bool parse_key_values_text(const std::string &text, RawMap *values, std::string *error_message)
{
    std::string section;
    std::istringstream input(text);
    std::string line;
    int line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        line = trim(strip_comment(line));
        if (line.empty())
        {
            continue;
        }

        if (line.front() == '[')
        {
            if (line.back() != ']')
            {
                *error_message = "invalid section header at line " + std::to_string(line_number);
                return false;
            }
            section = trim(line.substr(1, line.size() - 2));
            if (section.empty())
            {
                *error_message = "empty section header at line " + std::to_string(line_number);
                return false;
            }
            continue;
        }

        const size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos)
        {
            *error_message = "missing '=' at line " + std::to_string(line_number);
            return false;
        }

        const std::string key = trim(line.substr(0, eq_pos));
        const std::string value = trim(line.substr(eq_pos + 1));
        if (key.empty() || value.empty())
        {
            *error_message = "invalid key/value at line " + std::to_string(line_number);
            return false;
        }

        const std::string full_key = section.empty() ? key : (section + "." + key);
        if (values->find(full_key) != values->end())
        {
            *error_message = "duplicate key: " + full_key;
            return false;
        }
        (*values)[full_key] = value;
    }

    return true;
}

bool parse_key_values_file(const std::string &path, RawMap *values, std::string *error_message)
{
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
    {
        *error_message = "cannot open file: " + path;
        return false;
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    return parse_key_values_text(buffer.str(), values, error_message);
}

bool take_raw(const RawMap &values,
              std::set<std::string> *consumed,
              const std::string &key,
              std::string *raw,
              std::string *error_message)
{
    const auto it = values.find(key);
    if (it == values.end())
    {
        *error_message = "missing required key: " + key;
        return false;
    }
    consumed->insert(key);
    *raw = it->second;
    return true;
}

bool parse_bool_value(const std::string &raw, bool *value)
{
    if (raw == "true")
    {
        *value = true;
        return true;
    }
    if (raw == "false")
    {
        *value = false;
        return true;
    }
    return false;
}

bool parse_int_value(const std::string &raw, int *value)
{
    try
    {
        size_t consumed = 0;
        const long parsed = std::stol(raw, &consumed, 10);
        if (consumed != raw.size())
        {
            return false;
        }
        *value = static_cast<int>(parsed);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool parse_size_t_value(const std::string &raw, size_t *value)
{
    try
    {
        size_t consumed = 0;
        const unsigned long long parsed = std::stoull(raw, &consumed, 10);
        if (consumed != raw.size())
        {
            return false;
        }
        *value = static_cast<size_t>(parsed);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool parse_string_value(const std::string &raw, std::string *value)
{
    if (raw.size() < 2 || raw.front() != '"' || raw.back() != '"')
    {
        return false;
    }

    value->clear();
    bool escaped = false;
    for (size_t i = 1; i + 1 < raw.size(); ++i)
    {
        const char ch = raw[i];
        if (escaped)
        {
            switch (ch)
            {
                case 'n': value->push_back('\n'); break;
                case 't': value->push_back('\t'); break;
                case '\\': value->push_back('\\'); break;
                case '"': value->push_back('"'); break;
                default: return false;
            }
            escaped = false;
            continue;
        }
        if (ch == '\\')
        {
            escaped = true;
            continue;
        }
        value->push_back(ch);
    }
    return !escaped;
}

bool split_array_items(const std::string &raw, std::vector<std::string> *items)
{
    if (raw.size() < 2 || raw.front() != '[' || raw.back() != ']')
    {
        return false;
    }

    items->clear();
    std::string current;
    bool in_quotes = false;
    bool escaped = false;
    for (size_t i = 1; i + 1 < raw.size(); ++i)
    {
        const char ch = raw[i];
        if (escaped)
        {
            current.push_back(ch);
            escaped = false;
            continue;
        }
        if (ch == '\\')
        {
            current.push_back(ch);
            escaped = true;
            continue;
        }
        if (ch == '"')
        {
            current.push_back(ch);
            in_quotes = !in_quotes;
            continue;
        }
        if (!in_quotes && ch == ',')
        {
            items->push_back(trim(current));
            current.clear();
            continue;
        }
        current.push_back(ch);
    }

    if (!trim(current).empty() || raw.find(',') != std::string::npos)
    {
        items->push_back(trim(current));
    }
    if (items->size() == 1 && items->front().empty())
    {
        items->clear();
    }
    return !in_quotes && !escaped;
}

bool require_bool(const RawMap &values,
                  std::set<std::string> *consumed,
                  const std::string &key,
                  bool *target,
                  std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_bool_value(raw, target))
    {
        *error_message = "invalid bool for key: " + key;
        return false;
    }
    return true;
}

bool require_int(const RawMap &values,
                 std::set<std::string> *consumed,
                 const std::string &key,
                 int *target,
                 std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_int_value(raw, target))
    {
        *error_message = "invalid int for key: " + key;
        return false;
    }
    return true;
}

bool require_size_t(const RawMap &values,
                    std::set<std::string> *consumed,
                    const std::string &key,
                    size_t *target,
                    std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_size_t_value(raw, target))
    {
        *error_message = "invalid size_t for key: " + key;
        return false;
    }
    return true;
}

bool require_string(const RawMap &values,
                    std::set<std::string> *consumed,
                    const std::string &key,
                    std::string *storage,
                    const char **target,
                    std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }
    if (!parse_string_value(raw, storage))
    {
        *error_message = "invalid string for key: " + key;
        return false;
    }
    *target = storage->c_str();
    return true;
}

bool require_string_array(const RawMap &values,
                          std::set<std::string> *consumed,
                          const std::string &key,
                          std::array<std::string, VISION_NCNN_CONFIG_MAX_LABELS> *storage,
                          const char *(&target)[VISION_NCNN_CONFIG_MAX_LABELS],
                          size_t *count,
                          std::string *error_message)
{
    std::string raw;
    if (!take_raw(values, consumed, key, &raw, error_message))
    {
        return false;
    }

    std::vector<std::string> items;
    if (!split_array_items(raw, &items) || items.size() > VISION_NCNN_CONFIG_MAX_LABELS)
    {
        *error_message = "invalid string array size for key: " + key;
        return false;
    }

    for (size_t i = 0; i < VISION_NCNN_CONFIG_MAX_LABELS; ++i)
    {
        (*storage)[i].clear();
        target[i] = nullptr;
    }

    for (size_t i = 0; i < items.size(); ++i)
    {
        if (!parse_string_value(items[i], &(*storage)[i]))
        {
            *error_message = "invalid string array item for key: " + key;
            return false;
        }
        target[i] = (*storage)[i].c_str();
    }
    *count = items.size();
    return true;
}

ConfigSnapshot capture_config_snapshot()
{
    ConfigSnapshot snapshot;
    snapshot.vision_runtime = g_vision_runtime_config;
    snapshot.vision_processor = g_vision_processor_config;
    snapshot.strings = g_string_storage;
    snapshot.loaded_path = g_loaded_config_path;
    return snapshot;
}

void restore_config_snapshot(const ConfigSnapshot &snapshot)
{
    g_vision_runtime_config = snapshot.vision_runtime;
    g_vision_processor_config = snapshot.vision_processor;
    g_string_storage = snapshot.strings;
    g_loaded_config_path = snapshot.loaded_path;

    g_vision_runtime_config.udp_web_server_ip = g_string_storage.udp_web_server_ip.c_str();
    for (size_t i = 0; i < VISION_NCNN_CONFIG_MAX_LABELS; ++i)
    {
        g_vision_runtime_config.ncnn_labels[i] =
            g_string_storage.ncnn_labels[i].empty() ? nullptr : g_string_storage.ncnn_labels[i].c_str();
    }
}

void collect_restart_required_keys(const ConfigSnapshot &old_config,
                                   std::vector<std::string> *restart_required_keys)
{
    if (restart_required_keys == nullptr)
    {
        return;
    }

    restart_required_keys->clear();
    const auto push_if = [&](bool changed, const char *key) {
        if (changed)
        {
            restart_required_keys->emplace_back(key);
        }
    };

    push_if(old_config.vision_runtime.ncnn_input_width != g_vision_runtime_config.ncnn_input_width,
            "vision.runtime.ncnn.input_width");
    push_if(old_config.vision_runtime.ncnn_input_height != g_vision_runtime_config.ncnn_input_height,
            "vision.runtime.ncnn.input_height");
    push_if(old_config.vision_runtime.udp_web_video_port != g_vision_runtime_config.udp_web_video_port,
            "vision.runtime.web.video_port");
    push_if(old_config.vision_runtime.udp_web_meta_port != g_vision_runtime_config.udp_web_meta_port,
            "vision.runtime.web.meta_port");
    push_if(std::string(old_config.vision_runtime.udp_web_server_ip ? old_config.vision_runtime.udp_web_server_ip : "") !=
                std::string(g_vision_runtime_config.udp_web_server_ip ? g_vision_runtime_config.udp_web_server_ip : ""),
            "vision.runtime.web.server_ip");
    push_if(old_config.vision_runtime.ncnn_label_count != g_vision_runtime_config.ncnn_label_count,
            "vision.runtime.ncnn.label_count");

    for (size_t i = 0; i < VISION_NCNN_CONFIG_MAX_LABELS; ++i)
    {
        const std::string old_label = old_config.vision_runtime.ncnn_labels[i] ? old_config.vision_runtime.ncnn_labels[i] : "";
        const std::string new_label = g_vision_runtime_config.ncnn_labels[i] ? g_vision_runtime_config.ncnn_labels[i] : "";
        if (old_label != new_label)
        {
            restart_required_keys->emplace_back("vision.runtime.ncnn.labels");
            break;
        }
    }
}

void apply_runtime_changes_after_commit()
{
    vision_transport_udp_set_enabled(g_vision_runtime_config.udp_web_enabled);
    vision_transport_udp_set_max_fps(g_vision_runtime_config.udp_web_max_fps);
    vision_transport_udp_set_tcp_enabled(g_vision_runtime_config.udp_web_tcp_enabled);

    vision_thread_set_infer_enabled(g_vision_runtime_config.infer_enabled);
    vision_thread_set_ncnn_enabled(g_vision_runtime_config.ncnn_enabled);

    vision_image_processor_reload_config_from_globals();
}

bool apply_values(const RawMap &values, std::string *error_message)
{
    std::set<std::string> consumed;

    int schema_version = 0;
    if (!require_int(values, &consumed, "meta.schema_version", &schema_version, error_message))
    {
        return false;
    }
    if (schema_version != 1)
    {
        *error_message = "unsupported schema_version: " + std::to_string(schema_version);
        return false;
    }

    int udp_web_max_fps = 0;
    int udp_web_video_port = 0;
    int udp_web_meta_port = 0;
    size_t ncnn_label_count = 0;

    if (!require_bool(values, &consumed, "vision.runtime.infer_enabled", &g_vision_runtime_config.infer_enabled, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.ncnn_enabled", &g_vision_runtime_config.ncnn_enabled, error_message) ||
        !require_int(values, &consumed, "vision.runtime.ncnn.input_width", &g_vision_runtime_config.ncnn_input_width, error_message) ||
        !require_int(values, &consumed, "vision.runtime.ncnn.input_height", &g_vision_runtime_config.ncnn_input_height, error_message) ||
        !require_size_t(values, &consumed, "vision.runtime.ncnn.label_count", &ncnn_label_count, error_message) ||
        !require_string_array(values, &consumed, "vision.runtime.ncnn.labels", &g_string_storage.ncnn_labels, g_vision_runtime_config.ncnn_labels, &g_vision_runtime_config.ncnn_label_count, error_message))
    {
        return false;
    }

    if (ncnn_label_count != g_vision_runtime_config.ncnn_label_count)
    {
        *error_message = "vision.runtime.ncnn.label_count does not match labels array length";
        return false;
    }
    if (!require_bool(values, &consumed, "vision.runtime.web.enabled", &g_vision_runtime_config.udp_web_enabled, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.max_fps", &udp_web_max_fps, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.web.send_gray_jpeg", &g_vision_runtime_config.udp_web_send_gray_jpeg, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.gray_image_format", &g_vision_runtime_config.udp_web_gray_image_format, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.web.send_rgb_jpeg", &g_vision_runtime_config.udp_web_send_rgb_jpeg, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.rgb_image_format", &g_vision_runtime_config.udp_web_rgb_image_format, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.data_profile", &g_vision_runtime_config.udp_web_data_profile, error_message) ||
        !require_bool(values, &consumed, "vision.runtime.web.tcp_enabled", &g_vision_runtime_config.udp_web_tcp_enabled, error_message) ||
        !require_string(values, &consumed, "vision.runtime.web.server_ip", &g_string_storage.udp_web_server_ip, &g_vision_runtime_config.udp_web_server_ip, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.video_port", &udp_web_video_port, error_message) ||
        !require_int(values, &consumed, "vision.runtime.web.meta_port", &udp_web_meta_port, error_message))
    {
        return false;
    }

    g_vision_runtime_config.udp_web_max_fps = static_cast<uint32>(std::max(udp_web_max_fps, 0));
    g_vision_runtime_config.udp_web_video_port = static_cast<uint16>(std::max(udp_web_video_port, 0));
    g_vision_runtime_config.udp_web_meta_port = static_cast<uint16>(std::max(udp_web_meta_port, 0));

    if (g_vision_runtime_config.ncnn_label_count > VISION_NCNN_CONFIG_MAX_LABELS)
    {
        *error_message = "vision.runtime.ncnn.label_count exceeds max";
        return false;
    }

    if (consumed.size() != values.size())
    {
        std::ostringstream oss;
        bool first = true;
        for (const auto &entry : values)
        {
            if (consumed.find(entry.first) != consumed.end())
            {
                continue;
            }
            if (!first)
            {
                oss << ", ";
            }
            oss << entry.first;
            first = false;
        }
        *error_message = "unknown keys in config: " + oss.str();
        return false;
    }

    return true;
}

bool load_from_path(const std::string &path, std::string *error_message)
{
    RawMap values;
    if (!parse_key_values_file(path, &values, error_message))
    {
        return false;
    }
    return apply_values(values, error_message);
}

} // namespace

bool smartcar_config_load_from_default_locations(std::string *loaded_path, std::string *error_message)
{
    std::lock_guard<std::mutex> lock(g_config_mutex);
    const std::string target_path = executable_dir_config_path();
    if (target_path.empty())
    {
        if (error_message != nullptr)
        {
            *error_message = "cannot resolve config path from /proc/self/exe";
        }
        return false;
    }

    if (!load_from_path(target_path, error_message))
    {
        return false;
    }

    g_loaded_config_path = target_path;
    if (loaded_path != nullptr)
    {
        *loaded_path = target_path;
    }
    return true;
}

bool smartcar_config_apply_toml_text(const std::string &toml_text,
                                     std::vector<std::string> *restart_required_keys,
                                     std::string *error_message)
{
    std::lock_guard<std::mutex> lock(g_config_mutex);

    const ConfigSnapshot old_config = capture_config_snapshot();
    RawMap values;
    if (!parse_key_values_text(toml_text, &values, error_message))
    {
        return false;
    }
    if (!apply_values(values, error_message))
    {
        restore_config_snapshot(old_config);
        return false;
    }

    collect_restart_required_keys(old_config, restart_required_keys);
    apply_runtime_changes_after_commit();
    return true;
}

bool smartcar_config_read_loaded_text(std::string *toml_text,
                                      std::string *loaded_path,
                                      std::string *error_message)
{
    std::lock_guard<std::mutex> lock(g_config_mutex);

    const std::string path = g_loaded_config_path.empty() ? executable_dir_config_path() : g_loaded_config_path;
    if (path.empty())
    {
        if (error_message != nullptr)
        {
            *error_message = "cannot resolve config path from /proc/self/exe";
        }
        return false;
    }

    std::ifstream input(path, std::ios::binary);
    if (!input.is_open())
    {
        if (error_message != nullptr)
        {
            *error_message = "cannot open file: " + path;
        }
        return false;
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();
    if (toml_text != nullptr)
    {
        *toml_text = buffer.str();
    }
    if (loaded_path != nullptr)
    {
        *loaded_path = path;
    }
    return true;
}

std::string smartcar_config_loaded_path()
{
    std::lock_guard<std::mutex> lock(g_config_mutex);
    return g_loaded_config_path.empty() ? executable_dir_config_path() : g_loaded_config_path;
}
