#!/bin/bash

set -u

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
OUT_DIR="$SCRIPT_DIR/out"
LEGACY_OUT_DIR="$SCRIPT_DIR/../out"
CONFIG_FILE="$SCRIPT_DIR/build_target.env"
CONNECTION_PRESETS_FILE="$SCRIPT_DIR/connection_presets.json"
SYNC_TOML_SCRIPT="$SCRIPT_DIR/sync_connection_preset_to_toml.js"

# 默认目标。若存在 build_target.env，会优先读取该文件覆盖。
TARGET_PRESET="hotspot_a"
TARGET_HOST=""
TARGET_USER="root"
TARGET_PORT="22"
TARGET_APP_PATH="/home/root/infer"
TARGET_CONFIG_PATH="/home/root/infer/smartcar_config.toml"
MAKE_JOBS="12"
CAMERA_CAPTURE_WIDTH="320"
CAMERA_CAPTURE_FPS="120"

print_usage() {
    cat <<EOF
用法:
  ./build.sh
  通过 build_target.env 切换 TARGET_PRESET 即可
  如需临时覆盖:
    ./build.sh --preset hotspot_a
    ./build.sh --preset hotspot_b
    ./build.sh --camera-width 160
    ./build.sh --camera-width 320
    ./build.sh --camera-width 640
    ./build.sh --camera-fps 60
    ./build.sh --camera-fps 90
    ./build.sh --camera-fps 120
    ./build.sh --camera-fps 180
    ./build.sh --jobs 16

说明:
  1. 默认会读取同目录下的 build_target.env
  2. 热点/IP 预设统一维护在 connection_presets.json
  3. 推荐只改 build_target.env 里的 TARGET_PRESET
  4. 命令行参数优先级高于 build_target.env
  5. 当前目标:
     preset=$TARGET_PRESET
     host=$TARGET_HOST
     user=$TARGET_USER
     port=$TARGET_PORT
     app_path=$TARGET_APP_PATH
     config_path=$TARGET_CONFIG_PATH
     camera_width=$CAMERA_CAPTURE_WIDTH
     camera_fps=$CAMERA_CAPTURE_FPS
EOF
}

if [ -f "$CONFIG_FILE" ]; then
    # shellcheck disable=SC1090
    . "$CONFIG_FILE"
fi

if [[ "$CONNECTION_PRESETS_FILE" != /* ]]; then
    CONNECTION_PRESETS_FILE="$SCRIPT_DIR/${CONNECTION_PRESETS_FILE#./}"
fi

load_target_preset() {
    if [ ! -f "$CONNECTION_PRESETS_FILE" ]; then
        echo "找不到预设文件: $CONNECTION_PRESETS_FILE"
        exit 1
    fi

    local preset_output
    preset_output=$(node -e '
const fs = require("fs");
const file = process.argv[1];
const presetId = process.argv[2];
const data = JSON.parse(fs.readFileSync(file, "utf8"));
if (!data || !Array.isArray(data.presets)) {
  throw new Error("connection_presets.json 缺少 presets");
}
const preset = data.presets.find((item) => item.id === presetId);
if (!preset) {
  throw new Error(`未找到预设: ${presetId}`);
}
const values = [
  preset.build_target_host || preset.board_ssh_host || "",
  preset.build_target_user || preset.board_ssh_user || "root",
  String(preset.build_target_port || preset.board_ssh_port || 22),
  preset.build_target_app_path || "/home/root/infer",
  preset.build_target_config_path || preset.board_ssh_target_path || "/home/root/infer/smartcar_config.toml"
];
process.stdout.write(values.join("\n"));
' "$CONNECTION_PRESETS_FILE" "$TARGET_PRESET") || {
        echo "读取预设失败: $TARGET_PRESET"
        exit 1
    }

    mapfile -t preset_values <<< "$preset_output"
    TARGET_HOST="${preset_values[0]:-}"
    TARGET_USER="${preset_values[1]:-}"
    TARGET_PORT="${preset_values[2]:-22}"
    TARGET_APP_PATH="${preset_values[3]:-/home/root/infer}"
    TARGET_CONFIG_PATH="${preset_values[4]:-/home/root/infer/smartcar_config.toml}"
}

load_target_preset

while [ $# -gt 0 ]; do
    case "$1" in
        --preset)
            TARGET_PRESET="$2"
            load_target_preset
            shift 2
            ;;
        --host)
            TARGET_HOST="$2"
            shift 2
            ;;
        --user)
            TARGET_USER="$2"
            shift 2
            ;;
        --port)
            TARGET_PORT="$2"
            shift 2
            ;;
        --app-path)
            TARGET_APP_PATH="$2"
            shift 2
            ;;
        --config-path)
            TARGET_CONFIG_PATH="$2"
            shift 2
            ;;
        --jobs)
            MAKE_JOBS="$2"
            shift 2
            ;;
        --camera-width)
            CAMERA_CAPTURE_WIDTH="$2"
            shift 2
            ;;
        --camera-fps)
            CAMERA_CAPTURE_FPS="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            print_usage
            exit 1
            ;;
    esac
done

if [ -z "$TARGET_HOST" ] || [ -z "$TARGET_USER" ]; then
    echo "TARGET_HOST / TARGET_USER 不能为空。"
    exit 1
fi

if [ "$CAMERA_CAPTURE_WIDTH" != "160" ] && [ "$CAMERA_CAPTURE_WIDTH" != "320" ] && [ "$CAMERA_CAPTURE_WIDTH" != "640" ]; then
    echo "CAMERA_CAPTURE_WIDTH 仅支持 160 / 320 / 640，当前: $CAMERA_CAPTURE_WIDTH"
    exit 1
fi

if [ "$CAMERA_CAPTURE_FPS" != "60" ] && [ "$CAMERA_CAPTURE_FPS" != "90" ] && [ "$CAMERA_CAPTURE_FPS" != "120" ] && [ "$CAMERA_CAPTURE_FPS" != "180" ]; then
    echo "CAMERA_CAPTURE_FPS 仅支持 60 / 90 / 120 / 180，当前: $CAMERA_CAPTURE_FPS"
    exit 1
fi

if [ ! -d "$OUT_DIR" ] && [ -d "$LEGACY_OUT_DIR" ]; then
    OUT_DIR="$LEGACY_OUT_DIR"
fi

if [ ! -d "$OUT_DIR" ]; then
    mkdir -p "$OUT_DIR" || {
        echo "创建输出目录失败: $OUT_DIR"
        exit 1
    }
fi

UVC_RES_PRESET="1"
if [ "$CAMERA_CAPTURE_WIDTH" = "160" ]; then
    UVC_RES_PRESET="0"
elif [ "$CAMERA_CAPTURE_WIDTH" = "640" ]; then
    UVC_RES_PRESET="2"
fi

UVC_FPS_PRESET="2"
if [ "$CAMERA_CAPTURE_FPS" = "60" ]; then
    UVC_FPS_PRESET="0"
elif [ "$CAMERA_CAPTURE_FPS" = "90" ]; then
    UVC_FPS_PRESET="1"
elif [ "$CAMERA_CAPTURE_FPS" = "180" ]; then
    UVC_FPS_PRESET="3"
fi

echo "[BUILD] 目标预设: ${TARGET_PRESET}"
echo "[BUILD] 目标主板: ${TARGET_USER}@${TARGET_HOST}:${TARGET_PORT}"
echo "[BUILD] APP 目标路径: ${TARGET_APP_PATH}"
echo "[BUILD] 配置目标路径: ${TARGET_CONFIG_PATH}"
echo "[BUILD] 摄像头采图宽度: ${CAMERA_CAPTURE_WIDTH} (UVC_RES_PRESET=${UVC_RES_PRESET})"
echo "[BUILD] 摄像头采图帧率: ${CAMERA_CAPTURE_FPS} (UVC_FPS_PRESET=${UVC_FPS_PRESET})"

node "$SYNC_TOML_SCRIPT" "$CONNECTION_PRESETS_FILE" "$TARGET_PRESET" "$SCRIPT_DIR/smartcar_config.toml" || {
    echo "同步 smartcar_config.toml 中的电脑接收端 IP 失败。"
    exit 1
}

cd "$OUT_DIR" || {
    echo "无法进入 $OUT_DIR 目录，请检查目录是否存在。"
    exit 1
}

find . -mindepth 1 ! -name "本文件夹作用.txt" -exec rm -rf {} + || {
    echo "清理输出目录失败。"
    exit 1
}

cmake "$SCRIPT_DIR" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DUVC_RES_PRESET="${UVC_RES_PRESET}" -DUVC_FPS_PRESET="${UVC_FPS_PRESET}" || {
    echo "cmake 命令执行失败。"
    exit 1
}

echo "cmake 命令执行成功。"
make -j"$MAKE_JOBS" || {
    echo "make 编译失败。"
    exit 1
}

echo "生成APP"
app_binary_name=$(basename "$(dirname "$SCRIPT_DIR")")
app_binary_path="$OUT_DIR/$app_binary_name"
ncnn_model_dir="$SCRIPT_DIR/ncnn_model"
image_dir="$SCRIPT_DIR/image"

if [ ! -f "$app_binary_path" ]; then
    echo "找不到可执行文件: $app_binary_path"
    exit 1
fi

ssh -p "$TARGET_PORT" "${TARGET_USER}@${TARGET_HOST}" "
if [ -e '${TARGET_APP_PATH}' ] && [ ! -d '${TARGET_APP_PATH}' ]; then
    echo 'TARGET_APP_PATH_EXISTS_AS_FILE'
    exit 2
fi
mkdir -p '${TARGET_APP_PATH}'
" >/tmp/vision_infer_ssh_prepare.log 2>&1
ssh_status=$?
if [ "$ssh_status" -ne 0 ]; then
    if grep -q "TARGET_APP_PATH_EXISTS_AS_FILE" /tmp/vision_infer_ssh_prepare.log; then
        echo "目标路径已存在同名文件，无法作为目录使用: ${TARGET_APP_PATH}"
        echo "请先在主板上删除该文件，例如: rm -f ${TARGET_APP_PATH}"
    else
        echo "创建目标目录失败: ${TARGET_APP_PATH}"
        cat /tmp/vision_infer_ssh_prepare.log
    fi
    rm -f /tmp/vision_infer_ssh_prepare.log
    exit 1
fi
rm -f /tmp/vision_infer_ssh_prepare.log

scp -O -P "$TARGET_PORT" "$app_binary_path" "${TARGET_USER}@${TARGET_HOST}:${TARGET_APP_PATH}/" || {
    echo "APP 传输失败。"
    exit 1
}

scp -O -P "$TARGET_PORT" "$SCRIPT_DIR/smartcar_config.toml" "${TARGET_USER}@${TARGET_HOST}:${TARGET_CONFIG_PATH}" || {
    echo "配置文件传输失败。"
    exit 1
}

if [ ! -d "$ncnn_model_dir" ]; then
    echo "找不到 ncnn 模型目录: $ncnn_model_dir"
    exit 1
fi

ssh -p "$TARGET_PORT" "${TARGET_USER}@${TARGET_HOST}" "mkdir -p '${TARGET_APP_PATH}/ncnn_model'" || {
    echo "创建目标 ncnn_model 目录失败。"
    exit 1
}

scp -O -P "$TARGET_PORT" \
    "$ncnn_model_dir/tiny_classifier_fp32.ncnn.param" \
    "$ncnn_model_dir/tiny_classifier_fp32.ncnn.bin" \
    "$ncnn_model_dir/labels.txt" \
    "${TARGET_USER}@${TARGET_HOST}:${TARGET_APP_PATH}/ncnn_model/" || {
    echo "ncnn 模型文件传输失败。"
    exit 1
}

if [ ! -d "$image_dir" ]; then
    echo "找不到离线推理图片目录: $image_dir"
    exit 1
fi

ssh -p "$TARGET_PORT" "${TARGET_USER}@${TARGET_HOST}" "mkdir -p '${TARGET_APP_PATH}/image'" || {
    echo "创建目标 image 目录失败。"
    exit 1
}

scp -O -P "$TARGET_PORT" "$image_dir"/* "${TARGET_USER}@${TARGET_HOST}:${TARGET_APP_PATH}/image/" || {
    echo "离线推理图片传输失败。"
    exit 1
}

echo "传输完成"
