#!/usr/bin/env bash
# Isaac Sim 6.0.1 GUI 스트리밍 + ROS2 브리지(내장 Jazzy) 를 한 번에 켠다.
#
#   ./launch_isaac_ros.sh        # GPU 1 = RTX PRO 5000, 스트리밍 포트 49100 / 47998
#   ./launch_isaac_ros.sh 1      # 위와 같음
#   ./launch_isaac_ros.sh 0      # GPU 0 = RTX A6000,   스트리밍 포트 49200 / 48998
#
# WebRTC 클라이언트: Server 100.125.202.90, Ports 는 위 Signal / Stream.
# ROS 쪽 터미널:  conda activate isaac_ros && export ROS_DOMAIN_ID=42 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# Ctrl+C 로 안 꺼지면:  kill <시작할 때 출력되는 PID>
#
# 바꾸고 싶으면 실행 전에 환경변수로: PUBLIC_IP=... ROS_DOMAIN_ID=... ./launch_isaac_ros.sh 0

GPU="${1:-1}"
case "$GPU" in
  1) SIGNAL_PORT=49100; STREAM_PORT=47998; GPU_NAME="RTX PRO 5000" ;;
  0) SIGNAL_PORT=49200; STREAM_PORT=48998; GPU_NAME="RTX A6000" ;;
  *) echo "[launch] GPU 는 0(A6000) 또는 1(PRO 5000) 만 됩니다: '$GPU'"; exit 1 ;;
esac
PUBLIC_IP="${PUBLIC_IP:-100.125.202.90}"
ROS_DOMAIN="${ROS_DOMAIN_ID:-42}"

if ss -ltn 2>/dev/null | grep -q ":${SIGNAL_PORT} "; then
  echo "[launch] 포트 ${SIGNAL_PORT} 가 이미 쓰이는 중입니다 — GPU ${GPU} 용 Isaac 이 이미 켜져 있습니다."
  echo "[launch] 확인: pgrep -af isaacsim.exp.full.streaming"
  exit 1
fi

# conda (비대화형 셸에서도 activate 가 되게)
source "$(conda info --base)/etc/profile.d/conda.sh" || { echo "[launch] conda 를 못 찾았습니다"; exit 1; }
conda activate isaac || { echo "[launch] conda env 'isaac' 활성화 실패"; exit 1; }

# GPU 고정: 기존 스크립트(UUID 로 CUDA_VISIBLE_DEVICES 고정)가 있으면 그걸 쓴다
if [ -f "$HOME/Desktop/sj/isaac_env.sh" ]; then
  source "$HOME/Desktop/sj/isaac_env.sh" "$GPU"
else
  export CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES="$GPU"
fi

ulimit -n 65535 2>/dev/null || ulimit -n 4096 2>/dev/null || true

# ROS2: Isaac 내장 Jazzy 라이브러리 (시스템 ROS 설치 없이)
SITE_PACKAGES="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
ROS_LIB="$SITE_PACKAGES/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"
if [ ! -d "$ROS_LIB" ]; then
  echo "[launch] 내장 ROS 라이브러리가 없습니다: $ROS_LIB"
  exit 1
fi
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="$ROS_DOMAIN"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ROS_LIB"

echo "[launch] GPU ${GPU} (${GPU_NAME})  CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES}"
echo "[launch] 스트리밍 ${PUBLIC_IP}  Signal ${SIGNAL_PORT} / Stream ${STREAM_PORT}"
echo "[launch] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  (ROS 쪽도 같은 값이어야 토픽이 보입니다)"
echo "[launch] PID $$  — Ctrl+C 로 안 꺼지면: kill $$"

exec isaacsim isaacsim.exp.full.streaming.kit --no-window \
  --/exts/omni.kit.livestream.app/primaryStream/publicIp="$PUBLIC_IP" \
  --/exts/omni.kit.livestream.app/primaryStream/signalPort="$SIGNAL_PORT" \
  --/exts/omni.kit.livestream.app/primaryStream/streamPort="$STREAM_PORT"
