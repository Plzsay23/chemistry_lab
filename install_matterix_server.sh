#!/usr/bin/env bash
# MatteriX install + headless smoke test for the lab Isaac server (Ubuntu 22.04, GPU1 = RTX PRO 5000).
# Usage (on the server):  bash install_matterix_server.sh
# Versions come from the isaaclab-3.0.0b2.post1 wheel METADATA:
#   python>=3.12, torch>=2.10, isaacsim[all,extscache]==6.0.1.0
set -euo pipefail

ENV_NAME="${ENV_NAME:-matterix}"          # separate from KRIT's `isaac` env on purpose
WORK_DIR="${WORK_DIR:-$HOME/Desktop/sj}"
GPU_IDX="${GPU_IDX:-1}"                    # 1 = RTX PRO 5000, 0 = A6000

source "$(conda info --base)/etc/profile.d/conda.sh"

# 1) conda env
if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  conda create -y -n "$ENV_NAME" python=3.12
fi
conda activate "$ENV_NAME"
python -m pip install --upgrade pip

# 2) torch + Isaac Lab 3.0 (pulls Isaac Sim 6.0.1)
pip install -U torch==2.10.0 torchvision==0.25.0 --index-url https://download.pytorch.org/whl/cu128
pip install "isaaclab[isaacsim,all]==3.0.0b2.post1" --extra-index-url https://pypi.nvidia.com

# 3) git-lfs without sudo (shared server)
command -v git-lfs >/dev/null 2>&1 || conda install -y -c conda-forge git-lfs
git lfs install

# 4) clone Matterix + LFS assets
mkdir -p "$WORK_DIR" && cd "$WORK_DIR"
if [ ! -d Matterix ]; then
  git clone --recurse-submodules https://github.com/AccelerationConsortium/Matterix.git
fi
cd Matterix
git submodule update --init --recursive
git submodule foreach 'git lfs pull'

# 5) Matterix packages (matterix_sm, matterix_assets, matterix_tasks, matterix)
python -m pip install -r source.txt

# 6) checks
python - <<'PY'
import torch, isaaclab, importlib.metadata as md
print("torch", torch.__version__, "cuda", torch.version.cuda, "| devices:", torch.cuda.device_count())
print("isaacsim", md.version("isaacsim"), "| isaaclab", md.version("isaaclab"))
PY
ls source/matterix_assets/data/labware/beaker500ml/   # LFS files must not be ~130-byte pointers
python scripts/list_envs.py

# 7) headless smoke test on GPU1
if [ -f "$HOME/Desktop/sj/isaac_env.sh" ]; then
  source "$HOME/Desktop/sj/isaac_env.sh" "$GPU_IDX"
else
  export CUDA_VISIBLE_DEVICES="$GPU_IDX"
fi
ulimit -n 65535 || ulimit -n 4096
python scripts/zero_agent.py --task Matterix-Test-Beakers-Franka-v1 --num_envs 1 --headless 2>&1 | tee "$WORK_DIR/matterix_smoke.log"
