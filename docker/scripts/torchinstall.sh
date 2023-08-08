# https://github.com/NVIDIA-AI-IOT/YOLOv5-with-Isaac-ROS get the right torchvision version

export BUILD_VERSION=0.13.0

whoami
apt-get update
apt-get install -y libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev cuda-tools-11-4

git clone --branch v0.13.0 https://github.com/pytorch/vision torchvision   # see below for version of torchvision to download
cd torchvision && pip3 install . -v
pip3 install 'pillow<9'

apt purge -y cuda-tools-11-4