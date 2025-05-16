#!/bin/bash
# A basic setup script for our tag environment

# exit if failed
set -e

if [ -f README.md ]; then
  echo "Backing up existing README.md..."
  mv README.md README.md.bak
fi

echo "Creating UV environment"
uv init

if [ -f README.md ]; then
  echo "Removing uv-generated README.md..."
  rm README.md
fi

if [ -f main.py ]; then
  echo "Removing uv-generated main file..."
  rm main.py
fi

if [ -f README.md.bak ]; then
  echo "Restoring original README.md..."
  mv README.md.bak README.md
fi

echo "Adding required packages"
uv add torch genesis-world tensorboard pyrender pyopengl-accelerate

# Optional OMPL installation
read -p "Install OMPL for Motion Planning? (y/n): " install_ompl
if [[ "$install_ompl" == "y" || "$install_ompl" == "Y" ]]; then
    echo "Adding local OMPL module"
    wget -P ./modules/ompl/ https://github.com/ompl/ompl/releases/download/prerelease/ompl-1.7.0-cp310-cp310-manylinux_2_28_x86_64.whl
    uv add ./modules/ompl/ompl-1.7.0-cp310-cp310-manylinux_2_28_x86_64.whl
else
    echo "Skipping OMPL installation"
fi

echo "Adding debugging packages"
uv add ipython ipdb

echo "Adding Weights & Biases"
uv add wandb

echo "Completed!"