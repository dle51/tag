#!/bin/bash

EXP_NAME="DefaultFast"
LOG_PATH="logs/$EXP_NAME"

for ((ckpt=500; ckpt<=11500; ckpt+=500)); do
    echo "Running checkpoint $ckpt..."
    uv run scripts/go2_eval.py --exp-name=$EXP_NAME --ckpt=$ckpt --path=$LOG_PATH
done
