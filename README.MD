# Tag
### Robotic Tag with Preference Learning
[Genesis Physics Engine](https://github.com/Genesis-Embodied-AI/Genesis/tree/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8)

## Quick Start

1. Install [UV](https://docs.astral.sh/uv/) project manager if you haven't already
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

2. Run the script to create the UV project environment and add dependencies - Asks you if you want to install motion planning library (not required)
```bash
make setup
```

3. Run one of the user guide scripts and export the video the view! (or view in VSCode via ssh!)
```bash
uv run scripts/user_guide/control.py
```
```bash
# on local machine
scp dij:~/tag/mp4/user_guide/control_video.mp4 .
# use your favorite media player!
mpv control_video.mp4
```
