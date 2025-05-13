# Tag Initial Setup Guide
Quick guide on how to setup Genesis on the cluster server and get the first two initial scripts running

## Project Manager
We are using the UV project manager for now.
1. [UV](https://docs.astral.sh/uv/) - Python Package and Project Manager
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Making the Project Environment
Once you're in the cluster, you can make a new project directory or initialize from one that's already created.
To make a new project:
```bash
uv init tag
cd tag
```
You can confirm that the project and environment was created successfully by running
```console
user@login:~/tag$ uv run main.py
Hello from tag!
```
If you already have a directory, simply run the following command while in it
```bash
uv init
```

### Project Dependencies
There are some dependencies we need to install in different ways. We're going to either add the dependency directly to our pyproject.toml file, or we're going to pip it into a virtual environment.

Note. Setting this up locally requires a LOT of different dependency installations (looking at you CUDA drivers). Be warned...

```bash
# Required Dependencies
uv add torch genesis-world
# Unsure if these are required
uv add pyrender pyopengl-accelerate
# Dependency for Later
uv add tensorboard
```

## Running hello_genesis

While in the tag project directory, let's make a source directory
```bash
mkdir src
cd src
```

The first script that the Genesis user docs describes is just a simple procedure to load in a model. We simply have to modify two lines to get this to run correctly. I really recommend reading through the user guides [here](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/hello_genesis.html) to understand the script.
```python
# hello_genesis.py
# located in ~tag/src/hello_genesis.py
import genesis as gs
gs.init(backend=gs.gpu) # Changed this to GPU

scene = gs.Scene(show_viewer=False) # Change this to False for headless
plane = scene.add_entity(gs.morphs.Plane())
franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)

scene.build()

for i in range(1000):
    scene.step()
```

You can execute this script with the command
```bash
uv run src/hello_genesis.py
```

While we won't be able to see the model get rendered or anything, we should see the Genesis environment get created and fully execute before closing

#### Note:
 I had issues with my environment where it attempted to grab the Conda pyrender, despite not being in base and removing any mention of trying to run it outside my project environment. I resolved this issue for now by making the script run_clean.py:
```sh
#!/bin/bash
LD_LIBRARY_PATH= uv run "$@"
```
Making it executable:
```bash
chmod +x run_clean.sh
```
and executing the script:
```bash
./run_clean.sh src/hello_genesis.py
```

The second script described is for [visualization & rendering](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/visualization.html). This is important because we're running these simulations in an headless environment and this allows us to get an mp4 file to view the environment. Again, I heavily recommend reading the documentation. We have to add a few lines of code to this script to make it runnable in our environment.
```python
# simple_visualization.py
# located in ~tag/src/simple_visualization.py

# The next two lines are added to make sure this uses the egl interface
import os
os.environ["PYOPENGL_PLATFORM"] = "egl"
import genesis as gs

gs.init(backend=gs.gpu) # Modified like before

scene = gs.Scene(
    show_viewer = False, # Modified like before
    viewer_options = gs.options.ViewerOptions(
        res           = (1280, 960),
        camera_pos    = (3.5, 0.0, 2.5),
        camera_lookat = (0.0, 0.0, 0.5),
        camera_fov    = 40,
        max_FPS       = 60,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 1.0,
        show_link_frame  = False,
        show_cameras     = False,
        plane_reflection = True,
        ambient_light    = (0.1, 0.1, 0.1),
    ),
    renderer=gs.renderers.Rasterizer(),
)

plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)

cam = scene.add_camera(
    res    = (640, 480),
    pos    = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov    = 30,
    GUI    = False,
)

scene.build()

# render rgb, depth, segmentation, and normal
# rgb, depth, segmentation, normal = cam.render(rgb=True, depth=True, segmentation=True, normal=True)

cam.start_recording()
import numpy as np

for i in range(120):
    scene.step()
    cam.set_pose(
        pos    = (3.0 * np.sin(i / 60), 3.0 * np.cos(i / 60), 2.5),
        lookat = (0, 0, 0.5),
    )
    cam.render()
cam.stop_recording(save_to_filename='video.mp4', fps=60)
```
This will save an MP4 file named `video.mp4` to our project root directory. We can scp this file into our local machine and view it using a video player like VLC or MPV
