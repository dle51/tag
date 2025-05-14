# Parallelization Notes
A walkthrough of the components in the parallel_visualized.py and parallel_benchmark.py scripts from the Genesis User Guide

The main benefit of Genesis over other physics engines for RL learning is its efficiency in training in thousands of environments at once. Focusing on the parallel_visualized.py script first, we have a basic creation of the scene containing a plane, Franka arm, and a camera

```python
import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

import genesis as gs
import torch

### init
gs.init(backend=gs.gpu)

### scene
scene = gs.Scene(
    show_viewer    = False,
    rigid_options = gs.options.RigidOptions(
        dt                = 0.01,
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

### entities
plane = scene.add_entity(
    gs.morphs.Plane(),
)

franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)

### camera
cam = scene.add_camera(
    res = (640, 480),
    pos = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov = 60, # Setting this to 60 to see more models at once.
    GUI = False,
)
```

When we call the `build` method onto the scene, we add a few extra parameters. Here is the signature of the `build` method and a description of `n_envs` and `env_spacing` from the source code
```python
def build(
        self,
        n_envs=0,
        env_spacing=(0.0, 0.0),
        n_envs_per_row=None,
        center_envs_at_origin=True,
        compile_kernels=True,
    ):
'''
Parameters
n_envs : int
    n_envs: this specifies how many batched environments you want to create
env_spacing : tuple of float, shape (2,)
    env_spacing: the spawned parallel envs share identical states.
    For visualization purpose, you can specify this parameter to ask the visualizer to distribute all the envs in a grid with a distance of (x, y) in meters between each env.
    Note that this only affects the visualization behavior, and doesn’t change the actual position of the entities in each env.
'''
```

Genesis uses the words `batched environments` to refer to the multiple environments that are running in parallel time steps that are all feeding into a combined data set. From my understanding, this allows an agent in a supervised or reinforcement learning environment to get more data that is more diverse. 

We have the following in our script:
```python
### building parallel environments
B = 20
scene.build(n_envs=B, env_spacing=(1.0, 1.0))
```

Using `B` as the amount of parallel environments we are creating, we can creating 20 parallel environments 1 meter apart from each other (purely for visualization)
