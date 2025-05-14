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
    fov = 60,
    GUI = False,scripts/user_guide/parallel_visualized.py
)

### building parallel environments
B = 20
scene.build(n_envs=B, env_spacing=(1.0, 1.0))

### activate camera
cam.start_recording()

# control all the robots
franka.control_dofs_position(
    torch.tile(
        torch.tensor([0, 0, 0, -1.0, 0, 0, 0, 0.02, 0.02], device=gs.device), (B, 1)
    ),
)

for i in range(1000):
    scene.step()
    cam.render()

### saving video
cam.stop_recording(save_to_filename="parallel_visualized_video.mp4", fps=60)
