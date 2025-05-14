import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

import genesis as gs
import torch
import numpy as np

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
    GUI = False
)

### building parallel environments
B = 20
scene.build(n_envs=B, env_spacing=(1.0, 1.0))

### specifying joint names and dofs

jnt_names = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
    'joint7',
    'finger_joint1',
    'finger_joint2',
]
dofs_idx = [franka.get_joint(name).dof_idx_local for name in jnt_names]

### control gains
franka.set_dofs_kp(
    kp             = np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
    dofs_idx_local = dofs_idx,
)
franka.set_dofs_kv(
    kv             = np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
    dofs_idx_local = dofs_idx,
)
franka.set_dofs_force_range(
    lower          = np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    upper          = np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
    dofs_idx_local = dofs_idx,
)

### activate camera
cam.start_recording()

### Controlling the Robots
for i in range(900):
    if i == 0: # Controlling all Robots at Once
        franka.control_dofs_position(
            torch.tile(
                torch.tensor([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04], device=gs.device), (B, 1)
            ),
        )
    elif i == 180: # Controlling half the robots
        selection = [i for i in range(0, B, 2)]
        franka.control_dofs_position(
            position = torch.tile(
                torch.tensor([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04], device=gs.device), (len(selection), 1)
            ),
            envs_idx = torch.tensor(selection, device = gs.device),
        )
    elif i == 360: # Reset All
        franka.control_dofs_position(
            position = torch.zeros(B, 8, device = gs.device),
            dofs_idx_local = dofs_idx[1:],
        )
        franka.control_dofs_velocity(
            velocity = torch.tile(
                torch.tensor([1.0], device = gs.device), (B, 1)
            ),
            dofs_idx_local = dofs_idx[:1],
        )
    elif i == 540: # Free Fall All
        franka.control_dofs_force(
            torch.zeros(B, 9, device=gs.device),
        )
    
    # This is the control force computed based on the given control command
    # If using force control, it's the same as the given control command
    print('control force:', franka.get_dofs_control_force(dofs_idx))

    # This is the actual force experienced by the dof
    print('internal force:', franka.get_dofs_force(dofs_idx))

    scene.step()
    cam.render()

### saving video
cam.stop_recording(save_to_filename="./mp4/user_guide/parallel_visualized_video.mp4", fps=60)
