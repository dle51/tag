import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

import numpy as np
import genesis as gs

### init
gs.init()

### scene
scene = gs.Scene(
    show_viewer = False,
    rigid_options=gs.options.RigidOptions(
        enable_joint_limit = False,
    ),
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 1.0,
        show_link_frame = False,
        show_cameras = False,
        plane_reflection = True,
        ambient_light = (0.1, 0.1, 0.1),
    ),
)

### entities
plane = scene.add_entity(
    gs.morphs.Plane(),
)
robot = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)

### camera
cam = scene.add_camera(
    res = (1280, 720),
    pos = (5.25, 0.0, 3.75),
    lookat = (0, 0, 0.5),
    fov = 60,
    GUI = False,
)

### build
n_envs = 16
scene.build(n_envs=n_envs, env_spacing=(1.0, 1.0))

### ik&mp
target_quat = np.tile(np.array([0, 1, 0, 0]), [n_envs, 1]) # pointing downwards
center = np.tile(np.array([0.4, -0.2, 0.25]), [n_envs, 1])
#radius of the circle 
r = 0.1
# random speeds for each environment
angular_speed = np.random.uniform(-10, 10, n_envs)

ee_link = robot.get_link('hand')

### activate camera
cam.start_recording()

### movement
for i in range(0, 1000):
    target_pos = np.zeros([n_envs, 3])
    target_pos[:, 0] = center[:, 0] + np.cos(i/360*np.pi*angular_speed) * r
    target_pos[:, 1] = center[:, 1] + np.sin(i/360*np.pi*angular_speed) * r
    target_pos[:, 2] = center[:, 2]
    target_q = np.hstack([target_pos, target_quat])

    q = robot.inverse_kinematics(
        link     = ee_link,
        pos      = target_pos,
        quat     = target_quat,
        rot_mask = [False, False, True], # for demo purpose: only restrict direction of z-axis
    )

    robot.set_qpos(q)
    scene.step()
    cam.render()

cam.stop_recording(save_to_filename="./mp4/user_guide/parallel_ik_video.mp4", fps=60)
