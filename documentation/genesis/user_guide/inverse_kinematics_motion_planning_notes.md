# Inverse Kinematics & Motion Planning Notes
A walkthrough of the components found in the `ik_mp.py` script from the Genesis [user guide](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/inverse_kinematics_motion_planning.html).

#### Basic Introduction to Inverse Kinematics & Motion Planning
In our previous scripts, we defined a specific `position`, `velocity`, or `force` on each degree of freedom to manipulate the Franka arm into various positions. What if we wanted the gripper of the Franka arm to be in a specific position and orientation, like if we wanted it to pick up a cube and move it?

This becomes an inverse kinematics problem, where we begin with an end-effector, the gripper/hand of the Franka arm, the desired position and orientation that we want the end-effector to reach, and various constraints of the model, such as joint limits. Inverse kinematics solve for the appropriate joint configuration(s) that allows for the end-effector to be in the target position. With the start and end positions defined, we break down the movement into discrete motions through what's called motion planning to generate a suitable path that the model can take while considering constraints like obstacles, realistic movements, and shortest path efficiency.

[Resources on IK and MP](#Resources)

### ik_mp.py Script

We begin this script like the ones before
```python
import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

import numpy as np
import genesis as gs

### init
gs.init(backend = gs.gpu)

### scene creation
scene = gs.Scene(
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    show_viewer = False,
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 1.0,
        show_link_frame = False,
        show_cameras = False,
        plane_reflection = True,
        ambient_light = (0.1, 0.1, 0.1),
    ),
    renderer = gs.renderers.Rasterizer(),
)

### entities
plane = scene.add_entity(
    gs.morphs.Plane(),
)
cube = scene.add_entity(
    gs.morphs.Box(
        size = (0.04, 0.04, 0.04),
        pos  = (0.65, 0.0, 0.02),
    )
)
franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)

### camera
cam = scene.add_camera(
    res = (640, 480),
    pos = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov = 30,
    GUI = False,
)

### build
scene.build()
```

We then declare the dofs and constants for the PD controller

```python
### setting dofs
motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

### setting control gains for pd controller
franka.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
franka.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
)
franka.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([ 87,  87,  87,  87,  12,  12,  12,  100,  100]),
)
```

The way we define our dofs differ here than before. Because we're only using one model, we can simply use the global dof indices from the arm, and thus, can just use a simple ascending `numpy` array.

We then begin using inverse kinematics

```python
### inverse kinematics and motion planning

# get the end-effector link
end_effector = franka.get_link('hand')

# move to pre-grasp pose
qpos = franka.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.65, 0.0, 0.25]), # Note that in Mujoco (and Genesis), the z value refers to the "height"
    quat = np.array([0, 1, 0, 0]),
)

# gripper open pos
qpos[-2:] = 0.04
```

We declare the `hand` link from the Franka arm as the end-effector for the robot's inverse kinematic solver to hold onto. If we examine the panda.xml file from the Genesis source code, we have the line
```xml
<body name="hand" pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834">
```
that is used to identify what we should pass into the `rigid_entity.get_link()` method. I am not fully sure how to parse through this MJCF file and find the important links, but I believe this `hand` is being imported from the [`hand.xml`](https://github.com/Genesis-Embodied-AI/Genesis/blob/main/genesis/assets/xml/franka_emika_panda/hand.xml) file found in the same directory as the `panda.xml` file. More importantly, the go2 model utilized throughout Genesis is a URDF file, so it'll be more important to analyze that file.

After declaring the end-effector, we create `qpos` using the [`rigid_entity.inverse_kinematics()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/97732afcff357ebe5fe69cf50366758d4cb6fe3a/genesis/engine/entities/rigid_entity/rigid_entity.py#L812) method, where we specify the `link`, `pos`, and `quat`. `pos` refers to the target position and `quat` refers to the target orientation. In this case, we're moving the hand above the cube's location. It is a little difficult to understand the values behind `quat`, but Mujoco files natively use [Quaternions](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) for spatial orientation and rotation, as opposed to URDF which uses [Euler Angles](https://en.wikipedia.org/wiki/Euler_angles) (though the utils library of Genesis conveniently includes a conversion method between the two). Simply put, the array `[0, 1, 0, 0]` correlates to rotating the hand 180 degrees around the x-axis. Recall that in our previous simulations, the hand of the Franka pointed upwards when all the joints were set to 0, so this specifies that we want the hand to be pointing downwards.
This method returns the target positions of the joints in the target position in a `qpos` bundle. Recall that the last two degrees of free of the Franka arm are the sliding fingers in the hand, so we set those joints in our `qpos` to be in an open position.

Following this, we use the [`rigid_entity.plan_path()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/97732afcff357ebe5fe69cf50366758d4cb6fe3a/genesis/engine/entities/rigid_entity/rigid_entity.py#L1449) method to motion plan a route for the model.

```python
path = franka.plan_path(
    qpos_goal     = qpos,
    num_waypoints = 200, # 2s duration
)
```

We pass through the target position and the number of waypoints that correlates to the amount of time the model will take to reach the target position. This method returns a list of waypoints, with each waypoint being an array of the target positions for the dofs at that time step.

Activating the camera, we then execute the movement through the planned path
```python
### activate camera
cam.start_recording()

# executing planned path
for waypoint in path:
    franka.control_dofs_position(waypoint)
    scene.step()
    cam.render()

# reach last waypoint
for i in range(100):
    scene.step()
    cam.render()
```

We iterate through each waypoint in the path and send an instruction to the Franka arm's PD controller to move to the target position at that time step, stepping and rendering each time. We allow the model another 100 time steps because after we send the final waypoint, the PD controller still has to manipulate the model to the final target position.

From here, we instruct the model to "reach" by via inverse kinematics

```python
# reach
qpos = franka.inverse_kinematics(
    link = end_effector,
    pos  = np.array([0.65, 0.0, 0.130]),
    quat = np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(100):
    scene.step()
    cam.render()
```

The only value we manipulate here is the z axis, which means the robot is moving the hand downwards to be at the same level as the cube. We scrap the changes to the fingers of the hand and pass the movement to the main joints of the Franka arm. When you view the visualization of this script, its good to note that, even though we only changed one axis value, multiple joints move for the model to reach the target position. Following this, we send the instruction to grasp the cube

```python
# grasp
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)

for i in range(100):
    scene.step()
    cam.render()
```

Here is a good example of using the `rigid_entity.control_dofs_force()` method, we are applying 0.5 N of inward force to each of the gripper dofs, which results in the hand applying pressure to the cube, but not trying to reach an inner position and "crushing" the cube.

Lastly, we create the instruction to pick up the cube
```python
# lift
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.0, 0.28]),
    quat=np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(200):
    scene.step()
    cam.render()
```

Manipulating only the z-axis again, we create the target position to raise the hand of the Franka while maintaining the downright orientation of the hand. Because we aren't giving any instruction to the gripper dofs, they continue their previous instruction of applying force inwards, which allows the cube to stay inside the hand. Finally, we save the render.

```python
cam.stop_recording(save_to_filename="./mp4/user_guide/ik_mp_video.mp4", fps=60)
```

### Resources
[MathWorks Inverse Kinematics](https://www.mathworks.com/discovery/inverse-kinematics.html) \
[Introduction to Motion Planning](https://robotics.umich.edu/research/focus-areas/motion-planning/) \
[MIT Robotic Manipulation Course](https://manipulation.csail.mit.edu/Fall2024/) \
[Motion Planning Chapter for the Course Above](https://manipulation.csail.mit.edu/trajectories.html#section1)
