# Advanced Inverse Kinematic Notes
Overview of the `advanced_ik.py` script from the [Genesis User Guide](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/advanced_ik.html). \

In the previous IK script, we defined only one link, the gripper of the Franka arm, as the end-effector. In this script, we are going to define each of the fingers in the gripper as two separate target links and having them trace a horizontal circle in midair. We begin this script the same way as before.

```python
import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

import numpy as np
import genesis as gs

### init
gs.init(seed=0, precision='32', logging_level='debug')
```

In this example, the guide passes a few parameters we haven't seen before to the [`genesis.init()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/__init__.py#L49) method. Here, `seed` allows us to control the stochastic behavior of the `random`, `numpy`, and `torch` libraries. `taichi` only supports deterministic functions, so this setting doesn't affect it. `precision` defines the bit precision of `taichi`, `numpy`, and `torch`, where 32 bit is faster but less precise and 64 bit is the opposite. Finally, `logging_level` defines which logs get printed to console.

From here, we define our scene and entities like usual.

```python
### scene
scene = gs.Scene(
    show_viewer = False,
    rigid_options=gs.options.RigidOptions(
        enable_joint_limit=False,
        enable_collision=False,
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
scene.add_entity(
    gs.morphs.Plane(),
)
robot = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)
```

Then we create some target as a mean of visualization.

```python
### Target links for visualization
target_left = scene.add_entity(
    gs.morphs.Mesh(
        file='meshes/axis.obj',
        scale=0.1,
    ),
    surface=gs.surfaces.Default(color=(1, 0.5, 0.5, 1)),
)
target_right = scene.add_entity(
    gs.morphs.Mesh(
        file='meshes/axis.obj',
        scale=0.1,
    ),
    surface=gs.surfaces.Default(color=(0.5, 1.0, 0.5, 1)),
)
```

Looking at the [`scene.add_entity()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/engine/scene.py#L220) method, where we create a morph using the `meshes/axis.obj` file, which is just a small model pointing in the three axes. We also define a surface using the [`Default`](https://github.com/Genesis-Embodied-AI/Genesis/blob/main/genesis/options/surfaces.py#L599) class, which represents plastic in Genesis. We pass color values representing the intensity of `rbga`. The colors of the targets are
<span style="color:rgba(255,127,127,1)">target_left</span> and
<span style="color:rgba(127,255,127,1)">target_right</span>.

After adding these entities, we can add a camera for visualization and build the scene.

```python
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

We then define some target values and the links.

```python
### target values
target_quat = np.array([0, 1, 0, 0])
center = np.array([0.4, -0.2, 0.25])
r = 0.1

# Links
left_finger = robot.get_link('left_finger')
right_finger = robot.get_link('right_finger')
```

Our `target_quat` represents the target orientation of the hand. Recall from the `ik_mp.py` script that the values `[0, 1, 0, 0]` represent a rotation around the x-axis by 180 degrees, where the Franka arm's hand is pointing downwards. We're going to have the hand move around in a circle, so we also define the center point of this circle in space and the radius of the circle, `center` and `r` respectively. Setting our links in the same way as we set the entire hand as the end-effector previously, we can start the camera and movements calculations.

```python
### activate camera
cam.start_recording()

### Movement
for i in range(0, 2000):
    target_pos_left = center + np.array([np.cos(i/360*np.pi), np.sin(i/360*np.pi), 0]) * r
    target_pos_right = target_pos_left + np.array([0.0, 0.03, 0])

    target_left.set_qpos(np.concatenate([target_pos_left, target_quat]))
    target_right.set_qpos(np.concatenate([target_pos_right, target_quat]))
```

As we are moving the hand, or more accurately, the two fingers of the gripper around in a circle, we need to calculate the target position at the current time step. We really are just calculating the position of `target_left`, as `target_right` is based off that position. If you've taken a multivariable calculus course (or a really evil linear algebra course), you can probably see how the math here represents a vector-valued function of a circle, or more accurately, a parametric equation of a circle in vector form, in discrete time, where we calculate the displacement position of the object at a discrete time step and apply a Jacobian transformation to the function to restrict the circle to the xy-plane while the `z` value stays constant.

We aren't using the PD controller for this script, so we can have the visualization axes teleport to where the fingers will be after the inverse kinematic calculation. We can do this with the [`set_qpos()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/engine/entities/rigid_entity/rigid_entity.py#L1996) method.

Using this, we can bundle the target position and orientation together to form the `qpos` of just the targets, saving that to the object to be later passed into the inverse kinematic solver.

```python
q = robot.inverse_kinematics_multilink(
        links    = [left_finger, right_finger],
        poss     = [target_pos_left, target_pos_right],
        quats    = [target_quat, target_quat],
        rot_mask = [False, False, True], # only restrict direction of z-axis
    )
```

We use a new method [`rigid_entity.inverse_kinematics_multilink()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/engine/entities/rigid_entity/rigid_entity.py#L892) that solves for the target positions of the joints where multiple target links are in their desired positions and orientations.

We pass the `left_finger` and `right_finger` as the links to be used as end-effectors, `target_pos_left` and `target_pos_right` as the target positions of those two links respectively, the target orientations of the two end-effectors in quaternions labeled as `quat`, and a `rot_mask`, rotation mask, which can be use to restrict the rotation of a link on a desired axis. In this case, we only care about restricting the movement of the fingers on the z-axis so they continue pointing downwards, but we don't care about restricting the movement on the x and y axes (though I doubt the solver would implement a rotation on these axes anyways). There is also an argument called `pos_mask`, where we can restrict a link's movement on a specific axis, which could be useful in the future for us.

This method returns the target joint positions, `qpos`, where the end-effectors are in their target positions and orientations. We can then send the instruction to the Franka arm model to move to using [`rigid_entity.set_dofs_position()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/engine/entities/rigid_entity/rigid_entity.py#L2107).

```python
    robot.set_dofs_position(q)
    scene.visualizer.update()
    cam.render()
```

Remember that the `qpos` is just the target position we want at that specific time step. For example, when the script starts at second 0.00, we calculate the target position at that time step and send it to the model. If we were using the model's PD controller with the [`control_dofs_position()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/engine/entities/rigid_entity/rigid_entity.py#L2162) instruction, this movement would play out during the timeframe of 0.00 -> 0.01 seconds, where we then calculate a new `qpos` for a new instruction. We also utilize the [`scene.Visualizer.update()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/vis/visualizer.py#L164) method here, instead of [`scene.step`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/engine/scene.py#L714). This is because there isn't any physics-related things happening (like the PD controller). As the model is "teleporting" to the new position, we can simply update the visualization using `scene.Visualizer.update()` instead of simulating the space.

Saving the visualization, the script is complete.

```python
### Saving Video
cam.stop_recording(save_to_filename="./mp4/user_guide/advanced_ik_video.mp4", fps=60)
```

When viewing the video, we can see our two visualization meshes "attached" to where the fingers are. Something I found extremely interesting is, when the finger is moving on the y-axis, the two fingers will prefer to move themselves rather than have the arm move them, so the arm slows down and allows for the fingers to compensate. I doubt this is an optimization in the sense that the fingers use "less" energy to move than the entire arm, but rather, the fingers are the specified end-effectors, so their movement has priority during the inverse kinematics calculation.

Some small things to consider:
1. We set the `logging_level` to `debug`. Pay attention to what prints out when you launch the script compared to the previous scripts.
2. An object's `qpos` depends on what it is. For models like the Franka arm, `qpos` is identical to the `dofs_position` of the joints. For free meshes, like the visualization axes in our script, these "`joint`s" has 6 `dofs`, 3 translational and 3 rotational. The position is a the generalized coordinate `q` (which is why we labeled the `qpos` earlier as `q`), which is a essentially the xyz translation and the wxyz quaternion. In this script, we knew the desired quaternion of the orientation already (default), so we can just change the position of the mesh using `set_qpos()`.
