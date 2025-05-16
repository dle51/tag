# Parallel Inverse Kinematic Notes
Overview of the `parallel_ik.py` script from the [Genesis User Guide](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/advanced_ik.html). \

Combining two ideas, inverse kinematics and batched environments, we can have each environment's Franka arm's trace a circle like in the previous script `advanced_ik.py`, but we are setting the Franka's hand as the end-effector and having each environment trace the circle at a different speed.

The concept of applying different constants to each environment will be extremely important for us later, as it allows for us to randomize parameters in both the scene, like friction constants, and the model itself, like different initial joint positions, randomized center of mass positions, and more.

Let's begin our script just like before.

```python
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
```

A slight modification of the camera - I scaled the resolution up, moved the camera back slightly, and increased the FOV to allow for all the environments to be in view at once. This makes the video quite a bit larger, so feel free to reduce these settings.

Building the parallel environments like we did in `parallel_visualization.py`

```python
### build
n_envs = 16
scene.build(n_envs=n_envs, env_spacing=(1.0, 1.0))
```

We then set some target position values like in `advanced_ik.py`

```python
### target positions
# orient the fingers downwards
target_quat = np.tile(np.array([0, 1, 0, 0]), [n_envs, 1])
# center of the circle to be traced
center = np.tile(np.array([0.4, -0.2, 0.25]), [n_envs, 1])
#radius of the circle 
r = 0.1
# random speeds for each environment
angular_speed = np.random.uniform(-10, 10, n_envs)
```

We use the `numpy` functions [`numpy.array`](https://numpy.org/doc/2.1/reference/generated/numpy.array.html), [`numpy.tile`](https://numpy.org/doc/2.1/reference/generated/numpy.tile.html#numpy.tile), and [`numpy.random.uniform`](https://numpy.org/doc/2.2/reference/random/generated/numpy.random.uniform.html#numpy.random.uniform) to generate `n_envs` rows of the array present in `target_quat` and in `center`, as well as `n_envs` random values in the range `[-10, 10)` from a uniform distribution for the `angular_speed` values. 

Before activating the camera and having the models move, we set the end-effector to be the `hand` of the Franka arm.

```python
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
```

The target calculation in this script is similar to the previous `advanced_ik.py` script, so I'll break down each line quickly.

1. We declare the variable `target_pos` and initialize it to be a `n_envs` by `3` `numpy` matrix.
```python
target_pos = np.zeros([n_envs, 3])
```
2. We assign the first column of `target_pos`, which represents the x-axis, to be `center`'s first column (x-axis of center) plus a displacement value by solving a discrete-time parametric equation of a circle, where cosine represents the x-axis.
```python
target_pos[:, 0] = center[:, 0] + np.cos(i/360*np.pi*angular_speed) * r
```
3. We do the same here but with the second column of `target_pos` and `center`, but because this is the y-axis, we use sine instead.
```python
target_pos[:, 1] = center[:, 1] + np.sin(i/360*np.pi*angular_speed) * r
```
4. Because we don't want to manipulate the z-axis, we set the third column of `target_pos` to 0.
```python
target_pos[:, 2] = center[:, 2]
```
5. Here is an example of how we can calculate our `qpos` and save the value in our end-effector link, though we don't use it in this example. We use the [`numpy.hstack()`](https://numpy.org/doc/stable/reference/generated/numpy.hstack.html) method to merge together the target position and orientation. This method essentially takes two matrices and combines the last dimension if the other dimensions match. In this case, we have `target_pos`, which is a `16`x`3` matrix, and we have `target_quat`, which is a `16`x`4` matrix. The resulting matrix from the method is a `16`x`7` matrix, where the first 3 values in each row are the values from `target_pos` and the last 4 values in each row are from `target_quat`. This is the generalized coordinate `q` of 7 vectors.
```python
target_q = np.hstack([target_pos, target_quat])
```

After calculating, we send the `target_pos` and `target_quat` to the inverse kinematic solver.

```python
    q = robot.inverse_kinematics(
        link     = ee_link,
        pos      = target_pos,
        quat     = target_quat,
        rot_mask = [False, False, True], # for demo purpose: only restrict direction of z-axis
    )
    robot.set_qpos(q)
    scene.visualizer.update()
    cam.render()
```

Same as before, we set our `link` to the end-effector, the `pos` to our `target_pos`, `quat` to our target orientation, and set a `rot_mask` for demonstration before setting the joint positions of the model using [`rigid_entity.set_qpos()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/engine/entities/rigid_entity/rigid_entity.py#L1996), which functions the same as [`set_dofs_position()`](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1d88cb9525f121b7e296674abbbba9c8c8c2c8/genesis/engine/entities/rigid_entity/rigid_entity.py#L2107) in this usage. We update the scene visualizer like before and render a frame before saving the visualization.

```python
cam.stop_recording(save_to_filename="./mp4/user_guide/parallel_ik_video.mp4", fps=60)
```

Try running this script a couple times and you can see how each run differs. Also note that, because we're not using the PD controller, a lot of the movements seem sudden. If we change `set_qpos()` to `control_dofs_position()` and `scene.visualizer.update()` to `scene.step()`, you'll see a more realistic rotation, but you'll most likely see the Franka arm clip into itself when it goes from the initial 0 position of all 9 `dofs` to the first `qpos`. This is because inverse kinematics only consider "internal" constraints, like joint limits, but don't factor in collision. While we won't be using motion planning for tag, this would be a good place to implement it.
