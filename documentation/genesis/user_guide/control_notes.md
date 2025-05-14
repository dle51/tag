## Control Notes
A walkthrough of the components in control.py

We begin the script just like the previous script, adding one extra line

```python
import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

import numpy as np
import genesis as gs

### init
gs.init(backend=gs.gpu)

### scene
scene = gs.Scene(
    show_viewer = False,
    vis_options = gs.options.VisOptions(
        show_world_frame = True,
        world_frame_size = 1.0,
        show_link_frame = False,
        show_cameras = False,
        plane_reflection = True,
        ambient_light = (0.1, 0.1, 0.1),
    ),
    sim_options = gs.options.SimOptions(
        dt = 0.01,
    ),
    renderer = gs.renderers.Rasterizer(),
)
```

We add the `sim_options` parameter of dt, which specifies the timestep of the simulator. Following this, we add our entities and camera, then build our scene

```python
### entities
plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
    gs.morphs.MJCF(
        file  = 'xml/franka_emika_panda/panda.xml',
    ),
)

### camera
cam = scene.add_camera(
    res = (640, 480),
    pos = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov = 30,
    GUI = False,
)

### build scene
scene.build()
```

This is pretty much identical to our previous visualization script where the model loads and then falls to the ground. This script serves as an introduction to manipulating a robot entity through Genesis's built-in PD controller (proportional-derivative). We can select a `joint` in a model, such as one of the 9 joints in a Franka arm, and input in a position or velocity for that joint as a kind of instruction. Now, there is a lot that goes into joints and `dof` (degrees-of-freedom) that biometric and robotic courses go very in-depth about. There's a lot of online resources in understand the concepts at a low and high level, but here are some I found helpful: \
[PD Controller Introduction](https://maniskill.readthedocs.io/en/latest/user_guide/concepts/controllers.html) \
An introduction to PD controllers using the Franka Emika Arm (the one we're using in this script!).
[PID Control in Physics Bodies in Video Games](https://gamedev.net/tutorials/programming/math-and-physics/pid-control-of-physics-bodies-r3885/) \
*A PID Controller is the same concept of a PD controller, but has an additional integral term for stability*

Even though this is an article for Gamedev, they talk about how PID controllers interact with turning a model and collisions with other models that I found extremely helpful in the context of handling entities in any physics engines. You can even find online guides on very simple PID implementations of rolling a ball in Roblox. \
[Northwestern's Lecture on Degrees of Freedom of a Robot](https://modernrobotics.northwestern.edu/nu-gm-book-resource/2-2-degrees-of-freedom-of-a-robot/) \
The first half of this video had really nice visuals that helped my understanding of a joint vs. a degree-of-freedom. The second half of the video was math-based and I didn't understand much of it. \
[ROS <Joint> Documentation](https://wiki.ros.org/urdf/XML/joint) \
The ROS.org documentation of what a joint element. \
[Mujoco Joint Documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint) \
I used this documentation to identify the joints discussed later in this file.

Recall that we added a franka arm into our scene using the function
```python
gs.morphs.MJCF(file  = 'xml/franka_emika_panda/panda.xml')
```

If we go into the Genesis repository and navigate to `genesis/assets/xml/franka_emika_panda/panda.xml`, we can examine lines the xml file to determine how many joints there are and what type of joint they are. Keep in mind that it's better to go to the documentation of the model itself instead of trying to figure it out via the XML file. While I didn't use the documentation in this instance, this [brochure](https://pkj-robotics.dk/wp-content/uploads/2020/09/Franka-Emika_Brochure_EN_April20_PKJ.pdf) has a lot of the technical data we would need. We are looking for any declaration that looks 
```xml
<joint ... />
```
With this convention, we can identify 9 different joints:
```xml
<joint name="joint1"/> <!-- Line 138 -->
<joint name="joint2" range="-1.7628 1.7628"/> <!-- Line 144 -->
<joint name="joint3"/> <!-- Line 148 -->
<joint name="joint4" range="-3.0718 -0.0698"/> <!-- Line 159 -->
<joint name="joint5"/> <!-- Line 168 -->
<joint name="joint6" range="-0.0175 3.7525"/> <!-- Line 178 -->
<joint name="joint7"/> <!-- Line 200 -->
<joint name="finger_joint1" class="finger"/> <!-- Line 220 -->
<joint name="finger_joint2" class="finger"/> <!-- Line 232 -->
```

We should also take note line `11-13`, which defines what the `finger` class is
```xml
<default class="finger">
    <joint axis="0 1 0" type="slide" range="0 0.04"/>
</default>
```

Joints 1-7 don't contain a defined `type` or `class`, so by default, Mujoco sets these to `hinge` type joints, which just means they have one rotational degree of freedom. The last two joints are defined as a `slide` type joint, meaning they have one translational degree of freedom, defined by position and sliding direction. From this, we know that joints 1-7 are the rotating portions of the arm itself and that finger joints 1-2 are the pinchers at the top of the arm.

Getting back to the actual script now, we can list the model file and map them to the degree of freedom index inside the Genesis simulator.

```python
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
```

Here, we use the method `.dof_idx_local`, which has the description "Returns the local Degrees of Freedom indices of the joint in the entity". This only returns of the index with respect to the robot itself, so if we wanted to get the global index of a joint in the scene, like when we have multiple robots in one scene, we should use `dofs_idx()` instead, which is described by "Returns all the Degrees' of Freedom (DoF) indices of the joint in the rigid solver as a sequence".

We then have to define the control gains of the degrees of freedom, which relates to the equation of the PD controller itself where we are defining the constants.

```python
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
```

These values are usually imported in from the MJCF or URDF file, but we can fine tune these values ourself or use the [documentation](https://github.com/frankaemika/franka_ros/blob/develop/franka_description/robots/panda/joint_limits.yaml) of the model to set them. If you've taken a dynamical systems or even a differential equations course, these values will make a lot of sense. The `kp` value relates to the proportional gain, the stiffness, `kv` relates to the derivative gain, the damping, and `force_range` defines the minimum and maximum torque values for safety. These `set_dofs` methods take in the Numpy array and correlate each integer with the dofs index, such that joint 1 takes in a `kp` value of 4500, `kv` value of `450`, and `force_range` value of -87-87.

Following these declarations, we then start the recording and begin moving the arm.

```python
### activate camera
cam.start_recording()
```

Before using the PD controller, we can manipulate the arm in a non-realistic way by instantly modifying the state of the `dofs`.

```python
### hard reset
for i in range(750):
    if i == 0:
        franka.set_dofs_position(np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]), dofs_idx)
    elif i == 250:
        franka.set_dofs_position(np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]), dofs_idx)
    elif i == 500:
        franka.set_dofs_position(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]), dofs_idx)

    scene.step()
    cam.render()
```
Here, we use the method `rigid_entity.set_dofs_position`, which has the following signature
```python
def set_dofs_position(self, position, dofs_idx_local=None, envs_idx=None, *, zero_velocity=True, unsafe=False)
```

The numbers in this implementation likely refer to the rotational amount of the joint in radians. This loop represents the first 12 seconds of the related mp4 file, and we can see that ~8 seconds, the dofs of the model is set to all 0, which corresponds to the upright 'default' position. Like in the visualization script, we render after each step.

Next, we manipulate the arm using the PD controller.

```python
### pd control
for i in range(1250):
    if i == 0:
        franka.control_dofs_position(
            np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]),
            dofs_idx,
        )
    elif i == 250:
        franka.control_dofs_position(
            np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]),
            dofs_idx,
        )
    elif i == 500:
        franka.control_dofs_position(
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
            dofs_idx,
        )
    elif i == 750:
        # control first dof with velocity, and the rest with position
        franka.control_dofs_position(
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])[1:],
            dofs_idx[1:],
        )
        franka.control_dofs_velocity(
            np.array([1.0, 0, 0, 0, 0, 0, 0, 0, 0])[:1],
            dofs_idx[:1],
        )
    elif i == 1000:
        franka.control_dofs_force(
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
            dofs_idx,
        )
    # This is the control force computed based on the given control command
    # If using force control, it's the same as the given control command
    print('control force:', franka.get_dofs_control_force(dofs_idx))

    # This is the actual force experienced by the dof
    print('internal force:', franka.get_dofs_force(dofs_idx))

    scene.step()
    cam.render()

### saving video
cam.stop_recording(save_to_filename="controlVideo.mp4", fps=60)
```

In the three instances, when `0 <= i < 750`, we use `rigid_entity.control_dofs_position()` to have the dofs move to their intended positions instead of teleporting like when we used `set_dofs_position()`. Note that, unlike previously, using `control_*` stores the target value and the model will hold that position until a new target is given.

In the instance of `i == 750`, we use a position command for all but the first joint, where we instead use the method `rigid_entity.control_dofs_velocity()`, setting the first degree of freedom to 1 radians per second of velocity.

In the last instance of `i == 1000`, we use the method `rigid_body.control_dofs_force()`, where we set each dof to 0. The value here correlates to the amount of force acting on the degree of freedom in Newton-Metres for hinges and Newtons for the slide joints. By setting this value to 0, we are essentially 'resetting' the arm, allowing gravity to have full control on the model according to things such as friction. I was wondering why this would be useful, as this seems to be a very difficult and non-intuitive way of interacting with the model. Using force instead of position or velocity allows the model to interact with the environment in an adaptive way, such as trying to pick up an object where we want a claw to exert a certain amount of force on the object instead of attempting to reach a specific location.

Following this, we simply render a frame and save the mp4 file when completed.
