# Parallel Simulation Notes
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

Using `B` as the amount of parallel environments we are creating, we can creating 20 parallel environments 1 meter apart from each other (purely for visualization). When visualizing, we can't have that many parallel environments compared to a scene without a camera. From here, I imported the franka `dofs` and PD control gain values from the `control.py` script and started the recording.

```python
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
```

We can now move the Franka arm around just like how we did in the previous script, but I'm implementing a different way of sending the data. Recall that we modified the line `gs.init(backend=gs.gpu)` in all of our scripts to use a GPU as the backend instead of the CPU. Using Numpy works, but when we have a lot of data to send frequently (and with large batch sizes), there is a lot of data going between our CPU and GPU. We can reduce this overhead and improve performance by using [torch tensors](https://docs.pytorch.org/docs/stable/tensors.html). Tensors, to me, are extremely confusing (the official PyTorch [Introduction to PyTorch Tensors](https://www.youtube.com/watch?app=desktop&v=r7QDUPb2dCM) video is almost 40 minutes!) and I learned just enough to implement extremely simple procedures with them. I think that investing some more time into learning the basics will help with this project a lot.

While the user guide uses a singular movement, I recreated the same movements that were present in the `control.py` script. Before I go through that portion of the script, I found it helpful to look at the `rigid_entity.control_dofs_*` methods in the source code. We have the signature and the parameter descriptions
```python
def set_dofs_position(self, position, dofs_idx_local=None, envs_idx=None, *, zero_velocity=True, unsafe=False):
'''
position : array_like - This is velocity or force in the other two methods
    The position to set.
dofs_idx_local : None | array_like, optional
    The indices of the dofs to set. If None, all dofs will be set. Note that here this uses the local `q_idx`, not the scene-level one. Defaults to None.
envs_idx : None | array_like, optional
    The indices of the environments. If None, all environments will be considered. Defaults to None.
'''
```

Disregarding `zero_velocity` and `unsafe`, we have one new parameter, `envs_idx`, to consider now. With this parameter, we can give an instruction to *some* environments instead of all of them. While this doesn't seem beneficial to me at face value, we could use this to randomize certain variables in some environments, such as starting position, in our simulations to give more diverse data.

Getting back to the script, we begin with controlling all the models with one command

```python
for i in range(900):
    if i == 0: # Controlling all Robots at Once
        franka.control_dofs_position(
            torch.tile(
                torch.tensor([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04], device=gs.device), (B, 1)
            ),
        )
```

Here, we use the [torch.tile](https://docs.pytorch.org/docs/stable/generated/torch.tile.html) method, which takes in an input and a dimension, constructing a tensor by repeating the elements of the input until the dimensions are filled. Here is a better way of seeing what is actually happening in the script:
```python
>>> import torch
>>> B = 20
>>> x = torch.tensor([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04])
>>> x.tile(B, 1)
tensor([[1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400],
        [1.0000, 1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0400, 0.0400]])
>>> 
```

This tensor is being passed as the `position` argument of the method, and as there isn't any values passed for `dofs_idx_local` and `envs_idx`, it defaults to the indices being the `dof`, with each row being a different model. This can be seen in the first 3 seconds of the visualization video.

Next, I tried controlling half the robots in the simulation, moving every other one.
```python
    elif i == 180: # Controlling half the robots
        selection = [i for i in range(0, B, 2)]
        franka.control_dofs_position(
            position = torch.tile(
                torch.tensor([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04], device=gs.device), (len(selection), 1)
            ),
            envs_idx = torch.tensor(selection, device = gs.device),
        )
```

I create an array called `selection` to make it easier to select every other environment. The second argument of `torch.tile` is changed from `(B, 1)` to `(len(selection), 1)`, where the tensor has as many rows as the number of models receiving the instruction. `envs_idx` is set using a tensor of the selection array. While half of the Franka arms move via their new instruction, we can see that the other half is holding their previous position. This can be seen from during the next three seconds of the visualization.

The third instruction requires us to use two different `control` methods depending on the `dof`. In this instance, we're inputting an array of zeros into the position instruction, so we can use the method `torch.zeros` to make a tensor of all zeros.

```python
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
```

Looking at our tensor creation in the `position` method, we have `torch.zeros(B, 8, device = gs.device)`. We're creating a 2D tensor, a matrix, with B rows (one row for each arm) and 8 columns (representing all but the first dof). We can set the `dofs_idx_local` with a simple splice, but I bet we could make the `dofs_idx` array a tensor for the input. For velocity, we create a tensor with only one value and create a `B x 1` matrix for the first joint. This can be seen from seconds 6-8 in the visualization.

Lastly, we input 0 force for all of the arms.

```python
elif i == 540: # Free Fall All
    franka.control_dofs_force(
        torch.zeros(B, 9, device=gs.device),
    )
```

Looking at the visualization, we can see that half of the arms fall in a different way than the other half due to the way they are oriented straight up, with the "heavier" portion being the direction the arm falls to. We finish off this script by adding a print statement to show the control and internal forces, stepping through time and rendering, and saving the recording

```python
   # This is the control force computed based on the given control command
    # If using force control, it's the same as the given control command
    print('control force:', franka.get_dofs_control_force(dofs_idx))

    # This is the actual force experienced by the dof
    print('internal force:', franka.get_dofs_force(dofs_idx))

    scene.step()
    cam.render()

### saving video
cam.stop_recording(save_to_filename="./mp4/user_guide/parallel_visualized_video.mp4", fps=60)
```

If we take a peek at the `parallel_benchmark.py` script, there is no visualization, but the number of environments is set to 30000. We can run this script simply to see the amount of frames we get per second. When I ran it on the RTX 4090, I was getting just over 45 million frames a second, letting us train faster than real time.
