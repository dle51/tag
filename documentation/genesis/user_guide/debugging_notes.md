# Debugging Notes
A brief overview of the `debugging.py` script presented by the [Genesis User Guide](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/interactive_debugging.html).

Genesis implements an interface to access internal information and **all** attributes of the objects created in Genesis through the implementation of the `__repr__` method in every single class. This provides an extremely detailed overview of environments we create where we can view precise details regarding the scene, solvers, entities, links, collision geometry, and more. We can access this information using [pdb](https://docs.python.org/3/library/pdb.html) debugger, [ipdb](https://pypi.org/project/ipdb/), or the [ipython](https://github.com/ipython/ipython) shell. In this example, I'll use the `ipython` shell, but use your favorite!

Let's start with a simple static example and load up the `hello_genesis.py` script, but to spice things up, let's upload the `go2` robot file instead of the Franka Arm and add an extra camera to see some extra debugging information. Because this is a static example, we can exclude visualization and time steps.

```python
import genesis as gs

gs.init()

scene = gs.Scene(show_viewer=False)

plane = scene.add_entity(gs.morphs.Plane())
go2 = scene.add_entity(
    gs.morphs.URDF(
        file='urdf/go2/urdf/go2.urdf',
        pos = (0, 0, 0.5),
    ),
)

cam_0 = scene.add_camera()
scene.build()

# enter IPython's interactive mode
import IPython; IPython.embed()
```

Walking through this script and identifying the objects we create, we have `scene`, `plane`, `go2`, and `cam_0`. Like debugging any other python program, when we hit the breakpoint, we can input an object's name to get information about that object. Starting with scene, we get the output:
```python
──────────────── <gs.Scene> ────────────────
             't': <int>: 0
            'dt': <float>: 0.01
      'is_built': <bool>: True
         'cur_t': <float>: 0.0
    'fem_solver': <gs.FEMSolver>: <85fd4d9>, n_entities: 0
    'mpm_solver': <gs.MPMSolver>: <c0032d8>, n_entities: 0
    'pbd_solver': <gs.PBDSolver>: <e3da89c>, n_entities: 0
           'sim': <gs.Simulator>
    'sph_solver': <gs.SPHSolver>: <718572e>, n_entities: 0
           'uid': <gs.UID>('54311be-f4a8a4edea0b256261ea10bec')
      'show_FPS': <bool>: True
   'tool_solver': <gs.ToolSolver>: <727369d>, n_entities: 0
  'rigid_solver': <gs.RigidSolver>: <a335b67>, n_entities: 2
'active_solvers': <gs.List>(len=1, [
                      <gs.RigidSolver>: <a335b67>, n_entities: 2,
                  ])
 'avatar_solver': <gs.AvatarSolver>: <8f5a7b6>, n_entities: 0
        'viewer': None
       'gravity': <numpy.ndarray>: array([ 0.  ,  0.  , -9.81])
       'solvers': <gs.List>(len=8, [
                      <gs.ToolSolver>: <727369d>, n_entities: 0,
                      <gs.RigidSolver>: <a335b67>, n_entities: 2,
                      <gs.AvatarSolver>: <8f5a7b6>, n_entities: 0,
                      <gs.MPMSolver>: <c0032d8>, n_entities: 0,
                      <gs.SPHSolver>: <718572e>, n_entities: 0,
                      <gs.PBDSolver>: <e3da89c>, n_entities: 0,
                      <gs.FEMSolver>: <85fd4d9>, n_entities: 0,
                      <gs.SFSolver>: <30205e7>, n_entities: 0,
                  ])
      'emitters': <gs.List>(len=0, [])
      'entities': <gs.List>(len=2, [
                      <gs.RigidEntity>, idx: 0, morph: <gs.morphs.Plane>, material: <gs.materials.Rigid>,
                      <gs.RigidEntity>, idx: 1, morph: <gs.morphs.URDF(file='/data/user/tag/.venv/lib/python3.10/site-packages/genesis/assets/urdf/go2/urdf/go2.urdf')>, material: <gs.materials.Rigid>,
                  ])
 'requires_grad': <bool>: False
      'substeps': <int>: 1
    'visualizer': <gs.Visualizer>
```

The best place to find the meaning behind these parameters is the source code itself. Because we're examining a `scene`, we can simply check the [`scene.py`](https://github.com/Genesis-Embodied-AI/Genesis/blob/388aa1c98ab1e842fd44e5e11584c2c047623a49/genesis/engine/scene.py#L1048) file. 

From here, we can examine any object further inside of `scene` by appending it to `scene`. For example, if I wanted some more information on the `rigid_solver` in the scene, I can input `scene.rigid_solver` and get the output
```python
───────────────────── <gs.RigidSolver> ─────────────────────
               'n_qs': <int>: 19
             'n_dofs': <int>: 18
            'n_links': <int>: 14
            'n_geoms': <int>: 28
            'n_cells': <numpy.int64>: 611468
            'n_verts': <int>: 1946
            'n_faces': <int>: 3780
            'n_edges': <int>: 5670
           'n_joints': <int>: 14
           'n_vgeoms': <int>: 34
           'n_vverts': <int>: 384735
           'n_vfaces': <int>: 398644
# ... 
```

We can get information about stuff inside of lists too, like the first entity with `scene.entities[0]`, which is actually the `plane` object. 

The main object we'll probably be inspecting is the `go2` robot model. When we inspect that object in this environment, we get a whole bunch of information that I'll let you view yourself, but we get a lot of important information from the lists `geoms`, `vgeoms`, `joints`, `links`, `gravity_compensation`, and more. For example, we can get an understanding of the `joints` and `dofs` of the `go2` by inputting `go2.joints`
```python
<gs.List>(len=13, [
    <gs.RigidJoint>: <9f65d3c>, name: 'joint_base', idx: 1, type: <FREE: 4>,
    <gs.RigidJoint>: <9687584>, name: 'FL_hip_joint', idx: 2, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <9f5aca3>, name: 'FR_hip_joint', idx: 3, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <f7c1392>, name: 'RL_hip_joint', idx: 4, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <4f11225>, name: 'RR_hip_joint', idx: 5, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <53410dd>, name: 'FL_thigh_joint', idx: 6, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <432576c>, name: 'FR_thigh_joint', idx: 7, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <1de5bb9>, name: 'RL_thigh_joint', idx: 8, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <801692f>, name: 'RR_thigh_joint', idx: 9, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <ed59996>, name: 'FL_calf_joint', idx: 10, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <3f7e1b2>, name: 'FR_calf_joint', idx: 11, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <4067412>, name: 'RL_calf_joint', idx: 12, type: <REVOLUTE: 1>,
    <gs.RigidJoint>: <def45fb>, name: 'RR_calf_joint', idx: 13, type: <REVOLUTE: 1>,
])
```

When we inspect `go2` as a whole, we can see that
```python
'n_dofs': <int>: 18
```

As `free` joints have 6 degrees of freedom, we know the first 6 `dofs` of the `go2` robot correlate to the `joint_base`, then the next 12 correlate to the joints in the hip, thigh, and calf. Using the debugger, we can identify the joints of the `go2`, what type of joints they are, what degrees of freedom they correspond to, and the names of the joints much quicker than we could have by searching through documentation, though I am sure we will do both.

While I don't show it here, we can use `import IPython; IPython.embed()` as a breakpoint in our code at specific time steps (or use `breakpoint()` with `pdb`), so we can inspect the environment during it's runtime. More importantly, we can and should run methods on the environment during debugging. For example, we can run accessor methods on the `go2` object, like `get_contacts()`, `get_dofs_force()`, and even `get_jacobian()`. (You can look at all the methods [here](https://github.com/Genesis-Embodied-AI/Genesis/blob/3d1e5b7f0c2534c25617ccb10febae6652310bb9/genesis/engine/entities/rigid_entity/rigid_entity.py)) Here are some outputs to finish things off:
```python
In [23]: go2.get_pos()
Out[23]: tensor([0.0000, 0.0000, 0.5000], device='cuda:0')

In [24]: go2.get_contacts()
Out[24]: 
{'geom_a': array([], dtype=int32),
 'geom_b': array([], dtype=int32),
 'link_a': array([], dtype=int32),
 'link_b': array([], dtype=int32),
 'position': array([], shape=(0, 3), dtype=float32),
 'force_a': array([], shape=(0, 3), dtype=float32),
 'force_b': array([], shape=(0, 3), dtype=float32)}

In [25]: go2.get_mass()
Out[25]: 15.019000000000004

In [26]: go2.get_dofs_force()
Out[26]: 
tensor([ 0.0000e+00,  0.0000e+00, -1.4734e+02, -3.7973e-08,  1.3031e+00,
         0.0000e+00, -1.0204e+00,  1.0204e+00, -1.0204e+00,  1.0204e+00,
        -3.3987e-02, -3.3987e-02, -3.3987e-02, -3.3987e-02,  8.2788e-03,
         8.2788e-03,  8.2788e-03,  8.2788e-03], device='cuda:0')

In [27]: go2.get_jacobian(go2.get_link('RL_calf'))
Out[27]: 
tensor([[ 1.0000,  0.0000,  0.0000, -0.0000, -0.2130, -0.1420,  0.0000,  0.0000,
         -0.0000,  0.0000,  0.0000,  0.0000, -0.2130,  0.0000,  0.0000,  0.0000,
          0.0000,  0.0000],
        [ 0.0000,  1.0000,  0.0000,  0.2130,  0.0000, -0.1934,  0.0000,  0.0000,
          0.2130,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
          0.0000,  0.0000],
        [ 0.0000,  0.0000,  1.0000,  0.1420,  0.1934,  0.0000,  0.0000,  0.0000,
          0.0955,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
          0.0000,  0.0000],
        [ 0.0000,  0.0000,  0.0000,  1.0000,  0.0000,  0.0000,  0.0000,  0.0000,
          1.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
          0.0000,  0.0000],
        [ 0.0000,  0.0000,  0.0000,  0.0000,  1.0000,  0.0000,  0.0000,  0.0000,
          0.0000,  0.0000,  0.0000,  0.0000,  1.0000,  0.0000,  0.0000,  0.0000,
          1.0000,  0.0000],
        [ 0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  1.0000,  0.0000,  0.0000,
          0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
          0.0000,  0.0000]], device='cuda:0')
```