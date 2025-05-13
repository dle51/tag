## hello_genesis.py Notes
Walkthrough of the components of the hello_genesis.py script

```python
import genesis as gs
gs.init(backend=gs.gpu)
```
To begin, we import genesis into the script and declare that the backend is a GPU. Genesis is cross-platform, so we need to specify what hardware we're using. Genesis will automatically select CUDA as our backend, so we can also declare this as `gs.cuda` in this case. There are many more init settings, like time-step size or gravity, that will show up in other examples.

Next, we initialize the scene
```python
scene = gs.Scene(show_viewer=False)
```

The scene is where all objects, robots, cameras, etc. are placed. We can change a ton of different [configs](https://genesis-world.readthedocs.io/en/latest/api_reference/scene/scene.html) here, but for this program and all others on the cluster, we will make sure that the setting `show_viewer` is set to `False` because we are running the simulation without a display.

We can then start loading entities into the scene
```python
plane = scene.add_entity(gs.morphs.Plane())
franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)
```

[Entities](https://genesis-world.readthedocs.io/en/latest/api_reference/entity/index.html) in Genesis are robots and objects. Genesis is fully OOP, so we interact with these entities independently from one another. We add entities using a [morph](https://genesis-world.readthedocs.io/en/latest/api_reference/options/morph/index.html) parameter, which contains information about the geometry and pose of the entity. Here, we use `gs.morphs.MJCF` because we're using a Mujoco file and we are loading a franka arm using its direct path `xml/franka_emika_panda/panda.xml`. Again, there are many more settings we can include here, but we will see those later.

Finally, we can build the scene and run the script.

```python
scene.build()
for i in range(1000):
    scene.step()
```

This program is suppose to run with a display where we can control the franka arm using our keyboard, but because this is in headless mode, the scene simply builds then ends at the end of the loop.
