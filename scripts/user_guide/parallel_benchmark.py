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
)

### entities
plane = scene.add_entity(
    gs.morphs.Plane(),
)

franka = scene.add_entity(
    gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'),
)

### building parallel environments
B = 30000
scene.build(B)

# control all the robots
franka.control_dofs_position(
    torch.tile(
        torch.tensor([0, 0, 0, -1.0, 0, 0, 0, 0.02, 0.02], device=gs.device), (B, 1)
    ),
)

for i in range(1000):
    scene.step()
    