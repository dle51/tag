# Genesis User Guides
<!-- TODO: Update scripts to have hyperlinks to relevant information -->
<!-- TODO: Proofread documentation for errors (late night writing may be productive, but it definitely isn't good for grammar) -->
Notes covering the scripts and information found in the Genesis [user guide](https://genesis-world.readthedocs.io/en/latest/user_guide/index.html).

##### Getting Started
1. [Hello Genesis](./hello_genesis_notes.md) \
An introduction to Genesis |
[User Guide Link](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/hello_genesis.html) 
```bash
# From project root
uv run scripts/user_guide/hello_genesis.py
```

2. [Visualization & Rendering](./simple_visualization_notes.md) \
An introduction to rendering simulations in Genesis |
[User Guide Link](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/visualization.html)
```bash
uv run scripts/user_guide/simple_visualization.py
```

3. [Control Your Robot](./control_notes.md) \
An overview of manipulating models in Genesis |
[User Guide Link](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/control_your_robot.html)
```bash
uv run scripts/user_guide/control.py
```

4. [Parallel Simulation](./parallel_simulation_notes.md) \
An overview of scene-level parallelism in Genesis | 
[User Guide Link](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/parallel_simulation.html)
```bash
uv run scripts/user_guide/parallel_visualization.py # &
uv run scripts/user_guide/parallel_benchmark.py
```

5. [Inverse Kinematics & Motion Planning](./inverse_kinematics_motion_planning_notes.md) \
An introduction to Inverse Kinematics and Motion Planning in Genesis |
[User Guide Link](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/inverse_kinematics_motion_planning.html)
```bash
# Motion Planning - Requires OMPL
uv run scripts/user_guide/ik_mp.py
# No Motion Planning
uv run scripts/user_guide/inverse_kinematics.py
```

6. [Advanced Inverse Kinematics](./advanced_ik_notes.md) \
An introduction to multi-link inverse kinematics in Genesis |
[User Guide Link](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/advanced_ik.html)
```bash
uv run scripts/user_guide/advanced_ik.py
```

7. [Parallel Inverse Kinematics](./parallel_ik_notes.md) \
Implementing Inverse Kinematics in Batched Environments using Genesis |
[User Guide Link](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/advanced_ik.html)
```bash
uv run scripts/user_guide/parallel_ik.py
```

8. [Interactive Information Access and Debugging](./debugging_notes.md) \
Using IPython to debug scenes in Genesis |
[User Guide Link](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/interactive_debugging.html)
```bash
uv run scripts/user_guide_debugging.py
```
