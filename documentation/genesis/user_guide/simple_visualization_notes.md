## Visualization Notes
A walkthrough of the components in simple_visualization.py

At the beginning of this script, we add
```python
import os
os.environ["PYOPENGL_PLATFORM"] = "egl"
```

This specifies that are using the egl interface for the renderer later.

Compared to the hello_genesis.py script, we have a few more declarations when building the scene
```python
scene = gs.Scene(
    show_viewer = False,
    vis_options = gs.options.VisOptions(
        show_world_frame = True, # visualize the coordinate frame of `world` at its origin
        world_frame_size = 1.0, # length of the world frame in meter
        show_link_frame  = False, # do not visualize coordinate frames of entity links
        show_cameras     = False, # do not visualize mesh and frustum of the cameras added
        plane_reflection = True, # turn on plane reflection
        ambient_light    = (0.1, 0.1, 0.1), # ambient light setting
    ),
    renderer = gs.renderers.Rasterizer(), # using rasterizer for camera rendering
)
```

As before, we set `show_viewer` to `False` to run in headless mode. The original script included `viewer_options`, but we do not need to implement this in headless mode. The `vis_options` settings specify visualization parameters that the camera. The codeblock above details what each setting does. We also declare a renderer, which we specify the `Rasterizer` backend because we aren't raytracing.

After adding a plane and franka arm into the scene, we create a camera in the scene.
```python
cam = scene.add_camera(
    res    = (640, 480),
    pos    = (3.5, 0.0, 2.5),
    lookat = (0, 0, 0.5),
    fov    = 30,
    GUI    = False,
)
```

These camera settings are pretty straight-forward in what they do. `res` controls the resolution of the camera, `pos` places the camera in space inside of the scene, `lookat` chooses where the camera is pointing, `fov` specifies the field of view of the camera. For `GUI`, the user guide states that when enable, "each camera will create an opencv window to dynamically display the rendered image. Note that this is different from the viewer GUI."

Building the scene, we start the camera recording and after each step, we rotate the camera and render a frame.

```python
scene.build()
for i in range(120):
    scene.step()
    cam.set_pose(
        pos    = (3.0 * np.sin(i / 60), 3.0 * np.cos(i / 60), 2.5),
        lookat = (0, 0, 0.5),
    )
    cam.render()
cam.stop_recording(save_to_filename='simpleVisualizationVideo.mp4', fps=60)
```

`cam.set_pose()` changes where the camera is looking, and using some sin and cosine, we can rotate the camera around the franka arm each iteration. At the end of the script, we can save the camera's recording.
