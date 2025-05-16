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