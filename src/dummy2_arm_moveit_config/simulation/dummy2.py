import os
os.environ['PYOPENGL_PLATFORM'] = 'glx' #fix opengl context issue

import genesis as gs
gs.init(backend=gs.cpu)

# gs.init(
#     seed                = None,
#     precision           = '32',
#     debug               = True,
#     eps                 = 1e-12,
#     logging_level       = None,
#     backend             = gs.cpu,
#     theme               = 'dark',
#     logger_verbose_time = False
# )

scene = gs.Scene(show_viewer=True)
plane = scene.add_entity(gs.morphs.Plane())
franka = scene.add_entity(
    gs.morphs.URDF(file='urdf/dummy2/dummy2.urdf'),
)

scene.build()

for i in range(1000):
    scene.step()
