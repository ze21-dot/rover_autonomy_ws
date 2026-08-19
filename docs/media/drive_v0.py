from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

import numpy as np
import omni.kit.commands
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from omni.usd import get_context
from pxr import UsdPhysics

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

s, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False; cfg.fix_base = False; cfg.make_default_prim = False
omni.kit.commands.execute("URDFParseAndImportFile",
    urdf_path="/workspace/robot/karasimsek_isaac.urdf", import_config=cfg)

for _ in range(5):
    app.update()

stage = get_context().get_stage()
roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("ROOTS:", roots)
path = roots[0] if roots else None
if path is None:
    raise SystemExit("ArticulationRoot bulunamadi")

world.reset()
robot = Articulation(path)
robot.initialize()
names = list(robot.dof_names)
print("DOF:", names)
wheel_idx = [i for i, n in enumerate(names) if "Wheel" in n]
print("Teker:", [names[i] for i in wheel_idx])

cam = rep.create.camera(position=(3.0, 3.0, 1.6), look_at=(0, 0, 0.3))
rp = rep.create.render_product(cam, (1920, 1080))
w = rep.WriterRegistry.get("BasicWriter")
w.initialize(output_dir="/workspace/frames", rgb=True)
w.attach([rp])

t = np.zeros(len(names))
for i in wheel_idx: t[i] = -8.0
for _ in range(300):
    robot.set_joint_velocities(t)
    world.step(render=True)
print("=== SURUS TAMAM ===")
app.close()
