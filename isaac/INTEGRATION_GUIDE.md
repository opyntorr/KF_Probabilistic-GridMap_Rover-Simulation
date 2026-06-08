# INTEGRATION GUIDE — Gazebo→Isaac Sim 4.5 port modules into `scene_mecanum.py`

This guide tells the orchestrator EXACTLY how to wire the 7 port modules into
`/home/opyntorr/agv_uav_project_jetauto_Vilchis/isaac/scene_mecanum.py`.

Line numbers below refer to `scene_mecanum.py` as it stands now (550 lines). Anchors
used:
- argparse block: lines 49–69 (`args, _ = parser.parse_known_args()` at line 69).
- imports-after-SimulationApp: lines 82–92; `enable_extension(...)` calls: lines 94–99;
  `simulation_app.update()` at line 100.
- URDF import: lines 103–114; `prim_path` set at 110–113.
- `apply_jetauto_materials(stage, prim_path)`: line 121.
- `/physicsScene`: lines 139–149 (`TimeStepsPerSecond` at 149).
- `mat_path = "/physicsMaterial"`: lines 152–157.
- laberinto URDF block: lines 163–213.
- ground-plane logic: `_load_extras`/`_make_ground` at 217–218, AddGroundPlane 219–229.
- EXTRA_USDS loop: 247–281.
- roller-friction Traverse loop: 287–299.
- `/ActionGraph` `og.Controller.edit`: 324–366 (node names already present: **`OnTick`**
  = OnPlaybackTick, **`ReadSimTime`** = IsaacReadSimulationTime).
- gridmap RTX lidar: 369–384.
- `rclpy.init()` + `node`: 387–394.
- `timeline.play()` + first `update()`: 397–399.
- `art = SingleArticulation(prim_path); art.initialize()`: 401–402.
- settle loop (60 updates): 465–468.
- main loop: 530–537; `finally:`: 540–549 (`rclpy.shutdown()` at 547).

All modules live in the same dir as `scene_mecanum.py`, so `from <module> import ...`
works (same as the existing `from jetauto_materials import apply_jetauto_materials`).
No module has import-time side effects.

---

## (2) CONSOLIDATED LIST OF NEW CLI FLAGS

Add these to the argparse block (lines 49–68, before line 69). Existing flags reused
as-is: `--world`, `--gridmap`, `--no-extra`, `--ground`/`--no-ground`, `--record`,
`--record_fps`, `--x`, `--y`, `--z`, `--yaw`.

```python
# --- world_loader: reuses EXISTING --world laberinto (no new flag) ---
# --- aruco3d ---
parser.add_argument("--aruco", action="store_true",
                    help="añade marcadores ArUco 3D + cubo objetivo")
# --- drone ---
parser.add_argument("--drone", action="store_true",
                    help="spawn dron Tello kinemático en /drone1/cmd_vel")
# --- sensors ---
parser.add_argument("--imu", action="store_true",
                    help="publica IMU en /imu/data_raw (siempre seguro)")
parser.add_argument("--camera", action="store_true",
                    help="cámara RGBD Astra en /cam_1/* (necesita render)")
# --- cameras (multi-cam rig) ---
parser.add_argument("--cameras", action="store_true",
                    help="rig multi-cámara wheel/scene/chase[/top]")
parser.add_argument("--viewports", action="store_true",
                    help="un viewport GUI por cámara (requiere ventana, no --headless)")
parser.add_argument("--rec_cams", action="store_true",
                    help="graba el rig a disco (PNG/MP4 por cámara, replicator)")
parser.add_argument("--cam_res", default="1080p",
                    help="preset de resolución del rig (720p/1080p/1440p/4k)")
# --- perf_hud ---
parser.add_argument("--perf", action="store_true",
                    help="HUD de stats + imprime [PERF] fps/rtf/gpu/ram (+ /isaac/perf)")
# --- serpenteo_sweep physics knobs (consumed by the harness; defaults = no-op) ---
parser.add_argument("--solver-pos", type=int, default=-1)      # <0 = no tocar
parser.add_argument("--solver-vel", type=int, default=-1)      # <0 = no tocar
parser.add_argument("--phys-hz", type=float, default=-1.0)     # <=0 = 120 (default)
parser.add_argument("--contact-offset", type=float, default=-1.0)  # <0 = no tocar
parser.add_argument("--rest-offset", type=float, default=-1.0)     # <0 = no tocar
parser.add_argument("--maxdepen", type=float, default=-1.0)        # <0 = no tocar
parser.add_argument("--roller-shape", default="sphere")        # informativo (CSV)
parser.add_argument("--apply-perstep", action="store_true")    # acción en callback físico
```

IMPORTANT: `parser.parse_known_args()` (line 69) already tolerates unknown flags, but
the serpenteo harness flags must be DEFINED for them to take effect (otherwise they are
silently ignored and the sweep has no effect). All others are plain features.

---

## (1) PER-MODULE WIRING

### A) world_loader — maze / empty world  (flag: existing `--world laberinto`)

Public API (confirmed): `load_laberinto(stage, simulation_app, mat_path="/physicsMaterial",
parent="/World/Maze", mesh_dir=_MESH_DIR, usd_cache=_USD_CACHE, scale=0.001) -> (parent_path, bbox)`;
`load_vacio(stage, mat_path="/physicsMaterial", plane_path="/groundPlane", size=100.0) -> str`;
`ensure_maze_usds(...)`; `MAZE_SCALE`.

1. Import (near line 92, with the other post-SimulationApp imports):
   ```python
   from world_loader import load_laberinto, load_vacio
   ```
2. The asset converter ext is only needed for the maze. Add it gated (near line 96, the
   `if args.gridmap:` enable block):
   ```python
   if args.world == "laberinto":
       enable_extension("omni.kit.asset_converter")
   ```
   (This runs before the `simulation_app.update()` at line 100.)
3. **Replace the ENTIRE laberinto URDF block (lines 163–213)** with:
   ```python
   if args.world == "laberinto":
       _maze_path, _maze_bb = load_laberinto(stage, simulation_app, mat_path=mat_path)
   ```
   Must be AFTER `mat_path` exists (line 157) and BEFORE the `_make_ground` logic
   (line 218). The existing `_make_ground`/`_load_extras` checks `not args.world`
   (lines 217–218), so the maze's own `piso.stl` is the floor — NO change needed there.
4. Spawn: `_spawn_corner` (line 72) already spawns at (-1,-1, yaw=π/2) for
   `--world laberinto`. The maze auto-centers in xy with floor at z=0, so the robot
   lands inside. NO change needed.
5. (Optional) Empty/Kalman world: you MAY replace the inline `AddGroundPlaneCommand`
   block (lines 219–229) with `load_vacio(stage, mat_path=mat_path)` — identical command,
   just factored out. Not required; the inline block already works.

Notes: first conversion is slow (jaula.stl ~3.3 MB); cached USDs go to
`isaac/assets/worlds/usd/`. Most likely GPU tweak: maze rotation if STL comes in non-Z-up
(add an `Rx90` op on `/World/Maze` root, mirroring the old lines 179–181).

---

### B) aruco3d — 3D ArUco markers + goal cube  (flag: `--aruco`)

Public API (confirmed): `make_aruco3d(stage, prim_path, marker_id, size_m=0.18, pose=None,
mat_path=None, collision=False) -> str`; `make_goal_cube(stage, prim_path, pose=None,
size_m=0.15, mat_path=None, collision=True) -> str`; `place_robot_marker(stage,
robot_base_prim, marker_id=4, size_m=0.18, z=0.22, mat_path=None) -> str`.

1. Import (near line 121, next to `apply_jetauto_materials`):
   ```python
   from aruco3d import make_aruco3d, make_goal_cube, place_robot_marker
   ```
2. Call AFTER `mat_path` exists (line 157) AND AFTER `apply_jetauto_materials` (line 121),
   but BEFORE `timeline.play()` (line 398). The cleanest spot is right after the
   roller-friction loop (line 299) / before `/ActionGraph` (line 324):
   ```python
   if args.aruco:
       # cubo objetivo en el suelo (centro a z=size/2=0.075) a 1.5 m delante del spawn
       make_goal_cube(stage, "/World/CuboAruco", pose=((1.5, 0.0, 0.075),), mat_path=mat_path)
       # marcador ID4 en el techo del robot (hijo de base_link -> sigue al chasis)
       robot_root = "/" + str(prim_path).strip("/").split("/")[0]   # p.ej. /jetauto
       place_robot_marker(stage, robot_root + "/base_link", marker_id=4)
       # (opcional) marcador suelto en el piso:
       # make_aruco3d(stage, "/World/Aruco/m3", 3, pose=(0.0, 1.0, 0.0), mat_path=mat_path)
   ```
   `place_robot_marker` MUST be after `apply_jetauto_materials` so its
   `strongerThanDescendants` white/black bind wins over the robot's de-instanced mesh
   material. The cube center sits at the pose translation → for a cube resting on the
   floor pass `z = size_m/2` (0.075 for the default 0.15 m cube).

Notes: depends on cv2 (OpenCV) being importable in Isaac's python; markers are visual
(no collision) by default; the cube gets a box collider. The `/<root>/base_link` path
assumes the URDF importer keeps a prim named `base_link` directly under the articulation
root — if not, resolve by name (same `stage.Traverse()` pattern the lidar uses, line 373).

---

### C) drone — kinematic Tello  (flag: `--drone`)

Public API (confirmed): class `IsaacDrone(stage, simulation_app, prim_path="/World/drone1",
spawn=(0,0,0.1), mat_path="/physicsMaterial", ...)`; methods `step(dt)`, `spin_once(timeout_sec=0.0)`,
`post_play_init()`, `publish_odom()`, `takeoff()`, `land()`, `get_pose()`, `shutdown()`.
Topics: produces `/drone1/odom`, `/uav/camera/image`, `/uav/camera/camera_info`; consumes
`/drone1/cmd_vel`, `/drone1/takeoff`, `/drone1/land`.

1. Construct AFTER `timeline.play()` + first `update()` (lines 397–399) AND after
   `art = SingleArticulation(...); art.initialize()` (lines 401–402). A good spot is right
   after the settle loop (line 468) — the import is local so headless-without-`--drone`
   pays nothing:
   ```python
   drone = None
   if args.drone:
       from drone import IsaacDrone
       drone = IsaacDrone(stage, simulation_app, prim_path="/World/drone1",
                          spawn=(args.x, args.y, 0.1), mat_path=mat_path)
       drone.post_play_init()   # MUST be after timeline.play()+>=1 update()
       drone.takeoff()          # optional: auto-climb to z=2.2 so PID z>0.8 gate passes
   ```
2. In the main loop (lines 531–537), add three calls. Use fixed dt = 1/120 (the scene's
   TimeStepsPerSecond=120; one `simulation_app.update()` = one physics step):
   ```python
   while simulation_app.is_running():
       rclpy.spin_once(node, timeout_sec=0.0)        # existing
       if drone is not None:
           drone.spin_once()                          # process /drone1/cmd_vel + takeoff/land
           drone.step(1.0/120.0)                      # integrate twist, write kinematic pose (BEFORE update)
       targets = ik(cmd["vx"], cmd["vy"], cmd["wz"])  # existing
       art.apply_action(...)                          # existing
       simulation_app.update()                        # existing
       if drone is not None:
           drone.publish_odom()                       # publish /drone1/odom AFTER the step
       _rec_step()                                    # existing
   ```
3. In `finally:` (lines 540–549), BEFORE `rclpy.shutdown()` (line 547):
   ```python
   if drone is not None:
       drone.shutdown()
   ```

Notes: drone uses its OWN rclpy node (`isaac_drone`); its `rclpy.init()` is try/except
guarded, safe whether or not the scene already called `rclpy.init()` (it does, line 390).
`drone.shutdown()` destroys only its node (does NOT call `rclpy.shutdown()`), so the
scene's node survives. The drone adds NO nodes to `/ActionGraph`; its camera publishers
live in a sibling push graph **`/DroneCameraGraph`**.

---

### D) sensors — IMU + RGBD Astra camera  (flags: `--imu`, `--camera`)

Public API (confirmed): `add_imu(stage, simulation_app, base_link_prim="/jetauto/base_link",
topic="/imu/data_raw", frame_id="imu_link", imu_link_name="imu_link", graph_path="/ActionGraph") -> str|None`;
`add_rgbd_camera(stage, simulation_app, parent_prim=None, topic_base="/cam_1",
resolution=(640,480), frame_id="cam_1_optical_frame", ..., graph_path="/ActionGraph") -> str|None`.

Both functions ADD nodes to the EXISTING `/ActionGraph` and rely on the node names
already there: **`OnTick`** and **`ReadSimTime`** (lines 328–329). They must run AFTER
the `/ActionGraph` `og.Controller.edit` (ends line 366) and BEFORE `timeline.play()`
(line 398). They must also run AFTER the URDF import + an `update()` so the link prims
(`imu_link`, `cam_1_optical_frame`) exist — they do by line 366.

1. Extension (near line 96–99): the IMU needs the physics-sensor ext:
   ```python
   if args.imu:
       enable_extension("isaacsim.sensors.physics")   # for IsaacReadIMU + IMU sensor
   ```
   (The camera needs NO extra ext: `IsaacCreateRenderProduct` is in `isaacsim.core.nodes`,
   helpers in `isaacsim.ros2.bridge` already enabled at line 94.)
2. Import + call, right after line 366 (graph creation) and before line 369 / 398:
   ```python
   from sensors import add_imu, add_rgbd_camera
   if args.imu:
       add_imu(stage, simulation_app,
               base_link_prim=prim_path.replace("/base_footprint", "/base_link"),
               topic="/imu/data_raw", frame_id="imu_link")
   if args.camera:
       add_rgbd_camera(stage, simulation_app, topic_base="/cam_1",
                       resolution=(640, 480), frame_id="cam_1_optical_frame")
   ```

Notes: `prim_path` is `/jetauto/base_footprint`; the IMU/camera links are SIBLINGS under
`/jetauto` (e.g. `/jetauto/base_link`, `/jetauto/imu_link`). Both functions fall back to a
full `stage.Traverse()` by link NAME, so even a wrong `base_link_prim` still finds the
link. Safest: pass `base_link_prim="/jetauto"` (the robot root) or omit it. The camera
render product renders every tick (GPU-heavy at 120 Hz); if RTF tanks, add a
`frameSkipCount` on the `CamHelper*` nodes. Node names used: `ReadIMU`, `PublishIMU`,
`CamRenderProduct`, `CamHelperRgb/Depth/Info` (each created once).

---

### E) cameras — multi-camera rig + recorder  (flags: `--cameras`, `--viewports`, `--rec_cams`, `--cam_res`)

Public API (confirmed): `setup_cameras(stage, simulation_app, robot_prim,
wheel_link_prim=None, specs=None) -> dict[str,Camera]`; `update_chase_camera(stage, cams)`;
`create_viewports(camera_paths, resolution=None, tile=True) -> dict`; `set_viewport_camera(...)`;
class `Recorder(cameras, out_dir, fps=30.0, resolution=None, image_format="png",
rt_subframes=4, use_orchestrator=False)` with `.start()/.step()/.stop()/.to_mp4()`;
`set_resolution(...)`, `find_link_prim(...)`, `pngs_to_mp4(...)`, `RESOLUTION_PRESETS`.

1. Extension (near line 99): `if args.cameras or args.rec_cams or args.viewports:
   enable_extension("isaacsim.sensors.camera")`. Import at top (after SimulationApp):
   `import cameras`.
2. Build the rig AFTER `timeline.play()` + `art.initialize()` AND after the settle loop
   (line 468) — `Camera.initialize()` needs the timeline in play and the render warmed:
   ```python
   CAMS = {}
   if args.cameras or args.rec_cams or args.viewports:
       CAMS = cameras.setup_cameras(
           stage, simulation_app, robot_prim=prim_path,   # prim_path = /jetauto/base_footprint
           wheel_link_prim=None,                           # autodetecta "*wheel*"
           specs={"scene": {"resolution": args.cam_res},
                  "chase": {"resolution": args.cam_res},
                  "top":   {"enabled": args.gridmap}})      # cenital solo en gridmap, p.ej.
   ```
3. Viewports (GUI only):
   ```python
   if args.viewports and not args.headless:
       cameras.create_viewports(CAMS, resolution=(640, 360))
   ```
4. Disk recorder (alternative/complement to the existing `_rec_step` topdown ffmpeg):
   ```python
   cam_rec = None
   if args.rec_cams:
       import datetime
       outd = os.path.join(HERE, f"cams_{datetime.datetime.now():%Y%m%d_%H%M%S}")
       cam_rec = cameras.Recorder(CAMS, outd, fps=args.record_fps, resolution=args.cam_res)
   ```
5. In the main loop (after `simulation_app.update()`, line 536, next to `_rec_step()`):
   ```python
   if CAMS:
       cameras.update_chase_camera(stage, CAMS)           # chase/top follow the robot
   if cam_rec is not None:
       if not cam_rec._on and (abs(cmd["vx"])+abs(cmd["vy"])+abs(cmd["wz"]) > 1e-4):
           cam_rec.start()
       cam_rec.step()
   ```
6. In `finally:`: `if cam_rec is not None: cam_rec.stop(); cam_rec.to_mp4()`.

Notes: `mat_path`/`/physicsScene` untouched (cameras need neither). VRAM: each
viewport + each Recorder render product costs VRAM; with 1080p/4k on wheel+scene+chase+top
the single GPU may OOM — drop to `720p`. Each Camera gets its own render product (same
GPU-cost caveat as the sensors camera).

---

### F) perf_hud — performance HUD + [PERF] line  (flag: `--perf`)

Public API (confirmed): class `PerfMonitor(simulation_app, timeline=None, gpu_index=0,
report_period_s=1.0, publish_ros=False, ros_topic="/isaac/perf", ros_node=None,
viewport_overlay=False, prefix="[PERF]")` with `.tick(sim_time=None)`, `.shutdown()`,
`.last_line`; `enable_stats_overlay(display_options=3286) -> bool`;
`recommended_simulation_app_config(headless=False) -> dict`; constants
`DISPLAY_OPTIONS_STATS=3286`, `DISPLAY_OPTIONS_HIDDEN=3094`.

1. HUD overlay (windowed only). Simplest: in hot mode after imports/update (after line 100):
   ```python
   if args.perf:
       from perf_hud import enable_stats_overlay
       enable_stats_overlay()
   ```
   (Alternative: hardcode `display_options=3286` in the SimulationApp dict at line 80 when
   `args.perf and not args.headless` — but perf_hud can't be imported before SimulationApp,
   so use the literal value there.)
2. Instantiate AFTER `timeline.play()` + `update()` (after line 399); `timeline` exists
   (line 397). To share the scene's node, pass `ros_node=node` (node exists at line 391):
   ```python
   perf = None
   if args.perf:
       from perf_hud import PerfMonitor
       perf = PerfMonitor(simulation_app, timeline=timeline,
                          publish_ros=True, ros_node=node)
   ```
3. In the main loop (after `simulation_app.update()`, line 536): `if perf is not None: perf.tick()`.
4. In `finally:` BEFORE `rclpy.shutdown()` (line 547): `if perf is not None: perf.shutdown()`.

Notes: `publish_ros=True` reuses the scene's rclpy context (detects `rclpy.ok()`); passing
`ros_node=node` makes it publish `std_msgs/String` on `/isaac/perf` from the scene's node
(so it owns no node and `shutdown()` won't destroy the scene's node). GPU util is whole-device
on a shared GPU.

---

### G) serpenteo_sweep — anti-serpenteo physics sweep  (8 harness flags, no feature flag)

The harness is run by the orchestrator OUTSIDE the scene
(`source isaac/isaac_env.sh; python3 isaac/serpenteo_sweep.py [--quick]`). It launches
`scene_mecanum.py` via `$ISAACSIM/python.sh` and `kf_control_isaac.py` via system python3,
one config at a time. For it to have any effect, `scene_mecanum.py` must IMPLEMENT the 8
flags (defined in section 2). Wiring inside `scene_mecanum.py`:

- **(2) `--phys-hz`** — REPLACE line 149:
  ```python
  physx.CreateTimeStepsPerSecondAttr(args.phys_hz if args.phys_hz > 0 else 120.0)
  ```
- **(4) `--maxdepen`** — near `/physicsScene` creation (after line 149), on the chassis body:
  ```python
  if args.maxdepen >= 0.0:
      rb = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(prim_path))
      rb.CreateMaxDepenetrationVelocityAttr(args.maxdepen)
  ```
- **(3) `--contact-offset` / `--rest-offset`** — inside the roller-friction Traverse loop,
  in the `if p.HasAPI(UsdPhysics.CollisionAPI):` branch (lines 288–296), for prims whose
  path contains "roller"/"wheel":
  ```python
  if (args.contact_offset >= 0.0 or args.rest_offset >= 0.0) and \
     ("roller" in pth or "wheel" in pth):
      capi = PhysxSchema.PhysxCollisionAPI.Apply(p)   # PhysxSchema already imported (line 92)
      if args.contact_offset >= 0.0: capi.CreateContactOffsetAttr(args.contact_offset)
      if args.rest_offset    >= 0.0: capi.CreateRestOffsetAttr(args.rest_offset)
  ```
  (Always `rest_offset < contact_offset`; the harness defaults 0.0005 < 0.001 satisfy it.)
- **(1) `--solver-pos` / `--solver-vel`** — IMMEDIATELY after `art.initialize()` (line 402):
  ```python
  if args.solver_pos >= 0: art.set_solver_position_iteration_count(args.solver_pos)
  if args.solver_vel >= 0: art.set_solver_velocity_iteration_count(args.solver_vel)
  ```
  (MUST be after `initialize()`, with the articulation already in the stage.)
- **(6) `--apply-perstep`** — register a physics-step callback (after `art.initialize()` +
  gains set, ~line 452); and in the main loop SKIP the render-rate `art.apply_action()`
  when `args.apply_perstep`:
  ```python
  if args.apply_perstep:
      from omni.physx import get_physx_interface
      def _on_phys(dt):
          art.apply_action(ArticulationAction(
              joint_velocities=ik(cmd["vx"], cmd["vy"], cmd["wz"]),
              joint_indices=np.array(wheel_idx)))
      _sub = get_physx_interface().subscribe_physics_step_events(_on_phys)
  # ... in the while loop:
  if not args.apply_perstep:
      targets = ik(cmd["vx"], cmd["vy"], cmd["wz"])
      art.apply_action(ArticulationAction(joint_velocities=targets,
                                          joint_indices=np.array(wheel_idx)))
  ```
- **(5) `--roller-shape`** — NO code in `scene_mecanum.py`. The harness calls
  `regen_wheel_urdf(shape)` BEFORE launching, which rewrites
  `assets/jetauto_mecanum.urdf` (imported as-is at line 23/110). The flag is informational
  (for the CSV).

Notes: the harness launches the scene with `--headless --no-extra --world ''`. Results go
to `isaac/serpenteo_results.csv`; the best (residual ≤ 4 mm; tie-break RTF then
sim-to-real) is printed at the end. Risk: `phys-hz` 480 with GPU dynamics OFF (CPU,
line 146) may push RTF below the controller timeout — raise the harness margin if so.

---

## (3) RECOMMENDED INTEGRATION + GPU-TEST ORDER

Each step is independently testable; integrate AND GPU-verify in this order so a failure
isolates to one module. (HARD RULE: do not launch Isaac yourself — these are the
orchestrator's GPU runs.)

| # | Module(s) | What to add | GPU test command (after `source isaac/isaac_env.sh`) | Pass criterion |
|---|-----------|-------------|--------------------------------------------------------|----------------|
| 0 | (baseline) | nothing | `$ISAACSIM/python.sh isaac/scene_mecanum.py --no-extra` | robot rests, `/odom` `/tf` `/joint_states` publish |
| 1 | **world_loader** (worlds) | section A | `... scene_mecanum.py --world laberinto` | maze floor horizontal, robot rests inside, walls/colors right |
| 2 | **drone** (drone) | section C | `... scene_mecanum.py --drone` then publish `/drone1/cmd_vel` + `/drone1/takeoff` | drone climbs, `/drone1/odom` + `/uav/camera/image` publish |
| 3 | **cameras** (stitching/multicam) | section E | `... scene_mecanum.py --cameras --rec_cams` | rig builds, chase/scene render, MP4s written |
| 4 | **sensors + aruco3d** (sensors/arucos) | sections D, B | `... scene_mecanum.py --imu --camera --aruco` | `/imu/data_raw`, `/cam_1/image`, ArUco geometry visible to camera |
| 5 | **(ROS-stack)** | — (no new module; run the existing kf_control / nav stack against #0–#4 topics) | `... scene_mecanum.py --gridmap` + the ROS2 brain | `/scan` + nav stack closes the loop |
| 6 | **serpenteo_sweep** (serpenteo) | section G (the 8 physics flags) | `python3 isaac/serpenteo_sweep.py --quick` | CSV rows written, a config beats baseline residual |
| 7 | **perf_hud** (HUD) | section F | add `--perf` to any of the above | `[PERF] fps=.. rtf=.. gpu=..` prints, HUD shows (windowed) |
| 8 | **cameras viewports** (multicam GUI) | section E step 3 | `... scene_mecanum.py --cameras --viewports` (windowed) | one viewport window per camera |

Rationale: worlds first (everything sits on the floor), then the drone (independent
subsystem), then the camera rig and ArUco/sensors that the drone/stitching depend on,
then the full ROS stack, then the physics sweep (heaviest, longest), HUD last (pure
instrumentation), multicam GUI last (needs a window + most VRAM).

---

## (4) INTER-MODULE CONFLICTS & RESOLUTION

1. **Multiple modules edit `/ActionGraph`.** `sensors.add_imu` and
   `sensors.add_rgbd_camera` ADD nodes to the existing `/ActionGraph` (the one created at
   lines 324–366). They depend on the existing node names `OnTick` and `ReadSimTime`.
   - Resolution: call them AFTER line 366 (graph exists) and BEFORE `timeline.play()`
     (line 398). They use unique node names (`ReadIMU`, `PublishIMU`, `CamRenderProduct`,
     `CamHelperRgb/Depth/Info`) — no collision with the scene's nodes. Each function is
     called ONCE (do not double-call, or `og.Controller.edit` errors on duplicate node
     creation).
   - The **drone** does NOT touch `/ActionGraph`; it builds a sibling push graph
     **`/DroneCameraGraph`** — no conflict.

2. **`OnTick` name reuse across graphs.** Both `/ActionGraph` (scene) and
   `/DroneCameraGraph` (drone) define a node literally named `OnTick`. This is SAFE
   because the name is graph-local (controller-local), not a global prim clash — the
   drone's `OnTick` lives under `/DroneCameraGraph`. No action needed.

3. **Several modules create render products on the single GPU.** `sensors.add_rgbd_camera`
   (one offscreen RP every tick), `cameras` rig (one RP per camera), `cameras.Recorder`
   (one RP per camera), `drone` camera (one RP), and the existing `--record` TopCam + the
   `--gridmap` RTX lidar RP. Running many at once can saturate VRAM / tank RTF.
   - Resolution: do NOT enable all camera consumers simultaneously in one run. For the
     sweep/validation runs use `--no-extra` and NO cameras. Use `--cam_res 720p` if OOM.
     `Recorder.stop()` destroys its RPs to free VRAM.

4. **Two modules create their own rclpy node.** The scene creates `isaac_mecanum_drive`
   (line 391); `drone` creates `isaac_drone`; `perf_hud` (with `publish_ros=True`) creates
   `isaac_perf_hud` UNLESS you pass `ros_node=node`.
   - Resolution: all guard `rclpy.init()` with try/except (idempotent). In the main loop,
     spin each node separately via `spin_once` (already the pattern). Pass
     `ros_node=node` to `PerfMonitor` so it reuses the scene's node (avoids a 3rd node).
     Order in `finally:`: `drone.shutdown()` and `perf.shutdown()` (destroy their own
     nodes) BEFORE `rclpy.shutdown()` (line 547). `drone.shutdown()` deliberately does NOT
     call `rclpy.shutdown()`.

5. **TopCam prim-path: `--record` vs `cameras --top`.** The existing `--record` path
   creates `/World/TopCam` (lines 126, 477). `cameras.setup_cameras` with `top` enabled
   also creates a `/World/TopCam`.
   - Resolution: do NOT use `--record` and `cameras`'s `top` together. Either rename one
     (the cameras `top` is `enabled=False` by default; only the suggested
     `specs={"top": {"enabled": args.gridmap}}` turns it on — drop that key when
     `--record` is set), or pick one recorder. The two recorders are independent paths
     (`_rec_step` raw-ffmpeg vs `cameras.Recorder` replicator).

6. **`enable_extension` ordering.** All `enable_extension` calls must precede the relevant
   API use and ideally sit in the 94–99 block before line 100's `update()`:
   `omni.kit.asset_converter` (world_loader maze, drone mesh), `isaacsim.sensors.physics`
   (sensors IMU), `isaacsim.sensors.camera` (cameras rig, sensors needs none extra, drone
   self-enables it, `--record` already enables it). `enable_extension` is idempotent, so
   double-enabling (e.g. drone self-enabling `isaacsim.sensors.camera` after `--record`
   already did) is harmless.

7. **`mat_path` / `/physicsScene` shared state.** world_loader, aruco3d, drone, and the
   serpenteo offsets all consume the already-created `/physicsScene` and
   `mat_path="/physicsMaterial"` (lines 139–157). No module redefines them. cameras and
   perf_hud touch neither. No conflict; just ensure all of them are called AFTER line 157.

8. **`Usd.PrimRange` de-instance pass.** world_loader, aruco3d (via referenced geometry),
   and drone all rely on the `SetInstanceable(False)` de-instance trick before binding
   materials/collision (same as `jetauto_materials.py`). They each de-instance only their
   OWN subtree (`/World/Maze`, `/World/drone1/visual`, etc.), so no cross-talk.
