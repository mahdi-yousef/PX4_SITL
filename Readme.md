# Multicopter Interception via Image-Based Visual Servoing — SITL Testbed

This repository is a software-in-the-loop (SITL) simulation environment
built for my master's thesis, exploring image-based visual servoing (IBVS)
guidance for autonomous multicopter interception of a maneuvering aerial
target. It uses **PX4** (flight stack, in SITL) + **AirSim** (simulator and
target/camera environment) + **MAVSDK** (offboard control from Python).

The guidance approach is based on the ideas presented in **H. Yan, K. Yang,
Y. Cheng, Z. Wang, and D. Li, "Precise Interception Flight Targets by
Image-Based Visual Servoing of Multicopter," IEEE Transactions on
Industrial Electronics, 2025** (arXiv:2409.17497). Broadly, that work
combines an IBVS controller with a proportional navigation guidance (PNG)
law, adds a delayed Kalman filter to keep target position estimates
accurate despite image-processing latency, and includes a field-of-view
holding controller so the target doesn't drift out of frame during the
terminal approach. This project reimplements the core PNG-IBVS guidance
idea in a SITL environment as a first step before any hardware testing.

---

## Project layout

```
.
├── PX4_installation/   # PX4 SITL + AirSim (WSL2) + PyCharm setup guide
├── video/              # link to a recorded demo run
├── screenshots/        # simulation screenshots (see below)
├── PX4_SITL.py         # the SITL script: target drone + interceptor + detection loop
└── Readme.md           # this file
```

---

## How the simulation is structured

`PX4_SITL.py` runs two AirSim vehicles concurrently:

- **`Drone2` — the target** (`run_drone2_launch`). A second AirSim
  multirotor (`simpleflight`) spawned at runtime, armed, taken off, climbed
  to altitude, then set on a constant-velocity escape leg
  (`moveByVelocityAsync`). This stands in for a maneuvering intruder.
- **The PX4 vehicle — the interceptor** (`run_px4_interception`). Armed
  and taken off itself via `drone.action`, then switched into MAVSDK
  offboard mode. Each control-loop iteration:
  1. grabs a camera frame and runs AirSim's detection filter
     (`detect_target`, filtered to `Drone*`/`Cylinder*` mesh names) to get
     the target's bounding-box pixel centroid `(u, v)`,
  2. converts that pixel into a line-of-sight (LOS) unit vector in world
     NED using the pinhole model and the camera's current world pose
     (`pixel_to_NED_LOS`),
  3. converts the LOS vector to elevation/azimuth angles (`vec_to_angles`),
  4. applies the PNG-IBVS law below to get a commanded velocity direction,
     scales it by a saturated closing speed, rotates it into the body
     frame, and sends it as `VelocityBodyYawspeed` — with a small
     proportional yaw term keeping the target centered in frame,
  5. logs every iteration's LOS/command angles and velocities to
     `png_log.csv`, and plots them (`plot_png_log`, `plot_velocity_log`)
     once the run ends.

---

## Guidance law: PNG-IBVS

### The core idea

Classical proportional navigation (PN) guidance commands an interceptor to
null the **rotation rate of the line of sight (LOS)** to the target. If the
LOS direction stops rotating while range is closing, the two are on a
collision course. In an image-based visual servoing scheme, the LOS is
measured directly from the camera image rather than from a separate
navigation/estimation solution — the target's pixel position *is* the LOS
measurement.

The underlying rate law is:

```
σ̇ = k · λ̇
```

- **λ̇ (lambda-dot)** — the LOS rotation rate, per axis (elevation,
  azimuth).
- **σ̇ (sigma-dot)** — the rate of change of the commanded velocity-vector
  direction.
- **k** — the proportional navigation gain (`k = 3` in this
  implementation), analogous to the navigation constant `N` in classical
  PN. Larger `k` reacts to LOS drift more aggressively but is more
  sensitive to detection noise.

In code, this is implemented in its integrated (angle, not rate) form —
`sigma = k * lamda + sigma_offset` — where `sigma_offset` is a constant
fixed once at the first detection (`lamda_0`) so that the initial commanded
direction matches the vehicle's actual initial heading/geometry rather than
jumping discontinuously. Driving the *rate* of LOS rotation toward zero is
still the underlying objective; recomputing `sigma` directly from the
current `lamda` each frame is a discrete way of realizing that same
proportional relationship without explicitly differentiating a noisy pixel
signal.

### Getting λ from pixel coordinates: the pinhole model

Each detection gives a target centroid in pixel coordinates `(u, v)`. In
`pixel_to_NED_LOS`, the pinhole model turns that into a unit bearing vector
in the **camera frame**:

```
los_cam = normalize([f_pixels, u − cx, v − cy])
```

Here `f_pixels` acts as the forward (boresight) component and `(u − cx,
v − cy)` are the pixel offsets from the image center, so the vector's
direction already encodes the bearing angle implied by the pinhole
projection (a pixel offset from the principal point corresponds to an
angular offset from the optical axis, scaled by focal length).

That vector is then rotated from the camera frame into world NED using the
camera's current orientation quaternion (`simGetCameraInfo` →
`quat_to_rot_world2local`, transposed since camera→world is the inverse of
world→camera). `vec_to_angles` then converts the resulting world-frame LOS
vector into elevation/azimuth:

```
elevation = atan( z / sqrt(x² + y²) )
azimuth   = atan2( y, x )
```

This `λ = [elevation, azimuth]` is what feeds `sigma = k·λ + offset` each
iteration, and `angles_to_vec` converts the resulting commanded `sigma`
back into a unit direction vector for the velocity command.

---

## Guidance law: PNG-IBVS

### The core idea

Classical proportional navigation (PN) guidance commands an interceptor to
null the **rotation rate of the line of sight (LOS)** to the target. If the
LOS direction stops rotating while range is closing, the two are on a
collision course. In an image-based visual servoing scheme, the LOS is
measured directly from the camera image rather than from a separate
navigation/estimation solution — the target's pixel position *is* the LOS
measurement.

The proportional navigation control law used here has the form:

```
σ̇ = k · λ̇
```

- **λ̇ (lambda-dot)** — the LOS rotation rate (how fast the bearing to the
  target is changing), computed per-axis (horizontal/azimuth and
  vertical/elevation) from the image.
- **σ̇ (sigma-dot)** — the interceptor's commanded turn rate about that
  axis (e.g. yaw rate for the horizontal channel, a body-rate/velocity
  command for the vertical channel).
- **k** — a proportional (navigation) gain, analogous to the navigation
  constant `N` in classical PN. Larger `k` reacts to LOS drift more
  aggressively but is more sensitive to detection noise.

Driving `λ̇ → 0` on both axes is the closed-loop objective: as long as the
interceptor is closing range, a non-rotating LOS is a collision-course
condition.

### Getting λ (and λ̇) from pixel coordinates: the pinhole model

Each detection gives a target centroid in pixel coordinates `(u, v)`. With
the camera's intrinsics — focal length in pixels `(fx, fy)` and principal
point `(cx, cy)` — the pinhole projection model converts that pixel into a
bearing angle off the camera boresight:

```
azimuth   (horizontal LOS angle):  λ_h = atan2(u − cx, fx)
elevation (vertical LOS angle):    λ_v = atan2(v − cy, fy)
```

This is just the inverse of the pinhole projection: a pixel offset from
the principal point corresponds to an angular offset from the optical
axis, scaled by focal length.

The LOS **rate** is then the time-derivative of these angles. In practice,
with detections arriving frame-to-frame, this is computed as a finite
difference:

```
λ̇ ≈ (λ(t) − λ(t − Δt)) / Δt
```

or, equivalently and more directly from pixel velocity `(u̇, v̇)` (e.g. from
consecutive detections or optical flow of the bounding-box centroid), via
the small-angle/paraxial approximation:

```
λ̇_h ≈ u̇ / fx        λ̇_v ≈ v̇ / fy
```

Feeding these two rates into `σ̇ = k · λ̇` (one instance per axis) produces
the yaw-rate and vertical/lateral velocity commands sent to PX4 through
MAVSDK offboard mode — replacing the fixed `VelocityBodyYawspeed` call
currently in `run_px4_camera_routine`.

---

## Launching the simulation

1. Follow [`PX4_installation/Readme.md`](PX4_installation/Readme.md) once
   to set up PX4 SITL, the WSL2 ↔ AirSim networking, and the broadcast fix
   for PyCharm.
2. **Open AirSim** on Windows first (it needs to be listening before PX4
   connects).
3. In WSL2, run the shortcut set up in the install guide:
   ```bash
   px4
   ```
   Wait for the `pxh>` console to report a successful simulator connection
   and passing health checks.
4. In **PyCharm** (Windows), open and run `PX4_SITL.py`. It will:
   - connect to AirSim, then connect MAVSDK to the PX4 instance over the
     broadcast link,
   - arm and take off the PX4 interceptor, and spawn/arm/launch `Drone2`
     as the moving target,
   - run the PNG-IBVS control loop with the live detection window, logging
     to `png_log.csv` as it goes.
5. `c`/`a` in the detection window clear/re-add the detection mesh-name
   filter live. To stop, use **Ctrl+C** in the terminal (there's no `q`
   quit-key handler in this version) — this triggers the `finally` block,
   which closes the log file and pops up the LOS/velocity plots.

---

## Screenshots

Drop images into [`screenshots/`](screenshots) and reference them below,
e.g.:

```markdown
![AirSim environment](screenshots/airsim_environment.png)
![Detection bounding box](screenshots/detection_bbox.png)
```

Recommended screenshots to capture for the thesis writeup:

- **AirSim environment overview** — both vehicles visible (interceptor +
  `Drone2` target) in the simulated world.
- **PX4 `pxh>` console** — showing a successful simulator TCP connection
  and passing `is_global_position_ok` / `is_home_position_ok` checks.
- **Detection window** — the OpenCV view with the bounding box and label
  drawn on the target, ideally at a couple of different ranges.
- **PyCharm console output** — MAVSDK reporting "Connected to px4 drone!"
  and offboard mode starting successfully.
- **`settings.json` / project structure** (optional) — useful for
  reproducibility appendices.
- **The `plot_png_log` figure** — λ (LOS) vs σ (commanded) elevation and
  azimuth over the run, straight from `png_log.csv`.
- **The `plot_velocity_log` figure** — actual vs desired NED velocity
  components over the run.
