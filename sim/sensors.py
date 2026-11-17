"""Spot body cameras, headlights and the GUI presenter camera (Isaac Sim 6.1.0).

What this is
------------
The visual layer on top of the facade in ``sim/isaac_app.py``:

* ``create_spot_cameras`` attaches USD camera prims to Spot's ``body`` link at
  approximate spot_ros2 body-camera extrinsics and publishes each one through
  the verified OmniGraph ``ROS2CameraHelper`` / ``ROS2CameraInfoHelper``
  pattern (docs/isaac-api-notes.md section 4) as
  ``/demo_spot/camera/<name>/image`` (sensor_msgs/Image, rgb8) and
  ``/demo_spot/camera/<name>/camera_info``.
* ``add_nose_spotlight`` puts Spot's payload lights on its nose: a wide
  flood plus a narrow long-throw beam. The roofed tank is dark inside (only
  a dim fill, build_scene.py), so these are what lights the interior for the
  front cameras and the GUI; real Spot carries payload lights for exactly
  this job.
* ``ChaseCamera`` is the unpublished GUI viewport camera (``isaac_app
  --headless false``). Its ``CameraDirector`` (pure, below) implements the
  named presenter views of ``/demo_spot/payload/view``: ``auto`` (low 3/4
  chase outside, interior corner camera inside), ``chase``, ``interior``,
  ``overhead`` and ``door``.

The top half of the file is pure Python (math only, no Isaac/pxr imports) so
the geometry is unit-tested on a Mac: ``python3 -m unittest
tests.test_sensors``. Everything that touches USD / OmniGraph imports lazily
inside the function, and only runs inside the Isaac container after
``SimulationApp`` exists.

Conventions
-----------
Body frame is Spot's ``/World/Spot/body`` link: +X forward, +Y left, +Z up.
Measured link bounding box on the 6.1 asset: x -0.43..+0.44, y +/-0.12,
z -0.11..+0.09 m, so the nose face is at x = +0.444. USD cameras look down
their local -Z with +Y up; ``camera_rotation_xyz`` converts a (yaw, pitch
down) aim in the parent frame into XformCommonAPI XYZ Euler degrees.
"""

import math

# ---------------------------------------------------------------------------
# Camera set (spot_ros2 names) and approximate body-frame extrinsics
# ---------------------------------------------------------------------------

CAMERA_NAMES = ("frontleft", "frontright", "left", "right", "back")
DEFAULT_CAMERAS = ("frontleft", "frontright")

# name -> (translation xyz in body frame [m], yaw [deg, CCW from +X], pitch
# down [deg]). Approximates the real body-camera layout rather than the exact
# fisheye calibration: a forward stereo pair on the nose tilted ~20 deg down
# (the real pair looks down at the ground ahead), one camera per flank and one
# at the rear. Every lens sits 1-2 cm OUTSIDE the body-link bounding box so the
# near clip never slices through Spot's own shell.
CAMERA_EXTRINSICS = {
    "frontleft": ((0.46, 0.045, 0.02), 0.0, 20.0),
    "frontright": ((0.46, -0.045, 0.02), 0.0, 20.0),
    "left": ((-0.05, 0.135, 0.02), 90.0, 15.0),
    "right": ((-0.05, -0.135, 0.02), -90.0, 15.0),
    "back": ((-0.445, 0.0, 0.02), 180.0, 20.0),
}

# Pinhole approximation of the wide body cameras: 20.955 mm is the USD default
# horizontal aperture; f = 11 mm gives ~87 deg horizontal FOV at 4:3.
CAMERA_FOCAL_MM = 11.0
CAMERA_H_APERTURE_MM = 20.955
CAMERA_CLIP = (0.03, 200.0)

# Payload lights on the nose centerline. The flood (wide cone, 15 deg down)
# lands its pool ~1.5-3 m ahead where the front cameras look; the beam
# (narrow, nearly level) throws a hot spot onto the wall / ladder ahead so the
# GUI shots show where Spot is heading. UsdLux SphereLight + ShapingAPI
# (cone angle = half-angle, deg). Intensities were set by eye on the host
# (RTX Real-Time 2.0, default exposure) against the roofed tank's dim fill:
# the v1 60000 flood left the far end near-black (docs/evidence/09).
SPOTLIGHT_OFFSET = (0.47, 0.0, 0.0)
SPOTLIGHT_PITCH_DOWN_DEG = 15.0
SPOTLIGHT_CONE_DEG = 45.0
SPOTLIGHT_SOFTNESS = 0.4
SPOTLIGHT_RADIUS_M = 0.06
SPOTLIGHT_INTENSITY = 700000.0
BEAM_OFFSET = (0.47, 0.0, 0.03)
BEAM_PITCH_DOWN_DEG = 4.0
BEAM_CONE_DEG = 14.0
BEAM_SOFTNESS = 0.25
BEAM_RADIUS_M = 0.04
BEAM_INTENSITY = 2500000.0
# Outside in daylight the payload lights only add glare: the HUD's body-camera
# tile showed a blown-out white floor at spawn and after the retreat, and the
# GUI a white hot spot on the sunlit ground (rehearsal 2026-09-28). They are
# for the dark tank, so they fade in near the shell (``payload_light_scale``):
# full within LIGHTS_FULL_OUTSIDE_M of it (the door-probe seat is ~0.6 m
# out), off beyond LIGHTS_OFF_OUTSIDE_M (Spot sits ~1.8 m out after an exit
# or a retreat; spawn is 8 m out).
LIGHTS_FULL_OUTSIDE_M = 0.8
LIGHTS_OFF_OUTSIDE_M = 1.6


def parse_camera_list(spec):
    """``"all" | "none" | "a,b,..."`` -> tuple of camera names (stable order).

    Raises ValueError on an unknown name so a typo fails at launch, not in
    front of the audience with a silently missing feed.
    """
    text = (spec or "").strip().lower()
    if text in ("", "none"):
        return ()
    if text == "all":
        return tuple(CAMERA_NAMES)
    names = [n.strip() for n in text.split(",") if n.strip()]
    unknown = [n for n in names if n not in CAMERA_NAMES]
    if unknown:
        raise ValueError(
            "unknown camera name(s) %s; valid: %s, all, none"
            % (", ".join(unknown), ", ".join(CAMERA_NAMES)))
    # De-duplicate while keeping the canonical spot_ros2 order.
    return tuple(n for n in CAMERA_NAMES if n in names)


def frame_skip_count(render_hz, rate_hz):
    """ROS2CameraHelper ``frameSkipCount`` that turns the stage render rate
    into (approximately) the requested publish rate.

    The helper publishes on every rendered frame unless told to skip, so a
    20 Hz GUI render with rate_hz=10 needs skip=1. Never negative: asking for
    more than the render rate just publishes every frame.
    """
    if rate_hz <= 0 or render_hz <= 0:
        return 0
    return max(0, int(round(float(render_hz) / float(rate_hz))) - 1)


def camera_rotation_xyz(yaw_deg, pitch_down_deg):
    """XformCommonAPI XYZ Euler (deg) aiming a USD camera along a heading.

    A USD camera looks along local -Z with +Y up. Rotating X by 90 deg makes
    it look along +Y with +Z up; Z by -90 deg then turns that to +X. Reducing
    the X rotation tilts the view down, and adding yaw to the Z rotation turns
    it about the vertical. (Derivation checked in tests.test_sensors.)
    """
    return (90.0 - float(pitch_down_deg), 0.0, float(yaw_deg) - 90.0)


def light_rotation_xyz(pitch_down_deg):
    """XformCommonAPI XYZ Euler (deg) aiming a UsdLux light along parent +X,
    tilted down. Lights emit along local -Z; RotateY(-90) maps -Z to +X and
    RotateY(-(90 - p)) tilts that ``p`` degrees below the horizon."""
    return (0.0, -(90.0 - float(pitch_down_deg)), 0.0)


def rotate_xyz(rot_deg, vec):
    """Apply XformCommonAPI XYZ Euler rotation (X, then Y, then Z) to ``vec``.

    Only used by the tests and by diagnostics; kept here so both use the same
    convention as ``camera_rotation_xyz``.
    """
    x, y, z = vec
    ax, ay, az = (math.radians(a) for a in rot_deg)
    # X
    y, z = y * math.cos(ax) - z * math.sin(ax), y * math.sin(ax) + z * math.cos(ax)
    # Y
    x, z = x * math.cos(ay) + z * math.sin(ay), -x * math.sin(ay) + z * math.cos(ay)
    # Z
    x, y = x * math.cos(az) - y * math.sin(az), x * math.sin(az) + y * math.cos(az)
    return (x, y, z)


# ---------------------------------------------------------------------------
# Presenter camera (GUI viewport): named views and the auto director (pure)
# ---------------------------------------------------------------------------
#
# One unpublished USD camera (ChaseCamera.PATH) is the GUI viewport camera.
# CameraDirector decides where it is on every render tick. The presenter
# picks a view on /demo_spot/payload/view ({"camera": NAME}, host/set_view.sh):
#   auto      (default) low 3/4 rear chase outside the tank; once Spot is
#             through the door, the interior corner camera. The only cut is
#             the wall crossing (and back).
#   chase     the chase everywhere; inside it rides lower and closer, off the
#             shell.
#   interior  the fixed interior corner camera: high on the door side of the
#             tank, panning and zooming on Spot, so the curved wall, the
#             ladder on the far wall, the sludge and the pool of Spot's
#             headlight stay in frame.
#   overhead  under the roof, looking straight down, gliding with Spot (plan
#             view of the route between the pipe runs).
#   door      fixed lens outside, beside the cleanout door (probe close-up).
# A view change, a teleport (reset_spot.sh) and auto's wall crossing are hard
# cuts; everything else is low-pass smoothed, dt-exact.

VIEW_NAMES = ("auto", "chase", "interior", "overhead", "door")
DEFAULT_VIEW = "auto"
UI_MODES = ("presentation", "editor")
DEFAULT_UI_MODE = "presentation"
# Kit settings sim/isaac_app.py _set_ui_mode applies before every layout
# change. omni.kit.ui.actions binds ESCAPE -> cancel_fullscreen_ui on the
# Viewport window when it goes fullscreen (actions.py update_cancel_hotkey,
# gated by this setting, true in its extension.toml): one stray Esc in the
# DCV viewer dropped the demo from the fullscreen render to the 1440x863
# editor with every panel (host 2026-09-27, xdotool key Escape). Presentation
# is left on purpose only: set_view.sh --ui editor (or F11).
KIT_UI_SETTINGS = {"/exts/omni.kit.ui.actions/ui_hotkey_escape": False}

# Scene constants mirrored from sim/build_scene.py (this module stays pxr-free).
TANK_ROOF_Z = 4.0            # build_scene.TANK_HEIGHT: shell top = roof deck underside
DOOR_HALF_WIDTH = 0.65       # build_scene.DOOR_WIDTH / 2
DOOR_TOP_Z = 1.30            # build_scene.DOOR_HEIGHT

# Chase, outside: a low 3/4 rear view with the tank and door ahead of Spot.
CHASE_OUT_DIST = 3.4         # horizontal lens distance from Spot (m)
CHASE_OUT_EYE_Z = 1.15       # absolute lens height (m): just above Spot's back
CHASE_OUT_SIDE_DEG = 25.0    # lens this far round from dead astern, to Spot's right
# Walking away from the tank (exit, retreat) there is no room astern (the
# shell), so the lens goes ahead-right and Spot walks toward it.
CHASE_FRONT_SIDE_DEG = 35.0
FRONT_ON_COS = 0.4           # heading . outward radial above this -> front view
FRONT_OFF_COS = 0.0          # ... back to the rear view below this (hysteresis)
FRONT_RANGE_M = 5.0          # front view only this close to the shell
# Chase, inside ("chase" view only): lower-angle trailing inside the shell.
CHASE_IN_DIST = 2.4
CHASE_IN_EYE_Z = 1.5
CHASE_IN_SIDE_DEG = 30.0
CHASE_WALL_MARGIN = 0.5      # keep the lens this far from the shell
CHASE_LOOK_AHEAD = 1.0       # aim this far ahead of the body (rear views)
CHASE_FRONT_LEAD = 0.3       # ... and this far in front views
CHASE_TARGET_Z = 0.45        # aim height (standing body ~0.5 m, sitting ~0.2 m)
CHASE_FOCAL_MM = 16.0        # ~66 deg horizontal FOV (20.955 mm aperture)
CHASE_TAU_S = 0.8            # lens orbit smoothing (s)
CHASE_ORBIT_MAX_RAD_S = 0.6  # lens orbit speed cap round Spot (rad/s): no whip pans
TARGET_TAU_S = 0.5           # followed-body / aim smoothing (s)
HEADING_TAU_S = 0.6          # heading smoothing (s)
ZOOM_TAU_S = 1.2             # focal-length smoothing (s)
TELEPORT_M = 2.0             # a body jump this big between ticks is a reset: cut
# Exterior dressing (build_scene "camera_keepouts": barrels, pipe rack): the
# chase lens keeps this clear of them and needs a clear line to Spot, else it
# moves to Spot's other side (the exit front view used to park inside the
# pipe rack: host 2026-09-27, gp2).
KEEPOUT_CLEARANCE_M = 0.6
# The side of the shell a shot is on is the side a point CHASE_SIDE_PROBE
# behind Spot is on. In the doorway that means: entering, keep filming from
# outside through the 1.3 m opening until Spot's tail is under the lintel;
# leaving, keep the inside shot until it is out. (A lens above Spot in the
# doorway sits in, or stares into, the 2.7 m of wall above the door.)
CHASE_SIDE_PROBE = 0.5
CHASE_SIDE_HYST = 0.05       # side switch hysteresis (m): no cut ping-pong

# Interior corner camera (the auto view inside the tank).
INTERIOR_BEARING_DEG = -50.0  # lens bearing from the tank centre, from the door normal
INTERIOR_EYE_R_FRAC = 0.88    # x tank radius (5.28 m for the 6 m tank)
INTERIOR_EYE_Z = 2.6
INTERIOR_FRAME_M = 4.2        # scene width kept in frame around Spot (zoom)
INTERIOR_HFOV_DEG = (28.0, 70.0)
INTERIOR_LEAD_M = 0.4         # aim slightly ahead of Spot
# Overhead: under the roof deck, straight down, image right = door side.
OVERHEAD_EYE_Z = 3.85
OVERHEAD_TRAVEL_M = (3.8, 1.5)  # max lens offset from the centre along / across the door axis
OVERHEAD_FOCAL_MM = 12.0        # ~82 deg HFOV
# Door: fixed lens outside, beside the cleanout door.
DOOR_EYE_OUT_M = 2.3            # out along the door normal from the door centre
DOOR_EYE_SIDE_M = -2.0          # along the door tangent (the normal turned +90 deg)
DOOR_EYE_Z = 1.25
DOOR_FOCAL_MM = 18.0
DOOR_TRACK_M = (4.0, 6.5)       # aim at Spot within 4 m of the door, the door beyond 6.5 m


def parse_view_command(text):
    """``payload/view`` text -> ``(camera or None, ui or None)``.

    Accepts JSON ``{"camera": NAME}`` and/or ``{"ui": "presentation"|"editor"}``
    or a bare view name (``interior``). Raises ValueError otherwise, so the
    facade can log the bad request and keep the current view.
    """
    import json

    raw = (text or "").strip()
    try:
        d = json.loads(raw) if raw else None
    except ValueError:
        d = raw
    if isinstance(d, str):
        d = {"camera": d}
    if not isinstance(d, dict) or not d:
        raise ValueError(f'payload/view expects {{"camera": {"|".join(VIEW_NAMES)}}} '
                         f'and/or {{"ui": {"|".join(UI_MODES)}}}')
    unknown = sorted(set(d) - {"camera", "ui"})
    if unknown:
        raise ValueError(f"payload/view: unknown key(s) {', '.join(unknown)}")
    cam, ui = d.get("camera"), d.get("ui")
    if cam is not None:
        cam = str(cam).strip().lower()
        if cam not in VIEW_NAMES:
            raise ValueError(f"unknown camera {cam!r}; valid: {', '.join(VIEW_NAMES)}")
    if ui is not None:
        ui = str(ui).strip().lower()
        if ui not in UI_MODES:
            raise ValueError(f"unknown ui mode {ui!r}; valid: {', '.join(UI_MODES)}")
    if cam is None and ui is None:
        raise ValueError('payload/view: give "camera" and/or "ui"')
    return cam, ui


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _alpha(dt, tau):
    """Low-pass blend for one step of dt (exact, so the render rate does not
    change the feel)."""
    if dt <= 0.0:
        return 0.0
    return 1.0 - math.exp(-dt / max(tau, 1.0e-3))


def focal_for_hfov(hfov_deg, aperture_mm=CAMERA_H_APERTURE_MM):
    """Focal length (mm) giving a horizontal FOV for the horizontal aperture."""
    return 0.5 * aperture_mm / math.tan(math.radians(hfov_deg) / 2.0)


def hfov_for_focal(focal_mm, aperture_mm=CAMERA_H_APERTURE_MM):
    return math.degrees(2.0 * math.atan(0.5 * aperture_mm / focal_mm))


def shot_side(body_xyz, yaw, tank_center_xy, tank_radius, inside=None):
    """True if the shot belongs inside the shell: a point CHASE_SIDE_PROBE
    behind Spot is inside it. Pass the current side as ``inside`` to apply
    CHASE_SIDE_HYST hysteresis."""
    px, py = float(body_xyz[0]), float(body_xyz[1])
    cx, cy = (float(v) for v in tank_center_xy)
    probe_r = math.hypot(px - cx - CHASE_SIDE_PROBE * math.cos(yaw),
                         py - cy - CHASE_SIDE_PROBE * math.sin(yaw))
    if inside is None:
        return probe_r < tank_radius
    if inside:
        return probe_r < tank_radius + CHASE_SIDE_HYST
    return probe_r < tank_radius - CHASE_SIDE_HYST


def allowed_arc(qx, qy, dist, limit, outside):
    """Lens azimuths ``a`` for which ``q + dist * (cos a, sin a)`` (``q`` =
    Spot relative to the tank centre) stays outside (``outside``: radius >=
    ``limit``) or inside (radius <= ``limit``) the circle.

    Returns ``(centre_angle, half_width)`` with half_width in [0, pi] (pi =
    every azimuth), or None if no azimuth works. The allowed set is always
    one contiguous arc, which is what lets the orbit smoothing go round the
    permitted side instead of through the tank.
    """
    r = math.hypot(qx, qy)
    if r < 1.0e-6:
        ok = dist >= limit if outside else dist <= limit
        return (0.0, math.pi) if ok else None
    phi = math.atan2(qy, qx)
    m = (limit * limit - r * r - dist * dist) / (2.0 * dist * r)
    if outside:                       # need cos(a - phi) >= m
        if m <= -1.0:
            return (phi, math.pi)
        if m > 1.0:
            return None
        return (phi, math.acos(m))
    if m >= 1.0:                      # inside: need cos(a - phi) <= m
        return (_wrap(phi + math.pi), math.pi)
    if m < -1.0:
        return None
    return (_wrap(phi + math.pi), math.pi - math.acos(m))


def _clamp_to_arc(a, arc, prefer=None):
    """Nearest azimuth to ``a`` inside ``arc``. If ``a`` is outside it and
    ``prefer`` (the lens's current azimuth) is given, the arc edge nearest
    ``prefer`` wins instead: the lens stays on its side of the forbidden zone
    rather than flipping across whenever Spot's heading swings past it."""
    centre, half = arc
    rel = _wrap(a - centre)
    if abs(rel) <= half:
        return _wrap(a)
    if prefer is not None:
        return _wrap(centre + (half if _wrap(prefer - centre) >= 0.0 else -half))
    return _wrap(centre + max(-half, min(half, rel)))


def orbit_toward(a, a_des, alpha, arc, max_step=math.pi):
    """Low-pass the lens azimuth ``a`` toward ``a_des`` without leaving
    ``arc``: the lens orbits round the side it is allowed on, never through
    the tank (a Cartesian blend could cut through the shell or Spot). The
    move is also capped at ``max_step`` rad per call."""
    if arc is None or arc[1] >= math.pi - 1.0e-9:
        step = alpha * _wrap(a_des - a)
        return _wrap(a + max(-max_step, min(max_step, step)))
    centre, half = arc
    rel = max(-half, min(half, _wrap(a - centre)))
    rel_des = max(-half, min(half, _wrap(a_des - centre)))
    step = max(-max_step, min(max_step, alpha * (rel_des - rel)))
    return _wrap(centre + rel + step)


def chase_placement(body_xyz, yaw, tank_center_xy, tank_radius, inside, front=False,
                    current_az=None, side=1):
    """Desired chase lens placement ``(azimuth, dist, eye_z, arc)``.

    Outside: rear-right 3/4 (``front``: ahead-right) at CHASE_OUT_DIST,
    clamped to the azimuths that keep the lens CHASE_WALL_MARGIN outside the
    shell. Inside: rear-right at CHASE_IN_DIST, kept CHASE_WALL_MARGIN inside
    it (closer if Spot is too near the wall for the full distance). With
    ``current_az`` a blocked preference resolves to the arc edge on the
    lens's current side (see _clamp_to_arc). ``side`` -1 mirrors the shot
    to Spot's left.
    """
    px, py = float(body_xyz[0]), float(body_xyz[1])
    cx, cy = (float(v) for v in tank_center_xy)
    qx, qy = px - cx, py - cy
    sgn = 1.0 if side >= 0 else -1.0
    if inside:
        dist, eye_z = CHASE_IN_DIST, CHASE_IN_EYE_Z
        limit = max(tank_radius - CHASE_WALL_MARGIN, 0.1)
        a_pref = yaw + math.pi + sgn * math.radians(CHASE_IN_SIDE_DEG)
    else:
        dist, eye_z = CHASE_OUT_DIST, CHASE_OUT_EYE_Z
        limit = tank_radius + CHASE_WALL_MARGIN
        if front:
            a_pref = yaw - sgn * math.radians(CHASE_FRONT_SIDE_DEG)
        else:
            a_pref = yaw + math.pi + sgn * math.radians(CHASE_OUT_SIDE_DEG)
    arc = allowed_arc(qx, qy, dist, limit, outside=not inside)
    scale = 1.0
    while arc is None and scale > 0.2:   # inside, Spot near the wall: come closer
        scale -= 0.2
        arc = allowed_arc(qx, qy, dist * scale, limit, outside=not inside)
        if arc is not None:
            dist *= scale
    if arc is None:                      # doorway with the inside shot: lens toward the centre
        r = math.hypot(qx, qy)
        return _wrap(math.atan2(qy, qx) + math.pi), max(r - limit + 0.3, 0.3), eye_z, None
    return _clamp_to_arc(_wrap(a_pref), arc, prefer=current_az), dist, eye_z, arc


def interior_eye(tank_center_xy, tank_radius, door_normal_yaw=0.0):
    """The interior corner camera's fixed lens position."""
    b = float(door_normal_yaw) + math.radians(INTERIOR_BEARING_DEG)
    r = INTERIOR_EYE_R_FRAC * float(tank_radius)
    return (float(tank_center_xy[0]) + r * math.cos(b),
            float(tank_center_xy[1]) + r * math.sin(b), INTERIOR_EYE_Z)


def interior_focal_mm(eye, target):
    """Zoom that keeps INTERIOR_FRAME_M of scene around the target in frame."""
    d = math.sqrt(sum((float(t) - float(e)) ** 2 for e, t in zip(eye, target)))
    hfov = math.degrees(2.0 * math.atan(0.5 * INTERIOR_FRAME_M / max(d, 0.1)))
    lo, hi = INTERIOR_HFOV_DEG
    return focal_for_hfov(min(max(hfov, lo), hi))


def door_eye(tank_center_xy, tank_radius, door_normal_yaw=0.0):
    nx, ny = math.cos(door_normal_yaw), math.sin(door_normal_yaw)
    dx = float(tank_center_xy[0]) + float(tank_radius) * nx
    dy = float(tank_center_xy[1]) + float(tank_radius) * ny
    return (dx + DOOR_EYE_OUT_M * nx - DOOR_EYE_SIDE_M * ny,
            dy + DOOR_EYE_OUT_M * ny + DOOR_EYE_SIDE_M * nx, DOOR_EYE_Z)


def chase_camera_shot(body_xyz, yaw, tank_center_xy, tank_radius, inside=None,
                      door_normal_yaw=0.0):
    """Desired (unsmoothed) ``auto`` shot ``(eye, target, inside)``: the
    rear-right chase outside, the interior corner camera inside. The side is
    the one a point CHASE_SIDE_PROBE behind Spot is on; pass the current side
    as ``inside`` to apply CHASE_SIDE_HYST hysteresis."""
    px, py = float(body_xyz[0]), float(body_xyz[1])
    fx, fy = math.cos(yaw), math.sin(yaw)
    inside = shot_side(body_xyz, yaw, tank_center_xy, tank_radius, inside)
    if inside:
        eye = interior_eye(tank_center_xy, tank_radius, door_normal_yaw)
        target = (px + INTERIOR_LEAD_M * fx, py + INTERIOR_LEAD_M * fy, CHASE_TARGET_Z)
    else:
        a, dist, eye_z, _ = chase_placement(body_xyz, yaw, tank_center_xy, tank_radius, False)
        eye = (px + dist * math.cos(a), py + dist * math.sin(a), eye_z)
        target = (px + CHASE_LOOK_AHEAD * fx, py + CHASE_LOOK_AHEAD * fy, CHASE_TARGET_Z)
    return eye, target, inside


def chase_camera_target(body_xyz, yaw, tank_center_xy, tank_radius):
    """Desired (eye, target) world points (stateless; see chase_camera_shot)."""
    eye, target, _ = chase_camera_shot(body_xyz, yaw, tank_center_xy, tank_radius)
    return eye, target


def parse_keepouts(items):
    """build_scene ``camera_keepouts`` -> [("circle", x, y, r) | ("box", x0, y0, x1, y1)].
    Malformed entries are skipped (cosmetic data)."""
    out = []
    for item in items or ():
        try:
            if "r" in item:
                out.append(("circle", float(item["xy"][0]), float(item["xy"][1]), float(item["r"])))
            else:
                (x0, y0), (x1, y1) = item["min"], item["max"]
                out.append(("box", min(float(x0), float(x1)), min(float(y0), float(y1)),
                            max(float(x0), float(x1)), max(float(y0), float(y1))))
        except (KeyError, TypeError, ValueError, IndexError):
            continue
    return out


def keepout_distance(x, y, keepout):
    """Horizontal distance from (x, y) to a keep-out (0 inside it)."""
    if keepout[0] == "circle":
        _, cx, cy, r = keepout
        return max(0.0, math.hypot(x - cx, y - cy) - r)
    _, x0, y0, x1, y1 = keepout
    return math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))


def lens_clear(eye_xy, body_xy, keepouts, clearance=KEEPOUT_CLEARANCE_M):
    """True if the lens is ``clearance`` from every keep-out and its line to
    Spot does not pass through one."""
    for k in keepouts:
        if keepout_distance(eye_xy[0], eye_xy[1], k) < clearance:
            return False
        for i in range(1, 20):
            t = i / 20.0
            if keepout_distance(eye_xy[0] + t * (body_xy[0] - eye_xy[0]),
                                eye_xy[1] + t * (body_xy[1] - eye_xy[1]), k) <= 0.05:
                return False
    return True


class Shot(tuple):
    """``(eye, target, focal_mm, rot)``; ``rot`` is an explicit XformCommonAPI
    XYZ rotation (overhead) or None (aim the lens at ``target``)."""

    __slots__ = ()

    def __new__(cls, eye, target, focal_mm, rot=None):
        return tuple.__new__(cls, (tuple(eye), None if target is None else tuple(target),
                                   float(focal_mm), rot))

    eye = property(lambda self: self[0])
    target = property(lambda self: self[1])
    focal_mm = property(lambda self: self[2])
    rot = property(lambda self: self[3])


class CameraDirector:
    """Stateful presenter camera: ``step()`` once per render tick.

    Pure Python (unit-tested on a Mac). ``kind`` is the shot in use
    (chase_out / chase_in / interior / overhead / door), ``inside`` the side
    of the shell, ``cuts`` counts hard cuts, ``last_cut`` says whether the
    latest step was one.
    """

    def __init__(self, tank_center_xy=(0.0, 0.0), tank_radius=6.0, door_normal_yaw=0.0,
                 view=DEFAULT_VIEW, keepouts=None):
        self.center = (float(tank_center_xy[0]), float(tank_center_xy[1]))
        self.keepouts = parse_keepouts(keepouts)
        self.side = 1                  # chase lens on Spot's right (+1) or left (-1)
        self.radius = float(tank_radius)
        self.door_yaw = float(door_normal_yaw)
        self.door_xy = (self.center[0] + self.radius * math.cos(self.door_yaw),
                        self.center[1] + self.radius * math.sin(self.door_yaw))
        self.view = DEFAULT_VIEW
        self.inside = None
        self.front = False
        self.kind = None
        self.cuts = 0
        self.last_cut = False
        self._s = None                 # smoothed state of the current shot
        self._last_body = None
        self.set_view(view)

    def set_view(self, name):
        """Switch the named view (hard cut on the next step). Returns the
        canonical name; raises ValueError for an unknown one."""
        name = str(name).strip().lower()
        if name not in VIEW_NAMES:
            raise ValueError(f"unknown camera {name!r}; valid: {', '.join(VIEW_NAMES)}")
        if name != self.view:
            self.view = name
            self._s = None
        return name

    def _kind(self):
        if self.view == "auto":
            return "interior" if self.inside else "chase_out"
        if self.view == "chase":
            return "chase_in" if self.inside else "chase_out"
        return self.view

    def _update_front(self, body_xyz, yaw, cut):
        qx, qy = body_xyz[0] - self.center[0], body_xyz[1] - self.center[1]
        r = math.hypot(qx, qy)
        if r < 1.0e-6 or self.inside or r - self.radius > FRONT_RANGE_M:
            self.front = False
            return
        c = (math.cos(yaw) * qx + math.sin(yaw) * qy) / r
        if cut:
            self.front = c > FRONT_ON_COS
        elif self.front:
            self.front = c > FRONT_OFF_COS
        else:
            self.front = c > FRONT_ON_COS

    def step(self, body_xyz, yaw, dt):
        """Advance by ``dt`` seconds with Spot at ``body_xyz`` facing ``yaw``
        -> Shot."""
        body = tuple(float(v) for v in body_xyz)
        yaw = float(yaw)
        cut = self._s is None
        if (self._last_body is not None
                and math.hypot(body[0] - self._last_body[0], body[1] - self._last_body[1]) > TELEPORT_M):
            cut = True                  # reset_spot.sh teleport
        self._last_body = body
        self.inside = shot_side(body, yaw, self.center, self.radius, None if cut else self.inside)
        kind = self._kind()
        if kind != self.kind:
            cut = True
        self.kind = kind
        self._update_front(body, yaw, cut)
        if cut:
            self._s = None
            self.cuts += 1
        self.last_cut = cut
        shot_fn = {"chase_out": self._chase, "chase_in": self._chase, "interior": self._interior,
                   "overhead": self._overhead, "door": self._door}[kind]
        return shot_fn(body, yaw, 0.0 if cut else float(dt))

    # -- shots ------------------------------------------------------------

    def _follow(self, body, yaw, dt):
        """Smoothed body position and heading (the followed subject)."""
        s = self._s
        if s is None:
            s = self._s = {"body": body[:2], "yaw": yaw}
        else:
            a = _alpha(dt, TARGET_TAU_S)
            s["body"] = (s["body"][0] + a * (body[0] - s["body"][0]),
                         s["body"][1] + a * (body[1] - s["body"][1]))
            s["yaw"] = _wrap(s["yaw"] + _alpha(dt, HEADING_TAU_S) * _wrap(yaw - s["yaw"]))
        return s

    def _placement(self, bx, by, yaw, side, current_az):
        return chase_placement((bx, by), yaw, self.center, self.radius, self.inside,
                               front=self.front and not self.inside, current_az=current_az,
                               side=side)

    def _side_clear(self, bx, by, yaw, side):
        a, dist, _, _ = self._placement(bx, by, yaw, side, None)
        return lens_clear((bx + dist * math.cos(a), by + dist * math.sin(a)), (bx, by), self.keepouts)

    def _chase(self, body, yaw, dt):
        new = self._s is None
        s = self._follow(body, yaw, dt)
        bx, by = s["body"]
        if self.keepouts and not self.inside:
            # Stay on the current side unless it is blocked and the other is clear.
            if new:
                self.side = 1
            if not self._side_clear(bx, by, s["yaw"], self.side) and \
                    self._side_clear(bx, by, s["yaw"], -self.side):
                self.side = -self.side
        elif new:
            self.side = 1
        a_des, dist, eye_z, arc = self._placement(bx, by, s["yaw"], self.side, s.get("az"))
        lead_des = CHASE_FRONT_LEAD if (self.front and not self.inside) else CHASE_LOOK_AHEAD
        if "az" not in s:
            s["az"], s["dist"], s["lead"] = a_des, dist, lead_des
        else:
            s["az"] = orbit_toward(s["az"], a_des, _alpha(dt, CHASE_TAU_S), arc,
                                   max_step=CHASE_ORBIT_MAX_RAD_S * dt)
            s["dist"] += _alpha(dt, CHASE_TAU_S) * (dist - s["dist"])
            s["lead"] += _alpha(dt, CHASE_TAU_S) * (lead_des - s["lead"])
        ex, ey = bx + s["dist"] * math.cos(s["az"]), by + s["dist"] * math.sin(s["az"])
        # Numerical guard: never leave the lens on the wrong side of the margin ring.
        rx, ry = ex - self.center[0], ey - self.center[1]
        rr = math.hypot(rx, ry)
        if not self.inside and rr < self.radius + CHASE_WALL_MARGIN and rr > 1.0e-6:
            k = (self.radius + CHASE_WALL_MARGIN) / rr
            ex, ey = self.center[0] + rx * k, self.center[1] + ry * k
        elif self.inside and rr > self.radius - CHASE_WALL_MARGIN:
            k = (self.radius - CHASE_WALL_MARGIN) / rr
            ex, ey = self.center[0] + rx * k, self.center[1] + ry * k
        fx, fy = math.cos(s["yaw"]), math.sin(s["yaw"])
        target = (bx + s["lead"] * fx, by + s["lead"] * fy, CHASE_TARGET_Z)
        return Shot((ex, ey, eye_z), target, CHASE_FOCAL_MM)

    def _aim(self, desired, dt, tau=TARGET_TAU_S):
        s = self._s
        if s is None:
            s = self._s = {}
        if "target" not in s:
            s["target"] = tuple(desired)
        else:
            s["target"] = smooth_toward(s["target"], desired, dt, tau)
        return s["target"]

    def _interior(self, body, yaw, dt):
        eye = interior_eye(self.center, self.radius, self.door_yaw)
        target = self._aim((body[0] + INTERIOR_LEAD_M * math.cos(yaw),
                            body[1] + INTERIOR_LEAD_M * math.sin(yaw), CHASE_TARGET_Z), dt)
        focal_des = interior_focal_mm(eye, target)
        s = self._s
        s["focal"] = focal_des if "focal" not in s else s["focal"] + _alpha(dt, ZOOM_TAU_S) * (focal_des - s["focal"])
        return Shot(eye, target, s["focal"])

    def _overhead(self, body, yaw, dt):
        nx, ny = math.cos(self.door_yaw), math.sin(self.door_yaw)
        qx, qy = body[0] - self.center[0], body[1] - self.center[1]
        along = max(-OVERHEAD_TRAVEL_M[0], min(OVERHEAD_TRAVEL_M[0], qx * nx + qy * ny))
        across = max(-OVERHEAD_TRAVEL_M[1], min(OVERHEAD_TRAVEL_M[1], 0.4 * (-qx * ny + qy * nx)))
        desired = (self.center[0] + along * nx - across * ny,
                   self.center[1] + along * ny + across * nx, OVERHEAD_EYE_Z)
        s = self._s
        if s is None:
            s = self._s = {}
        s["eye"] = desired if "eye" not in s else smooth_toward(s["eye"], desired, dt, CHASE_TAU_S)
        # Zero X/Y rotation looks straight down with image right = +X; turn
        # it so image right is the door side (door_normal_yaw).
        return Shot(s["eye"], None, OVERHEAD_FOCAL_MM, rot=(0.0, 0.0, math.degrees(self.door_yaw)))

    def _door(self, body, yaw, dt):
        eye = door_eye(self.center, self.radius, self.door_yaw)
        nx, ny = math.cos(self.door_yaw), math.sin(self.door_yaw)
        door_aim = (self.door_xy[0] - 0.3 * nx, self.door_xy[1] - 0.3 * ny, 0.65)
        d = math.hypot(body[0] - self.door_xy[0], body[1] - self.door_xy[1])
        near, far = DOOR_TRACK_M
        w = min(1.0, max(0.0, (far - d) / (far - near)))
        desired = tuple(a + w * (b - a) for a, b in zip(door_aim, (body[0], body[1], CHASE_TARGET_Z)))
        return Shot(eye, self._aim(desired, dt), DOOR_FOCAL_MM)


def look_at_rotation_xyz(eye, target, fallback_yaw=0.0):
    """XYZ Euler (deg) that points a USD camera at ``eye`` toward ``target``."""
    dx, dy, dz = (float(t) - float(e) for e, t in zip(eye, target))
    horiz = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if horiz > 1.0e-3 else float(fallback_yaw)
    pitch_down = math.degrees(math.atan2(-dz, max(horiz, 1.0e-3)))
    return camera_rotation_xyz(math.degrees(yaw), pitch_down)


def smooth_toward(current, desired, dt, tau=CHASE_TAU_S):
    """First-order low-pass of a 3-vector; dt-exact so render rate changes
    (10 Hz headless vs 20 Hz GUI) do not change the feel."""
    if current is None or dt <= 0.0:
        return tuple(desired) if current is None else tuple(current)
    alpha = 1.0 - math.exp(-dt / max(tau, 1.0e-3))
    return tuple(c + alpha * (d - c) for c, d in zip(current, desired))


# ---------------------------------------------------------------------------
# Isaac-side construction (lazy imports; container only)
# ---------------------------------------------------------------------------


def _xform(prim, translate, rotate_xyz_deg):
    from pxr import Gf, UsdGeom

    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(*translate))
    api.SetRotate(Gf.Vec3f(*rotate_xyz_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    return api


def _define_camera(stage, path, width, height):
    from pxr import Gf, UsdGeom

    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(CAMERA_FOCAL_MM)
    cam.CreateHorizontalApertureAttr(CAMERA_H_APERTURE_MM)
    # Square pixels: vertical aperture follows the image aspect ratio, which
    # keeps fx == fy in the published camera_info.
    cam.CreateVerticalApertureAttr(CAMERA_H_APERTURE_MM * float(height) / float(width))
    cam.CreateClippingRangeAttr(Gf.Vec2f(*CAMERA_CLIP))
    return cam


def _ros_camera_graph(graph_path, camera_path, width, height, namespace,
                      topic_base, frame_id, skip):
    """One push/ON_DEMAND graph per camera (camera_periodic.py pattern).

    Per-camera graphs isolate failures: a bad camera leaves the others and the
    facade running. useSystemTime=True stamps images with the ROS wall clock,
    matching odometry, so the HUD/agent can pair frames with poses.
    """
    import omni.graph.core as og
    import usdrt.Sdf

    keys = og.Controller.Keys
    (graph, nodes, _, _) = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("renderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("renderProduct.outputs:execOut", "rgb.inputs:execIn"),
                ("renderProduct.outputs:execOut", "info.inputs:execIn"),
                ("renderProduct.outputs:renderProductPath", "rgb.inputs:renderProductPath"),
                ("renderProduct.outputs:renderProductPath", "info.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("renderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("renderProduct.inputs:width", int(width)),
                ("renderProduct.inputs:height", int(height)),
                ("rgb.inputs:type", "rgb"),
                ("rgb.inputs:nodeNamespace", namespace),
                ("rgb.inputs:topicName", topic_base + "/image"),
                ("rgb.inputs:frameId", frame_id),
                ("rgb.inputs:frameSkipCount", int(skip)),
                ("rgb.inputs:useSystemTime", True),
                ("info.inputs:nodeNamespace", namespace),
                ("info.inputs:topicName", topic_base + "/camera_info"),
                ("info.inputs:frameId", frame_id),
                ("info.inputs:frameSkipCount", int(skip)),
                ("info.inputs:useSystemTime", True),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    return graph


def create_spot_cameras(stage, robot_prim_path, names, rate_hz=10.0, render_hz=None,
                        width=640, height=480, namespace="/demo_spot",
                        frame_prefix="demo_spot/", log=print):
    """Attach and publish Spot body cameras.

    Args:
        stage: the ``pxr.Usd.Stage`` (``omni.usd.get_context().get_stage()``).
        robot_prim_path: Spot root prim, e.g. ``/World/Spot``; cameras are
            parented to ``<robot_prim_path>/body`` so they ride with the torso.
        names: iterable of CAMERA_NAMES entries (see ``parse_camera_list``).
        rate_hz: target publish rate per camera.
        render_hz: stage render rate (isaac_app ``--render-hz``); the helper
            publishes once per rendered frame, so this sets frameSkipCount.
            Defaults to rate_hz (skip nothing).
        width, height: image size (640x480 keeps two cameras inside the RTF
            budget on the L4 host).
        namespace / frame_prefix: ROS namespace and TF prefix (spot_ros2 style
            ``<prefix><name>_fisheye`` frame ids).
        log: callable for one-line status messages (stdout by default so host
            scripts can grep them).

    Returns:
        dict name -> camera prim path for every camera that came up. A camera
        whose graph fails is logged and skipped; it never raises, because the
        facade must keep running without video.
    """
    body_path = robot_prim_path.rstrip("/") + "/body"
    if not stage.GetPrimAtPath(body_path).IsValid():
        log(f"CAMERAS FAILED: body link {body_path} not found; no cameras created")
        return {}
    skip = frame_skip_count(render_hz if render_hz else rate_hz, rate_hz)
    created = {}
    for name in names:
        try:
            (tx, ty, tz), yaw_deg, pitch_deg = CAMERA_EXTRINSICS[name]
            cam_path = f"{body_path}/cameras/{name}"
            cam = _define_camera(stage, cam_path, width, height)
            _xform(cam.GetPrim(), (tx, ty, tz), camera_rotation_xyz(yaw_deg, pitch_deg))
            _ros_camera_graph(
                graph_path=f"/SpotCameraGraph_{name}",
                camera_path=cam_path, width=width, height=height,
                namespace=namespace, topic_base=f"camera/{name}",
                frame_id=f"{frame_prefix}{name}_fisheye", skip=skip)
            created[name] = cam_path
        except Exception as error:  # noqa: BLE001 - one bad camera must not kill the rest
            log(f"CAMERA {name} FAILED (continuing without it): {error}")
    log(f"CAMERAS enabled={','.join(created) or 'none'} {width}x{height} "
        f"rate={rate_hz:g}Hz render={render_hz or rate_hz:g}Hz frameSkipCount={skip} "
        f"topics={namespace}/camera/<name>/image,camera_info")
    return created

def _nose_light(stage, path, radius, intensity, cone_deg, softness, pitch_down_deg, offset,
                color=(1.0, 0.95, 0.88)):
    from pxr import Gf, UsdLux

    light = UsdLux.SphereLight.Define(stage, path)
    light.CreateRadiusAttr(float(radius))
    light.CreateIntensityAttr(float(intensity))
    light.CreateColorAttr(Gf.Vec3f(*color))  # warm white LED
    shaping = UsdLux.ShapingAPI.Apply(light.GetPrim())
    shaping.CreateShapingConeAngleAttr(float(cone_deg))
    shaping.CreateShapingConeSoftnessAttr(float(softness))
    _xform(light.GetPrim(), offset, light_rotation_xyz(pitch_down_deg))
    return path


def add_nose_spotlight(stage, robot_prim_path, intensity=SPOTLIGHT_INTENSITY):
    """Spot's payload lights, parented to the body link: a wide flood
    (``nose_light``) plus a narrow long-throw beam (``nose_beam``) that puts
    a visible hot spot on the tank wall / ladder ahead. Returns the flood's
    path (both are best effort; a failed beam leaves the flood)."""
    body = robot_prim_path.rstrip("/") + "/body"
    path = _nose_light(stage, body + "/nose_light", SPOTLIGHT_RADIUS_M, intensity,
                       SPOTLIGHT_CONE_DEG, SPOTLIGHT_SOFTNESS, SPOTLIGHT_PITCH_DOWN_DEG,
                       SPOTLIGHT_OFFSET)
    try:
        _nose_light(stage, body + "/nose_beam", BEAM_RADIUS_M, BEAM_INTENSITY, BEAM_CONE_DEG,
                    BEAM_SOFTNESS, BEAM_PITCH_DOWN_DEG, BEAM_OFFSET)
    except Exception as error:  # noqa: BLE001 - cosmetic
        print(f"HEADLIGHT beam failed (flood only): {error}", flush=True)
    return path


def payload_light_scale(xy, tank_center_xy=(0.0, 0.0), tank_radius=6.0,
                        full_m=LIGHTS_FULL_OUTSIDE_M, off_m=LIGHTS_OFF_OUTSIDE_M):
    """0..1 factor on the payload lights' intensity with Spot's body at
    ``xy``: 1 inside the tank and up to ``full_m`` outside the shell, 0 from
    ``off_m`` outside, smoothstep between (no visible pop)."""
    outside = math.hypot(float(xy[0]) - tank_center_xy[0],
                         float(xy[1]) - tank_center_xy[1]) - float(tank_radius)
    if outside <= full_m:
        return 1.0
    if outside >= off_m:
        return 0.0
    u = (outside - full_m) / (off_m - full_m)
    return 1.0 - u * u * (3.0 - 2.0 * u)


class PayloadLights:
    """Fades Spot's payload lights (``add_nose_spotlight``) with its
    distance from the tank shell (``payload_light_scale``). ``update(xy)``
    runs on render ticks, writes the intensity only when the factor moved
    (>= 0.02, or reached 0 / 1) and never raises: after an error the lights
    stay as they were."""

    STEP = 0.02

    def __init__(self, stage, robot_prim_path, tank_center_xy=(0.0, 0.0), tank_radius=6.0):
        from pxr import UsdLux

        body = robot_prim_path.rstrip("/") + "/body"
        self.tank_center_xy = (float(tank_center_xy[0]), float(tank_center_xy[1]))
        self.tank_radius = float(tank_radius)
        self._lights = []
        for name, base in (("nose_light", SPOTLIGHT_INTENSITY), ("nose_beam", BEAM_INTENSITY)):
            prim = stage.GetPrimAtPath(body + "/" + name)
            if prim and prim.IsValid():
                self._lights.append((UsdLux.SphereLight(prim).GetIntensityAttr(), base))
        self.scale = None
        self.enabled = bool(self._lights)

    def update(self, xy):
        if not self.enabled:
            return None
        try:
            scale = payload_light_scale(xy, self.tank_center_xy, self.tank_radius)
            last = self.scale
            if last is not None and (scale == last or (abs(scale - last) < self.STEP
                                                         and scale not in (0.0, 1.0))):
                return last
            for attr, base in self._lights:
                attr.Set(float(base * scale))
            self.scale = scale
        except Exception as error:  # noqa: BLE001 - cosmetic, never stop the sim
            self.enabled = False
            print(f"HEADLIGHT fade disabled after error: {error}", flush=True)
        return self.scale


class ChaseCamera:
    """The GUI viewport camera, driven by a CameraDirector (named views).

    ``update`` is called on render ticks only (that is when the viewport
    samples it) and never raises: a failed update disables it and leaves the
    camera where it was.
    """

    PATH = "/World/ChaseCamera"

    def __init__(self, stage, tank_center_xy=(0.0, 0.0), tank_radius=6.0, door_normal_yaw=0.0,
                 view=DEFAULT_VIEW, keepouts=None):
        from pxr import Gf, UsdGeom

        cam = UsdGeom.Camera.Define(stage, self.PATH)
        cam.CreateFocalLengthAttr(CHASE_FOCAL_MM)
        cam.CreateHorizontalApertureAttr(CAMERA_H_APERTURE_MM)
        # 16:9 like the pinned viewport resolution, so framing is what the
        # director computed (horizontal FOV from the horizontal aperture).
        cam.CreateVerticalApertureAttr(CAMERA_H_APERTURE_MM * 9.0 / 16.0)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500.0))
        self._focal_attr = cam.GetFocalLengthAttr()
        self._api = UsdGeom.XformCommonAPI(cam.GetPrim())
        self._Gf = Gf
        self._rot_order = UsdGeom.XformCommonAPI.RotationOrderXYZ
        self.director = CameraDirector(tank_center_xy, tank_radius, door_normal_yaw, view,
                                       keepouts=keepouts)
        self._last_t = None
        self._focal = None
        self.failed = False

    @property
    def view(self):
        return self.director.view

    def set_view(self, name):
        """Named view (VIEW_NAMES); hard cut on the next update. Raises
        ValueError for an unknown name."""
        return self.director.set_view(name)

    def update(self, body_xyz, yaw, sim_time):
        if self.failed or body_xyz is None:
            return
        try:
            dt = 0.0 if self._last_t is None else sim_time - self._last_t
            self._last_t = sim_time
            shot = self.director.step(body_xyz, yaw, dt)
            rot = shot.rot if shot.rot is not None else look_at_rotation_xyz(
                shot.eye, shot.target, fallback_yaw=yaw)
            self._api.SetTranslate(self._Gf.Vec3d(*shot.eye))
            self._api.SetRotate(self._Gf.Vec3f(*rot), self._rot_order)
            if self._focal is None or abs(shot.focal_mm - self._focal) > 0.02:
                self._focal_attr.Set(float(shot.focal_mm))
                self._focal = shot.focal_mm
        except Exception as error:  # noqa: BLE001 - cosmetic; never break the loop
            self.failed = True
            print(f"CHASE-CAMERA disabled after error: {error}", flush=True)
