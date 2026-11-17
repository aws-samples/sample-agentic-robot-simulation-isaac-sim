"""Procedural confined-space scene for the Spot oil-and-gas demo (Isaac Sim 6.1.0).

What this is
------------
Builds a crude-oil-tank confined-space entry scene out of primitive USD geometry
(boxes + cylinders, no booleans) so every wall segment gets a real static
collider. Authored entirely under ``/World/ConfinedSpace``.

Scene contents (meters, Z-up, stage units = 1 m):
  * Flat concrete ground slab with physics (optional, ``add_ground=True``).
  * Tank: thin-walled cylindrical shell, radius 6 m, height 4 m, approximated
    by 24 box segments on a circle; one segment is omitted at +X to leave the
    API-650-style flush cleanout doorway (1.3 m clear width x 1.3 m clear
    height, at floor level), with a box door frame (jambs + lintel) and a
    wall-fill panel above the opening so the shell is continuous above 1.3 m.
  * Tank floor: thin dark disc, top surface raised 0.02 m.
  * Interior: 3 sludge patches (flattened boxes), an internal ladder mock
    (boxes) against the far wall, and 2 pipe runs (cylinders, centerline
    0.4 m) that partially block the door-to-ladder line so Spot must route
    around them (each run leaves a >1.3 m gap on one side).
  * Exterior dressing: 2 barrels (cylinders) and a simple pipe rack
    (boxes + cylinders).
  * Roof (visual only, no collider): a fixed cone roof on a deck at the
    shell top (z 4 m), like a crude storage tank. It makes the interior a
    real dark confined space: v1 was open-topped, so the sun lit half of
    the interior and shadowed the far end to near-black (docs/evidence/07,
    09). Nothing the mission uses moves: shell, door, floor, pipes, ladder
    and waypoints are unchanged, and nothing reaches the roof.
  * Lighting: exterior = dim dome + sun (distant) + a weak sphere light at
    the doorway. Interior (the tank reads dark, but legible on a
    projector): daylight spilling in through the cleanout door (rect
    light), daylight from a roof manway above the ladder (rect light), and a
    very dim cool fill under the roof (4 sphere lights). Spot's own
    headlights (sim/sensors.py add_nose_spotlight) do the rest.

Doorway geometry check (why Spot fits): Spot standing is ~1.10 x 0.50 m
footprint, 0.61-0.70 m tall. Clear opening is 1.30 m wide (jamb inner faces
at y = +/-0.65) and 1.30 m tall (lintel underside at z = 1.30), giving
>= 0.40 m lateral clearance per side and >= 0.60 m headroom - comfortably
more than the required 0.10 m per side. The gap bottom is at ground level
(the 0.02 m tank-floor lip is a trivial step).

How it runs
-----------
Runs ONLY inside the Isaac Sim 6.1.0 container via ``/isaac-sim/python.sh``.
It is imported by ``sim/isaac_app.py`` AFTER ``SimulationApp`` has been
constructed; this module never instantiates ``SimulationApp`` itself.

    from build_scene import build_scene
    info = build_scene(None)          # uses the current omni.usd stage
    # info["spawn_xy"], info["entry_xy"], info["interior_waypoints"]

``build_scene`` accepts a ``pxr.Usd.Stage``, any object exposing a ``.stage``
attribute, or ``None`` (falls back to ``omni.usd.get_context().get_stage()``).
``save_scene(path)`` is an optional helper that exports the flattened stage
to a ``.usd``/``.usda`` file for offline inspection.

Only APIs evidenced in docs/isaac-api-notes.md are used verbatim; standard
USD calls that were not literally exercised in the recon are tagged
``# VERIFY-ON-HOST``.
"""

import math

# pxr is only importable inside the Isaac container (after SimulationApp init
# in the importing process). Kept at module top per module contract; this file
# is never imported on the Mac.
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ---------------------------------------------------------------------------
# Scene constants
# ---------------------------------------------------------------------------

ROOT = "/World/ConfinedSpace"

TANK_RADIUS = 6.0          # shell centerline radius (m)
TANK_HEIGHT = 4.0          # shell height (m)
WALL_THICKNESS = 0.10      # radial thickness of each shell segment (m)
NUM_SEGMENTS = 24          # box segments around the circle
DOOR_SEGMENT = 0           # omitted segment index -> doorway faces +X
DOOR_WIDTH = 1.30          # clear width between jamb inner faces (m)
DOOR_HEIGHT = 1.30         # clear height under the lintel (m)

FLOOR_LIP = 0.02           # tank floor top surface height (m)

# Colors (simple preview-surface diffuse; do NOT depend on vMaterials paths).
RUSTED_STEEL = (0.30, 0.13, 0.08)
DARK_STEEL = (0.15, 0.10, 0.08)
CONCRETE_GRAY = (0.50, 0.50, 0.50)
FLOOR_DARK = (0.10, 0.09, 0.085)   # lifted from 0.05 so the headlight pool reads
SLUDGE_BROWN = (0.09, 0.06, 0.03)
PIPE_STEEL = (0.35, 0.33, 0.30)
BARREL_BLUE = (0.10, 0.20, 0.45)


# ---------------------------------------------------------------------------
# Stage / prim helpers
# ---------------------------------------------------------------------------


def _resolve_stage(world_or_stage):
    """Accept a Usd.Stage, an object with a .stage attribute, or None."""
    if world_or_stage is None:
        import omni.usd  # deferred: only valid after SimulationApp init

        return omni.usd.get_context().get_stage()
    if isinstance(world_or_stage, Usd.Stage):
        return world_or_stage
    stage = getattr(world_or_stage, "stage", None)
    if stage is not None:
        return stage
    raise TypeError(
        "build_scene expects a Usd.Stage, an object with a .stage attribute, "
        "or None; got %r" % (world_or_stage,)
    )


def _make_material(stage, path, rgb, roughness=0.7, metallic=0.0):
    """Simple UsdPreviewSurface material (renders in RTX; no vMaterials)."""
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _finish_gprim(gprim, material, rgb, collision=True):
    """Bind material, author displayColor fallback, enable static collision."""
    prim = gprim.GetPrim()
    gprim.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    if material is not None:
        # VERIFY-ON-HOST: UsdShade.MaterialBindingAPI.Apply/Bind is standard
        # USD but was not literally exercised in the recon notes.
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    if collision:
        # Static collider: CollisionAPI without RigidBodyAPI, per the USD
        # Physics schema (task-sanctioned pattern).
        # VERIFY-ON-HOST: UsdPhysics.CollisionAPI.Apply not verbatim in the
        # recon notes; confirm colliders register in the 6.1 PhysX pipeline.
        UsdPhysics.CollisionAPI.Apply(prim)


def _xform(prim, translate, rotate_xyz_deg=(0.0, 0.0, 0.0), scale=None):
    """Author translate/rotate/scale via XformCommonAPI (recon-notes pattern)."""
    api = UsdGeom.XformCommonAPI(prim)
    api.SetTranslate(Gf.Vec3d(*translate))
    api.SetRotate(Gf.Vec3f(*rotate_xyz_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    if scale is not None:
        # VERIFY-ON-HOST: XformCommonAPI.SetScale (notes only show
        # SetTranslate/SetRotate; SetScale is the same API family).
        api.SetScale(Gf.Vec3f(*scale))


def _box(stage, path, size, translate, rot_z_deg=0.0, material=None,
         rgb=(0.5, 0.5, 0.5), collision=True):
    """Axis-aligned unit cube scaled to ``size`` (sx, sy, sz), then placed."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    _xform(cube.GetPrim(), translate, (0.0, 0.0, rot_z_deg), size)
    _finish_gprim(cube, material, rgb, collision)
    return cube


def _cylinder(stage, path, radius, height, translate, axis="Z", material=None,
              rgb=(0.5, 0.5, 0.5), collision=True):
    """UsdGeom.Cylinder with explicit axis, extent, placement, collision."""
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    cyl.CreateAxisAttr(axis)
    h2 = height / 2.0
    if axis == "Z":
        extent = [Gf.Vec3f(-radius, -radius, -h2), Gf.Vec3f(radius, radius, h2)]
    elif axis == "Y":
        extent = [Gf.Vec3f(-radius, -h2, -radius), Gf.Vec3f(radius, h2, radius)]
    else:  # "X"
        extent = [Gf.Vec3f(-h2, -radius, -radius), Gf.Vec3f(h2, radius, radius)]
    cyl.CreateExtentAttr(extent)
    _xform(cyl.GetPrim(), translate)
    # VERIFY-ON-HOST: analytic cylinder collision approximation under PhysX
    # (default convex approximation is fine for a demo; confirm no warnings).
    _finish_gprim(cyl, material, rgb, collision)
    return cyl


# ---------------------------------------------------------------------------
# Scene sections
# ---------------------------------------------------------------------------


def _build_ground(stage, mats):
    _box(
        stage, ROOT + "/Ground/slab",
        size=(40.0, 40.0, 0.25),
        translate=(0.0, 0.0, -0.125),  # top surface exactly at z = 0
        material=mats["concrete"], rgb=CONCRETE_GRAY,
    )


def _build_tank_shell(stage, mats):
    """24 box segments on the circle; DOOR_SEGMENT omitted for the doorway."""
    seg_angle = 2.0 * math.pi / NUM_SEGMENTS
    # tan-based length at the OUTER radius: neighbours overlap through the
    # full wall thickness (mid-line tan-length only touches at the mid-plane
    # and leaves small wedge gaps on the outer skin) -> no cracks in the wall.
    seg_len = 2.0 * (TANK_RADIUS + WALL_THICKNESS / 2.0) * math.tan(seg_angle / 2.0)
    for k in range(NUM_SEGMENTS):
        if k == DOOR_SEGMENT:
            continue  # the doorway gap
        theta = k * seg_angle
        _box(
            stage, ROOT + "/Tank/Shell/segment_%02d" % k,
            # local x = radial thickness, local y = tangential length
            size=(WALL_THICKNESS, seg_len, TANK_HEIGHT),
            translate=(
                TANK_RADIUS * math.cos(theta),
                TANK_RADIUS * math.sin(theta),
                TANK_HEIGHT / 2.0,
            ),
            rot_z_deg=math.degrees(theta),
            material=mats["rust"], rgb=RUSTED_STEEL,
        )


def _build_doorway(stage, mats):
    """Frame around the gap + wall fill above the 1.3 m opening.

    Raw gap left by the missing segment is ~1.57 m wide; the jambs narrow it
    to exactly DOOR_WIDTH = 1.30 m and overlap the neighbouring segments so
    there is no crack.
    """
    x = TANK_RADIUS
    jamb_w = 0.22           # tangential; spans y 0.65..0.87 (overlaps shell)
    jamb_y = DOOR_WIDTH / 2.0 + jamb_w / 2.0   # +/-0.76
    jamb_h = DOOR_HEIGHT + 0.20
    for name, sign in (("jamb_left", 1.0), ("jamb_right", -1.0)):
        _box(
            stage, ROOT + "/Tank/DoorFrame/" + name,
            size=(0.25, jamb_w, jamb_h),
            translate=(x, sign * jamb_y, jamb_h / 2.0),
            material=mats["dark_steel"], rgb=DARK_STEEL,
        )
    # Lintel (frame header): underside defines the 1.30 m clear height.
    _box(
        stage, ROOT + "/Tank/DoorFrame/lintel",
        size=(0.25, DOOR_WIDTH + 2.0 * jamb_w, 0.24),
        translate=(x, 0.0, DOOR_HEIGHT + 0.12),
        material=mats["dark_steel"], rgb=DARK_STEEL,
    )
    # Wall fill above the opening keeps the shell continuous up to 4 m.
    fill_h = TANK_HEIGHT - DOOR_HEIGHT
    _box(
        stage, ROOT + "/Tank/DoorFrame/wall_above_door",
        size=(WALL_THICKNESS, 1.70, fill_h),
        translate=(x, 0.0, DOOR_HEIGHT + fill_h / 2.0),
        material=mats["rust"], rgb=RUSTED_STEEL,
    )


def _build_tank_floor(stage, mats):
    _cylinder(
        stage, ROOT + "/Tank/floor",
        radius=TANK_RADIUS - 0.02, height=2.0 * FLOOR_LIP,
        translate=(0.0, 0.0, 0.0),  # spans -0.02..+0.02 -> top raised 0.02
        material=mats["floor"], rgb=FLOOR_DARK,
    )


def _build_interior(stage, mats):
    # --- sludge patches (flattened boxes on the tank floor) ---
    patches = [
        ((2.8, -2.2), (1.8, 1.4, 0.06)),
        ((-1.0, 3.0), (2.2, 1.6, 0.05)),
        ((-3.5, -2.5), (1.4, 1.1, 0.05)),
    ]
    for i, ((px, py), (sx, sy, sz)) in enumerate(patches):
        _box(
            stage, ROOT + "/Interior/sludge_%d" % i,
            size=(sx, sy, sz),
            translate=(px, py, FLOOR_LIP + sz / 2.0),
            material=mats["sludge"], rgb=SLUDGE_BROWN, )

    # --- internal ladder mock against the far wall (opposite the doorway) ---
    lx = -(TANK_RADIUS - WALL_THICKNESS / 2.0 - 0.20)  # ~0.2 m standoff
    rail_h = 3.6
    for name, ry in (("rail_left", 0.25), ("rail_right", -0.25)):
        _box(
            stage, ROOT + "/Interior/Ladder/" + name,
            size=(0.06, 0.06, rail_h),
            translate=(lx, ry, FLOOR_LIP + rail_h / 2.0),
            material=mats["dark_steel"], rgb=DARK_STEEL,
        )
    z = 0.40
    rung = 0
    while z <= 3.5:
        _box(
            stage, ROOT + "/Interior/Ladder/rung_%02d" % rung,
            size=(0.05, 0.50, 0.05),
            translate=(lx, 0.0, z),
            material=mats["dark_steel"], rgb=DARK_STEEL,
        )
        z += 0.35
        rung += 1

    # --- two pipe runs at 0.4 m centerline that Spot must route around ---
    # Run A blocks the NORTH half at x = -2 (gap south of y = -0.7).
    # Run B blocks the SOUTH half at x = +2.5 (gap north of y = +0.6).
    # A straight doorway->ladder line along y=0 is therefore blocked twice;
    # each gap is > 1.3 m wide so Spot can weave through.
    pipe_z = 0.40
    runs = [
        ("pipe_run_a", -2.0, -0.7, 5.5),
        ("pipe_run_b", 2.5, -5.3, 0.6),
    ]
    for name, px, y0, y1 in runs:
        length = y1 - y0
        cy = (y0 + y1) / 2.0
        _cylinder(
            stage, ROOT + "/Interior/" + name,
            radius=0.12, height=length,
            translate=(px, cy, pipe_z), axis="Y",
            material=mats["pipe"], rgb=PIPE_STEEL,
        )
        # simple support feet so the run doesn't float visually
        for j, sy in enumerate((y0 + 0.4, y1 - 0.4)):
            _box(
                stage, ROOT + "/Interior/%s_support_%d" % (name, j),
                size=(0.15, 0.15, pipe_z - 0.12),
                translate=(px, sy, FLOOR_LIP + (pipe_z - 0.12) / 2.0),
                material=mats["dark_steel"], rgb=DARK_STEEL,
            )


# Exterior dressing footprints (used for the geometry below and reported to
# the GUI presenter camera as keep-outs, so a lens never parks inside them).
BARRELS_XY = ((9.5, 3.0), (10.3, 3.6))
BARREL_RADIUS = 0.30
PIPE_RACK_BOX = ((8.75, -5.2), (10.25, -1.8))   # posts, beams and the 3.4 m pipes (xy)


def _build_exterior(stage, mats):
    # --- two barrels near the tank, off the spawn->door line ---
    for i, (bx, by) in enumerate(BARRELS_XY):
        _cylinder(
            stage, ROOT + "/Exterior/barrel_%d" % i,
            radius=BARREL_RADIUS, height=0.90,
            translate=(bx, by, 0.45), axis="Z",
            material=mats["barrel"], rgb=BARREL_BLUE,
        )

    # --- simple pipe rack: 4 posts + 2 cross beams + 3 pipes ---
    rack = ROOT + "/Exterior/PipeRack"
    post_h = 1.2
    for i, (px, py) in enumerate(((8.9, -4.8), (10.1, -4.8), (8.9, -2.2), (10.1, -2.2))):
        _box(
            stage, rack + "/post_%d" % i,
            size=(0.12, 0.12, post_h),
            translate=(px, py, post_h / 2.0),
            material=mats["dark_steel"], rgb=DARK_STEEL,
        )
    for i, by in enumerate((-4.8, -2.2)):
        _box(
            stage, rack + "/beam_%d" % i,
            size=(1.5, 0.12, 0.12),
            translate=(9.5, by, post_h - 0.06),
            material=mats["dark_steel"], rgb=DARK_STEEL,
        )
    for i, px in enumerate((9.1, 9.5, 9.9)):
        _cylinder(
            stage, rack + "/pipe_%d" % i,
            radius=0.09, height=3.4,
            translate=(px, -3.5, post_h + 0.09), axis="Y",
            material=mats["pipe"], rgb=PIPE_STEEL,
        )


def _build_roof(stage, mats):
    """Fixed cone roof on a flat deck at the shell top. Visual only (no
    collider: nothing ever reaches it, and it keeps PhysX unchanged)."""
    deck_r = TANK_RADIUS + WALL_THICKNESS / 2.0 + 0.08   # covers the 24-gon's corners (6.10 m)
    _cylinder(
        stage, ROOT + "/Tank/Roof/deck",
        radius=deck_r, height=0.08,
        translate=(0.0, 0.0, TANK_HEIGHT + 0.04),       # z 4.00 .. 4.08
        material=mats["rust"], rgb=RUSTED_STEEL, collision=False,
    )
    cone_h = 0.75
    cone = UsdGeom.Cone.Define(stage, ROOT + "/Tank/Roof/cone")
    cone.CreateRadiusAttr(deck_r + 0.05)
    cone.CreateHeightAttr(cone_h)
    cone.CreateAxisAttr("Z")
    cone.CreateExtentAttr([Gf.Vec3f(-deck_r - 0.05, -deck_r - 0.05, -cone_h / 2.0),
                           Gf.Vec3f(deck_r + 0.05, deck_r + 0.05, cone_h / 2.0)])
    _xform(cone.GetPrim(), (0.0, 0.0, TANK_HEIGHT + 0.08 + cone_h / 2.0))
    _finish_gprim(cone, mats["rust"], RUSTED_STEEL, collision=False)


# Interior light levels, set by eye on the host (RTX Real-Time 2.0, default
# exposure, the GUI presenter camera and the HUD body cameras). The tank must
# read as a dark confined space lit by Spot's headlights, yet stay legible on
# a projector: walls, ladder, sludge and pipes visible in silhouette.
DOOR_SPILL_INTENSITY = 3500.0     # daylight through the cleanout door
MANWAY_INTENSITY = 14000.0        # daylight through a roof manway above the ladder
FILL_INTENSITY = 8000.0           # each of 4 dim fill lights under the roof
DAYLIGHT = (1.0, 0.97, 0.92)
# Entry work light (real tank entries light the entry from outside): the door
# face is on the shadow side of the tank, so the probe beat (Spot at the
# door for ~2 min of the demo) and the sit after a retreat read near-black on
# a projector (leadership rehearsal 2026-09-28, docs/evidence/v5). A shaped
# sphere light on a mast 2.6 m out and 1.6 m to the side, aimed at the door
# mouth; the emitter itself is hidden.
WORK_LIGHT_XYZ = (TANK_RADIUS + 2.6, -1.6, 2.4)
WORK_LIGHT_AIM_XYZ = (TANK_RADIUS + 0.4, 0.0, 0.5)
WORK_LIGHT_INTENSITY = 400000.0
WORK_LIGHT_RADIUS_M = 0.1
WORK_LIGHT_CONE_DEG = 45.0
WORK_LIGHT_COLOR = (1.0, 0.95, 0.85)


def work_light_rotation_xyz(src=WORK_LIGHT_XYZ, aim=WORK_LIGHT_AIM_XYZ):
    """XformCommonAPI XYZ Euler (deg) aiming a UsdLux light (emits along
    local -Z) from ``src`` at ``aim``: tilt about Y below the horizon, then
    yaw about Z (the sim/sensors.py light_rotation_xyz convention)."""
    dx, dy, dz = (aim[i] - src[i] for i in range(3))
    yaw = math.degrees(math.atan2(dy, dx))
    pitch_down = math.degrees(math.atan2(-dz, math.hypot(dx, dy)))
    return (0.0, -(90.0 - pitch_down), yaw)
SKYLIGHT = (0.85, 0.92, 1.0)
FILL_COLOR = (0.80, 0.86, 0.95)


def _hide_from_camera(light):
    """Light the scene but do not show the emitter itself to cameras (RTX
    attribute visibleInPrimaryRay, as authored by Kit's light property
    widget: extscache/omni.kit.property.light-1.0.16+00c488ae/omni/kit/
    property/light/scripts/prim_light_widget.py)."""
    light.GetPrim().CreateAttribute("visibleInPrimaryRay", Sdf.ValueTypeNames.Bool).Set(False)


def _build_lights(stage):
    """Exterior: dim dome + sun + weak doorway light. Interior (under the
    roof): door spill, roof-manway daylight on the ladder, dim fill.

    UsdLux authoring (Define / CreateIntensityAttr / ShapingAPI) was
    exercised on the host (Isaac Sim 6.1.0 RTX): the v1 dome/sun/doorway
    lights and Spot's nose light rendered as authored.
    """
    dome = UsdLux.DomeLight.Define(stage, ROOT + "/Lights/dome")
    dome.CreateIntensityAttr(80.0)  # dim ambient; the roof keeps it out of the tank

    distant = UsdLux.DistantLight.Define(stage, ROOT + "/Lights/sun")
    distant.CreateIntensityAttr(1500.0)
    distant.CreateAngleAttr(0.53)
    # Aim roughly from the +X / -Y sky toward the site (emits along local -Z).
    _xform(distant.GetPrim(), (0.0, 0.0, 10.0), (-40.0, 0.0, 55.0))

    door_light = UsdLux.SphereLight.Define(stage, ROOT + "/Lights/doorway")
    door_light.CreateRadiusAttr(0.08)
    door_light.CreateIntensityAttr(2000.0)  # weak; only hints at the opening
    _xform(door_light.GetPrim(), (TANK_RADIUS + 0.9, 0.0, 1.15))

    work = UsdLux.SphereLight.Define(stage, ROOT + "/Lights/entry_work_light")
    work.CreateRadiusAttr(WORK_LIGHT_RADIUS_M)
    work.CreateIntensityAttr(WORK_LIGHT_INTENSITY)
    work.CreateColorAttr(Gf.Vec3f(*WORK_LIGHT_COLOR))
    shaping = UsdLux.ShapingAPI.Apply(work.GetPrim())
    shaping.CreateShapingConeAngleAttr(WORK_LIGHT_CONE_DEG)
    shaping.CreateShapingConeSoftnessAttr(0.5)
    _xform(work.GetPrim(), WORK_LIGHT_XYZ, work_light_rotation_xyz())
    _hide_from_camera(work)

    # Daylight spilling in through the 1.3 x 1.3 m cleanout door: a rect
    # light just inside the opening, emitting into the tank. RectLight emits
    # along local -Z; RotateY(+90) turns that to -X (the door faces +X).
    spill = UsdLux.RectLight.Define(stage, ROOT + "/Lights/door_spill")
    spill.CreateWidthAttr(1.1)
    spill.CreateHeightAttr(1.1)
    spill.CreateIntensityAttr(DOOR_SPILL_INTENSITY)
    spill.CreateColorAttr(Gf.Vec3f(*DAYLIGHT))
    _xform(spill.GetPrim(), (TANK_RADIUS - 0.2, 0.0, 0.7), (0.0, 90.0, 0.0))
    _hide_from_camera(spill)

    # Roof manway above the internal ladder (real tanks have one there):
    # a shaft of skylight down the far wall, so the far end is not black.
    manway = UsdLux.RectLight.Define(stage, ROOT + "/Lights/roof_manway")
    manway.CreateWidthAttr(0.7)
    manway.CreateHeightAttr(0.7)
    manway.CreateIntensityAttr(MANWAY_INTENSITY)
    manway.CreateColorAttr(Gf.Vec3f(*SKYLIGHT))
    _xform(manway.GetPrim(), (-(TANK_RADIUS - 0.9), 0.0, TANK_HEIGHT - 0.1))
    _hide_from_camera(manway)

    # Very dim cool fill under the roof (shadowed by the shell and roof, so
    # none of it leaks outside).
    for i in range(4):
        theta = math.radians(45.0 + 90.0 * i)
        fill = UsdLux.SphereLight.Define(stage, ROOT + "/Lights/interior_fill_%d" % i)
        fill.CreateRadiusAttr(0.3)
        fill.CreateIntensityAttr(FILL_INTENSITY)
        fill.CreateColorAttr(Gf.Vec3f(*FILL_COLOR))
        _xform(fill.GetPrim(), (3.0 * math.cos(theta), 3.0 * math.sin(theta), TANK_HEIGHT - 0.6))
        _hide_from_camera(fill)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def build_scene(world_or_stage=None, add_ground=True):
    """Build the confined-space scene under /World/ConfinedSpace.

    Args:
        world_or_stage: a ``pxr.Usd.Stage``, an object with a ``.stage``
            attribute, or ``None`` (uses the current ``omni.usd`` stage).
        add_ground: set False if the caller already provides a ground plane
            (e.g. the Grid environment from spot_standalone.py) to avoid
            two coplanar surfaces z-fighting at z = 0.

    Returns:
        dict with (extra keys are informational):
            spawn_xy            (x, y) ~8 m outside the doorway, on its axis
            spawn_yaw           yaw (rad) that faces the doorway from spawn
            entry_xy            (x, y) 1 m outside the doorway center
            doorway_xy          (x, y) of the doorway center on the wall line
            door_normal_yaw     outward doorway normal (rad; 0 = +X)
            tank_center_xy      (x, y) tank axis; tank_radius shell radius
            interior_waypoints  [(x, y)] near-entry -> weave north of pipe
                                run B -> tank center -> weave south of pipe
                                run A -> far-ladder. Consecutive straight
                                segments are collision-free (>0.4 m lateral
                                clearance from every pipe end).
    """
    stage = _resolve_stage(world_or_stage)

    UsdGeom.Xform.Define(stage, ROOT)
    UsdGeom.Scope.Define(stage, ROOT + "/Looks")

    mats = {
        "rust": _make_material(stage, ROOT + "/Looks/rusted_steel", RUSTED_STEEL, 0.85, 0.25),
        "dark_steel": _make_material(stage, ROOT + "/Looks/dark_steel", DARK_STEEL, 0.6, 0.5),
        "concrete": _make_material(stage, ROOT + "/Looks/concrete", CONCRETE_GRAY, 0.9, 0.0),
        "floor": _make_material(stage, ROOT + "/Looks/tank_floor", FLOOR_DARK, 0.8, 0.2),
        # Wet, glossy sludge: the headlight picks it out as a specular sheen.
        "sludge": _make_material(stage, ROOT + "/Looks/sludge", SLUDGE_BROWN, 0.3, 0.0),
        "pipe": _make_material(stage, ROOT + "/Looks/pipe_steel", PIPE_STEEL, 0.5, 0.6),
        "barrel": _make_material(stage, ROOT + "/Looks/barrel_blue", BARREL_BLUE, 0.4, 0.3),
    }

    if add_ground:
        _build_ground(stage, mats)
    _build_tank_shell(stage, mats)
    _build_doorway(stage, mats)
    _build_tank_floor(stage, mats)
    _build_roof(stage, mats)
    _build_interior(stage, mats)
    _build_exterior(stage, mats)
    _build_lights(stage)

    # Doorway faces +X; its center sits on the wall line at (TANK_RADIUS, 0).
    return {
        "spawn_xy": (TANK_RADIUS + 8.0, 0.0),        # (14, 0), 8 m from door
        "spawn_yaw": math.pi,                         # face -X, toward door
        "entry_xy": (TANK_RADIUS + 1.0, 0.0),         # (7, 0), 1 m outside
        "doorway_xy": (TANK_RADIUS, 0.0),
        # Outward normal of the doorway (tank center -> door), i.e. the angle
        # of the omitted shell segment; consumers face door_normal_yaw + pi to
        # look into the tank. Published on /demo_spot/payload/site.
        "door_normal_yaw": DOOR_SEGMENT * 2.0 * math.pi / NUM_SEGMENTS,
        "tank_center_xy": (0.0, 0.0),
        "tank_radius": TANK_RADIUS,
        # NOTE: the two pipe runs both cross y = 0 (run B spans y -5.3..0.6 at
        # x = 2.5, run A spans y -0.7..5.5 at x = -2), so waypoints must weave:
        # a straight y = 0 doorway->ladder line would clip both pipes at
        # z 0.28..0.52, which Spot can neither step over nor pass under.
        # Informational, GUI only (sim/sensors.py CameraDirector): exterior
        # dressing the presenter camera must stay clear of and see past.
        "camera_keepouts": [{"xy": xy, "r": BARREL_RADIUS} for xy in BARRELS_XY]
                           + [{"min": PIPE_RACK_BOX[0], "max": PIPE_RACK_BOX[1]}],
        "interior_waypoints": [
            (4.6, 0.0),    # near-entry, just inside the doorway
            (2.5, 1.6),    # weave north of pipe run B's end (y = 0.6)
            (0.0, 0.0),    # tank center (between the two pipe-run gaps)
            (-2.0, -1.7),  # weave south of pipe run A's end (y = -0.7)
            (-4.8, 0.0),   # in front of the ladder on the far wall
        ],
    }


def save_scene(path, world_or_stage=None):
    """Optional helper: export the flattened stage to ``path`` (.usd/.usda).

    VERIFY-ON-HOST: Usd.Stage.Export is standard USD but was not exercised
    in the recon notes.
    """
    stage = _resolve_stage(world_or_stage)
    stage.Export(str(path))
