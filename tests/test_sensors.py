"""Unit tests for the pure-math half of sim.sensors — stdlib only, run on a Mac.

Run from the repo root:

    python3 -m unittest tests.test_sensors

The USD / OmniGraph half of sim/sensors.py imports pxr lazily, so importing
the module here never touches Isaac. These tests pin the conventions that are
easy to get silently wrong on the host (camera aim, light aim, frame skip,
chase-camera wall clamp) where a mistake only shows up as a black or sideways
video feed.
"""

import math
import os
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

from sim import sensors


def _close(a, b, tol=1e-6):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


class TestParseCameraList(unittest.TestCase):
    def test_keywords(self):
        self.assertEqual(sensors.parse_camera_list("all"), sensors.CAMERA_NAMES)
        self.assertEqual(sensors.parse_camera_list("none"), ())
        self.assertEqual(sensors.parse_camera_list(""), ())
        self.assertEqual(sensors.parse_camera_list(None), ())

    def test_list_is_canonical_order_and_deduped(self):
        self.assertEqual(sensors.parse_camera_list("back, frontleft,back"),
                         ("frontleft", "back"))

    def test_default_matches_contract(self):
        self.assertEqual(sensors.parse_camera_list("frontleft,frontright"),
                         sensors.DEFAULT_CAMERAS)

    def test_unknown_name_fails_loudly(self):
        with self.assertRaises(ValueError):
            sensors.parse_camera_list("frontleft,hand")


class TestFrameSkip(unittest.TestCase):
    def test_equal_rates_publish_every_frame(self):
        self.assertEqual(sensors.frame_skip_count(10, 10), 0)

    def test_gui_20hz_render_halves(self):
        self.assertEqual(sensors.frame_skip_count(20, 10), 1)
        self.assertEqual(sensors.frame_skip_count(30, 10), 2)

    def test_never_negative_or_crashy(self):
        self.assertEqual(sensors.frame_skip_count(5, 10), 0)
        self.assertEqual(sensors.frame_skip_count(10, 0), 0)


class TestAimConventions(unittest.TestCase):
    """USD camera looks along local -Z, up +Y; lights emit along -Z."""

    def _aim(self, rot):
        return (sensors.rotate_xyz(rot, (0.0, 0.0, -1.0)),
                sensors.rotate_xyz(rot, (0.0, 1.0, 0.0)))

    def test_forward_camera_looks_plus_x_up_z(self):
        fwd, up = self._aim(sensors.camera_rotation_xyz(0.0, 0.0))
        self.assertTrue(_close(fwd, (1.0, 0.0, 0.0)), fwd)
        self.assertTrue(_close(up, (0.0, 0.0, 1.0)), up)

    def test_pitch_down_tilts_toward_ground(self):
        fwd, up = self._aim(sensors.camera_rotation_xyz(0.0, 20.0))
        want = (math.cos(math.radians(20)), 0.0, -math.sin(math.radians(20)))
        self.assertTrue(_close(fwd, want), fwd)
        self.assertGreater(up[2], 0.9)  # image stays upright

    def test_side_and_back_yaw(self):
        for yaw, want in ((90.0, (0, 1, 0)), (-90.0, (0, -1, 0)), (180.0, (-1, 0, 0))):
            fwd, up = self._aim(sensors.camera_rotation_xyz(yaw, 0.0))
            self.assertTrue(_close(fwd, want), (yaw, fwd))
            self.assertTrue(_close(up, (0, 0, 1)), (yaw, up))

    def test_extrinsics_point_where_named(self):
        for name, (_, yaw, pitch) in sensors.CAMERA_EXTRINSICS.items():
            fwd, _ = self._aim(sensors.camera_rotation_xyz(yaw, pitch))
            self.assertLess(fwd[2], 0.0, name)  # every camera looks slightly down
        fl = sensors.CAMERA_EXTRINSICS["frontleft"][0]
        fr = sensors.CAMERA_EXTRINSICS["frontright"][0]
        self.assertGreater(fl[1], fr[1])  # left is +Y

    def test_lenses_clear_of_body_bbox(self):
        # Measured /World/Spot/body bbox on the 6.1 asset (see module docstring).
        for name, ((x, y, z), _, _) in sensors.CAMERA_EXTRINSICS.items():
            inside = (-0.429 <= x <= 0.444) and (-0.122 <= y <= 0.122) and (-0.105 <= z <= 0.095)
            self.assertFalse(inside, name)

    def test_light_points_forward_and_down(self):
        d = sensors.rotate_xyz(sensors.light_rotation_xyz(15.0), (0.0, 0.0, -1.0))
        want = (math.cos(math.radians(15)), 0.0, -math.sin(math.radians(15)))
        self.assertTrue(_close(d, want), d)


class TestPayloadLightFade(unittest.TestCase):
    """Rehearsal 2026-09-28: outside in daylight the payload lights blew the
    HUD body-camera tile out to white (spawn, after the retreat). Positions
    below are the storyline's (tank r 6 at the origin, door on +X)."""

    def test_on_at_the_door_and_inside_off_where_spot_sits_outside(self):
        scale = sensors.payload_light_scale
        self.assertEqual(scale((0.0, 0.0)), 1.0)          # tank centre
        self.assertEqual(scale((4.6, 0.0)), 1.0)          # interior waypoint 1
        self.assertEqual(scale((6.58, -0.03)), 1.0)       # door-probe seat
        self.assertEqual(scale((7.8, 0.02)), 0.0)         # sitting after exit / retreat
        self.assertEqual(scale((14.0, 0.0)), 0.0)         # spawn
        self.assertTrue(0.0 < scale((7.2, 0.0)) < 1.0)    # walking up: fading in
        xs = [6.0 + 0.05 * i for i in range(41)]
        values = [scale((x, 0.0)) for x in xs]
        self.assertEqual(values, sorted(values, reverse=True))   # monotonic, no pop
        self.assertTrue(all(abs(a - b) < 0.2 for a, b in zip(values, values[1:])))

    def test_update_writes_only_on_change_and_never_raises(self):
        class Attr:
            def __init__(self):
                self.values = []

            def Set(self, v):
                self.values.append(v)

        lights = object.__new__(sensors.PayloadLights)
        flood, beam = Attr(), Attr()
        lights._lights = [(flood, sensors.SPOTLIGHT_INTENSITY), (beam, sensors.BEAM_INTENSITY)]
        lights.tank_center_xy, lights.tank_radius = (0.0, 0.0), 6.0
        lights.scale, lights.enabled = None, True
        self.assertEqual(lights.update((14.0, 0.0)), 0.0)
        self.assertEqual(lights.update((13.0, 0.0)), 0.0)          # unchanged: no write
        self.assertEqual(flood.values, [0.0])
        self.assertEqual(lights.update((6.58, 0.0)), 1.0)
        self.assertEqual(flood.values[-1], sensors.SPOTLIGHT_INTENSITY)
        self.assertEqual(beam.values[-1], sensors.BEAM_INTENSITY)

        class Broken(Attr):
            def Set(self, v):
                raise RuntimeError("stage gone")

        lights._lights = [(Broken(), 1.0)]
        lights.update((14.0, 0.0))                                  # logs, disables
        self.assertFalse(lights.enabled)
        self.assertIsNone(lights.update((6.0, 0.0)))


class TestChaseCamera(unittest.TestCase):
    C, R = (0.0, 0.0), 6.0

    def test_outside_trails_behind_heading(self):
        # Low 3/4 rear view: CHASE_OUT_DIST away, CHASE_OUT_SIDE_DEG round
        # from dead astern to Spot's right, at a fixed low lens height.
        eye, target = sensors.chase_camera_target((14.0, 0.0, 0.5), math.pi, self.C, self.R)
        dx, dy = eye[0] - 14.0, eye[1]
        self.assertAlmostEqual(math.hypot(dx, dy), sensors.CHASE_OUT_DIST)
        self.assertGreater(dx, 0.0)                    # behind Spot (it faces -X)
        self.assertGreater(dy, 0.0)                    # Spot's right when facing -X is +Y
        self.assertAlmostEqual(math.degrees(math.atan2(dy, dx)), sensors.CHASE_OUT_SIDE_DEG)
        self.assertAlmostEqual(eye[2], sensors.CHASE_OUT_EYE_Z)
        self.assertLess(eye[2], self.DOOR_TOP_Z)        # low: under the lintel height
        self.assertLess(target[0], 14.0)  # looks ahead, toward the tank

    def test_inside_stays_inside_shell(self):
        limit = self.R - sensors.CHASE_WALL_MARGIN
        for x, y, yaw in ((4.6, 0.0, math.pi), (0.0, 0.0, 0.3), (-4.8, 0.0, 0.0),
                          (2.5, 1.6, 2.0), (-2.0, -1.7, -1.0)):
            eye, _ = sensors.chase_camera_target((x, y, 0.5), yaw, self.C, self.R)
            self.assertLessEqual(math.hypot(eye[0], eye[1]), limit + 1e-9, (x, y, yaw))

    # Doorway geometry from sim/build_scene.py: 1.30 m clear opening on the
    # +X wall line (x = 6); lintel 0.25 m deep (x 5.875..6.125) and the
    # 4 m shell above it. Anything crossing that slab at z >= 1.3 is wall.
    DOOR_FACES_X = (5.875, 6.125)
    DOOR_TOP_Z = 1.30

    def _through_wall(self, a, b):
        (ax, _, az), (bx, _, bz) = a, b
        for xw in self.DOOR_FACES_X:
            if min(ax, bx) < xw < max(ax, bx):
                if az + (bz - az) * (xw - ax) / (bx - ax) >= self.DOOR_TOP_Z:
                    return True
        return False

    def _lens_in_wall(self, eye):
        return (self.DOOR_FACES_X[0] <= math.hypot(eye[0], eye[1]) <= self.DOOR_FACES_X[1]
                and eye[2] >= self.DOOR_TOP_Z)

    def _walk(self, x0, x1, yaw, dt=0.1, speed=0.5):
        """Auto-view (eye, target, inside) states along a y = 0 walk through
        the doorway (CameraDirector, as the GUI viewport runs it)."""
        director = sensors.CameraDirector(self.C, self.R)
        x, n = x0, int(abs(x1 - x0) / (speed * dt))
        for _ in range(n + 1):
            body = (x, 0.0, 0.5)
            shot = director.step(body, yaw, dt)
            yield body, (shot.eye, shot.target, director.inside)
            x += math.copysign(speed * dt, x1 - x0)

    def test_doorway_entry_films_through_the_opening(self):
        # Regression: the old shot parked the lens above Spot in the doorway,
        # i.e. inside (x = 6) or staring into the 2.7 m of wall above the door.
        for body, (eye, target, _) in self._walk(8.0, 3.0, math.pi):
            self.assertFalse(self._lens_in_wall(eye), (body, eye))
            self.assertFalse(self._through_wall(eye, body), (body, eye))
            self.assertFalse(self._through_wall(eye, target), (body, eye, target))

    def test_doorway_exit_keeps_spot_in_view(self):
        for body, (eye, _, _) in self._walk(3.0, 9.0, 0.0):
            self.assertFalse(self._lens_in_wall(eye), (body, eye))
            self.assertFalse(self._through_wall(eye, body), (body, eye))

    def test_side_switch_is_one_hard_cut(self):
        states = [shot for _, shot in self._walk(8.0, 3.0, math.pi)]
        flips = [i for i in range(1, len(states)) if states[i][2] != states[i - 1][2]]
        self.assertEqual(len(flips), 1)
        i = flips[0]
        want, _, _ = sensors.chase_camera_shot(
            (8.0 - i * 0.05, 0.0, 0.5), math.pi, self.C, self.R, inside=True)
        self.assertTrue(_close(states[i][0], want, 1e-9))  # no blend across the shell

    def test_side_hysteresis(self):
        # Probe exactly on the shell: the current side wins until it is
        # CHASE_SIDE_HYST past it, so policy sway cannot ping-pong the cut.
        x = self.R - sensors.CHASE_SIDE_PROBE
        for side in (True, False):
            _, _, got = sensors.chase_camera_shot((x, 0.0, 0.5), math.pi, self.C, self.R,
                                                  inside=side)
            self.assertEqual(got, side)
        _, _, got = sensors.chase_camera_shot(
            (x - 2 * sensors.CHASE_SIDE_HYST, 0.0, 0.5), math.pi, self.C, self.R, inside=False)
        self.assertTrue(got)

    def test_outside_eye_never_inside_the_shell(self):
        limit = self.R + sensors.CHASE_WALL_MARGIN
        for x, y, yaw in ((6.6, 0.0, 0.0), (7.0, 0.0, 0.0), (8.0, 2.0, 0.3), (6.8, 1.0, math.pi / 2)):
            eye, _, inside = sensors.chase_camera_shot((x, y, 0.5), yaw, self.C, self.R)
            self.assertFalse(inside, (x, y, yaw))
            self.assertGreaterEqual(math.hypot(eye[0], eye[1]), limit - 1e-9, (x, y, yaw))

    def test_look_at_aims_at_target(self):
        eye, target = (0.0, 0.0, 2.0), (3.0, 4.0, 0.0)
        rot = sensors.look_at_rotation_xyz(eye, target)
        fwd = sensors.rotate_xyz(rot, (0.0, 0.0, -1.0))
        d = [t - e for e, t in zip(eye, target)]
        n = math.sqrt(sum(v * v for v in d))
        self.assertTrue(_close(fwd, [v / n for v in d], 1e-6), fwd)

    def test_smoothing_is_rate_independent(self):
        a = (0.0, 0.0, 0.0)
        for _ in range(10):
            a = sensors.smooth_toward(a, (1.0, 0.0, 0.0), 0.1)
        b = (0.0, 0.0, 0.0)
        for _ in range(20):
            b = sensors.smooth_toward(b, (1.0, 0.0, 0.0), 0.05)
        self.assertAlmostEqual(a[0], b[0], places=9)
        self.assertEqual(sensors.smooth_toward(None, (1, 2, 3), 0.1), (1, 2, 3))


# Interior route from sim/build_scene.py (door at +X on the 6 m shell).
WAYPOINTS = [(4.6, 0.0), (2.5, 1.6), (0.0, 0.0), (-2.0, -1.7), (-4.8, 0.0)]
PIPES = (((2.5, -5.3), (2.5, 0.6)), ((-2.0, -0.7), (-2.0, 5.5)))  # run B, run A at z 0.4, r 0.12


def walk_poses(points, start_yaw=math.pi, speed=0.5, turn=0.8, dt=0.1, dwell=None):
    """(x, y, yaw) every dt along a polyline: turn in place, then walk."""
    x, y = points[0]
    yaw, out = start_yaw, []
    for i, (tx, ty) in enumerate(points[1:], 1):
        want = math.atan2(ty - y, tx - x)
        while abs(sensors._wrap(want - yaw)) > 1e-3:
            d = sensors._wrap(want - yaw)
            yaw = sensors._wrap(yaw + max(-turn * dt, min(turn * dt, d)))
            out.append((x, y, yaw))
        n = max(1, int(math.hypot(tx - x, ty - y) / (speed * dt)))
        out.extend((x + (tx - x) * k / n, y + (ty - y) * k / n, yaw) for k in range(1, n + 1))
        x, y = tx, ty
        out.extend((x, y, yaw) for _ in range(int((dwell or {}).get(i, 0.0) / dt)))
    return out


# Spawn -> probe seat at the door mouth (6 s dwell) -> 5 waypoints -> back
# out the same way -> sit 1.8 m outside: the scripted storyline's path.
STORY = walk_poses([(14.0, 0.0), (6.57, 0.0)] + WAYPOINTS + WAYPOINTS[::-1][1:]
                   + [(6.57, 0.0), (7.8, 0.0)], dwell={1: 6.0, 6: 3.0})
# Staged-spike retreat: in to the tank centre, turn, straight back out.
RETREAT = walk_poses([(6.57, 0.0)] + WAYPOINTS[:3] + [WAYPOINTS[1], WAYPOINTS[0], (7.8, 0.0)])


def _off_axis_deg(eye, target, point):
    f = [t - e for e, t in zip(eye, target)]
    b = [p - e for e, p in zip(eye, point)]
    cos = sum(u * v for u, v in zip(f, b)) / (math.sqrt(sum(u * u for u in f)) * math.sqrt(sum(v * v for v in b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def _shell_blocks(eye, point, radius=6.0, n=600):
    """True if the straight eye -> point line crosses the tank shell anywhere
    but through the 1.3 x 1.3 m doorway, or passes under the roof deck from
    outside (the shell is 4 m tall and roofed)."""
    prev = None
    for i in range(n + 1):
        t = i / n
        x, y, z = (e + t * (p - e) for e, p in zip(eye, point))
        outside = math.hypot(x, y) > radius
        if prev is not None and outside != prev and not (
                x > 0.0 and abs(y) < sensors.DOOR_HALF_WIDTH and z < sensors.DOOR_TOP_Z):
            return True
        prev = outside
    return False


def _pipe_blocks(eye, point, clear=0.12):
    """True if the sight line passes within a pipe's radius of its axis."""
    for (ax, ay), (bx, by) in PIPES:
        for i in range(401):
            t = i / 400.0
            x, y, z = (e + t * (p - e) for e, p in zip(eye, point))
            if min(ay, by) <= y <= max(ay, by) and math.hypot(x - ax, z - 0.4) < clear:
                return True
    return False


class TestCameraDirector(unittest.TestCase):
    C, R = (0.0, 0.0), 6.0

    def _run(self, poses, view="auto", dt=0.1):
        d = sensors.CameraDirector(self.C, self.R, 0.0, view)
        return d, [(pose, d.step((pose[0], pose[1], 0.5), pose[2], dt), d.last_cut, d.kind)
                   for pose in poses]

    def test_auto_cuts_only_at_the_wall_crossings(self):
        d, steps = self._run(STORY)
        cuts = [i for i, (_, _, cut, _) in enumerate(steps) if cut]
        self.assertEqual(len(cuts), 3, cuts)               # first frame, in, out
        kinds = [steps[i][3] for i in cuts]
        self.assertEqual(kinds, ["chase_out", "interior", "chase_out"])
        self.assertEqual(d.cuts, 3)

    def test_auto_moves_smoothly_between_cuts(self):
        # No jumps: 0.1 s apart the lens moves < 0.12 m and the aim point
        # < 0.15 m (Spot walks 0.05 m per step); the zoom < 1 mm.
        for poses in (STORY, RETREAT):
            _, steps = self._run(poses)
            for (_, a, _, _), (pose, b, cut, _) in zip(steps, steps[1:], strict=False):
                if cut:
                    continue
                self.assertLess(math.dist(a.eye, b.eye), 0.12, pose)
                self.assertLess(math.dist(a.target, b.target), 0.15, pose)
                self.assertLess(abs(a.focal_mm - b.focal_mm), 1.0, pose)

    def test_auto_keeps_spot_in_frame_and_in_sight(self):
        blocked = []
        for poses in (STORY, RETREAT):
            _, steps = self._run(poses)
            for pose, shot, _, kind in steps:
                body = (pose[0], pose[1], 0.5)
                half = sensors.hfov_for_focal(shot.focal_mm) / 2.0
                self.assertLess(_off_axis_deg(shot.eye, shot.target, body), 0.5 * half, (pose, kind))
                self.assertFalse(_pipe_blocks(shot.eye, body), (pose, kind))
                if _shell_blocks(shot.eye, body):
                    blocked.append(pose)
        # Only in the doorway on the way out may a jamb hide Spot's centre,
        # for at most 0.3 s (the inside shot holds until the tail is out).
        self.assertLessEqual(len(blocked), 3, blocked)
        for x, y, _ in blocked:
            self.assertLess(abs(x - self.R), 0.7, blocked)

    def test_lens_placement_is_physical(self):
        for view in sensors.VIEW_NAMES:
            _, steps = self._run(STORY, view)
            for pose, shot, _, kind in steps:
                r = math.hypot(shot.eye[0], shot.eye[1])
                self.assertFalse(self.R - 0.15 <= r <= self.R + 0.15, (view, pose, shot.eye))
                if r < self.R:
                    self.assertLess(shot.eye[2], sensors.TANK_ROOF_Z - 0.1, (view, pose))
                if kind == "chase_out":
                    self.assertGreaterEqual(r, self.R + sensors.CHASE_WALL_MARGIN - 1e-6, (view, pose))
                if kind in ("chase_in", "interior", "overhead"):
                    self.assertLessEqual(r, self.R - sensors.CHASE_WALL_MARGIN + 1e-6, (view, pose))
                self.assertGreater(shot.eye[2], 0.8, (view, pose))

    def test_exit_turns_to_a_front_view(self):
        # Walking out of the door there is no room astern (the shell): the
        # lens goes ahead-right and Spot walks toward it, then sits in frame.
        d, steps = self._run(STORY)
        pose, shot, _, kind = steps[-1]
        self.assertEqual(kind, "chase_out")
        self.assertTrue(d.front)
        self.assertGreater(shot.eye[0], pose[0])            # in front of Spot (facing +X)
        self.assertLess(shot.eye[1], 0.0)                    # on Spot's right

    # sim/build_scene.py camera_keepouts: the two barrels and the pipe rack.
    KEEPOUTS = ({"xy": (9.5, 3.0), "r": 0.3}, {"xy": (10.3, 3.6), "r": 0.3},
                {"min": (8.75, -5.2), "max": (10.25, -1.8)})

    def test_lens_stays_clear_of_the_exterior_dressing(self):
        # Regression (host gp2): the exit front view parked the lens inside
        # the pipe rack, so the frame was a black post. With the keep-outs
        # the lens moves to Spot's other side and keeps a clear line to it.
        keep = sensors.parse_keepouts(self.KEEPOUTS)
        self.assertEqual(len(keep), 3)
        for poses in (STORY, RETREAT):
            d = sensors.CameraDirector(self.C, self.R, 0.0, "auto", keepouts=self.KEEPOUTS)
            for x, y, yaw in poses:
                shot = d.step((x, y, 0.5), yaw, 0.1)
                if d.kind != "chase_out":
                    continue
                gap = min(sensors.keepout_distance(shot.eye[0], shot.eye[1], k) for k in keep)
                self.assertGreater(gap, sensors.KEEPOUT_CLEARANCE_M - 0.05, ((x, y), shot.eye))
                self.assertTrue(sensors.lens_clear(shot.eye[:2], (x, y), keep, clearance=0.0), (x, y))
            self.assertTrue(d.front)
            self.assertEqual(d.side, -1)                      # the rack is on Spot's right
            self.assertGreater(shot.eye[1], 0.0)

    def test_keepout_geometry(self):
        box = ("box", 0.0, 0.0, 2.0, 1.0)
        self.assertEqual(sensors.keepout_distance(1.0, 0.5, box), 0.0)
        self.assertAlmostEqual(sensors.keepout_distance(3.0, 0.5, box), 1.0)
        self.assertAlmostEqual(sensors.keepout_distance(3.0, 2.0, box), math.sqrt(2.0))
        self.assertAlmostEqual(sensors.keepout_distance(0.0, 2.0, ("circle", 0.0, 0.0, 0.5)), 1.5)
        self.assertFalse(sensors.lens_clear((-1.0, 0.5), (3.0, 0.5), [box], clearance=0.0))
        self.assertTrue(sensors.lens_clear((-1.0, 2.5), (3.0, 2.5), [box]))
        self.assertEqual(sensors.parse_keepouts([{"xy": (1, 2)}, {"min": (0, 0)}, None, "x"]), [])

    def test_interior_zoom_frames_spot_and_stays_in_range(self):
        lo, hi = sensors.INTERIOR_HFOV_DEG
        _, steps = self._run(STORY)
        hfovs = [sensors.hfov_for_focal(shot.focal_mm) for _, shot, _, kind in steps if kind == "interior"]
        self.assertTrue(hfovs)
        self.assertTrue(all(lo - 1e-6 <= h <= hi + 1e-6 for h in hfovs), (min(hfovs), max(hfovs)))
        # Far end (ladder, ~9 m from the lens) is tighter than the door end.
        self.assertLess(min(hfovs), max(hfovs) - 10.0)

    def test_every_view_follows_spot_inside_the_tank(self):
        inside = [p for p in STORY if math.hypot(p[0], p[1]) < 4.9]
        for view in ("auto", "chase", "interior", "overhead"):
            _, steps = self._run(inside, view)
            for pose, shot, _, kind in steps[5:]:
                body = (pose[0], pose[1], 0.5)
                self.assertFalse(_shell_blocks(shot.eye, body), (view, pose))
                if shot.rot is None:
                    half = sensors.hfov_for_focal(shot.focal_mm) / 2.0
                    self.assertLess(_off_axis_deg(shot.eye, shot.target, body), half, (view, pose))
                else:  # overhead: straight down, Spot within the footprint
                    fwd = sensors.rotate_xyz(shot.rot, (0.0, 0.0, -1.0))
                    self.assertTrue(_close(fwd, (0.0, 0.0, -1.0)), fwd)
                    half_w = (shot.eye[2] - 0.5) * math.tan(math.radians(
                        sensors.hfov_for_focal(shot.focal_mm) / 2.0))
                    self.assertLess(abs(body[0] - shot.eye[0]), half_w, (view, pose))
                    self.assertLess(abs(body[1] - shot.eye[1]), half_w * 9.0 / 16.0, (view, pose))

    def test_overhead_image_right_is_the_door_side(self):
        d = sensors.CameraDirector(self.C, self.R, 0.0, "overhead")
        shot = d.step((0.0, 0.0, 0.5), 0.0, 0.1)
        right = sensors.rotate_xyz(shot.rot, (1.0, 0.0, 0.0))
        self.assertTrue(_close(right, (1.0, 0.0, 0.0)), right)   # door at +X on the right
        d = sensors.CameraDirector(self.C, self.R, math.pi / 2, "overhead")
        right = sensors.rotate_xyz(d.step((0.0, 0.0, 0.5), 0.0, 0.1).rot, (1.0, 0.0, 0.0))
        self.assertTrue(_close(right, (0.0, 1.0, 0.0)), right)

    def test_door_view_sees_the_probe_seat(self):
        d = sensors.CameraDirector(self.C, self.R, 0.0, "door")
        for _ in range(30):
            shot = d.step((6.57, 0.0, 0.5), math.pi, 0.1)
        self.assertGreater(math.hypot(*shot.eye[:2]), self.R + 1.0)
        self.assertFalse(_shell_blocks(shot.eye, (6.57, 0.0, 0.5)))
        self.assertLess(_off_axis_deg(shot.eye, shot.target, (6.57, 0.0, 0.5)), 5.0)

    def test_view_switch_and_teleport_are_cuts(self):
        d = sensors.CameraDirector(self.C, self.R)
        d.step((14.0, 0.0, 0.5), math.pi, 0.1)
        d.step((13.95, 0.0, 0.5), math.pi, 0.1)
        self.assertFalse(d.last_cut)
        self.assertEqual(d.set_view("  Overhead "), "overhead")
        d.step((13.9, 0.0, 0.5), math.pi, 0.1)
        self.assertTrue(d.last_cut)
        self.assertEqual(d.kind, "overhead")
        d.set_view("overhead")                              # same view: no cut
        d.step((13.85, 0.0, 0.5), math.pi, 0.1)
        self.assertFalse(d.last_cut)
        d.set_view("auto")
        d.step((0.0, 0.0, 0.5), 0.0, 0.1)
        d.step((14.0, 0.0, 0.5), math.pi, 0.1)             # reset_spot.sh teleport
        self.assertTrue(d.last_cut)
        self.assertEqual(d.kind, "chase_out")
        with self.assertRaises(ValueError):
            d.set_view("drone")
        self.assertEqual(d.view, "auto")

    def test_orbit_never_leaves_the_allowed_arc(self):
        # Lens outside the 6.5 m ring at 3.4 m from Spot standing just out of
        # the door facing out: every smoothed azimuth stays allowed.
        q, dist, limit = (6.6, 0.0), sensors.CHASE_OUT_DIST, 6.5
        arc = sensors.allowed_arc(q[0], q[1], dist, limit, outside=True)
        a = sensors._clamp_to_arc(math.radians(150.0), arc)
        for _ in range(100):
            a = sensors.orbit_toward(a, math.radians(-150.0), 0.2, arc)
            eye = (q[0] + dist * math.cos(a), q[1] + dist * math.sin(a))
            self.assertGreaterEqual(math.hypot(*eye), limit - 1e-9)

    def test_allowed_arc_matches_geometry(self):
        for qx, qy, dist, limit, outside in ((7.0, 1.0, 3.4, 6.5, True), (3.0, -2.0, 2.4, 5.5, False),
                                             (5.0, 0.0, 2.4, 5.5, False), (0.0, 0.0, 2.0, 5.5, False)):
            arc = sensors.allowed_arc(qx, qy, dist, limit, outside)
            for k in range(360):
                a = math.radians(k)
                r = math.hypot(qx + dist * math.cos(a), qy + dist * math.sin(a))
                ok = r >= limit - 1e-9 if outside else r <= limit + 1e-9
                inside_arc = arc is not None and abs(sensors._wrap(a - arc[0])) <= arc[1] + 1e-9
                if abs(r - limit) > 1e-6:
                    self.assertEqual(ok, inside_arc, (qx, qy, dist, limit, outside, k))


REPO = pathlib.Path(__file__).resolve().parent.parent


class TestPresentationUi(unittest.TestCase):
    """Regression (host 2026-09-27): one Esc in the fullscreen viewport ran
    Kit's cancel_fullscreen_ui hotkey and put every editor panel on the
    audience's screen."""

    def test_escape_hotkey_is_disabled(self):
        self.assertIs(sensors.KIT_UI_SETTINGS["/exts/omni.kit.ui.actions/ui_hotkey_escape"], False)

    def test_ui_mode_applies_the_settings_before_the_fullscreen_action(self):
        src = (REPO / "sim" / "isaac_app.py").read_text()
        body = src[src.index("def _set_ui_mode("):]
        body = body[:body.index("\ndef ", 1)]
        applied = body.index("sensors.KIT_UI_SETTINGS")
        self.assertLess(applied, body.index('execute_action("omni.kit.ui.actions", "toggle_fullscreen"'))
        self.assertLess(applied, body.index("set_fullscreen(want)"))


@unittest.skipUnless(shutil.which("bash"), "needs bash")
class TestRunSimLayout(unittest.TestCase):
    """host/run_sim.sh ensure_gui_layout with a stub dcv: raise a no-viewer
    console to 1920x1080, but never override an attached viewer's layout
    (that would crop / scale the fullscreen render in a smaller viewer)."""

    def _run(self, layout, conns, conns_rc=0):
        src = (REPO / "host" / "run_sim.sh").read_text()
        func = re.search(r"^ensure_gui_layout\(\) \{\n.*?^\}\n", src, re.S | re.M).group(0)
        with tempfile.TemporaryDirectory() as tmp:
            calls = pathlib.Path(tmp, "calls")
            pathlib.Path(tmp, "sudo").write_text('#!/bin/sh\nexec "$@"\n')
            pathlib.Path(tmp, "dcv").write_text(
                "#!/bin/sh\n"
                'case "$1" in\n'
                '  describe-session) printf "Session: \\n\\tid: console\\n\\tdisplay layout: %s+0+0\\n" "$STUB_LAYOUT" ;;\n'
                '  list-connections) [ -n "$STUB_CONNS" ] && printf "%s\\n" "$STUB_CONNS"; exit "$STUB_RC" ;;\n'
                '  set-display-layout) echo "$*" >> "$STUB_CALLS" ;;\n'
                "esac\n")
            for name in ("sudo", "dcv"):
                os.chmod(os.path.join(tmp, name), 0o755)
            env = dict(os.environ, PATH=tmp + os.pathsep + os.environ.get("PATH", ""),
                       STUB_LAYOUT=layout, STUB_CONNS=conns, STUB_RC=str(conns_rc),
                       STUB_CALLS=str(calls))
            env.pop("SPOT_GUI_LAYOUT", None)
            # A fixed script: the function's source reaches it as a positional
            # parameter, never spliced into the shell code.
            out = subprocess.run(["bash", "-c", 'log() { echo "LOG $*"; }\neval "$1"\nensure_gui_layout',
                                  "bash", func], env=env, capture_output=True, text=True, timeout=30)
            self.assertEqual(out.returncode, 0, out.stderr)
            return calls.read_text() if calls.exists() else "", out.stdout

    def test_no_viewer_small_console_is_raised(self):
        calls, _ = self._run("1024x768", "There are no clients connected to the session.")
        self.assertIn("--session console 1920x1080+0+0", calls)

    def test_attached_viewer_keeps_its_layout(self):
        calls, out = self._run("1440x900", "Connection ID: 1\n  user: ubuntu\n  client address: 127.0.0.1")
        self.assertEqual(calls, "")
        self.assertIn("viewer attached", out)

    def test_big_enough_console_is_left_alone(self):
        calls, _ = self._run("1920x1080", "There are no clients connected to the session.")
        self.assertEqual(calls, "")

    def test_unknown_connection_state_falls_back_to_raising(self):
        calls, _ = self._run("1024x768", "", conns_rc=1)
        self.assertIn("1920x1080+0+0", calls)


class TestParseViewCommand(unittest.TestCase):
    def test_json_and_bare_names(self):
        self.assertEqual(sensors.parse_view_command('{"camera": "Interior"}'), ("interior", None))
        self.assertEqual(sensors.parse_view_command('{"ui": "editor"}'), (None, "editor"))
        self.assertEqual(sensors.parse_view_command('{"camera": "door", "ui": "presentation"}'),
                         ("door", "presentation"))
        self.assertEqual(sensors.parse_view_command(" overhead "), ("overhead", None))
        self.assertEqual(sensors.parse_view_command('"chase"'), ("chase", None))

    def test_rejects_bad_requests(self):
        for text in ("", "{}", "[]", "drone", '{"camera": "drone"}', '{"ui": "fullscreen"}',
                     '{"camera": "auto", "fov": 30}', "42"):
            with self.assertRaises(ValueError, msg=text):
                sensors.parse_view_command(text)


if __name__ == "__main__":
    unittest.main()
