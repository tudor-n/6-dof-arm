"""Offline geometry tests - no camera, no arm, no serial port required.

Run with:  python test_geometry.py        (or: pytest test_geometry.py)

These cover the maths that the vision->arm registration depends on. The pose tests
build a synthetic camera looking at the real marker layout from vision.json, so the
whole solve -> average -> raycast chain can be checked in software against known
ground truth.
"""
import math
import numpy as np
import cv2

import vision as V
import server as S

TOL_MM = 0.5          # closed-loop raycast tolerance
TOL_DEG = 0.05


# --------------------------------------------------------------------------- helpers

def rot(axis, deg):
    a = np.asarray(axis, float)
    a /= np.linalg.norm(a)
    return cv2.Rodrigues((a * math.radians(deg)).reshape(3, 1))[0]


def ang_between(Ra, Rb):
    """Geodesic angle between two rotations, in degrees."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def equivalent_rvec(rvec):
    """The other Rodrigues vector describing the SAME rotation.

    R(a, t) == R(-a, 2pi - t), so negating the axis and using the explement angle
    leaves the rotation untouched while flipping the vector. solvePnP's LM refinement
    can return either form, which is exactly what breaks naive averaging.
    """
    r = np.asarray(rvec, float).ravel()
    t = np.linalg.norm(r)
    return (-r / t) * (2 * math.pi - t)


def synthetic_scene(cam_pos=(150.0, -450.0, 700.0)):
    """A camera high up on the robot's right, looking down at the measured markers.

    Built as a look-at so the object->camera rotation genuinely lands near 180 deg, the
    same regime the real rig sits in - which is what makes the Rodrigues wrap bite.
    """
    vis = V.Vision(".", 0, lambda *a, **k: None)
    assert vis.K is not None, "camera_matrix.npy missing - cannot run pose tests"

    cam_pos = np.array(cam_pos, float)
    target = np.array([140.0, 0.0, 5.0])
    fwd = target - cam_pos
    fwd /= np.linalg.norm(fwd)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, world_up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R_cam = np.vstack([right, down, fwd])          # rows: camera x, y, z in base frame
    rvec = cv2.Rodrigues(R_cam)[0]
    tvec = (-R_cam @ cam_pos).reshape(3, 1)
    return vis, rvec, tvec, cam_pos


# --------------------------------------------------------------------- rotation average

def test_equivalent_rvec_is_the_same_rotation():
    rv = (np.array([0.3, -2.9, 0.6])).reshape(3, 1)
    assert ang_between(cv2.Rodrigues(rv)[0], cv2.Rodrigues(equivalent_rvec(rv))[0]) < 1e-6


def test_naive_rvec_mean_is_broken_near_180_degrees():
    """Documents the bug: at |rvec| ~ pi the axis sign flips and the mean collapses."""
    truth = rot([1.0, 0.08, 0.05], 179.0)
    base = cv2.Rodrigues(truth)[0]
    samples = [base if i % 2 == 0 else equivalent_rvec(base).reshape(3, 1) for i in range(14)]

    naive = cv2.Rodrigues(np.mean(samples, axis=0))[0]
    assert ang_between(naive, truth) > 45.0, "expected the naive mean to be badly wrong"

    good = cv2.Rodrigues(V.avg_rotation(samples))[0]
    assert ang_between(good, truth) < 1.0, f"SO(3) mean off by {ang_between(good, truth):.2f} deg"


def test_avg_rotation_cancels_noise():
    truth = rot([0.9, 0.1, 0.2], 176.0)
    rng = np.random.default_rng(7)
    samples = []
    for i in range(15):
        noisy = rot([0.9, 0.1, 0.2], 176.0) @ rot(rng.normal(size=3), rng.normal(0, 0.4))
        rv = cv2.Rodrigues(noisy)[0]
        samples.append(equivalent_rvec(rv).reshape(3, 1) if i % 3 == 0 else rv)
    out = cv2.Rodrigues(V.avg_rotation(samples))[0]
    assert ang_between(out, truth) < 0.5


def test_avg_rotation_is_a_proper_rotation():
    rng = np.random.default_rng(3)
    samples = [cv2.Rodrigues(rot(rng.normal(size=3), rng.uniform(150, 200)))[0] for _ in range(9)]
    R = cv2.Rodrigues(V.avg_rotation(samples))[0]
    assert abs(np.linalg.det(R) - 1.0) < 1e-9
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-9)


def test_avg_rotation_single_sample_is_identity_op():
    rv = cv2.Rodrigues(rot([0.2, 1.0, -0.3], 171.0))[0]
    out = V.avg_rotation([rv])
    # compare the vectors directly: ang_between uses acos, which is ill-conditioned at 0
    assert np.max(np.abs(out.ravel() - rv.ravel())) < 1e-12


# ------------------------------------------------------------------------- camera pose

def test_camera_centre_round_trip():
    _, rvec, tvec, cam_pos = synthetic_scene()
    assert np.allclose(V.camera_centre(rvec, tvec), cam_pos, atol=1e-6)


def test_solve_recovers_the_synthetic_pose():
    vis, rvec, tvec, cam_pos = synthetic_scene()
    ids = sorted(vis.markers)
    obj = np.array([vis.markers[i] for i in ids], np.float32)
    px = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, vis.K, vis.Z)[0].reshape(-1, 2)

    got = vis.solve_pose(obj, px.astype(np.float32))
    assert got is not None, "solve_pose rejected a perfectly valid synthetic scene"
    rv, tv, err = got
    assert err < 0.5, f"reprojection error {err:.2f}px on noiseless input"
    assert ang_between(cv2.Rodrigues(rv)[0], cv2.Rodrigues(rvec)[0]) < 1.0
    assert np.allclose(V.camera_centre(rv, tv), cam_pos, atol=5.0)


def test_pose_gate_rejects_a_camera_below_the_table():
    """Contract: a solve that lands the camera under the table must be refused."""
    vis, good_rvec, good_tvec, _ = synthetic_scene()
    assert vis.pose_is_sane(good_rvec, good_tvec)

    # same look-at geometry, reflected through the table: camera underneath, aiming up
    _, bad_rvec, bad_tvec, bad_pos = synthetic_scene(cam_pos=(150.0, -450.0, -690.0))
    assert bad_pos[2] < vis.table_z(bad_pos[0], bad_pos[1])
    assert not vis.pose_is_sane(bad_rvec, bad_tvec)


def ippe_branches(vis, rvec, tvec):
    """Both solutions of the coplanar-PnP ambiguity, with each one's reprojection error.

    IPPE needs exactly coplanar object points, so the markers are flattened onto their
    fitted plane first.
    """
    ids = sorted(vis.markers)
    obj = np.array([[vis.markers[i][0], vis.markers[i][1],
                     vis.table_z(vis.markers[i][0], vis.markers[i][1])] for i in ids], np.float32)
    px = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, vis.K, vis.Z)[0].reshape(-1, 2).astype(np.float32)
    n, rvs, tvs, _ = cv2.solvePnPGeneric(obj, px, vis.K, vis.Z, flags=cv2.SOLVEPNP_IPPE)
    out = []
    for r, t in zip(rvs, tvs):
        pr = cv2.projectPoints(obj.reshape(-1, 1, 3), r, t, vis.K, vis.Z)[0].reshape(-1, 2)
        out.append((r, t, float(np.mean(np.linalg.norm(pr - px, axis=1)))))
    return out


def test_gates_keep_exactly_one_branch_of_the_planar_ambiguity():
    """Reproduce the ambiguity in software and check solve_pose's criteria disambiguate.

    Both of solve_pose's filters are applied here, because in this geometry each one
    catches a different thing: see the two tests below.
    """
    vis, rvec, tvec, cam_pos = synthetic_scene()
    branches = ippe_branches(vis, rvec, tvec)
    assert len(branches) == 2, f"expected an ambiguous pair, got {len(branches)}"

    kept = [(r, t) for r, t, e in branches
            if vis.pose_is_sane(r, t) and e <= V.MAX_SOLVE_REPROJ_PX]
    assert len(kept) == 1, f"gates kept {len(kept)} of 2 branches, expected exactly 1"
    assert np.allclose(V.camera_centre(*kept[0]), cam_pos, atol=5.0)


def test_the_twin_branch_is_caught_by_reprojection_not_by_height():
    """Documents WHICH gate does the work for this marker layout.

    The twin puts the camera on the far side of the table in Y but still well above it,
    so the height check passes it; the strongly non-square, obliquely viewed quadrilateral
    means the twin cannot fit the pixels, so reprojection is what rejects it. A more
    frontoparallel or square layout would flip which gate matters, which is why both exist.
    """
    vis, rvec, tvec, cam_pos = synthetic_scene()
    branches = ippe_branches(vis, rvec, tvec)
    twin = max(branches, key=lambda b: np.linalg.norm(V.camera_centre(b[0], b[1]) - cam_pos))
    assert vis.pose_is_sane(twin[0], twin[1]), "twin is above the table, so height cannot catch it"
    assert twin[2] > 10.0, f"twin reprojects at {twin[2]:.1f}px - expected it to be clearly unfittable"


def test_solve_pose_survives_pixel_noise():
    vis, rvec, tvec, cam_pos = synthetic_scene()
    ids = sorted(vis.markers)
    obj = np.array([vis.markers[i] for i in ids], np.float32)
    px = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, vis.K, vis.Z)[0].reshape(-1, 2)
    rng = np.random.default_rng(11)
    noisy = (px + rng.normal(0, 0.4, px.shape)).astype(np.float32)

    got = vis.solve_pose(obj, noisy)
    assert got is not None, "solve_pose rejected a realistically noisy scene"
    rv, tv, err = got
    assert err < V.MAX_SOLVE_REPROJ_PX
    assert np.linalg.norm(V.camera_centre(rv, tv) - cam_pos) < 40.0


def test_pixel_to_table_closes_the_loop():
    """Project known table points to pixels, raycast back, expect the originals."""
    vis, rvec, tvec, _ = synthetic_scene()
    vis.rvec, vis.tvec = rvec, tvec
    vis.s["object_z"] = 0.0
    vis.s["x_trim"] = vis.s["y_trim"] = 0.0

    for x, y in [(140.0, 0.0), (60.0, 170.0), (230.0, -240.0), (220.0, 250.0), (300.0, 60.0)]:
        z = vis.table_z(x, y)
        u, v = vis.project([[x, y, z]])[0]
        back = vis.pixel_to_table(u, v, raw=True)
        assert back is not None, f"raycast failed for ({x}, {y})"
        assert math.hypot(back[0] - x, back[1] - y) < TOL_MM, \
            f"({x}, {y}) -> ({back[0]:.2f}, {back[1]:.2f})"


def test_markers_reproject_onto_themselves():
    vis, rvec, tvec, _ = synthetic_scene()
    vis.rvec, vis.tvec = rvec, tvec
    vis.s["object_z"] = 0.0
    for mid, p in vis.markers.items():
        u, v = vis.project([p])[0]
        back = vis.pixel_to_table(u, v, raw=True, z_off=p[2] - vis.table_z(p[0], p[1]))
        assert math.hypot(back[0] - p[0], back[1] - p[1]) < TOL_MM, f"marker {mid}"


# ------------------------------------------------------------------- arm kinematics / TCP

def cfg_with(**tool):
    c = S.load_cfg()
    c["tool"] = {**S.DEFAULT_TOOL, **tool}
    return c


def test_zero_tool_offset_matches_the_old_tip():
    """With all offsets zero the TCP must sit exactly on the old L3 centreline tip."""
    cfg = cfg_with(along=0.0, perp=0.0, lateral=0.0)
    servo = [90.0, 70.0, 120.0, 150.0, 90.0, 90.0]
    L = cfg["links"]
    yaw, al, be, ga = [math.radians(a) for a in S.joint_angles(servo, cfg)]
    a1 = math.pi / 2 - al
    a2 = a1 - be
    p = a2 - ga
    r = L["L1"] * math.cos(a1) + L["L2"] * math.cos(a2) + L["L3"] * math.cos(p)
    z = L["L0"] + L["L1"] * math.sin(a1) + L["L2"] * math.sin(a2) + L["L3"] * math.sin(p)
    expect = (r * math.cos(yaw), r * math.sin(yaw), z)
    got = S.fk(servo, cfg)[0][-1]
    assert np.allclose(got, expect, atol=1e-9)


def test_along_offset_extends_the_tool():
    servo = [90.0, 70.0, 120.0, 150.0, 90.0, 90.0]
    a = S.fk(servo, cfg_with(along=0.0, perp=0.0, lateral=0.0))[0][-1]
    b = S.fk(servo, cfg_with(along=25.0, perp=0.0, lateral=0.0))[0][-1]
    assert abs(math.dist(a, b) - 25.0) < 1e-6


def test_roll_moves_an_offset_tcp_but_not_a_centred_one():
    servo_a = [90.0, 70.0, 120.0, 150.0, 45.0, 90.0]
    servo_b = [90.0, 70.0, 120.0, 150.0, 135.0, 90.0]

    centred = cfg_with(along=0.0, perp=0.0, lateral=0.0)
    assert np.allclose(S.fk(servo_a, centred)[0][-1], S.fk(servo_b, centred)[0][-1], atol=1e-9)

    offset = cfg_with(along=0.0, perp=30.0, lateral=0.0)
    pa = S.fk(servo_a, offset)[0][-1]
    pb = S.fk(servo_b, offset)[0][-1]
    assert math.dist(pa, pb) > 20.0, "wrist roll must move a TCP that is off the roll axis"


def test_perp_offset_is_perpendicular_to_the_tool_axis():
    """At roll == zero the perpendicular offset must not change the tool's reach along
    its own axis, only displace it sideways by exactly the offset."""
    servo = [90.0, 70.0, 120.0, 150.0, S.load_cfg()["joints"][4]["zero"], 90.0]
    a = np.array(S.fk(servo, cfg_with(along=0.0, perp=0.0, lateral=0.0))[0][-1])
    b = np.array(S.fk(servo, cfg_with(along=0.0, perp=40.0, lateral=0.0))[0][-1])
    assert abs(np.linalg.norm(b - a) - 40.0) < 1e-6


TARGETS = [(220.0, 0.0, 60.0, -90.0), (180.0, 120.0, 90.0, -60.0),
           (160.0, -140.0, 40.0, -90.0), (260.0, 40.0, 120.0, -45.0)]


def round_trip(cfg, roll_servo):
    """IK a TCP target, FK the answer back, return the worst miss in mm.

    ik() yields only the four arm joints, so the roll it solved for has to be put back
    before fk() can reproduce the same tool point.
    """
    worst, tested = 0.0, 0
    for x, y, z, pitch in TARGETS:
        servo, reach, lim = S.ik(x, y, z, pitch, cfg, roll_servo)
        if reach or lim:
            continue
        got = S.fk(list(servo) + [roll_servo, 90.0], cfg)[0][-1]
        worst = max(worst, math.dist(got, (x, y, z)))
        tested += 1
    assert tested, "no target was reachable - the round trip tested nothing"
    return worst


def test_ik_fk_round_trip_with_a_tool_offset():
    """A rolled, offset tool is the case the old centreline-only model could not express."""
    assert round_trip(cfg_with(along=12.0, perp=30.0, lateral=8.0), 120.0) < 1.0


def test_ik_fk_round_trip_across_the_roll_range():
    cfg = cfg_with(along=12.0, perp=30.0, lateral=8.0)
    for roll in (0.0, 45.0, 90.0, 135.0, 180.0):
        assert round_trip(cfg, roll) < 1.0, f"round trip broke at roll {roll}"


def test_ik_round_trip_still_works_without_a_tool_offset():
    assert round_trip(cfg_with(along=0.0, perp=0.0, lateral=0.0), 90.0) < 1.0



def test_joint_angles_still_reports_four_arm_joints():
    cfg = S.load_cfg()
    assert len(S.joint_angles([90.0] * 6, cfg)) == 4


# ------------------------------------------------------------------------------- runner

def main():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    fails = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as e:
            fails.append(name)
            print(f"  FAIL  {name}: {e}")
        except Exception as e:
            fails.append(name)
            print(f"  ERROR {name}: {type(e).__name__}: {e}")
    print(f"\n{len(tests) - len(fails)}/{len(tests)} passed")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
