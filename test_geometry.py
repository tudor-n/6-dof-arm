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


# ------------------------------------------------- tape measurements (non-FK ground truth)

# Tudor's hand measurements, 2026-10-19. The only numbers in this project that do not come
# from the arm itself.
TAPE = {"0-1": 392.0, "0-2": 152.0, "0-3": 420.0, "1-2": 436.0, "1-3": 174.0, "2-3": 401.0}

# The arm-measured model as it stood before the tape exposed it: a trapezoid where reality is
# a rectangle. Kept as a fixture so these tests keep exercising the defect after vision.json
# was corrected to the hand-measured values.
FK_MARKERS = {0: [52.7, 170.8, 5.2], 1: [39.5, -180.9, 5.0],
              2: [221.4, 254.9, 5.0], 3: [239.4, -240.4, 5.4]}


def vision_with_fk_markers():
    vis = V.Vision(".", 0, lambda *a, **k: None)
    vis.markers = {k: list(v) for k, v in FK_MARKERS.items()}
    vis.touched = {k: list(v) for k, v in FK_MARKERS.items()}
    vis.tape = dict(TAPE)
    vis._fit_plane()
    return vis


def test_shape_from_distances_round_trips_a_known_rectangle():
    truth = {0: np.array([0.0, 0.0]), 1: np.array([400.0, 0.0]),
             2: np.array([0.0, 160.0]), 3: np.array([400.0, 160.0])}
    tape = {f"{i}-{j}": float(np.linalg.norm(truth[i] - truth[j]))
            for i in range(4) for j in range(i + 1, 4)}
    got, closure = V.shape_from_distances(tape, [0, 1, 2, 3])
    assert closure < 1e-6, f"closure {closure}"
    for i in range(4):
        for j in range(i + 1, 4):
            assert abs(np.linalg.norm(got[i] - got[j]) - tape[f"{i}-{j}"]) < 1e-6


def test_shape_from_distances_reports_closure_error_on_inconsistent_tape():
    bad = dict(TAPE)
    bad["2-3"] = 300.0                      # 100 mm off what the other five imply
    _, closure = V.shape_from_distances(bad, [0, 1, 2, 3])
    assert closure > 50.0, f"inconsistent tape should show a big closure error, got {closure:.1f}"


def test_real_tape_is_self_consistent():
    """The six measurements must close as a planar quadrilateral, or they are untrustworthy."""
    _, closure = V.shape_from_distances(TAPE, [0, 1, 2, 3])
    assert closure < 5.0, f"tape closes to {closure:.1f} mm - re-measure before trusting it"


def test_shape_from_distances_rejects_a_missing_pair():
    short = {k: v for k, v in TAPE.items() if k != "1-3"}
    try:
        V.shape_from_distances(short, [0, 1, 2, 3])
    except ValueError as e:
        assert "1-3" in str(e)
    else:
        raise AssertionError("expected a ValueError naming the missing pair")


def test_tape_check_catches_the_fk_shape_distortion():
    """The arm-measured model really did disagree with the tape - this pins the magnitude."""
    vis = vision_with_fk_markers()
    chk = vis.tape_check()
    assert chk is not None and len(chk["rows"]) == 6
    assert not chk["ok"], "expected the FK-derived model to fail the tape check"
    assert chk["worst"] > 50.0, f"worst disagreement only {chk['worst']} mm"


def test_the_shipped_marker_model_passes_the_tape_check():
    """vision.json now carries hand-measured positions; they must agree with the tape."""
    vis = V.Vision(".", 0, lambda *a, **k: None)
    chk = vis.tape_check()
    assert chk is not None, "vision.json should ship tape measurements"
    assert chk["ok"], f"the shipped marker model disagrees with the tape: {chk}"


def test_tape_check_returns_none_without_tape():
    vis = V.Vision(".", 0, lambda *a, **k: None)
    vis.tape = {}
    assert vis.tape_check() is None


def test_markers_from_tape_produces_a_shape_correct_model():
    """The whole point: the rebuilt model must match the tape, not the arm's distortion."""
    vis = vision_with_fk_markers()
    assert not vis.tape_check()["ok"], "precondition: the FK model fails the tape"

    out, info = vis.markers_from_tape(apply=False)
    assert out is not None, info
    assert vis.tape_check()["worst"] > 50.0, "apply=False must not modify the stored model"

    vis.markers = out
    chk = vis.tape_check()
    assert chk["ok"], f"rebuilt model still disagrees with the tape: {chk}"
    assert chk["worst"] < 2.0, f"worst {chk['worst']} mm"


def test_markers_from_tape_keeps_the_layout_the_right_way_round():
    """A rigid fit cannot undo a mirror, so both reflections are tried. Markers 0 and 2 are
    on the robot's left (+Y) and 1 and 3 on its right; that must survive the rebuild."""
    vis = vision_with_fk_markers()
    out, _ = vis.markers_from_tape(apply=False)
    assert out[0][1] > 0 and out[2][1] > 0, f"0 and 2 should be +Y (robot left), got {out}"
    assert out[1][1] < 0 and out[3][1] < 0, f"1 and 3 should be -Y (robot right), got {out}"
    assert out[0][0] < out[2][0], "marker 0 is nearer the base than marker 2"
    assert out[1][0] < out[3][0], "marker 1 is nearer the base than marker 3"


def test_markers_from_tape_reports_the_fk_residual():
    vis = vision_with_fk_markers()
    _, info = vis.markers_from_tape(apply=False)
    assert "RMS residual" in info
    assert "forward kinematics is distorted" in info,         "a 38 mm RMS placement residual must be called out, not buried"


# ----------------------------------- homography as a cross-check on the intrinsics

def scene_with_homography(K_scale=1.0):
    """Synthetic scene with both mappings live. K_scale fakes a focal-length error."""
    vis, rvec, tvec, cam_pos = synthetic_scene()
    if K_scale != 1.0:
        vis.K = vis.K.copy()
        vis.K[0, 0] *= K_scale
        vis.K[1, 1] *= K_scale
    vis.rvec, vis.tvec = rvec, tvec
    vis.s["object_z"] = 0.0
    vis.s["x_trim"] = vis.s["y_trim"] = 0.0
    ids = sorted(vis.markers)
    # pixels the markers land on, using the TRUE intrinsics the scene was built with
    true_K = np.load("camera_matrix.npy")
    obj = np.array([vis.markers[i] for i in ids], np.float32)
    px = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, true_K, vis.Z)[0].reshape(-1, 2)
    centers = {i: px[k] for k, i in enumerate(ids)}
    vis._update_homography(centers)
    vis.proj = dict(centers)
    return vis, centers


def test_homography_maps_the_markers_onto_themselves():
    vis, centers = scene_with_homography()
    assert vis.H is not None
    for mid, uv in centers.items():
        got = vis.pixel_to_table_h(uv[0], uv[1])
        want = vis.markers[mid][:2]
        assert math.hypot(got[0] - want[0], got[1] - want[1]) < 0.5, f"marker {mid}: {got} vs {want}"


def test_homography_needs_all_four_markers():
    vis, centers = scene_with_homography()
    vis._update_homography({k: v for k, v in centers.items() if k != 2})
    assert vis.H is None
    assert vis.pixel_to_table_h(500, 500) is None


def test_two_mappings_agree_when_the_intrinsics_are_right():
    """With the real camera matrix the raycast and the homography must coincide."""
    vis, _ = scene_with_homography()
    chk = vis.intrinsics_check()
    assert chk is not None and chk["n"] >= 20
    assert chk["worst"] < 1.0, f"mappings disagree by {chk['worst']} mm with correct intrinsics"


def test_the_check_actually_detects_a_wrong_focal_length():
    """Guards against a vacuous test: a 10% focal error must show up."""
    good = scene_with_homography(1.0)[0].intrinsics_check()["worst"]
    bad = scene_with_homography(1.10)[0].intrinsics_check()["worst"]
    assert bad > 10.0, f"a 10% focal error only moved the check to {bad} mm - no detecting power"
    assert bad > good * 10, f"good {good} mm vs bad {bad} mm - not discriminating"


def test_homography_cannot_detect_a_bad_marker_model():
    """The point of keeping PnP: 4 points fit a homography exactly however wrong they are,
    so it reports nothing. This is why the reprojection error was the only warning."""
    vis, centers = scene_with_homography()
    vis.markers = {k: [v[0] * 1.3, v[1] * 0.7, v[2]] for k, v in vis.markers.items()}
    vis._update_homography(centers)
    for mid, uv in centers.items():
        got = vis.pixel_to_table_h(uv[0], uv[1])
        want = vis.markers[mid][:2]
        assert math.hypot(got[0] - want[0], got[1] - want[1]) < 0.5, \
            "a homography always fits 4 points perfectly - that is the blindness being documented"


# --------------------------------------------------- servo gain (commanded vs actual degrees)

def cfg_gain(g, idx=None):
    """Config with gain g on joint idx, or on all four arm joints when idx is None."""
    c = S.load_cfg()
    c["tool"] = {"along": 0.0, "perp": 0.0, "lateral": 0.0}
    for i, j in enumerate(c["joints"]):
        if idx is None or i == idx:
            j["gain"] = g
        else:
            j["gain"] = 1.0
    return c


def test_unit_gain_changes_nothing():
    """Default must reproduce the original kinematics exactly."""
    servo = [120.0, 70.0, 120.0, 150.0, 90.0, 90.0]
    base = S.load_cfg()
    for j in base["joints"]:
        j.pop("gain", None)                      # a config predating the gain field
    assert np.allclose(S.joint_angles(servo, base), S.joint_angles(servo, cfg_gain(1.0)))
    assert np.allclose(S.fk(servo, base)[0][-1], S.fk(servo, cfg_gain(1.0))[0][-1])


def test_gain_scales_the_actual_angle():
    # derive the servo value from the configured zero rather than hardcoding one: the base
    # zero is a measured quantity and has already moved once (90 -> 97.56)
    zero = S.load_cfg()["joints"][0]["zero"]
    servo = [zero + 30.0, 90.0, 90.0, 90.0, 90.0, 90.0]      # 30 deg commanded off zero
    full = S.joint_angles(servo, cfg_gain(1.0))[0]
    part = S.joint_angles(servo, cfg_gain(0.75))[0]
    assert abs(full - 30.0) < 1e-9, f"unit gain on 30 deg commanded should give 30, got {full}"
    assert abs(part - 22.5) < 1e-9, f"0.75 gain on 30 deg commanded should give 22.5, got {part}"


def test_an_unmodelled_gain_makes_the_arm_undershoot_the_commanded_yaw():
    """The real-world failure: a command computed in ignorance of the gain sends the arm
    somewhere else. Built from IK on a genuine forward target rather than a hand-picked
    servo list, so the pose actually reaches over the table instead of folding behind
    the base - where planar reach goes negative and azimuth wraps by 180 degrees.
    """
    want = (180.0, 120.0, 60.0, -80.0)      # verified cleanly reachable
    naive_cfg = cfg_gain(1.0)
    servo, reach, lim = S.ik(*want, naive_cfg, 90.0)
    assert not (reach or lim), "test target must be cleanly reachable"
    cmd = list(servo) + [90.0, 90.0]

    believed = S.fk(cmd, naive_cfg)[0][-1]
    assert math.dist(believed, want[:3]) < 1.0, "FK must agree with the IK that produced it"

    actual = S.fk(cmd, cfg_gain(0.75, idx=0))[0][-1]
    r = math.hypot(actual[0], actual[1])
    assert r > 0, "pose must reach forward for the azimuth comparison to mean anything"

    # a sub-unit base gain turns the arm less than asked, so it lands short of the commanded
    # azimuth and nearer the centre line
    assert abs(actual[1]) < abs(believed[1]), f"|Y| {abs(actual[1]):.1f} vs {abs(believed[1]):.1f}"
    assert abs(math.degrees(math.atan2(actual[1], actual[0]))) < \
           abs(math.degrees(math.atan2(believed[1], believed[0])))
    assert math.dist(actual, believed) > 20.0, "a 25% yaw gain error should be plainly visible"


def test_fk_planar_keeps_the_radius_signed_behind_the_base():
    """A folded-back pose has negative planar reach; the TCP must not report it positive."""
    servo = [130.0, 70.0, 120.0, 150.0, 90.0, 90.0]
    cfg = cfg_gain(1.0)
    chain = S.fk_planar(servo, cfg)
    assert chain[4][0] < 0, "this pose should fold the L3 tip behind the base"
    assert chain[-1][0] < 0, "the TCP radius must carry the same sign as the chain"
    assert abs(chain[-1][0] - chain[4][0]) < 1e-9, "no tool offset, so TCP sits on the L3 tip"


def test_ik_inverts_the_gain():
    """IK must command a bigger angle to achieve the same pose through a lossy servo."""
    for g in (1.0, 0.9, 0.75, 0.706):
        cfg = cfg_gain(g)
        for x, y, z, pitch in TARGETS:
            servo, reach, lim = S.ik(x, y, z, pitch, cfg, 90.0)
            if reach or lim:
                continue
            got = S.fk(list(servo) + [90.0, 90.0], cfg)[0][-1]
            assert math.dist(got, (x, y, z)) < 1.0, \
                f"gain {g}: asked ({x},{y},{z}) got {tuple(round(v, 1) for v in got)}"


def test_ik_commands_a_larger_base_angle_at_lower_gain():
    want = (200.0, 150.0, 60.0, -80.0)
    hi = S.ik(*want, cfg_gain(1.0), 90.0)[0][0]
    lo = S.ik(*want, cfg_gain(0.75), 90.0)[0][0]
    zero = S.load_cfg()["joints"][0]["zero"]
    assert abs(lo - zero) > abs(hi - zero), \
        f"lower gain must need a bigger command: {lo} vs {hi} (zero {zero})"


def test_zero_gain_is_treated_as_unity_rather_than_dividing_by_zero():
    servo, reach, lim = S.ik(200.0, 100.0, 60.0, -80.0, cfg_gain(0.0), 90.0)
    assert all(math.isfinite(v) for v in servo)


def test_joint_ranges_report_actual_degrees():
    wide = S.joint_ranges(cfg_gain(1.0))[0]
    narrow = S.joint_ranges(cfg_gain(0.5))[0]
    assert abs((narrow[1] - narrow[0]) - (wide[1] - wide[0]) * 0.5) < 1e-6, \
        "halving the gain must halve the reachable angular span"


# ------------------------------------------------- servo calibration fitting (zero + gain)

import fit_servo_calibration as FC


def synth_pairs(zero, sign, gain, servos, noise=0.0, seed=0):
    """(servo, actual angle) pairs for a joint with known calibration."""
    rng = np.random.default_rng(seed)
    return [[sv, sign * (sv - zero) * gain + (rng.normal(0, noise) if noise else 0.0)]
            for sv in servos]


def test_fit_recovers_a_known_calibration():
    for zero, sign, gain in [(90.0, 1, 1.0), (90.0, 1, 0.75), (89.5, -1, 0.82), (180.0, -1, 0.706)]:
        pairs = synth_pairs(zero, sign, gain, [40, 70, 90, 120, 150])
        got, err = FC.fit_joint("shoulder", pairs)
        assert err is None, err
        f = got["fit"]
        assert abs(f["zero"] - zero) < 0.05, f"zero {f['zero']} vs {zero}"
        assert f["sign"] == sign
        assert abs(f["gain"] - gain) < 0.001, f"gain {f['gain']} vs {gain}"
        assert got["r2"] > 0.9999


def test_fit_separates_zero_from_gain():
    """The point of a two-parameter fit: a zero error and a gain error look alike at one
    operating point and only separate across a range."""
    pairs = synth_pairs(75.0, 1, 0.75, [40, 70, 90, 120, 150])
    got, _ = FC.fit_joint("shoulder", pairs)
    assert abs(got["fit"]["zero"] - 75.0) < 0.05
    assert abs(got["fit"]["gain"] - 0.75) < 0.001


def test_fit_survives_realistic_measurement_noise():
    pairs = synth_pairs(90.0, 1, 0.75, [40, 60, 90, 120, 150], noise=0.4, seed=5)
    got, _ = FC.fit_joint("shoulder", pairs)
    assert abs(got["fit"]["gain"] - 0.75) < 0.02
    assert abs(got["fit"]["zero"] - 90.0) < 1.5


def test_fit_flags_an_outlier_instead_of_swallowing_it():
    pairs = synth_pairs(90.0, 1, 0.75, [40, 70, 90, 120, 150])
    pairs[2][1] += 9.0                      # one badly misread gauge
    got, _ = FC.fit_joint("shoulder", pairs)
    assert got["notes"], "a 9 degree outlier must produce a warning"
    assert any("residual" in n or "r2" in n for n in got["notes"])


def test_plate_tilt_is_subtracted_from_pitch_joints_only():
    """Gravity-referenced readings carry the desk slope; joint angles are plate-relative."""
    pairs = synth_pairs(90.0, 1, 0.75, [40, 70, 90, 120, 150])
    tilted = [[sv, a + 6.0] for sv, a in pairs]

    pitch, _ = FC.fit_joint("shoulder", tilted, plate_tilt=6.0)
    assert abs(pitch["fit"]["zero"] - 90.0) < 0.05, "tilt should be removed for a pitch joint"

    base, _ = FC.fit_joint("base", tilted, plate_tilt=6.0)
    assert abs(base["fit"]["zero"] - 90.0) > 1.0, \
        "base yaw is not gravity-referenced, so plate tilt must NOT be applied to it"


def test_uncorrected_plate_tilt_corrupts_the_zero():
    """Documents the trap the --plate-tilt flag exists to avoid."""
    pairs = synth_pairs(90.0, 1, 0.75, [40, 70, 90, 120, 150])
    tilted = [[sv, a + 6.0] for sv, a in pairs]
    got, _ = FC.fit_joint("shoulder", tilted, plate_tilt=0.0)
    off = abs(got["fit"]["zero"] - 90.0)
    assert off > 5.0, f"ignoring a 6 deg plate tilt should shift the zero, moved {off:.2f}"
    assert abs(got["fit"]["gain"] - 0.75) < 0.001, "a pure offset must not disturb the gain"


def test_fit_refuses_too_few_points():
    got, err = FC.fit_joint("elbow", [[90, 0.0]])
    assert got is None and "at least 2" in err


def test_fit_refuses_a_joint_that_did_not_move():
    got, err = FC.fit_joint("elbow", [[90, 5.0], [90, 5.0], [90, 5.0]])
    assert got is None and err


def test_marker_azimuths_match_the_shipped_marker_model():
    """The base can be calibrated off the markers, so those azimuths must stay in step."""
    vis = V.Vision(".", 0, lambda *a, **k: None)
    for mid, az in FC.MARKER_AZIMUTH.items():
        x, y = vis.markers[mid][:2]
        assert abs(math.degrees(math.atan2(y, x)) - az) < 0.05, \
            f"marker {mid}: vision.json says {math.degrees(math.atan2(y, x)):.2f}, "\
            f"fit_servo_calibration says {az}"


# ------------------------- calibrating from ruler measurements instead of an angle gauge

def axes_from_servo(servo, cfg):
    """The four axis positions a ruler would read, in (forward, height) table coordinates."""
    planar = S.fk_planar(servo, cfg)
    return planar[1], planar[2], planar[3], planar[4]      # shoulder, elbow, wrist, L3 tip


def test_axis_geometry_round_trips_against_fk():
    """Angles derived from axis positions must equal the ones fk() was built from."""
    cfg = cfg_gain(1.0)
    for servo in ([90.0, 70.0, 120.0, 150.0, 90.0, 90.0],
                  [90.0, 110.0, 60.0, 120.0, 90.0, 90.0],
                  [90.0, 140.0, 40.0, 100.0, 90.0, 90.0]):
        got = FC.angles_from_axes(*axes_from_servo(servo, cfg))
        want = S.joint_angles(servo, cfg)
        for name, idx in (("shoulder", 1), ("elbow", 2), ("wrist pitch", 3)):
            assert abs(got[name] - want[idx]) < 1e-6, \
                f"{name}: {got[name]} vs {want[idx]}"


def test_axis_geometry_recovers_the_link_lengths():
    cfg = cfg_gain(1.0)
    got = FC.angles_from_axes(*axes_from_servo([90.0, 70.0, 120.0, 150.0, 90.0, 90.0], cfg))
    for k in ("L1", "L2", "L3"):
        assert abs(got["links"][k] - cfg["links"][k]) < 1e-6, k
    assert abs(got["L0"] - cfg["links"]["L0"]) < 1e-6


def test_axis_method_recovers_zero_and_gain_through_the_full_fit():
    """End to end: a rig with known zeros and gains, measured only as axis positions."""
    truth = {"shoulder": (82.0, 0.76), "elbow": (89.5, 0.74), "wrist pitch": (180.0, 0.75)}
    cfg = S.load_cfg()
    for j in cfg["joints"]:
        t = truth.get(j["name"])
        if t:
            j["zero"], j["gain"] = t
        else:
            j["gain"] = 1.0

    poses = []
    for sv in ([90.0, 70.0, 120.0, 150.0, 90.0, 90.0], [90.0, 90.0, 90.0, 150.0, 90.0, 90.0],
               [90.0, 110.0, 60.0, 120.0, 90.0, 90.0], [90.0, 140.0, 40.0, 100.0, 90.0, 90.0]):
        sh, el, wr, tip = axes_from_servo(sv, cfg)
        poses.append({"servo": sv[:4], "elbow": list(el), "wrist": list(wr), "tip": list(tip)})

    sh0 = axes_from_servo([90.0] * 6, cfg)[0]
    pairs, links = FC.pairs_from_axis_poses(poses, list(sh0))

    for name, (z, g) in truth.items():
        got, err = FC.fit_joint(name, pairs[name])
        assert err is None, err
        assert abs(got["fit"]["zero"] - z) < 0.05, f"{name} zero {got['fit']['zero']} vs {z}"
        assert abs(got["fit"]["gain"] - g) < 0.002, f"{name} gain {got['fit']['gain']} vs {g}"

    for k in ("L1", "L2", "L3"):
        assert max(links[k]) - min(links[k]) < 1e-6, f"{k} should be constant across poses"
        assert abs(sum(links[k]) / len(links[k]) - cfg["links"][k]) < 1e-6


def test_axis_method_needs_no_plate_tilt_correction():
    """The payoff: table-frame measurements are plate-relative, so the desk slope cancels."""
    cfg = cfg_gain(1.0)
    sh, el, wr, tip = axes_from_servo([90.0, 70.0, 120.0, 150.0, 90.0, 90.0], cfg)
    plain = FC.angles_from_axes(sh, el, wr, tip)
    # the same physical arm on a 4.39 deg slope: a ruler referenced to the TABLE reads the
    # identical numbers, because the table tilted along with the arm
    assert abs(plain["shoulder"] - S.joint_angles([90.0, 70.0, 120.0, 150.0], cfg)[1]) < 1e-6


def test_pairs_from_axis_poses_rejects_malformed_input():
    try:
        FC.pairs_from_axis_poses([{"servo": [90, 70, 120, 150], "elbow": [0, 0]}], [0, 83])
    except ValueError as e:
        assert "malformed" in str(e)
    else:
        raise AssertionError("a pose missing wrist/tip should raise")


def test_plate_tilt_would_corrupt_axis_derived_angles():
    """Why the script refuses to apply the tilt to axis poses: it would double-correct."""
    cfg = cfg_gain(1.0)
    poses = []
    for sv in ([90.0, 70.0, 120.0, 150.0, 90.0, 90.0], [90.0, 110.0, 60.0, 120.0, 90.0, 90.0],
               [90.0, 140.0, 40.0, 100.0, 90.0, 90.0]):
        sh, el, wr, tip = axes_from_servo(sv, cfg)
        poses.append({"servo": sv[:4], "elbow": list(el), "wrist": list(wr), "tip": list(tip)})
    sh0 = axes_from_servo([90.0] * 6, cfg)[0]
    pairs, _ = FC.pairs_from_axis_poses(poses, list(sh0))

    right, _ = FC.fit_joint("shoulder", pairs["shoulder"], plate_tilt=0.0)
    wrong, _ = FC.fit_joint("shoulder", pairs["shoulder"], plate_tilt=4.39)
    assert abs(right["fit"]["zero"] - S.load_cfg()["joints"][1]["zero"]) < 0.05
    assert abs(wrong["fit"]["zero"] - right["fit"]["zero"]) > 3.0, \
        "applying a tilt to already-plate-relative data must visibly shift the zero"


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
