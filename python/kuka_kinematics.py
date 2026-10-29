"""Python port of the KUKA forward and inverse kinematics used for drawing.

Mirrors the MATLAB code in ../matlab/kuka (forward_kuka.m, inverse_kuka.m,
FrameTransformation.m) so the math can be checked without MATLAB.

Usage (from the repository root):
    pip install -r python/requirements.txt
    python python/kuka_kinematics.py

It solves inverse kinematics for every point of the jug drawing, checks each
solution with forward kinematics, and saves a 3D plot of the arm's DH frames.
"""

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
DESIGN = ROOT / "matlab" / "kuka" / "designs" / "jug.xlsx"
FIGURE = ROOT / "docs" / "images" / "kuka_model_3d.png"

# Tool offset corrections found by calibration (calibrate_kuka.m): [delta_a6, delta_d6] in mm
CALIBRATION_DELTA = (1.2958, 1.6840)


def dh_table(delta=CALIBRATION_DELTA):
    """KUKA DH parameters as rows of (d, a, alpha) in mm and rad, matching mykuka_search.m."""
    a6 = 296.23 + delta[0]
    d6 = 161.44 + delta[1]
    return np.array([
        [400.0, 25.0, np.pi / 2],
        [0.0, 315.0, 0.0],
        [0.0, 35.0, np.pi / 2],
        [365.0, 0.0, -np.pi / 2],
        [0.0, 0.0, np.pi / 2],
        [d6, -a6, 0.0],
    ])


def link_transform(d, a, alpha, theta):
    """Homogeneous transform A_i = Rot_z(theta) Trans_z(d) Trans_x(a) Rot_x(alpha)."""
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1],
    ])


def forward_kuka(q, dh, return_all=False):
    """End effector pose H_6^0 (or every intermediate frame if return_all)."""
    H = np.eye(4)
    frames = [H]
    for (d, a, alpha), theta in zip(dh, q):
        H = H @ link_transform(d, a, alpha, theta)
        frames.append(H)
    return frames if return_all else H


def inverse_kuka(H, dh):
    """Elbow-up inverse kinematics by kinematic decoupling (port of inverse_kuka.m)."""
    d1, a1 = dh[0, 0], dh[0, 1]
    a2 = dh[1, 1]
    a3 = dh[2, 1]
    d4 = dh[3, 0]
    d6, a6 = dh[5, 0], dh[5, 1]
    R_d, o_d = H[:3, :3], H[:3, 3]

    # 1. Wrist centre
    xc, yc, zc = o_d - R_d @ np.array([a6, 0.0, d6])

    # 2. Arm joints
    theta1 = np.arctan2(yc, xc)
    gamma = np.arctan2(a3, d4)
    r = np.hypot(xc, yc)
    L = np.hypot(d4, a3)
    D = ((r - a1) ** 2 + (zc - d1) ** 2 - a2 ** 2 - d4 ** 2 - a3 ** 2) / (2 * a2 * L)
    theta3 = np.arctan2(D, np.sqrt(max(0.0, 1 - D ** 2))) - gamma
    phi = theta3 - np.pi / 2 + gamma
    theta2 = np.arctan2(zc - d1, r - a1) - np.arctan2(L * np.sin(phi), a2 + L * np.cos(phi))

    # 3. Wrist joints from R_6^3 = (R_3^0)^T R_d
    H30 = np.eye(4)
    for (d, a, alpha), theta in zip(dh[:3], (theta1, theta2, theta3)):
        H30 = H30 @ link_transform(d, a, alpha, theta)
    R63 = H30[:3, :3].T @ R_d
    theta4 = np.arctan2(R63[1, 2], R63[0, 2])
    theta5 = np.arctan2(np.sqrt(max(0.0, 1 - R63[2, 2] ** 2)), R63[2, 2])
    theta6 = np.arctan2(R63[2, 1], -R63[2, 0])
    return np.array([theta1, theta2, theta3, theta4, theta5, theta6])


# Paper plane from the three calibration points (FrameTransformation.m)
CAL_POINTS = np.array([
    [675.71, 179.56, 28.02],
    [681.04, -148.32, 24.14],
    [620.14, -319.63, 25.31],
])


def paper_to_base():
    """Return a function mapping paper-frame points to robot base-frame points."""
    plane = np.linalg.pinv(CAL_POINTS) @ np.ones(3)  # a x + b y + c z = 1
    v2 = plane / np.linalg.norm(plane)
    v1 = np.array([0.0, 0.0, 1.0])
    axis = np.cross(v1, v2)
    s, c = np.linalg.norm(axis), v1 @ v2
    if s < 1e-12:
        R = np.eye(3)
    else:
        k = axis / s
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + s * K + (1 - c) * K @ K
    origin = np.array([0.0, 0.0, 1.0 / plane[2]])
    return lambda p: R @ p + origin


# Pencil pointing straight down onto the paper
R_DRAW = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]], dtype=float)


def main():
    dh = dh_table()
    to_base = paper_to_base()

    xy = pd.read_excel(DESIGN, header=None).dropna().to_numpy(dtype=float)
    paper = np.c_[550 + 10 * xy[:, 0], 10 * xy[:, 1], np.full(len(xy), -3.0)]
    targets = np.array([to_base(p) for p in paper])

    errors, joints = [], []
    for p in targets:
        H = np.eye(4)
        H[:3, :3], H[:3, 3] = R_DRAW, p
        q = inverse_kuka(H, dh)
        joints.append(q)
        errors.append(np.linalg.norm(forward_kuka(q, dh)[:3, 3] - p))
    print(f"Jug design: {len(targets)} points")
    print(f"Max FK(IK) position error: {max(errors):.2e} mm")

    # 3D plot of the DH frames at one drawing pose
    frames = forward_kuka(joints[len(joints) // 3], dh, return_all=True)
    origins = np.array([f[:3, 3] for f in frames])
    fig = plt.figure(figsize=(5.2, 4.6))
    ax = fig.add_subplot(projection="3d")
    xs, ys = np.meshgrid(np.linspace(450, 800, 2), np.linspace(-250, 250, 2))
    ax.plot_surface(xs, ys, np.full_like(xs, targets[:, 2].mean()), color="#eeeeee", alpha=0.6)
    ax.plot(*targets.T, color="#c0392b", lw=2.6, label="Jug drawing path")
    ax.plot(*origins.T, "-o", color="#2c3e50", lw=3, ms=5, mfc="#f39c12", label="Joint frames (DH)")
    for f in frames[1:]:
        for col, colour in zip(range(3), ["#e74c3c", "#27ae60", "#2980b9"]):
            ax.quiver(*f[:3, 3], *(f[:3, col] * 60), color=colour, lw=1)
    ax.plot([0], [0], [0], "ks", ms=7)
    ax.set_xlim(-50, 750)
    ax.set_ylim(-400, 400)
    ax.set_zlim(0, 800)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(22, -58)
    ax.set_xlabel("x (mm)", fontsize=8)
    ax.set_ylabel("y (mm)", fontsize=8)
    ax.set_zlabel("z (mm)", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.legend(fontsize=8, loc="upper left", frameon=False)
    fig.tight_layout()
    FIGURE.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIGURE, dpi=200)
    print(f"Saved {FIGURE.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
