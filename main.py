"""Follow a 3D reference trajectory with PID, LQR, or MPC."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from controllers import CONTROLLERS, FigureEightTrajectory


DRONES = {
    "skydio": {
        "file": "skydio_x2.xml",
        "radius": 2.0,
        "height": 1.4,
        "camera_distance": 6.2,
        "path_point_size": 0.012,
    },
    "crazyflie": {
        "file": "crazyflie_2.xml",
        "radius": 0.8,
        "height": 0.75,
        "camera_distance": 2.8,
        "path_point_size": 0.004,
    },
}


def choose_drone() -> str:
    print("Choose a drone:")
    print("  1. Skydio X2")
    print("  2. Bitcraze Crazyflie 2")
    choice = input("Selection [1]: ").strip() or "1"
    if choice not in {"1", "2"}:
        raise SystemExit("Selection must be 1 or 2.")
    return "skydio" if choice == "1" else "crazyflie"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone", choices=DRONES)
    parser.add_argument("--controller", choices=CONTROLLERS, default="mpc")
    return parser.parse_args()


def add_path_to_viewer(
    viewer: mujoco.viewer.Handle,
    trajectory: FigureEightTrajectory,
    base_height: float,
    point_size: float,
):
    """Draw yellow reference points and return a movable green target marker."""
    viewer.user_scn.ngeom = 0
    rotation = np.eye(3).reshape(-1)
    offset = np.array([0.0, 0.0, base_height])

    for point in trajectory.display_points():
        geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.full(3, point_size),
            point + offset,
            rotation,
            np.array([1.0, 0.65, 0.05, 0.55], dtype=np.float32),
        )
        viewer.user_scn.ngeom += 1

    target = viewer.user_scn.geoms[viewer.user_scn.ngeom]
    mujoco.mjv_initGeom(
        target,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.full(3, point_size * 2.0),
        offset,
        rotation,
        np.array([0.1, 1.0, 0.25, 0.9], dtype=np.float32),
    )
    viewer.user_scn.ngeom += 1
    return target


def main() -> None:
    args = parse_args()
    drone_name = args.drone or choose_drone()
    config = DRONES[drone_name]
    model_path = Path(__file__).parent / "models" / str(config["file"])

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    trajectory = FigureEightTrajectory(
        float(config["radius"]), float(config["height"])
    )
    controller = CONTROLLERS[args.controller]()
    print(f"Controller: {args.controller.upper()}")

    body_name = "skydio_x2" if drone_name == "skydio" else "crazyflie_2"
    drone_body = model.body(body_name)
    mass = float(model.body_subtreemass[drone_body.id])
    base_height = float(model.body_pos[drone_body.id, 2])
    gravity_compensation = np.array([0.0, 0.0, -model.opt.gravity[2]])
    acceleration = np.zeros(3)
    next_control_time = 0.0
    started = time.monotonic()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -18
        viewer.cam.distance = float(config["camera_distance"])
        viewer.cam.lookat[:] = (0.0, 0.0, float(config["height"]) / 2.0)
        target_marker = add_path_to_viewer(
            viewer,
            trajectory,
            base_height,
            float(config["path_point_size"]),
        )

        while viewer.is_running():
            step_started = time.monotonic()
            elapsed = step_started - started

            if elapsed >= next_control_time:
                state = np.concatenate((data.qpos[:3].copy(), data.qvel[:3].copy()))
                acceleration = controller.control(state, elapsed, trajectory)
                next_control_time = elapsed + controller.dt

            # Controllers output acceleration; MuJoCo actuators need force.
            data.ctrl[:3] = mass * (acceleration + gravity_compensation)
            target_marker.pos[:] = trajectory.position(elapsed) + np.array(
                [0.0, 0.0, base_height]
            )
            mujoco.mj_step(model, data)
            viewer.sync()

            delay = model.opt.timestep - (time.monotonic() - step_started)
            if delay > 0:
                time.sleep(delay)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
