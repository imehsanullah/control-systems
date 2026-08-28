"""Record one simulation cycle for each drone/controller pair."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import imageio.v2 as imageio
import mujoco
import numpy as np

from controllers import CONTROLLERS, FigureEightTrajectory
from main import DRONES


OUTPUT = Path(__file__).parent / "gifs"
FPS = 8


def add_path(scene, trajectory, base_height, point_size, target):
    rotation = np.eye(3).reshape(-1)
    offset = np.array([0.0, 0.0, base_height])
    for point in trajectory.display_points(90):
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.full(3, point_size),
            point + offset,
            rotation,
            np.array([1.0, 0.72, 0.0, 1.0], dtype=np.float32),
        )
        scene.ngeom += 1

    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.full(3, 2.0 * point_size),
        target + offset,
        rotation,
        np.array([0.1, 1.0, 0.25, 0.95], dtype=np.float32),
    )
    scene.ngeom += 1


def record(drone_name: str, controller_name: str) -> None:
    config = DRONES[drone_name]
    model = mujoco.MjModel.from_xml_path(
        str(ROOT / "models" / str(config["file"]))
    )
    data = mujoco.MjData(model)
    trajectory = FigureEightTrajectory(
        float(config["radius"]), float(config["height"])
    )
    controller = CONTROLLERS[controller_name]()

    body_name = "skydio_x2" if drone_name == "skydio" else "crazyflie_2"
    body_id = model.body(body_name).id
    mass = float(model.body_subtreemass[body_id])
    base_height = float(model.body_pos[body_id, 2])
    gravity = np.array([0.0, 0.0, -model.opt.gravity[2]])

    camera = mujoco.MjvCamera()
    camera.azimuth = 135
    camera.elevation = -22
    camera.distance = float(config["camera_distance"])
    camera.lookat[:] = (0.0, 0.0, float(config["height"]) / 2.0)

    frames = []
    acceleration = np.zeros(3)
    control_stride = round(controller.dt / model.opt.timestep)
    render_stride = round(1.0 / (FPS * model.opt.timestep))
    total_steps = round(trajectory.cycle_time / model.opt.timestep)

    with mujoco.Renderer(model, height=300, width=400) as renderer:
        for step in range(total_steps):
            time_s = step * model.opt.timestep
            if step % control_stride == 0:
                state = np.concatenate((data.qpos[:3].copy(), data.qvel[:3].copy()))
                acceleration = controller.control(state, time_s, trajectory)
            data.ctrl[:3] = mass * (acceleration + gravity)
            mujoco.mj_step(model, data)

            if step % render_stride == 0:
                renderer.update_scene(data, camera=camera)
                add_path(
                    renderer.scene,
                    trajectory,
                    base_height,
                    float(config["path_point_size"]),
                    trajectory.position(time_s),
                )
                frames.append(renderer.render().copy())

    OUTPUT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT / f"{drone_name}_{controller_name}.gif"
    imageio.mimsave(
        path,
        frames,
        duration=1000 / FPS,
        loop=0,
        palettesize=64,
        subrectangles=True,
    )
    print(path)


if __name__ == "__main__":
    for drone in DRONES:
        for controller_name in CONTROLLERS:
            record(drone, controller_name)
