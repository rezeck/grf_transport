#!/usr/bin/env python3
"""Transporte da caixa no Genesis, sem ROS e sem Gazebo.

Cada HeRo roda o controlador GRF de src/grf_colletive_transport.cpp (ver
grf.py): sente só o que o laser de 0,5 m alcança e amostra a velocidade por
Metropolis-Hastings. Torque, atrito e contato seguem o URDF.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import grf  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
HERO_COMMON = Path(
    os.environ.get("HERO_COMMON", REPO.parent / "hero_common")
).resolve()

PACKAGES = {
    "grf_transport": REPO,
    "hero_description": HERO_COMMON / "hero_description",
    "hero_gazebo": HERO_COMMON / "hero_gazebo",
}

ROBOTS = (
    ("hero_0", (-0.3148, -0.3163, 0.08)),
    ("hero_1", (-0.1150, -0.3162, 0.08)),
    ("hero_2", (0.0845, -0.3162, 0.08)),
    ("hero_3", (0.2845, -0.3162, 0.08)),
    ("hero_4", (-0.1472, -0.1162, 0.08)),
    ("hero_5", (0.0536, -0.1162, 0.08)),
    ("hero_6", (0.2520, -0.1162, 0.08)),
    ("hero_7", (-0.1797, 0.0838, 0.08)),
    ("hero_8", (0.0195, 0.0838, 0.08)),
    ("hero_9", (0.2205, 0.0838, 0.08)),
)

BOX_POS = (0.0, 1.1, 0.28)
TARGET_POS = (0.0, -1.2, 0.04)
GOAL = np.array([TARGET_POS[0], TARGET_POS[1]])

# Prisma do STL: cubo de 0.4 m. Inércia do URDF (0.0017) é de um corpo
# menor e faz a caixa tombar com qualquer empurrão.
BOX_SIZE = 0.4
BOX_MASS = 0.3
BOX_INERTIA = BOX_MASS * (BOX_SIZE**2 + BOX_SIZE**2) / 12.0

WHEEL_RADIUS = 0.025
WHEEL_ARMATURE = 1e-5
WHEEL_TRACK = 0.0646
# Torque abaixo do effort 0.1 N·m do URDF.
WHEEL_TORQUE = 0.05

BOX_FRICTION = 0.35
ARENA_FRICTION = 0.9
BASE_FRICTION = 0.02
WHEEL_FRICTION = 1.0

# Igual ao C++: o controlador para quando a caixa chega a 0,1 m do alvo.
ARRIVE_TOL = 0.10
# O C++ roda a 10 Hz; o passo do Genesis é 0,01 s.
CONTROL_EVERY = 10
VIEWER_HOLD_STEPS = 3000


def rewrite_packages(text: str, src: Path) -> str:
    def replace(match: re.Match[str]) -> str:
        pkg, rel = match.group(1), match.group(2)
        root = PACKAGES.get(pkg)
        if root is None:
            raise SystemExit(f"Pacote URDF desconhecido: {pkg} em {src}")
        path = (root / rel).resolve()
        if not path.exists():
            raise SystemExit(f"Mesh ausente: {path}")
        return path.as_posix()

    return re.sub(r"package://([^/]+)/([^\s\"']+)", replace, text)


def write_xml(root: ET.Element, dest: Path) -> Path:
    dest.write_text(ET.tostring(root, encoding="unicode"))
    return dest


def prepare_hero(src: Path, dest: Path) -> Path:
    """Tira a colisão do chapéu. No Gazebo ela tem maxContacts 0."""
    root = ET.fromstring(rewrite_packages(src.read_text(), src))
    for link in root.findall("link"):
        if link.get("name") == "hat_link":
            for collision in list(link.findall("collision")):
                link.remove(collision)
    return write_xml(root, dest)


def prepare_box(src: Path, dest: Path) -> Path:
    """Troca a malha aberta por um cubo e corrige a inércia."""
    root = ET.fromstring(rewrite_packages(src.read_text(), src))
    link = root.find("link")
    for collision in list(link.findall("collision")):
        link.remove(collision)
    collision = ET.SubElement(link, "collision")
    origin = ET.SubElement(collision, "origin")
    origin.set("xyz", "0 0 0")
    origin.set("rpy", "0 0 0")
    box = ET.SubElement(ET.SubElement(collision, "geometry"), "box")
    box.set("size", f"{BOX_SIZE} {BOX_SIZE} {BOX_SIZE}")
    inertia = link.find("inertial").find("inertia")
    for axis in ("ixx", "iyy", "izz"):
        inertia.set(axis, f"{BOX_INERTIA:.6f}")
    for axis in ("ixy", "ixz", "iyz"):
        inertia.set(axis, "0")
    return write_xml(root, dest)


def prepare_visual(src: Path, dest: Path) -> Path:
    root = ET.fromstring(rewrite_packages(src.read_text(), src))
    return write_xml(root, dest)


def as_vec(value, n: int) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    return np.asarray(value, dtype=float).reshape(-1)[:n]


def yaw_of(entity) -> float:
    w, x, y, z = as_vec(entity.get_quat(), 4)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def wheel_speed(linear: float, angular: float) -> np.ndarray:
    half = angular * WHEEL_TRACK / 2.0
    return np.array(
        [(linear - half) / WHEEL_RADIUS, (linear + half) / WHEEL_RADIUS]
    )


def command_drive(robot, dofs, velocity: np.ndarray) -> None:
    """Holonômico para diferencial, como no C++: v = |u|, w = 2 * erro de rumo."""
    speed = min(float(np.linalg.norm(velocity)), grf.VMAX)
    if speed < 1e-6:
        robot.control_dofs_velocity(np.zeros(2), dofs)
        return
    err = wrap(math.atan2(velocity[1], velocity[0]) - yaw_of(robot))
    robot.control_dofs_velocity(wheel_speed(speed, 2.0 * err), dofs)


def build_scene(show_viewer: bool, tmp_path: Path):
    import genesis as gs

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.01, gravity=(0.0, 0.0, -9.81)),
        rigid_options=gs.options.RigidOptions(
            enable_collision=True,
            enable_self_collision=False,
            enable_joint_limit=False,
            # O cone padrão transforma atrito em força normal e a caixa decola.
            friction_cone=gs.friction_cone.elliptic,
            contact_resolution=gs.contact_resolution.signorini,
            impratio=5.0,
            constraint_timeconst=0.02,
            max_collision_pairs=4000,
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(0.1, -3.4, 2.6),
            camera_lookat=(0.0, 0.0, 0.1),
            camera_fov=40,
        ),
        show_viewer=show_viewer,
    )

    hero_urdf = prepare_hero(
        HERO_COMMON / "hero_description" / "robot" / "hero_light.urdf",
        tmp_path / "hero.urdf",
    )
    box_urdf = prepare_box(
        REPO / "models" / "rectangular_prism" / "rectangular_prism.urdf",
        tmp_path / "box.urdf",
    )
    target_urdf = prepare_visual(
        REPO / "models" / "rectangular_prism" / "rectangular_prism_target.urdf",
        tmp_path / "target.urdf",
    )
    arena_urdf = prepare_visual(
        HERO_COMMON / "hero_gazebo" / "worlds" / "models" / "arena_0" / "arena.urdf",
        tmp_path / "arena.urdf",
    )
    # Piso da malha em z=0.03. Um plano em z=0 deixa o robô dentro da laje.
    arena = scene.add_entity(
        gs.morphs.URDF(
            file=str(arena_urdf),
            fixed=True,
            collision=True,
            convexify=False,
            decimate=False,
        )
    )
    box = scene.add_entity(
        gs.morphs.URDF(file=str(box_urdf), pos=BOX_POS, fixed=False, convexify=False)
    )
    scene.add_entity(
        gs.morphs.URDF(
            file=str(target_urdf), pos=TARGET_POS, fixed=True, collision=False
        )
    )
    robots = [
        scene.add_entity(
            gs.morphs.URDF(file=str(hero_urdf), pos=pos, fixed=False, convexify=False)
        )
        for _name, pos in ROBOTS
    ]
    scene.build()
    return scene, arena, box, robots


def configure_contacts(arena, box, robots) -> list[list[int]]:
    """Atrito e motores das rodas. Devolve os índices dos dois dofs de cada robô."""
    box.get_link("object_link").set_friction(BOX_FRICTION)
    arena.get_link("arena_link").set_friction(ARENA_FRICTION)
    torque = np.full(2, WHEEL_TORQUE)
    wheel_idx = []
    for robot in robots:
        robot.get_link("base_link").set_friction(BASE_FRICTION)
        robot.get_link("left_wheel_link").set_friction(WHEEL_FRICTION)
        robot.get_link("right_wheel_link").set_friction(WHEEL_FRICTION)
        dofs = [
            robot.get_joint("left_wheel_joint").dofs_idx_local[0],
            robot.get_joint("right_wheel_joint").dofs_idx_local[0],
        ]
        wheel_idx.append(dofs)
        # O Genesis põe armature 0,1 em toda junta, o que deixa a roda ~10^4
        # vezes mais pesada que a do HeRo (3e-6 kg·m²) e o robô não acelera.
        robot.set_dofs_armature(np.full(2, WHEEL_ARMATURE), dofs)
        robot.set_dofs_kp(np.full(2, 4.0), dofs)
        robot.set_dofs_kv(np.full(2, 0.08), dofs)
        robot.set_dofs_force_range(-torque, torque, dofs)
        robot.control_dofs_velocity(np.zeros(2), dofs)
    return wheel_idx


def run_transport(scene, box, robots, wheel_idx, steps_settle: int, steps: int) -> bool:
    """Roda o GRF até a caixa chegar a ARRIVE_TOL do alvo. True se chegou."""
    zero = np.zeros(2)
    for _ in range(steps_settle):
        scene.step()

    controller = grf.GRFController(GOAL)
    agents = [
        grf.Agent(as_vec(robot.get_pos(), 3)[:2], yaw_of(robot)) for robot in robots
    ]
    settled = as_vec(box.get_pos(), 3)
    print(f"caixa após assentar: x={settled[0]:.3f} y={settled[1]:.3f} z={settled[2]:.3f}")
    robot_z = [float(as_vec(robot.get_pos(), 3)[2]) for robot in robots]
    print(f"robôs no piso: z={min(robot_z):.3f}..{max(robot_z):.3f}")
    max_box_z = float(settled[2])
    min_robot_z, max_robot_z = min(robot_z), max(robot_z)

    arrived = False
    for step in range(steps):
        box_xyz = as_vec(box.get_pos(), 3)
        max_box_z = max(max_box_z, float(box_xyz[2]))
        distance = float(np.linalg.norm(GOAL - box_xyz[:2]))
        if distance < ARRIVE_TOL:
            arrived = True
            break
        for robot in robots:
            z = float(as_vec(robot.get_pos(), 3)[2])
            min_robot_z, max_robot_z = min(min_robot_z, z), max(max_robot_z, z)
        if step % CONTROL_EVERY == 0:
            for agent, robot in zip(agents, robots):
                agent.position = as_vec(robot.get_pos(), 3)[:2]
                agent.theta = yaw_of(robot)
            velocities = controller.step(agents, box_xyz[:2], yaw_of(box))
            for robot, dofs, velocity in zip(robots, wheel_idx, velocities):
                command_drive(robot, dofs, velocity)
        scene.step()
        if step % 1000 == 999:
            print(
                f"t={0.01 * (step + 1):.0f}s caixa x={box_xyz[0]:.3f} "
                f"y={box_xyz[1]:.3f} z={box_xyz[2]:.3f} alvo={distance:.3f} m"
            )

    for robot, dofs in zip(robots, wheel_idx):
        robot.control_dofs_velocity(zero, dofs)
    final = as_vec(box.get_pos(), 3)
    end_goal = float(np.linalg.norm(final[:2] - GOAL))
    print(
        f"caixa no fim: x={final[0]:.3f} y={final[1]:.3f} z={final[2]:.3f} "
        f"alvo={end_goal:.3f} m"
    )
    print(f"z caixa={max_box_z:.3f} robô={min_robot_z:.3f}..{max_robot_z:.3f}")
    if min_robot_z < 0.03:
        raise SystemExit("Os robôs atravessaram o piso da arena.")
    if max_box_z > settled[2] + 0.05:
        raise SystemExit("A caixa levantou.")
    if max_robot_z > 0.12:
        raise SystemExit("Os robôs saltaram.")
    if not arrived:
        print(f"caixa não chegou: faltam {end_goal:.3f} m (tolerância {ARRIVE_TOL} m)")
    return arrived


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps-settle", type=int, default=200)
    parser.add_argument("--steps", type=int, default=30000)
    args = parser.parse_args()

    if not (HERO_COMMON / "hero_description").is_dir():
        raise SystemExit(f"hero_common não encontrado em {HERO_COMMON}")

    import genesis as gs

    # O cone elíptico no Metal fica lento demais para esta cena.
    gs.init(backend=gs.cpu, logging_level="warning")

    with tempfile.TemporaryDirectory(prefix="grf-genesis-") as tmp:
        scene, arena, box, robots = build_scene(not args.headless, Path(tmp))
        wheel_idx = configure_contacts(arena, box, robots)
        arrived = run_transport(scene, box, robots, wheel_idx, args.steps_settle, args.steps)
        print("transporte ok" if arrived else "transporte incompleto")
        if not args.headless:
            for _ in range(VIEWER_HOLD_STEPS):
                scene.step()
    raise SystemExit(0 if arrived else 1)


if __name__ == "__main__":
    main()
