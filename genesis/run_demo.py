#!/usr/bin/env python3
"""Transporte da caixa no Genesis, sem ROS e sem Gazebo.

Os HeRo contornam a caixa e empurram devagar até o alvo (o outro lado da
arena). Torque, atrito e o contato seguem o URDF: a caixa desliza e não
decola quando encosta na parede.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

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
BOX_HALF = BOX_SIZE / 2.0

WHEEL_RADIUS = 0.025
WHEEL_TRACK = 0.0646
# v_max da UI do HeRo. Torque abaixo do effort 0.1 N·m do URDF.
V_MAX = 0.05
W_MAX = 0.40
WHEEL_TORQUE = 0.05


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


def push_slots(box_xy: np.ndarray, direction: np.ndarray) -> list[np.ndarray]:
    lateral = np.array([-direction[1], direction[0]])
    slots = []
    # A primeira fila mira um pouco dentro da face, senão para antes de encostar.
    for depth in (BOX_HALF, BOX_HALF + 0.09):
        for offset in (-0.16, -0.08, 0.0, 0.08, 0.16):
            slots.append(box_xy - direction * depth + lateral * offset)
    return slots


def assign_slots(positions: list[np.ndarray], slots: list[np.ndarray]) -> list[int]:
    remaining = set(range(len(slots)))
    chosen = []
    for pos in positions:
        slot = min(remaining, key=lambda i: float(np.linalg.norm(slots[i] - pos)))
        remaining.remove(slot)
        chosen.append(slot)
    return chosen


def lane_offsets(positions: list[np.ndarray], sides: list[float]) -> list[float]:
    """Cada robô fica numa faixa fora da caixa, para não empilharem no mesmo ponto."""
    offsets = [0.0] * len(positions)
    for side in (-1.0, 1.0):
        group = [i for i, value in enumerate(sides) if value == side]
        group.sort(key=lambda i: positions[i][0])
        for rank, index in enumerate(group):
            offsets[index] = side * (BOX_HALF + 0.12 + rank * 0.09)
    return offsets


def approach_target(
    robot_xy: np.ndarray,
    slot: np.ndarray,
    box_xy: np.ndarray,
    direction: np.ndarray,
    lane: float,
    phase: int,
) -> tuple[np.ndarray, int]:
    """Sai para a faixa, passa por trás da caixa e só então encosta para empurrar."""
    lateral = np.array([-direction[1], direction[0]])
    along = float(np.dot(robot_xy - box_xy, direction))
    rear = -(BOX_HALF + 0.1)
    if phase == 0:
        target = box_xy + lateral * lane + direction * along
        if abs(float(np.dot(robot_xy - box_xy, lateral)) - lane) < 0.05:
            phase = 1
    if phase == 1:
        target = box_xy + lateral * lane + direction * rear
        if along < rear + 0.06:
            phase = 2
    if phase == 2:
        target = slot
    return target, phase


def command_drive(robot, dofs, target_xy: np.ndarray, creep: float) -> None:
    pos = as_vec(robot.get_pos(), 3)[:2]
    error = target_xy - pos
    distance = float(np.linalg.norm(error))
    if distance < 0.03:
        robot.control_dofs_velocity(np.zeros(2), dofs)
        return
    yaw_error = wrap(math.atan2(error[1], error[0]) - yaw_of(robot))
    # Continua andando na curva. Parar para girar deixa o grupo travado.
    align = max(0.4, math.cos(min(abs(yaw_error), math.pi / 2)))
    linear = min(creep, 0.8 * distance) * align
    angular = float(np.clip(2.2 * yaw_error, -W_MAX, W_MAX))
    robot.control_dofs_velocity(wheel_speed(linear, angular), dofs)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps-settle", type=int, default=200)
    parser.add_argument("--steps", type=int, default=20000)
    args = parser.parse_args()

    if not (HERO_COMMON / "hero_description").is_dir():
        raise SystemExit(f"hero_common não encontrado em {HERO_COMMON}")

    import genesis as gs

    # O cone elíptico no Metal fica lento demais para esta cena.
    gs.init(backend=gs.cpu, logging_level="warning")

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
        show_viewer=not args.headless,
    )

    with tempfile.TemporaryDirectory(prefix="grf-genesis-") as tmp:
        tmp_path = Path(tmp)
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
                file=str(target_urdf),
                pos=TARGET_POS,
                fixed=True,
                collision=False,
            )
        )
        robots = []
        for name, pos in ROBOTS:
            robot = scene.add_entity(
                gs.morphs.URDF(
                    file=str(hero_urdf),
                    pos=pos,
                    fixed=False,
                    convexify=False,
                )
            )
            robots.append((name, robot))

        scene.build()

        box.get_link("object_link").set_friction(0.35)
        arena.get_link("arena_link").set_friction(0.9)
        wheel_idx = []
        zero = np.zeros(2)
        torque = np.full(2, WHEEL_TORQUE)
        for _name, robot in robots:
            robot.get_link("base_link").set_friction(0.02)
            robot.get_link("left_wheel_link").set_friction(1.0)
            robot.get_link("right_wheel_link").set_friction(1.0)
            dofs = [
                robot.get_joint("left_wheel_joint").dofs_idx_local[0],
                robot.get_joint("right_wheel_joint").dofs_idx_local[0],
            ]
            wheel_idx.append(dofs)
            robot.set_dofs_kp(np.full(2, 4.0), dofs)
            robot.set_dofs_kv(np.full(2, 0.08), dofs)
            robot.set_dofs_force_range(-torque, torque, dofs)
            robot.control_dofs_velocity(zero, dofs)

        for _ in range(args.steps_settle):
            scene.step()

        settled = as_vec(box.get_pos(), 3)
        positions = [as_vec(robot.get_pos(), 3)[:2] for _name, robot in robots]
        to_goal = GOAL - settled[:2]
        direction = to_goal / max(float(np.linalg.norm(to_goal)), 1e-6)
        slot_of = assign_slots(positions, push_slots(settled[:2], direction))
        sides = [1.0 if pos[0] >= settled[0] else -1.0 for pos in positions]
        lanes = lane_offsets(positions, sides)
        phases = [0] * len(robots)
        start_goal = float(np.linalg.norm(to_goal))
        print(
            f"caixa após assentar: x={settled[0]:.3f} y={settled[1]:.3f} z={settled[2]:.3f}"
        )

        robot_z = [float(as_vec(robot.get_pos(), 3)[2]) for _n, robot in robots]
        print(f"robôs no piso: z={min(robot_z):.3f}..{max(robot_z):.3f}")
        max_box_z = float(settled[2])
        max_robot_z = max(robot_z)
        min_robot_z = min(robot_z)
        arrived = False
        final = settled
        for step in range(args.steps):
            box_xyz = as_vec(box.get_pos(), 3)
            max_box_z = max(max_box_z, float(box_xyz[2]))
            to_goal = GOAL - box_xyz[:2]
            distance = float(np.linalg.norm(to_goal))
            if distance < 0.35:
                arrived = True
                for (_name, robot), dofs in zip(robots, wheel_idx):
                    robot.control_dofs_velocity(zero, dofs)
            else:
                direction = to_goal / distance
                slots = push_slots(box_xyz[:2], direction)
                # Perto do alvo, anda mais devagar para não lançar a caixa.
                creep = 0.04 + 0.02 * min(distance / 1.5, 1.0)
                for i, ((_name, robot), dofs) in enumerate(zip(robots, wheel_idx)):
                    robot_xyz = as_vec(robot.get_pos(), 3)
                    max_robot_z = max(max_robot_z, float(robot_xyz[2]))
                    min_robot_z = min(min_robot_z, float(robot_xyz[2]))
                    target, phases[i] = approach_target(
                        robot_xyz[:2],
                        slots[slot_of[i]],
                        box_xyz[:2],
                        direction,
                        lanes[i],
                        phases[i],
                    )
                    command_drive(robot, dofs, target, creep)
            scene.step()
            if step % 1000 == 999:
                print(
                    f"t={0.01 * (step + 1):.0f}s caixa x={box_xyz[0]:.3f} "
                    f"y={box_xyz[1]:.3f} z={box_xyz[2]:.3f}"
                )
            if arrived:
                final = as_vec(box.get_pos(), 3)
                break
        else:
            final = as_vec(box.get_pos(), 3)

        end_goal = float(np.linalg.norm(final[:2] - GOAL))
        print(
            f"caixa no fim: x={final[0]:.3f} y={final[1]:.3f} z={final[2]:.3f} "
            f"alvo={end_goal:.3f} m"
        )
        print(
            f"z caixa={max_box_z:.3f} robô={min_robot_z:.3f}..{max_robot_z:.3f}"
        )
        if min_robot_z < 0.03:
            raise SystemExit("Os robôs atravessaram o piso da arena.")
        if max_box_z > settled[2] + 0.05:
            raise SystemExit("A caixa levantou.")
        if max_robot_z > 0.12:
            raise SystemExit("Os robôs saltaram.")
        if end_goal > start_goal - 0.8:
            raise SystemExit("A caixa não foi para o outro lado.")
        print("transporte ok" if arrived else "caixa a caminho do alvo")
        if not args.headless:
            for _ in range(300):
                scene.step()


if __name__ == "__main__":
    main()
