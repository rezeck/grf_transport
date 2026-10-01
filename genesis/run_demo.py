#!/usr/bin/env python3
"""Cena do transporte coletivo no Genesis, sem ROS e sem Gazebo.

Carrega o URDF do HeRo, a arena e o prisma retangular. A caixa é um corpo
livre (massa e inércia do URDF) e cai no chão. Os robôs empurram na direção
+Y para mostrar o contato.
"""

from __future__ import annotations

import argparse
import os
import re
import tempfile
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

# Poses do launch/spawn_robots.launch. Z acima do piso da arena.
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

BOX_POS = (0.0, 1.1, 0.23)
TARGET_POS = (0.0, -1.2, 0.04)
# Olhar para +Y, onde a caixa começa. Eixos das rodas estão em Y.
ROBOT_EULER = (0.0, 0.0, 90.0)


def materialize_urdf(src: Path, dest: Path) -> Path:
    """Troca package:// por caminho absoluto. O Genesis não resolve ROS."""
    text = src.read_text()

    def replace(match: re.Match[str]) -> str:
        pkg, rel = match.group(1), match.group(2)
        root = PACKAGES.get(pkg)
        if root is None:
            raise SystemExit(f"Pacote URDF desconhecido: {pkg} em {src}")
        path = (root / rel).resolve()
        if not path.exists():
            raise SystemExit(f"Mesh ausente: {path}")
        return path.as_posix()

    text = re.sub(r"package://([^/]+)/([^\s\"']+)", replace, text)
    dest.write_text(text)
    return dest


def as_vec3(value) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu()
    return np.asarray(value).reshape(-1)[:3]


def box_pose(entity) -> np.ndarray:
    return as_vec3(entity.get_pos())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps-settle", type=int, default=200)
    parser.add_argument("--steps-push", type=int, default=1200)
    parser.add_argument("--wheel-speed", type=float, default=18.0, help="rad/s")
    args = parser.parse_args()

    if not (HERO_COMMON / "hero_description").is_dir():
        raise SystemExit(f"hero_common não encontrado em {HERO_COMMON}")

    import genesis as gs

    backend = gs.cpu
    if not args.headless:
        backend = getattr(gs, "metal", gs.cpu)
    try:
        gs.init(backend=backend, logging_level="warning")
    except Exception:
        gs.init(backend=gs.cpu, logging_level="warning")

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=0.01, gravity=(0.0, 0.0, -9.81)),
        rigid_options=gs.options.RigidOptions(
            enable_collision=True,
            # O URDF limita a roda a 0.3 rad/s. Isso é teto do atuador do
            # Gazebo, não um batente mecânico. Com o limite ligado o robô
            # anda ~7 mm/s e não alcança a caixa.
            enable_joint_limit=False,
        ),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(-0.02, -3.2, 2.4),
            camera_lookat=(0.0, 0.4, 0.15),
            camera_fov=45,
        ),
        show_viewer=not args.headless,
    )

    scene.add_entity(gs.morphs.Plane())

    with tempfile.TemporaryDirectory(prefix="grf-genesis-") as tmp:
        tmp_path = Path(tmp)
        hero_urdf = materialize_urdf(
            HERO_COMMON / "hero_description" / "robot" / "hero_light.urdf",
            tmp_path / "hero.urdf",
        )
        box_urdf = materialize_urdf(
            REPO / "models" / "rectangular_prism" / "rectangular_prism.urdf",
            tmp_path / "box.urdf",
        )
        target_urdf = materialize_urdf(
            REPO / "models" / "rectangular_prism" / "rectangular_prism_target.urdf",
            tmp_path / "target.urdf",
        )
        arena_urdf = materialize_urdf(
            HERO_COMMON / "hero_gazebo" / "worlds" / "models" / "arena_0" / "arena.urdf",
            tmp_path / "arena.urdf",
        )

        scene.add_entity(
            gs.morphs.URDF(
                file=str(arena_urdf),
                pos=(0.0, 0.0, 0.0),
                fixed=True,
                # O casco convexo preenche a arena e a caixa senta em cima
                # do bloco. A malha côncava mantém o piso e as paredes.
                convexify=False,
                decimate=False,
            ),
        )
        box = scene.add_entity(
            gs.morphs.URDF(
                file=str(box_urdf),
                pos=BOX_POS,
                fixed=False,
                convexify=True,
            ),
        )
        scene.add_entity(
            gs.morphs.URDF(
                file=str(target_urdf),
                pos=TARGET_POS,
                fixed=True,
                collision=False,
            ),
        )

        robots = []
        for name, pos in ROBOTS:
            robot = scene.add_entity(
                gs.morphs.URDF(
                    file=str(hero_urdf),
                    pos=pos,
                    euler=ROBOT_EULER,
                    fixed=False,
                    convexify=True,
                ),
            )
            robots.append((name, robot))

        scene.build()

        wheel_idx = []
        for name, robot in robots:
            left = robot.get_joint("left_wheel_joint").dofs_idx_local[0]
            right = robot.get_joint("right_wheel_joint").dofs_idx_local[0]
            dofs = [left, right]
            wheel_idx.append(dofs)
            robot.set_dofs_kp(np.array([20.0, 20.0]), dofs)
            robot.set_dofs_kv(np.array([0.5, 0.5]), dofs)
            robot.set_dofs_force_range(
                np.array([-0.8, -0.8]),
                np.array([0.8, 0.8]),
                dofs,
            )

        for _ in range(args.steps_settle):
            scene.step()
        settled = box_pose(box)
        print(
            f"caixa após assentar: x={settled[0]:.3f} y={settled[1]:.3f} z={settled[2]:.3f}"
        )

        speed = np.array([args.wheel_speed, args.wheel_speed])
        for _ in range(args.steps_push):
            for (_, robot), dofs in zip(robots, wheel_idx):
                robot.control_dofs_velocity(speed, dofs)
            scene.step()

        for name, robot in robots:
            pos = as_vec3(robot.get_pos())
            print(f"{name}: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}")
        pushed = box_pose(box)
        delta = pushed - settled
        print(
            f"caixa após empurrar: x={pushed[0]:.3f} y={pushed[1]:.3f} z={pushed[2]:.3f}"
        )
        print(f"deslocamento: dx={delta[0]:.3f} dy={delta[1]:.3f} dz={delta[2]:.3f}")
        if not (0.15 < settled[2] < 0.40):
            raise SystemExit("A caixa não assentou no piso da arena.")
        if abs(float(delta[1])) < 0.03 and abs(float(delta[0])) < 0.03:
            raise SystemExit("A caixa não se moveu no contato com os robôs.")
        print("física da caixa ok")
        if not args.headless:
            for _ in range(400):
                scene.step()


if __name__ == "__main__":
    main()
