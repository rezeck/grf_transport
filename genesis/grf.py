"""Controlador GRF do transporte, portado de src/grf_colletive_transport.cpp.

Cada robô só vê o que o laser de 0,5 m alcança. A cada passo ele amostra
velocidades por Metropolis-Hastings a partir da energia local (obstáculo,
objeto e vizinhos), sem comunicação e sem líder. Nomes e constantes seguem o
C++ para facilitar a comparação.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# config/params.yaml
SENSING = 0.5
SAFEZONE = 0.15
DT = 0.5
MASS = 8000.0
VMAX = 0.06
WORLDSIZE = 4.0
MCMC_STEPS = 60
MCMC_SIGMA = 0.05
BURN_IN = 0.6
LASER_RES = 0.1

# config/arena0.yaml, multiplicado por WORLDSIZE
ARENA_LINES = np.array(
    [
        [-0.486, 0.447, -0.486, -0.447],
        [-0.447, 0.486, 0.447, 0.486],
        [0.486, 0.447, 0.486, -0.447],
        [-0.447, -0.486, 0.447, -0.486],
        [0.456, 0.483, 0.478, 0.464],
        [-0.479, -0.465, -0.457, -0.484],
        [-0.484, 0.455, -0.464, 0.478],
        [-0.484, 0.453, -0.484, 0.445],
        [0.471, -0.474, 0.484, -0.452],
        [-0.482, -0.458, -0.474, -0.468],
        [0.456, -0.484, 0.478, -0.465],
    ]
) * WORLDSIZE

# config/rectangular_prism.yaml (cubo de 0,4 m)
OBJECT_LINES = np.array(
    [
        [-0.20, -0.20, 0.20, -0.20],
        [0.20, -0.20, 0.20, 0.20],
        [0.20, 0.20, -0.20, 0.20],
        [-0.20, 0.20, -0.20, -0.20],
    ]
)

LASER_ANGLES = np.arange(0.0, 2.0 * math.pi, LASER_RES)


def coulomb_buckingham(r, eps, eps0, r0, alpha, q1, q2):
    return eps * (
        (6.0 / (alpha - 6.0)) * math.exp(alpha) * (1.0 - r / r0)
        - (alpha / (alpha - 6.0)) * (r0 / r) ** 6
    ) + (q1 * q2) / (4.0 * math.pi * eps0 * r)


def kinetic_energy(v: float, m: float) -> float:
    # O C++ usa 0,5 * v * m, sem elevar ao quadrado: v já é a velocidade².
    return 0.5 * v * m


def saturation(v: np.ndarray, norm: float) -> np.ndarray:
    """Limita cada componente a `norm`, mantendo a direção (como no C++)."""
    ax, ay = abs(v[0]), abs(v[1])
    if ax < 1e-12 and ay < 1e-12:
        return v.copy()
    if ay >= ax and ay > norm:
        return v * (norm / ay)
    if ax > ay and ax > norm:
        return v * (norm / ax)
    return v.copy()


def orientation(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> int:
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if abs(val) <= 1e-7:
        return 0
    return 1 if val > 0 else 2


def transform_lines(local: np.ndarray, pos: np.ndarray, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    start = local[:, :2] @ rot.T + pos
    end = local[:, 2:] @ rot.T + pos
    return np.hstack([start, end])


def ray_hits(origin: np.ndarray, angles: np.ndarray, lines: np.ndarray) -> np.ndarray:
    """Ponto mais próximo atingido por cada raio de alcance SENSING.

    Devolve um array (n, 2) só com os raios que acertaram algo.
    """
    d = np.stack([np.cos(angles), np.sin(angles)], axis=1) * SENSING
    p = lines[:, :2]
    e = lines[:, 2:] - p
    # origin + t*d = p + u*e
    den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
    w = p[None, :, :] - origin[None, None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (w[..., 0] * e[None, :, 1] - w[..., 1] * e[None, :, 0]) / den
        u = (w[..., 0] * d[:, None, 1] - w[..., 1] * d[:, None, 0]) / den
    ok = (np.abs(den) > 1e-12) & (t >= 0.0) & (t <= 1.0) & (u >= 0.0) & (u <= 1.0)
    t = np.where(ok, t, np.inf)
    best = t.min(axis=1)
    hit = np.isfinite(best)
    return origin + d[hit] * best[hit, None]


def segments_cross(a: np.ndarray, b: np.ndarray, lines: np.ndarray) -> bool:
    d = b - a
    p = lines[:, :2]
    e = lines[:, 2:] - p
    den = d[0] * e[:, 1] - d[1] * e[:, 0]
    w = p - a
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (w[:, 0] * e[:, 1] - w[:, 1] * e[:, 0]) / den
        u = (w[:, 0] * d[1] - w[:, 1] * d[0]) / den
    ok = (np.abs(den) > 1e-12) & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)
    return bool(ok.any())


@dataclass
class Agent:
    position: np.ndarray
    theta: float
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))


class GRFController:
    def __init__(self, target: np.ndarray, rng: np.random.Generator | None = None):
        self.target = np.asarray(target, dtype=float)
        self.rng = rng or np.random.default_rng()
        self.velocity: list[np.ndarray] = []

    # --- potenciais -------------------------------------------------------

    def target_occlusion(self, pos: np.ndarray, objects: np.ndarray) -> float:
        dist_target = np.linalg.norm(pos - self.target)
        cw = ccw = 0
        for obj in objects:
            if (
                np.linalg.norm(pos - obj) >= SENSING
                or dist_target < np.linalg.norm(obj - self.target)
            ):
                continue
            wise = orientation(pos, self.target, obj)
            if wise == 1:
                cw += 1
            elif wise == 2:
                ccw += 1
            else:
                cw += 1
                ccw += 1
        if cw == 0 or ccw == 0:
            return 2.2
        rate = min(cw / ccw, ccw / cw)
        return 5.0 if rate > 0.4 else 2.2

    def go_cwise(self, pos: np.ndarray, theta: float, objects: np.ndarray) -> bool:
        goal = pos + np.array([math.cos(theta), math.sin(theta)]) * SENSING * 0.5
        cw = ccw = 0
        for obj in objects:
            if np.linalg.norm(pos - obj) >= SENSING:
                continue
            wise = orientation(pos, goal, obj)
            if wise == 1:
                cw += 1
            elif wise == 2:
                ccw += 1
            else:
                cw += 1
                ccw += 1
        return cw >= ccw

    def u_object(self, agent: Agent, v: np.ndarray, objects: np.ndarray) -> float:
        if len(objects) < 2:
            return 0.0
        pos = agent.position + v * DT
        factor = self.target_occlusion(pos, objects)
        cwise = self.go_cwise(pos, agent.theta, objects)

        energy = 0.0
        grad = np.zeros(2)
        mass = MASS
        n = len(objects)
        for i in range(n - 1):
            a, b = objects[i], objects[(i + 1) % n]
            dist_i = np.linalg.norm(pos - a) + 1e-9
            dist_j = np.linalg.norm(pos - b)
            if dist_i >= SENSING or dist_j >= SENSING:
                return 1e3
            if factor >= 5.0:
                return energy + coulomb_buckingham(dist_i, 0.04, 0.04, 0.8, 0.2, 8e6, -1.0)
            energy += coulomb_buckingham(dist_i, 0.04, 0.04, 0.8, 0.2, 100.0, -1.0)
            dv = (b - a) if cwise else (a - b)
            grad += dv - v
            mass += 0.01 * MASS
        grad = saturation(grad, 1.0)
        group_speed = float(grad @ grad) + 1e-9
        my_speed = float(v @ v)
        return (
            energy
            + kinetic_energy(group_speed, mass)
            + kinetic_energy(VMAX - my_speed, mass)
        )

    def u_obstacle(self, agent: Agent, v: np.ndarray, obstacles: np.ndarray) -> float:
        pos = agent.position + v * DT
        energy = 0.0
        for obs in obstacles:
            dist = np.linalg.norm(pos - obs) + 1e-9
            if dist <= SAFEZONE:
                energy += coulomb_buckingham(dist, 0.04, 0.04, 0.8, 0.05, 1.0, 1.0)
        return energy

    def u_neighbors(self, agent: Agent, v: np.ndarray, neighbors: list[Agent]) -> float:
        pos = agent.position + v * DT
        group_vrel = np.zeros(2)
        group_mass = MASS
        energy = 0.0
        for other in neighbors:
            other_pos = other.position + other.velocity * DT
            dist = np.linalg.norm(pos - other_pos) + 1e-9
            if dist > SENSING:
                return 1e3
            energy += coulomb_buckingham(dist, 0.04, 0.04, 0.8, 0.65, 50.0, -1.0)
            if SAFEZONE < dist < SENSING:
                group_vrel += other.velocity - v
                group_mass += MASS
        group_vrel = saturation(group_vrel, VMAX)
        group_speed = float(group_vrel @ group_vrel) + 1e-9
        my_speed = float(v @ v)
        return (
            energy
            + kinetic_energy(group_speed, group_mass)
            + kinetic_energy(VMAX - my_speed, group_mass)
        )

    # --- amostragem -------------------------------------------------------

    def metropolis_hastings(
        self,
        agent: Agent,
        neighbors: list[Agent],
        objects: np.ndarray,
        obstacles: np.ndarray,
    ) -> np.ndarray:
        def energy(v: np.ndarray) -> float:
            return (
                self.u_obstacle(agent, v, obstacles)
                + self.u_object(agent, v, objects)
                + self.u_neighbors(agent, v, neighbors)
            )

        chain = [agent.velocity.copy()]
        last_u = energy(chain[0])
        for _ in range(MCMC_STEPS):
            sample = saturation(self.rng.normal(agent.velocity, MCMC_SIGMA), VMAX)
            u = energy(sample)
            d_e = u - last_u
            if d_e < 0.0 or self.rng.random() < math.exp(-min(d_e, 700.0)):
                chain.append(sample)
                last_u = u
            else:
                chain.append(chain[-1])
        tail = chain[int(BURN_IN * len(chain)) :]
        return saturation(np.mean(tail, axis=0), VMAX)

    # --- passo de controle --------------------------------------------------

    def step(
        self,
        agents: list[Agent],
        object_pos: np.ndarray,
        object_yaw: float,
    ) -> list[np.ndarray]:
        """Atualiza a velocidade de cada robô. Usa só o que o laser dele vê."""
        object_lines = transform_lines(OBJECT_LINES, object_pos, object_yaw)
        occluders = np.vstack([ARENA_LINES, object_lines])
        snapshot = [Agent(a.position.copy(), a.theta, a.velocity.copy()) for a in agents]

        neighbors: list[list[Agent]] = [[] for _ in agents]
        for i in range(len(agents) - 1):
            for j in range(i + 1, len(agents)):
                if np.linalg.norm(snapshot[i].position - snapshot[j].position) > SENSING:
                    continue
                if segments_cross(snapshot[i].position, snapshot[j].position, occluders):
                    continue
                neighbors[i].append(snapshot[j])
                neighbors[j].append(snapshot[i])

        out = []
        for i, agent in enumerate(snapshot):
            angles = LASER_ANGLES + agent.theta
            objects = ray_hits(agent.position, angles, object_lines)
            obstacles = ray_hits(agent.position, angles, ARENA_LINES)
            v = self.metropolis_hastings(agent, neighbors[i], objects, obstacles)
            agents[i].velocity = v
            out.append(v)
        return out
