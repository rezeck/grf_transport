# Genesis (sem Gazebo / ROS)

Branch `genesis-sim`. A cena do `demo.launch` roda no [Genesis](https://genesis-world.readthedocs.io/en/latest/): plano, arena fixa, 10 HeRo (`hero_light.urdf`), prisma dinâmico e alvo fixo.

Os `package://` do URDF são reescritos para caminhos absolutos. Meshes do HeRo vêm de `../hero_common` (ou `HERO_COMMON`).

```bash
python3.12 -m venv .venv-genesis
.venv-genesis/bin/pip install -r genesis/requirements.txt
.venv-genesis/bin/python genesis/run_demo.py --headless
```

Sem `--headless` abre o viewer (Metal neste Mac). A caixa cai, assenta, e os robôs empurram na direção +Y.
