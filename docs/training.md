# Tanítási jegyzetek és javaslatok

Ez a fájl tartalmazza a tanítás menedzseléséhez szükséges legfontosabb instrukciókat: hogyan tárold a checkpointokat, milyen metrikákat ments, és alap hyperparaméter-javaslatokat.

1) Checkpointing és logolás

- Ments minden `n` epizód / `n` tréning-lépés után modellt és optimizer állapotot (pl. `checkpoints/epoch_{i}.pt`).
- Használj egyszerű JSON vagy YAML metrikafájlt az epizód jutalom, lépésszám, sikerarány rögzítésére, illetve TensorBoard / Weights & Biases integrációt (opcionális).

2) Metrikák

- Érdemes gyűjteni: epizód összjutalom, epizód hossz (lépések), siker (cél elérése), ütközések száma, átlagos távolság a célhoz.

3) Hyperparaméter-javaslatok (kezdőértékek)

- DQN (diszkrét akciótér):
  - replay buffer: 100_000
  - batch size: 64
  - gamma: 0.99
  - lr: 1e-4
  - target update: 1000 lépés

- PPO (folyamatos):
  - clip_eps: 0.2
  - epochs: 4
  - batch size: 64
  - lr: 3e-4

4) Tanítási ciklus (magas szinten)

- Inicializáld a környezetet és az ügynököt.
- Gyűjts adatot (step -> állapot, akció, jutalom, következő állapot, done).
- DQN: tölts minibatch-eket a replay buffer-ből, frissítsd a hálózatot.
- Policy gradient / PPO: gyűjts rollouot, számold a GAE-t, és futtass policy/value update-et.

5) Generalizáció és curriculum

- Kezdd egyszerű pályákkal és növeld a nehézséget (pálya randomizáció, több fal, zaj a szenzoron).
- Randomize starting pose és goal position minden epizódnál a jobb általánosítás érdekében.

6) Mentés és reprodukálhatóság

- Rögzítsd a seed-et (numpy, torch, random, és a környezet seed-je), a használt csomagok verzióit (`requirements.txt` / `pip freeze`) és a futtatási parancsokat.
