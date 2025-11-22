# jan_fwl_aimaze

Ez a csomag egy ROS 2 alapú projekt része, melynek célja egy mobil robot (például TurtleBot) megtanítása arra, hogy labirintusból kijusson megerősítéses tanulás (Reinforcement Learning, RL) segítségével.

## Cél

- A robotnak meg kell tanulnia megtalálni a kijáratot különböző kezdőpozíciókból és labirintus-pályákból.
- A tanulás célja hatékony, ütközésmentes útvonal megtalálása a lehető legkevesebb lépéssel.

## Az ügynök és a környezet

- Ügynök (Agent): a robot vezérlője, amely az érzékelők alapján döntéseket hoz (diszkrét akciók).
- Környezet (Environment): a Gazebo/Ignition alapú szimulációs világ, amely visszajelzést ad az ügynöknek (szenzoradatok, jutalmak, termináció).

## Állapot (State)

- Fő bemenet: LIDAR/laser távolságmérések (sensor_msgs/LaserScan). Az olvasásokat javasolt előfeldolgozni (aggregálás, normalizálás, kieső értékek kezelése).
- Opcionális: odometria (nav_msgs/Odometry) vagy becsült pozíció/irány a céltól.

## Akciók (Action)

- Javasolt egyszerű, diszkrét akciótér: {előre, balra, jobbra, hátra} vagy diszkrét (v, ω) párok.
- Az akciók hatására `geometry_msgs/Twist` üzenetek kerülnek publikálásra a `cmd_vel` topicra.

## Jutalmazás (Reward)

- Pozitív jutalom, ha a robot közelebb kerül a célhoz (Δtávolság = előző_távolság - aktuális_távolság).
- Nagy negatív jutalom ütközés esetén (epizód vége).
- Kis negatív lépésbüntetés minden időlépésnél (pl. -0.01) a gyors megoldás ösztönzésére.

## Termináció (Episode termination)

- Az epizód véget ér, ha a robot eléri a célzónát, ütközik, vagy eléri a maximális lépésszámot/timeoutot.

## Integráció (ROS 2 / Szimuláció)

- Bemeneti topicok:
  - `/scan` — sensor_msgs/LaserScan (LIDAR adatok)
  - opcionálisan `/odom` — nav_msgs/Odometry
- Kimeneti topic:
  - `cmd_vel` — geometry_msgs/Twist (robot vezérlése)
- Szimuláció: Gazebo vagy Ignition használata a világ és a robot fizikájának modellezéséhez.

## DRL keretrendszer javaslatok

- Mély megerősítéses algoritmusok: DQN (diszkrét akciótérhez), PPO vagy TD3 (ha folyamatos akciótérre váltanak).
- Könyvtárak: PyTorch ajánlott egyszerű prototípushoz; a TensorFlow is használható.

## Gyors teszt / Futtatás (helyi szimuláción)

1. A workspace gyökérében build:

```bash
colcon build --packages-select jan_fwl_aimaze
```

2. Forrásolás és a példa launch futtatása:

```bash
source install/setup.bash
ros2 launch jan_fwl_aimaze launch_example1.launch.py
```

Megjegyzés: A pontos elindítási lépések a helyi ROS 2 disztribúciótól függenek; részletes, disztribúció-specifikus útmutatót a `docs/` részben adunk majd meg.


