# Szimulációs környezet beállítása (Gazebo / Ignition)

Ez a leírás rövid útmutatót ad arra vonatkozóan, hogyan készítsük elő a labirintus-szimulációt ROS 2 alatt. Használati utasítások és disztribúció-specifikus megjegyzések találhatók alább.

1) Követelmények

- ROS 2 (ajánlott disztribúciók: Humble vagy Rolling; Foxy régebbi, de egyes rendszereken még használható). Telepítsd a hozzá tartozó Gazebo/ Ignition verziót.
- Gazebo (classic) vagy Ignition Fortress/Edifice/… a ROS 2 disztribúcióhoz illeszkedően.

2) Labirintus világ és robot modell

- Készíts egy `.world` vagy `.sdf` fájlt, amely tartalmazza a falakat, kezdőpontokat és a célzónát.
- Robotmodell: használhatsz meglévő TurtleBot 3 modell(ek)et vagy egyszerű differential drive modellt.

3) Indítás (példa)

- Példa launch szerkezet a `launch/launch_example1.launch.py` fájlra támaszkodva:

  - Indítsd el a szimulátort a labirintus világ betöltésével.
  - Indítsd el a robot spawn node-ot a kezdőpozíciókkal.
  - Indítsd el az RL környezet wrapper node-ot, amely figyeli a `/scan` és `/odom` topicokat és kezeli a `cmd_vel`-t.

4) Disztribúció-specifikus megjegyzések

- Humble (ajánlott): használj Ignition Fortress / Gazebo Garden megfelelő integrációt; ellenőrizd, hogy a `ros_ign_bridge` telepítve van-e ha Ignition-t használsz.
- Foxy: hagyományos Gazebo verziók használatosak; figyelj a `gazebo_ros` csomag kompatibilitására.

5) Tuning és tesztelés

- Ellenőrizd a LIDAR `/scan` topicot RViz-ben vagy egyszerű `ros2 topic echo` paranccsal.
- Állíts be több kezdőpozíciót (launch param) és ellenőrizd, hogy a robot minden esetben képes mozogni.

6) Hibakeresési tippek

- Ha a LIDAR üzenetek hiányoznak: ellenőrizd a robot spawn és a sensor plugin konfigurációját.
- Ha a robot nem reagál `cmd_vel`-re: ellenőrizd a vezérlő (controller) és a tf tree helyességét.
