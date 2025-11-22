# RL specifikációk (Állapot / Akció / Jutalom)

Ez a dokumentum pontosítja az RL környezet legfontosabb elemeit: hogyan reprezentáljuk az állapotot, milyen akcióteret használunk és hogyan számoljuk a jutalmat.

1) Állapot (State)

- Fő bemenet: LIDAR sugárzások (`sensor_msgs/LaserScan`). Ajánlott az eredeti 360 vagy 720 mérés aggregálása 24–36 sávra (pl. medián vagy átlag), majd normalizálás [0,1]-re.
- Opcionális kiegészítők: relativ célirány és cél távolsága (normalizált), jelenlegi lineáris és szögsebesség (odom).
- Hiányzó adatok kezelése: ha egy sugár végtelen vagy NaN, helyettesítsük maximális mérési tartomány értékével és alkalmazzunk klipet.

2) Akciók (Action)

- Javasolt egyszerű diszkrét készlet (kezdéshez):
  - 0: előre (v = 0.15 m/s, ω = 0)
  - 1: balra (v = 0.0 m/s, ω = +0.6 rad/s)
  - 2: jobbra (v = 0.0 m/s, ω = -0.6 rad/s)
  - 3: hátra (v = -0.05 m/s, ω = 0)
- Alternatíva: diszkrét (v, ω) rács (például 3×3 kombináció), vagy folyamatos akciótér PPO/TD3 esetén.

3) Jutalmazás (Reward)

- Alapformula (példa):

  r_t = k*(d_{t-1} - d_t) + R_goal*I_goal - R_collision*I_collision - ε

  ahol:
  - d_t: robot távolsága a céltól időtlen t-ben
  - I_goal: indikátor, 1 ha cél elérve
  - I_collision: indikátor, 1 ha ütközés történt
  - ε: kis lépésbüntetés (pl. 0.01)
  - k: skálázó faktor (pl. 1.0)
  - R_goal: nagy pozitív jutalom cél esetén (pl. +10)
  - R_collision: nagy negatív jutalom (pl.  -10 … -50)

- Extra büntetés javasolt, ha a robot háromszor egymás után ugyanazt az akciót hajtja végre vagy ha forog egyhelyben (detektált stagnálás).

4) Epizód végzése (Termination)

- Epizód vége, ha:
  - Robot eléri a célzónát (távolság < threshold).
  - Ütközés: kontakt vagy LIDAR közelérték kisebb, mint biztonsági küszöb.
  - Maximális lépésszám/timeout elérése.

5) Megjegyzések

- Állítsuk be a jutalmak és skálákat kísérleti úton, hogy a jel-zaj arány (reward signal) megfelelő legyen a tanuláshoz.
- Dokumentáljuk minden választást a jegyzőkönyvben (miért választottuk ezt a jutalmat/akciókészletet), mert a beadandó bírálatakor ez fontos pont.
