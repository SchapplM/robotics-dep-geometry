# Geometrie-Toolbox

Dieses Repo enthält eine Sammlung von Matlab-Funktionen zum Thema Geometrie. Darunter fallen Berechnungen zu grundlegenden Geometrischen Körpern und Objekten (Kreis, Quader, Kugel, Linie) und weitere Berechnungen, wie z.B. Kollisionserkennung.

Moritz Schappler, schappler@imes.uni-hannover.de, 2018-03  
(C) Institut für mechatronische Systeme, Universität Hannover

## Benutzung

Für die Benutzung des Repos muss die Pfadinitialisierung durchgeführt werden mit dem Skript `geometry_toolbox_path_init.m` aus dem Hauptordner.

### Modultests

Die einzelnen Funktionen des Repos können über die Modultests aus dem Ordner `examples_tests` gestartet werden. Die Funktionsfähigkeit des Gesamt-Repos kann mit dem Gesamt-Testskript `geometry_toolbox_test_repo.m` aus dem Hauptordner geprüft werden.

## Inhalt

Hier erfolgt eine kurze Übersicht der im Repo enthaltenen Funktionen. Die Funktionsweise ist in den jeweiligen Funktionsdateien und Modultests erklärt.

* Schnittpunktberechnungen
  * Kreis-Kreis: `intersect_circles.m`
  * Linie-Quader: `find_intersection_line_box.m`
  * Linie-Kapsel: `find_intersection_line_capsule.m`
  * Linie-Zylinder: `find_intersection_line_cylinder.m`
* Berechnung von Punkten auf Körper-Oberflächen
  * auf Quader-Oberfläche: `box_random_surface_point.m`, `box_random_surface_point_equal.m`
  * auf Zylinder-Oberfläche: `cylinder_random_surface_point.m`, `cylinder_random_surface_point_equal.m`


## Erweiterung

Weitere allgemeine Funktionen sollen unter Verwendung des Benennungsschemas und grundsätzlicher Programmierkonventionen ergänzt werden und damit allen Institutsangehörigen zur Verfügung gestellt werden.

