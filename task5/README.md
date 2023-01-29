# 5. Verarbeitung von Sensorinformationen

**Abgabe:** Erstellen Sie für die Bearbeitung einen Branch `task5` und implementieren Sie ihre Lösung in diesem Branch.
Sobald Sie fertig sind, stellen Sie einen Merge-Request gegen den Branch `master`.
Die Abgabe gilt als bestanden, wenn die CI-Tests ohne Fehler durchlaufen und der Merge-Request akzeptiert wurde.

**Deadline:** Der Merge-Request muss bis zum 24. Januar 2023 um 23:59 erstellt worden sein.

## 5.1 Sensorbasierte Steuerung

Das Ziel dieser Übung ist es, Sensordaten einzulesen, auszuwerten und den Roboter entsprechend zu steuern.
Dazu berechnen Sie die Position einer Säule anhand der Messungen eines Laserscans und steuern den Roboter auf diese
Säule zu.
Implementieren Sie die Lösung in einem Package ``my_basic_kinematics``.

### 5.1.1 Simulationsumgebung

Um die Daten des Laserscanners in der Simulation sehen zu können müssen Sie Ihr turtlebot3_gazebo updaten.

```bash
cd ~/ros2_ws/src/turtlebot3_simulations
git pull
cd ~/ros2_ws/
colcon build --symlink-install
```

Verwenden Sie für diese Aufgabe das launch-file ``turtlebot3_pillar_world.launch.py`` um den Turtlebot in einer Welt mit
einer Säule zu laden.

```bash
ros2 launch turtlebot3_gazebo turtlebot3_pillar_world.launch.py
```

### 5.1.2 Extrahieren der Säulenposition aus dem Laserscan

Erstellen Sie das Package ``my_basic_kinematics``.
Schreiben Sie darin einen Node ```visual_servoing``` welcher Messungen eines 2D Laserscanners über des Topic ``/scan``
abonniert und in den Daten die Säule extrahiert. Steuern Sie dann Ihren Roboter auf diese Säule zu und halten Sie kurz
vor der Säule an.

Publizieren Sie einen RViz-Marker auf dem Topic ```min_distance_marker``` um die Position Ihres extrahierten Objektes in
RViz zu visualisieren. Beachten Sie, dass Sie auch den korrekten Frame für den Marker eintragen damit RViz weiß auf
welches Koordinaten System sich die Koordinaten beziehen.

<details>
<summary><b>Hinweise zur Implementierung:</b>
Klicken Sie <b>hier</b> um weitere Informationen zu erhalten.</summary>

**LaserScan Message**

* https://docs.ros2.org/galactic/api/sensor_msgs/msg/LaserScan.html

Beachten Sie bei der Message des 2D-Laserscanners folgende Hinweise:

* ``range_min`` ist nicht die minimale Distanz der aktuellen Messung, sondern der kleinste Wert den der Sensor überhaupt
  messen kann.
* Den Winkel unter dem eine Entfernung (``range``) gemessen wurde erhalten Sie über die Position im Array. Der erste
  Wert im Array wurde bei ``angle_min`` aufgenommen. Jeder weitere Punkt ist dann um ein
  Winkelinkrement ``angle_increment`` weiter.

```python
std_msgs/msg/Header header
float angle_min         # Winkel unter dem der Scan beginnt
float angle_max         # Winkel bei dem der Scan endet
float angle_increment   # Winkelinkrement zwischen zwei Laserstrahlen.
float time_increment
float scan_time
float range_min         # Minimale Reichweite des Sensors
float range_max         # Maximale Reichweite des Sensors
float[] ranges          # Entfernungsmessungen beginnen mit der Messung zum Winkel angle_min
float[] intensities
```

**Marker Message**

* http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html

```python
p_marker.header.frame_id = "base_scan"
p_marker.header.stamp = self.get_clock().now().to_msg()
p_marker.ns = ""
p_marker.id = 1
```

</details>

### 5.1.3 Implementierung eines Reglers zum Anfahren der Säule

Erweitern Sie Ihren ```visual_servoing```-Node um einen Regler, der den Roboter auf die Säule zusteuert und kurz vor
Erreichen der Säule anhält.

**Hinweis:** Als Grundlage können Sie Ihren Code aus der ``approach_goal``-Implementierung verwenden und
im ```visual_servoing``` an die neue Aufgabe anpassen.

## 5.2 Implementierung eines realistischen Odometrie-Modells

Wie Sie in der Übung 5.1 gesehen haben, liefert Gazebo für den Laserscan Messwerte mit einem entsprechenden Rauschen.
Für die Odometrie des Differenzialantriebs liefert Gazebo allerdings die perfekten Werte ohne Berücksichtigung von
Unsicherheiten.
Dies entspricht nicht der Realität.
Daher sollen Sie einen Node schreiben der die ideale Odometrie so verändert, dass eine realistische Odometrie entsteht.
Hierfür implementieren Sie zuerst einen Node ``probabilistic_diffdrive`` der auf die perfekten Odometriewerte aus der
Simulation ein entsprechendes Rauschen aufaddiert und dann wieder publiziert.
In einem zweiten Schritt visualisieren Sie dann die Odometrie mit der entsprechenden Kovarianz in einem Python-Plot.

### 5.2.1 Implementierung einer Odometrie mit Noise Model

Implementieren Sie in Ihrem ``my_basic_kinematics``-Package einen Node ``probabilistic_diffdrive`` der auf die Odometrie
Daten aus der Simulation folgendes Rauschen addiert:

* $`k_d = (0.02\,\text{m})^2 / 1\,\text{m}`$
* $`k_θ = (5\,\text{Grad})^2 / 360\,\text{Grad}`$
* $`k_{Drift} = (2\,\text{Grad})^2 / 1\,\text{m}`$

Zur Erläuterung der Parameter siehe Folien 5-23 bis 5-26.
Publizieren Sie die Odometrie mit entsprechenden
Kovarianzmatrizen wieder als ``nav_msgs/Odometry`` auf dem Topic ``/odom_with_noise``.

Beachten Sie, dass Sie in dieser Node auch die Fehlerfortpflanzung für die Odometrie, wie im Skript zur Koppelnavigation
angegeben, berechnen müssen, um die entsprechenden Kovarianzen in der Odometrie-Message einfügen zu können.

<details>
<summary><b>Hinweise zur Implementierung:</b>
Klicken Sie <b>hier</b> um weitere Informationen zu erhalten.</summary>

<ins>Zustand des Roboters</ins><br />
Beachten Sie bei der Implementierung, dass Sie sich in Ihrem ``probabilistic_diffdrive``-Node neben den Kovarianzen auch
den Zustand des Roboters mit $`(x, y, \theta)`$ abspeichern müssen, damit Sie bei jeder neu erhaltenen Odometriemessung
den Steuerbefehl $`u`$ auf Ihren internen Zustand anwenden können.

<ins>Steuerbefehl</ins><br />
Den Steuerbefehl $`u`$ können Sie aus der Differenz der letzten und der aktuell empfangenen Odometriemessung ermitteln.

<ins>Odometrie-Message mit Kovarianz</ins><br />
Die ``nav_msgs/Odometry``-Message enthält zwei Arrays für Kovarianzen in 3D. Eines für die Kovarianzen bezüglich der
3D-Pose des Roboters und eine bezüglich der Geschwindigkeiten.
Sie müssen in dieser Übung lediglich die Kovarianzen für die Pose bestimmen.

* http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html

```python
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

Weiter reicht es aus, wenn Sie die Kovarianzen nur für die 2D-Pose bestimmen und dann nur diese Werte in der Matrix
eintragen.
Alle anderen Einträge der Kovarianzmatrix können Sie auf ``0`` setzen.
Da die Kovarianzmatrix als flaches Array in der Message angeben ist, müssen Sie folgende Werte bestimmen.

```python
pose.covariance[0]  # x, x
pose.covariance[1]  # x, y
pose.covariance[5]  # x, theta

pose.covariance[6]  # y, x
pose.covariance[7]  # y, y
pose.covariance[11]  # y, theta

pose.covariance[30]  # theta, x
pose.covariance[31]  # theta, y
pose.covariance[35]  # theta, theta
```

</details>

#### 5.2.2. Visualisierung der Unsicherheiten

Implementieren Sie hierfür einen Node ``plot_odom`` der Ihre Odometrie mit Kovarianz empfängt und sich intern die
Messwerte mit Kovarianzen abspeichert.
Nachdem der Roboter einen kompletten Kreis gefahren ist plotten Sie dann die Kovarianzellipse wie in der Abbildung auf
Folie 5-26 in eine Datei ``odometrie.png`` im Verzeichnis task5.
Verwenden Sie hierfür wieder matplotlib.

Für das Fahren des Roboters verwenden Sie Ihren ``cuvre_drive``-Node und lassen den Roboter auf einem Kreis mit Radius
2m fahren.
Plotten Sie die Kovarianzellipse in entsprechenden Abständen zu den Positionen des Roboters.

<details>
<summary><b>Hinweise zur Implementierung:</b>
Klicken Sie <b>hier</b> um weitere Informationen zu erhalten.</summary>

<ins>Hilfsfunktionen zum Plotten von Ellipsen<ins>

```python
import math
import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt


# --------
# plot position and sigma ellipse
#
def plot_position_covariance(position, sigma_position, color='b'):
    assert sigma_position.shape == (2, 2), 'sigma_pose should be 2*2 matrix'
    assert len(position) == 2, 'position must be of the form [x,y]'

    # if sigma_position is close to (0,0; 0,0) there will be problems with singularity
    if sigma_position[0, 0] < 1.0E-10 or sigma_position[1, 1] < 1.0E-10:
        return
    d, V = la.eig(la.inv(sigma_position))

    # ellipse not rotated:
    gamma = np.linspace(0, 2 * math.pi, 80)
    xp = np.sin(gamma) / math.sqrt(d[0])
    yp = np.cos(gamma) / math.sqrt(d[1])

    # rotate and move ellipse
    Xp = np.vstack((xp, yp))
    Xp = V.dot(Xp)
    xp = Xp[0, :] + position[0]
    yp = Xp[1, :] + position[1]
    plt.plot(xp, yp, color + '-', linewidth=0.5)
    plt.plot(position[0], position[1], 'ko', markersize=6)


# --------
# plot position and sigma ellipse and
# plot orientation as cone.
#
def plot_pose_covariance(pose, sigma_pose, color='b'):
    assert sigma_pose.shape == (3, 3), 'sigma_pose should be 3x3 matrix'
    assert len(pose) == 3, 'pose must be of the form [x,y,theta]'

    # plot ellipse
    plot_position_covariance(pose[0:2], sigma_pose[0:2, 0:2], color)

    # plot orientation and sigma theta as cone
    theta = pose[2]
    sigma_theta = math.sqrt(sigma_pose[2, 2])
    d = math.sqrt(sigma_pose[0, 0] + sigma_pose[1, 1])  # cone length
    theta1 = theta - sigma_theta / 2
    theta2 = theta + sigma_theta / 2
    x0 = pose[0]
    y0 = pose[1]
    x1 = x0 + d * math.cos(theta1)
    y1 = y0 + d * math.sin(theta1)
    x2 = x0 + d * math.cos(theta2)
    y2 = y0 + d * math.sin(theta2)
    plt.plot((x0, x1), (y0, y1), 'c-', linewidth=0.5)
    plt.plot((x0, x2), (y0, y2), 'c-', linewidth=0.5)
```

</details>
