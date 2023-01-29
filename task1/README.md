# 1. Koordinatensysteme und Transformationen
## Pythonfunktionen für Koordinatentransformationen
Zum Rechnen mit Rotations- und Transformationsmatrizen sind folgende Funktionen in Python zu definieren. 
Benutzen Sie dazu den Datentyp `array` aus dem Paket `numpy`.

| Funktion       | Beschreibung                                                                                                                        |
|----------------|-------------------------------------------------------------------------------------------------------------------------------------|
| `rot(theta)`   | liefert eine 2D-Rotationsmatrix mit Drehwinkel theta zurück.                                                                        |
| `rotx(theta)`  | liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drehachse `x` zurück.                                            |
| `roty(theta)`  | liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drehachse `y` zurück.                                            |
| `rotz(theta)`  | liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drehachse `z` zurück.                                            |
| `rot2trans(r)` | wandelt die Rotationsmatrix `r` in eine homogene Transformationsmatrix um und liefert diese zurück.                                 |
| `trans(t)`     | liefert eine homogene Translationsmatrix mit Translation `t` zurück. `t` ist ein Tupel der Größe 2 bzw. 3 für den 2D- bzw. 3D-Fall. |

Im Package `task1` finden Sie die Dateien `transformations.py`, `task1.py` und ein Package `test`. 

In `transformations.py` befinden sich die vorbereiteten Funktionsrümpfe, die Sie nach der vorigen Funktionsbeschreibung vervollständigen sollen. 
`task1.py` verwendet die Funktionen aus `transformations.py`, um die Aufgaben 2.1 und 2.2 aus der Vorlesung zu lösen.
Auch diese Funktionen sollen Sie vervollständigen.

`test_task1.py` und `test_transformations.py` enthalten Tests, die ihre Implementierungen überprüfen sollen. 
Einige Tests sind bereits implementiert. 
So enthält `test_transformations.py` beispielsweise einfache Unit-Tests, die die oben beschriebenen Funktionen testen.
Erstellen Sie weitere Testfunktionen, die ihre Implementierungen mit den Lösungen von Aufgabe 2.1 und 2.2 überprüfen.
Diese sollten sich dann in `test_task1.py` befinden.
Erweitern Sie gegebenenfalls die bereits vorhandenen Tests, um weitere Fälle abzudecken.

Die Tests können mit
```
python -m pytest task1
```
oder 
```
pytest task1
```
im root des Repositories ausgeführt werden. 

## Abgabe
Bearbeiten Sie die Aufgabe mit ihrem Team auf Ihrem Fork in Branch `task1`. 
Sobald Sie fertig sind, stellen Sie einen Merge Request gegen den Branch `master` im Abgabe-Repository.
Die Abgabe gilt als bestanden, wenn die CI-Tests ohne Fehler durchlaufen und der Merge-Request akzeptiert wurde. 

## Deadline
Der Merge-Request muss bis zum 01. November 2022 um 23:59 erstellt worden sein.