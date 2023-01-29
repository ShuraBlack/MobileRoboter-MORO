# MORO-Labor WiSe 2023
## Bedienungsanleitung
- Klonen Sie dieses Repo um die darin enthaltenen Aufgaben zu bearbeiten.
- Erstellen Sie für jede neue Aufgabe einen Branch nach dem Schema `task<Nummer>` (z.B. für Aufgabe 1: `task1`).
- Sobald Sie die Aufgabe bearbeitet haben stellen Sie einen Merge-Request gegen den Branch `master` in diesem Repo.
- Die Aufgabe gilt als bestanden, wenn die CI-Pipeline alle Tests erfolgreich ausgeführt hat und der Merge-Request akzeptiert wurde.

Sie können selbstverständlich weitere Sub-Branches zum Bearbeiten der Aufgaben erstellen. Lediglich der Merge-Request
mit Ihrer Abgabe muss von einem Branch mit Namen nach dem Schema `task<Nummer>` ausgehen.

Verwenden Sie für Funktionsaufrufe ausschließlich Parameter in SI-Einheiten. 
Beispielsweise werden Entfernungen in Meter, Zeiten in Sekunden und Winkel in Radianten angegeben.
Sollten Sie Einheiten zwischenzeitlich nicht in SI-Einheiten verarbeiten, machen Sie dies
durch den Variablennamen kenntlich. Speichern Sie Winkel beispielsweise in Grad, können Sie
die zugehörige Variable beispielsweise `alpha_deg` nennen.
