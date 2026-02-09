# AStarPathfinder – Dokumentation

Beschreibung der A*-Implementierung für Grid2D-basiertes Path Planning.

**Paket:** `de.egb.controlcore.planning`  
**Datei:** `AStarPathfinder.java`

---

## 1. Überblick

`AStarPathfinder` implementiert den A*-Algorithmus für Rastergraphen. Er arbeitet mit `Grid2D` und liefert einen kürzesten Pfad zwischen Start- und Zielzelle unter Berücksichtigung von Hindernissen und variablen Zellkosten.

---

## 2. Konstruktor

```java
AStarPathfinder(Grid2D grid, Grid2D.Connectivity connectivity)
AStarPathfinder(Grid2D grid, Grid2D.Connectivity connectivity,
                boolean preventCornerCutting, Heuristic heuristic)
```

| Parameter | Beschreibung |
|-----------|--------------|
| `grid` | Das Raster (muss nicht null sein) |
| `connectivity` | `FOUR` oder `EIGHT` |
| `preventCornerCutting` | Verhindert diagonale Schritte durch Hindernissecken (nur bei EIGHT) |
| `heuristic` | `MANHATTAN` oder `EUCLIDEAN` |

Standard (einfacher Konstruktor): `preventCornerCutting = true`, `heuristic = EUCLIDEAN`.

---

## 3. Heuristiken

| Heuristik | Formel | Verwendung |
|-----------|--------|------------|
| `MANHATTAN` | \|dx\| + \|dy\| | Klassisch, immer zulässig |
| `EUCLIDEAN` | √(dx² + dy²) | Weniger Expansionen bei 8-Nachbarn |

Beide sind admissible für 4- und 8-Nachbarschaft.

---

## 4. findPath

```java
List<Grid2D.Cell> findPath(Grid2D.Cell start, Grid2D.Cell goal)
```

**Rückgabe:**
- Pfad von Start bis Ziel (inklusive beider Zellen), in Reihenfolge
- Leere Liste, wenn kein Pfad existiert oder Start/Ziel ungültig

**Randfälle:**
- Start == Ziel → `List.of(start)`
- Außerhalb des Rasters oder nicht begehbar → leere Liste
- Ziel von Hindernissen eingeschlossen → leere Liste

---

## 5. Bewegungs kosten

- **Kardinal (4er-Schritt):** `grid.getCost(nx, ny)`
- **Diagonal (8er-Schritt):** `grid.getCost(nx, ny) × √2`

Die Diagonale ist geometrisch √2-mal länger; der Faktor wird auf die Zellkosten angewendet.

---

## 6. Ablauf (vereinfacht)

1. g-Map, Parent-Map, Open (PriorityQueue), Closed (HashSet) initialisieren
2. Start in Open mit f = h(start, goal)
3. Solange Open nicht leer:
   - Zelle mit kleinstem f aus Open entnehmen
   - Bereits in Closed → überspringen
   - Ist Ziel → Pfad rekonstruieren, zurückgeben
   - Zelle in Closed aufnehmen
   - Für jeden Nachbarn: tentative_g berechnen, bei Verbesserung Parent setzen, in Open einfügen
4. Open leer → kein Pfad

---

## 7. Verwendungsbeispiel

```java
Grid2D grid = new Grid2D(30, 20);

// Hindernisse
grid.setWalkable(10, 8, false);
grid.setWalkable(10, 9, false);
grid.setWalkable(10, 10, false);

AStarPathfinder pathfinder = new AStarPathfinder(grid, Grid2D.Connectivity.EIGHT);
List<Grid2D.Cell> path = pathfinder.findPath(grid.cell(0, 0), grid.cell(29, 19));

for (Grid2D.Cell c : path) {
    System.out.println(c);  // (0, 0), (1, 0), ...
}
```

---

## 8. Integration mit Trajektorie

Der A*-Pfad liefert Wegpunkte (Grid-Zellen). Für die Robotersteuerung:

1. Zellen in Weltkoordinaten (m) umrechnen (Skalierung, Offset)
2. `TrapezoidalTrajectory2D` oder ähnliches entlang der Wegpunkte
3. `Se2PController` folgt der Trajektorie

Die Umrechnung Raster ↔ Weltkoordinaten muss je nach Feldgröße und Rasterauflösung implementiert werden.
