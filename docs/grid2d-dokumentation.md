# Grid2D – Dokumentation

Ausführliche Beschreibung der `Grid2D`-Klasse – einer 2D-Rasterstruktur für Path-Planning-Algorithmen (insbesondere A*).

**Paket:** `de.egb.controlcore.planning`  
**Datei:** `Grid2D.java`

---

## 1. Übersicht

`Grid2D` repräsentiert ein rechteckiges Raster, in dem jede Zelle (Cell) entweder begehbar oder blockiert sein kann und individuelle Bewegungskosten haben kann. Die Klasse dient als Graphen-Grundlage für Suchalgorithmen wie A*.

**Kernfunktionen:**
- Verwaltung von Begehbarkeit (Hindernisse)
- Variable Kosten pro Zelle (z.B. für unterschiedliches Terrain)
- Ermittlung von Nachbarzellen (4- oder 8-Nachbarschaft)
- Corner-Cutting-Verhinderung bei diagonalen Bewegungen

---

## 2. Koordinatensystem

Das Raster verwendet die übliche Matrix-Konvention:
- **x**: Spaltenindex (0 = links, `width - 1` = rechts)
- **y**: Zeilenindex (0 = oben, `height - 1` = unten)

```
     x →  0   1   2   3   ...
  y
  ↓ 0   [·] [·] [·] [·]
  1   [·] [·] [·] [·]
  2   [·] [·] [·] [·]
  ...
```

**Array-Indexierung:** `walkable[y][x]` und `cost[y][x]` – Zeile zuerst (row-major), dann Spalte.

---

## 3. Innere Klassen und Enum

### 3.1 Cell

```java
public static final class Cell {
    public final int x;
    public final int y;
}
```

**Zweck:** Unveränderliche Repräsentation einer Rasterzelle als Knoten für Suchalgorithmen.

**Wichtige Eigenschaften:**
- **Immutable** – x und y sind `final`
- **equals/hashCode** – ermöglicht Nutzung in `HashSet` und `HashMap` (z.B. Closed List bei A*)
- **hashCode-Implementierung:** `(x * 73856093) ^ (y * 19349663)` – typische Methode, um 2D-Koordinaten gut über den 32-Bit-Raum zu verteilen und Kollisionen zu reduzieren

**Factory:** `grid.cell(x, y)` erzeugt eine `Cell`-Instanz (mit Bounds-Check).

### 3.2 Connectivity

```java
public enum Connectivity { FOUR, EIGHT }
```

| Wert | Bedeutung | Nachbarn |
|------|-----------|----------|
| **FOUR** | Von-Neumann-Nachbarschaft | nur links, rechts, oben, unten (4 Stück) |
| **EIGHT** | Moore-Nachbarschaft | zusätzlich die 4 diagonalen Nachbarn (8 Stück) |

```
FOUR:              EIGHT:
    [ ]                [·][·][·]
  [ ][x][ ]            [·][x][·]
    [ ]                [·][·][·]
```

---

## 4. Felder ( interne Daten )

| Feld | Typ | Beschreibung |
|------|-----|--------------|
| `width` | int | Anzahl Spalten |
| `height` | int | Anzahl Zeilen |
| `walkable` | boolean[][] | `walkable[y][x] == true` ⇔ Zelle ist begehbar |
| `cost` | double[][] | Kosten für das Betreten der Zelle (≥ 0) |

Beide Arrays sind `[height][width]` – Zeile (y) zuerst.

---

## 5. Konstruktor

```java
public Grid2D(int width, int height)
```

**Initialisierung:**
- Alle Zellen sind begehbar (`walkable = true`)
- Alle Zellen haben Kosten 1.0 (`cost = 1.0`)

**Validierung:** `width` und `height` müssen > 0 sein, sonst `IllegalArgumentException`.

---

## 6. Öffentliche API

### 6.1 Abmessungen

| Methode | Rückgabe | Beschreibung |
|---------|----------|--------------|
| `width()` | int | Anzahl Spalten |
| `height()` | int | Anzahl Zeilen |

### 6.2 Grenzenprüfung

| Methode | Beschreibung |
|---------|--------------|
| `inBounds(int x, int y)` | Liefert `true`, wenn (x,y) innerhalb des Rasters liegt. Keine Exception. |
| `requireInBounds(int x, int y)` | Prüft ebenfalls, wirft aber `IllegalArgumentException`, wenn außerhalb. Intern verwendet. |

### 6.3 Begehbarkeit

| Methode | Beschreibung |
|---------|--------------|
| `isWalkable(int x, int y)` | Gibt zurück, ob die Zelle begehbar ist. Wirft bei ungültigen Koordinaten. |
| `setWalkable(int x, int y, boolean value)` | Setzt Begehbarkeit (z.B. `false` für Hindernisse). |

### 6.4 Kosten

| Methode | Beschreibung |
|---------|--------------|
| `getCost(int x, int y)` | Gibt die Kosten der Zelle zurück (≥ 0). |
| `setCost(int x, int y, double value)` | Setzt die Kosten. `value` muss ≥ 0, endlich und nicht NaN sein. |

**Verwendung bei A*:**  
Kosten für einen Schritt nach (nx, ny):
- Kardinal (4er-Nachbar): `getCost(nx, ny)`
- Diagonal (8er-Nachbar): `getCost(nx, ny) * √2`

### 6.5 Cell-Erzeugung

| Methode | Beschreibung |
|---------|--------------|
| `cell(int x, int y)` | Erzeugt eine `Cell`-Instanz. Prüft Grenzen. |

### 6.6 Nachbarn

```java
List<Cell> neighbors(int x, int y, Connectivity conn)
List<Cell> neighbors(int x, int y, Connectivity conn, boolean preventCornerCutting)
```

**Parameter:**
- `x`, `y` – Zelle, deren Nachbarn bestimmt werden
- `conn` – `FOUR` oder `EIGHT`
- `preventCornerCutting` – nur bei `EIGHT` relevant (siehe unten)

**Rückgabe:** Liste begehbarer Nachbarzellen innerhalb des Rasters. Leere Liste, wenn die Ausgangszelle selbst nicht begehbar ist.

**Standard:** `neighbors(x, y, conn)` ruft intern `preventCornerCutting = false` auf.

---

## 7. Corner-Cutting-Verhinderung (preventCornerCutting)

Bei 8-Nachbarschaft kann man sich diagonal bewegen. Ohne zusätzliche Logik wäre folgendes erlaubt:

```
  [■][ ]      Roboter in (1,1) könnte diagonal nach (2,0) laufen
  [ ][x]      und damit die Ecke des Hindernisses „durchschneiden“.
```

`preventCornerCutting = true` verhindert das: Ein diagonaler Schritt ist nur erlaubt, wenn **beide** angrenzenden kardinalen Nachbarn (in Bewegungsrichtung) frei sind.

**Beispiel – Diagonal Schritt nach rechts-oben (dx=+1, dy=-1):**
- Benötigt freie Zellen: `(x+1, y)` und `(x, y-1)`
- Sind beide begehbar, darf `(x+1, y-1)` als Nachbar hinzugefügt werden

**Implementierung in `addDiagonal`:**
- `ax = x + dx`, `ay = y` → Zelle in x-Richtung
- `bx = x`, `by = y + dy` → Zelle in y-Richtung
- Beide müssen `inBounds` und `walkable` sein

**Empfehlung für A*:** `preventCornerCutting = true` verwenden, um unrealistische Diagonalpfade durch Hindernissecken zu vermeiden.

---

## 8. Ablauf der Nachbarschaftsberechnung

1. Prüfen: (x,y) in Bounds und begehbar → sonst leere Liste
2. **Kardinale Nachbarn:** (x±1, y), (x, y±1) über `tryAdd` – nur hinzufügen wenn in Bounds und begehbar
3. Bei `EIGHT`: vier diagonale Richtungen über `addDiagonal` prüfen
4. Bei `preventCornerCutting`: zusätzlich Freiheit der benachbarten kardinalen Zellen prüfen
5. Ergebnisliste zurückgeben

`tryAdd` und `addDiagonal` fügen nur Zellen hinzu, die im Raster liegen und begehbar sind.

---

## 9. Verwendung mit A*

Typischer Ablauf:

```java
Grid2D grid = new Grid2D(20, 15);

// Hindernisse setzen
grid.setWalkable(5, 5, false);
grid.setWalkable(5, 6, false);

// Start und Ziel
Cell start = grid.cell(0, 0);
Cell goal = grid.cell(19, 14);

// Nachbarn für A* (8-connected mit Corner-Cutting-Verhinderung)
List<Cell> neighbors = grid.neighbors(start.x, start.y, Connectivity.EIGHT, true);

// Bewegungskosten für A* (Beispiel: Schritt von (x,y) nach (nx,ny))
double cost = grid.getCost(nx, ny);
boolean isDiagonal = (Math.abs(nx - x) + Math.abs(ny - y)) == 2;
if (isDiagonal) cost *= Math.sqrt(2);
```

---

## 10. Design-Entscheidungen

| Entscheidung | Begründung |
|--------------|------------|
| `Cell` als eigene Klasse | Eindeutige Knoten-Identität, sauberes `equals`/`hashCode` für Collections |
| Primitive Arrays statt List/Map | Geringer Overhead, schneller Zugriff über [y][x] |
| `cost` als `double` | Flexible Gewichtung (z.B. 1.0, √2, 2.0 für Schweres Terrain) |
| `walkable` und `cost` getrennt | Blockierte Zellen können trotzdem Kosten haben; Konzept bleibt klar |
| `preventCornerCutting` als Parameter | Flexibel: 8-Nachbarn mit oder ohne Corner-Cutting |
| `requireInBounds` bei öffentlichen Methoden | Fail-fast bei falschen Koordinaten statt stiller Fehler |

---

## 11. Einschränkungen und Erweiterungsmöglichkeiten

**Aktuell:**
- Keine integrierte Umrechnung Weltkoordinaten (m) ↔ Rasterzellen
- Keine vordefinierte Heuristik (liegt in der Verantwortung von A*)
- Keine Thread-Sicherheit bei gleichzeitigen `setWalkable`/`setCost`-Aufrufen

**Mögliche Erweiterungen:**
- `worldToCell(double wx, double wy)` / `cellToWorld(int x, int y)` bei festem Rastermaßstab
- `getMovementCost(int x1, int y1, int x2, int y2)` als Hilfsmethode für A*
- Immutable-Variante für parallele Suche
