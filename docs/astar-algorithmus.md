# A*-Algorithmus – Erklärung

Ausführliche Beschreibung des A*-Algorithmus für die spätere Eigenimplementierung im Path-Planning-Kontext (z.B. SSL Roboter-Navigation).

---

## 1. Überblick

**A*** (ausgesprochen „A-Stern“) ist ein optimaler, best-first Suchalgorithmus für kürzeste Wege in gewichteten Graphen. Er wurde 1968 von Hart, Nilsson und Raphael vorgestellt und ist eine Erweiterung von Dijkstra.

**Eigenschaften:**
- Findet einen kürzesten Pfad (falls einer existiert)
- Nutzt eine Heuristik, um die Suche zu beschleunigen
- Ist bei geeigneter Heuristik vollständig und optimal

**Typische Anwendungen:** Spiele, Robotik, GPS-Navigation, Rasterkarten.

---

## 2. Idee und Intuition

### 2.1 Unterschied zu Dijkstra

**Dijkstra** expandiert immer den Knoten mit dem kleinsten **bekannten Abstand** zum Start. Er kennt die Richtung zum Ziel nicht und expandiert „in alle Richtungen“.

**A\*** nutzt zusätzlich eine **Heuristik** h(n): eine Schätzung der Restkosten von Knoten n zum Ziel. Dadurch bevorzugt er Knoten, die vermutlich näher am Ziel liegen, und expandiert gezielter.

### 2.2 Die Bewertungsfunktion f(n)

Für jeden Knoten n berechnet A*:

```
f(n) = g(n) + h(n)
```

| Term | Bedeutung |
|------|-----------|
| **g(n)** | Tatsächliche Kosten vom Start bis zu n (bekannt, akkumuliert) |
| **h(n)** | Geschätzte Restkosten von n bis zum Ziel (Heuristik) |
| **f(n)** | Geschätzte Gesamtkosten eines Pfades über n (Start → n → Ziel) |

A* expandiert immer den Knoten mit dem **kleinsten f-Wert**. Damit versucht er, den vielversprechendsten Pfad zuerst zu erkunden.

---

## 3. Admissible Heuristik

Die Heuristik h(n) muss **admissible** (zulässig) sein:

> h(n) überschätzt niemals die echten Restkosten.

Formal: Für jeden Knoten n gilt: **h(n) ≤ h*(n)**, wobei h*(n) die echten minimalen Kosten von n zum Ziel sind.

**Warum?** Nur so garantiert A*, dass ein einmal gefundenes Ziel auch wirklich optimal ist. Wenn h zu groß wäre, könnte A* einen besseren Pfad überspringen.

**Monotonie (konsistent):** Für jede Kante (n → n') mit Kantenkosten c:

> h(n) ≤ c(n, n') + h(n')

Das impliziert Admissibility und sorgt dafür, dass jeder Knoten höchstens einmal expandiert wird (kein „Zurückgehen“ zu bereits expandierten Knoten mit schlechterem f).

---

## 4. Typische Heuristiken für 2D-Raster

### 4.1 Manhattan-Distanz

Wenn sich der Roboter nur horizontal/vertikal bewegen darf (4-Nachbarn):

```
h(n) = |x_ziel - x_n| + |y_ziel - y_n|
```

Admissible, weil jede kürzeste Route mindestens so viele Schritte braucht.

### 4.2 Euklidische Distanz

Wenn diagonale Bewegung erlaubt ist (8-Nachbarn) oder für kontinuierliche Räume:

```
h(n) = √((x_ziel - x_n)² + (y_ziel - y_n)²)
```

Admissible, weil die Luftlinie die kürzeste Verbindung ist.

### 4.3 Diagonaler Abstand (Octile)

Für 8-Nachbarn mit unterschiedlichen Kosten für Diagonale vs. Gerade: Kombination aus Manhattan und Diagonale, je nach Bewegungsmodell.

---

## 5. Algorithmusablauf (Schritt für Schritt)

### 5.1 Datenstrukturen

- **Open List (Prioritätswarteschlange):** Knoten, die noch expandiert werden können. Sortiert nach f(n). Oft als Min-Heap.
- **Closed List (Set):** Bereits expandierte Knoten. Verhindert doppelte Expansion.
- **Parent-Map:** Für jeden Knoten n: Vorgänger auf dem bisher besten Pfad von Start zu n. Wird zum Rekonstruieren des Pfades genutzt.

### 5.2 Pseudocode (konzeptionell)

```
1. Open = {Start}, Closed = {}
2. g(Start) = 0, f(Start) = h(Start)
3. Solange Open nicht leer:
   a. n = Knoten in Open mit kleinstem f-Wert
   b. Wenn n == Ziel: Pfad über Parent-Map rekonstruieren, FERTIG
   c. n aus Open entfernen, zu Closed hinzufügen
   d. Für jeden Nachbar n' von n:
      - Wenn n' in Closed: überspringen
      - tentative_g = g(n) + Kantenkosten(n, n')
      - Wenn n' nicht in Open ODER tentative_g < g(n'):
         - Parent(n') = n
         - g(n') = tentative_g
         - f(n') = g(n') + h(n')
         - n' zu Open hinzufügen (bzw. f-Wert in Open aktualisieren)
4. Kein Pfad gefunden (Ziel unerreichbar)
```

### 5.3 Wichtige Details

**Update von Nachbarn:** Ein Nachbar n' kann schon in Open sein (von einem früheren Pfad). Dann nur aktualisieren, wenn der neue Weg zu n' günstiger ist (kleineres tentative_g).

**Pfad-Rekonstruktion:** Ab Ziel rückwärts über Parent(n) bis zum Start gehen, dann umdrehen.

---

## 6. 4-Nachbarn vs. 8-Nachbarn (Raster)

### 4-Nachbarn (von-Neumann-Nachbarschaft)

```
    [ ]
[N ][x][O]
    [S ]
```

Nur links, rechts, oben, unten. Kantenkosten oft 1 pro Schritt.

### 8-Nachbarn (Moore-Nachbarschaft)

```
[NW][N ][NO]
[W ][x ][O ]
[SW][S ][SO]
```

Inklusive Diagonalen. Diagonale Schrittlänge ≈ √2, daher oft Kantenkosten √2 für diagonale Schritte, 1 für gerade.

---

## 7. Kontinuierliche Räume (für Robotik)

A* arbeitet klassisch auf Graphen. Für kontinuierliche 2D-Räume:

**Diskretisierung:** Raum in Raster (Grid) oder andere Zellen einteilen. Jede Zelle = Knoten, Nachbarn = benachbarte Zellen. Hindernisse = Zellen, die nicht betretbar sind.

**Alternativen (später):**
- **Voronoi-Diagramme** als Graphen
- **Visibility Graphs**
- **RRT* / PRM** für hochdimensionale Räume (A* oft zu aufwändig)

Für den Einstieg ist ein einfaches Raster meist am übersichtlichsten.

---

## 8. Laufzeit und Speicher

- **Zeit:** O(b^d) im Worst Case, wobei b Verzweigungsfaktor (durchschnittliche Nachbaranzahl) und d Pfadtiefe. Mit guter Heuristik oft deutlich weniger als Dijkstra.
- **Speicher:** O(b^d) für Open + Closed. Kann bei großen Karten problematisch werden.

**Verbesserungen:**
- Bidirektionale Suche (von Start und Ziel gleichzeitig)
- Iterative Deepening A* (IDA*): spart Speicher, kann Zeit kosten
- Jump Point Search (JPS): für Raster mit vielen freien Flächen, reduziert expandierte Knoten

---

## 9. Spezialfälle

### Kein Pfad möglich

Wenn das Ziel von Hindernissen umschlossen ist: A* expandiert irgendwann alle erreichbaren Knoten, Open wird leer → Rückgabe „kein Pfad“.

### Gewichtete Kanten

Kanten können unterschiedliche Kosten haben (z.B. unterschiedliches Terrain). Die Kosten fließen in g(n) und damit in f(n) ein. Die Heuristik muss weiterhin admissible bleiben.

### Mehrere Ziele

Entweder A* einmal pro Ziel laufen lassen, oder ein „Ziel-Set“ verwenden: Sobald ein Knoten im Ziel-Set ist, Abbruch.

---

## 10. Checkliste für die Implementierung

- [ ] Repräsentation des Graphen/Rasters (Knoten, Nachbarn, Hindernisse)
- [ ] Heuristik h(n) wählen (Manhattan oder Euklidisch)
- [ ] Open List als Prioritätswarteschlange (Min-Heap nach f)
- [ ] Closed Set für expandierte Knoten
- [ ] Parent-Map für Pfad-Rekonstruktion
- [ ] Kantenkosten korrekt (gerade vs. diagonal)
- [ ] Behandeln von „kein Pfad“

---

## 11. Projekt-Implementierung

**Klasse:** `de.egb.controlcore.planning.AStarPathfinder`

**Verwendung:**
```java
Grid2D grid = new Grid2D(20, 15);
grid.setWalkable(5, 5, false);  // Hindernis

AStarPathfinder pathfinder = new AStarPathfinder(grid, Grid2D.Connectivity.EIGHT);
List<Grid2D.Cell> path = pathfinder.findPath(grid.cell(0, 0), grid.cell(19, 14));
```

Siehe auch [code-dokumentation.md](code-dokumentation.md) Abschnitt 4.9.

---

## 12. Literatur

- Hart, P. E.; Nilsson, N. J.; Raphael, B.: *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Transactions on Systems Science and Cybernetics, 1968.
- Russell, S.; Norvig, P.: *Artificial Intelligence: A Modern Approach* – Kapitel zu Suchalgorithmen.
