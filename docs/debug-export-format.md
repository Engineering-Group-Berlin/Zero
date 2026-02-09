# Debug Export Format (JSONL)

Die Datei `build/debug/recording.jsonl` enthält pro Zeile einen vollständigen Weltzustand als JSON-Objekt. Jede Zeile entspricht einem Frame (ca. 100 ms Abstand).

## Format

- **Datei**: JSONL (eine JSON-Zeile pro Frame)
- **Encoding**: UTF-8
- **Programmstart**: `./gradlew run --args="--debug-export"`

---

## Vollständige Struktur

```json
{
  "t": <number>,           // Wall-clock Zeit in Sekunden (seit Programmstart)
  "tVision": <number>,     // Vision-Capture-Zeit in Sekunden (0 wenn keine Vision)
  "ball": {
    "p": { "x": <number>, "y": <number> },
    "v": { "x": <number>, "y": <number> },
    "valid": <boolean>     // true wenn Ball von Vision erkannt
  },
  "blue": [
    {
      "id": <int>,         // 0–15
      "pose": {
        "p": { "x": <number>, "y": <number> },
        "theta": <number>  // Radiant [-π, π)
      },
      "twist": {
        "v": { "x": <number>, "y": <number> },
        "omega": <number>  // rad/s
      },
      "present": <boolean> // true wenn von Vision erkannt
    }
    // ... 16 Einträge (id 0–15)
  ],
  "yellow": [
    // gleiche Struktur wie blue, 16 Einträge
  ],
  "ctrl": [
    {
      "team": <string>,       // "blue" | "yellow"
      "robotId": <int>,
      "targetPose": { "p": {...}, "theta": <number> },
      "targetTwist": { "v": {...}, "omega": <number> },
      "output": { "v": {...}, "omega": <number> }   // Se2PController-Ausgabe (m/s, rad/s)
    }
    // ... ein Eintrag pro gesteuertem Roboter (aktuell: gelber Roboter 0)
  ]
}
```

---

## Typen

| Feld     | Typ      | Einheit       | Beschreibung                                      |
|----------|----------|---------------|---------------------------------------------------|
| `t`      | number   | s             | Wall-clock Zeit seit Start                        |
| `tVision`| number   | s             | Zeitstempel der Vision-Capture                    |
| `ball.p` | Vec2     | m             | Ballposition (x, y)                               |
| `ball.v` | Vec2     | m/s           | Ballgeschwindigkeit                               |
| `ball.valid` | boolean | —          | Ball sichtbar                                     |
| `pose.p` | Vec2     | m             | Roboterposition (x, y)                            |
| `pose.theta` | number | rad       | Roboterorientierung [-π, π)                       |
| `twist.v`| Vec2     | m/s           | Roboter-Geschwindigkeit (vx, vy)                  |
| `twist.omega` | number | rad/s     | Winkelgeschwindigkeit                             |
| `present`| boolean  | —             | Roboter von Vision erkannt                        |
| **ctrl** (Se2PController) |
| `ctrl[].team` | string | —        | "blue" oder "yellow"                              |
| `ctrl[].robotId` | int | —       | Roboter-ID                                        |
| `ctrl[].targetPose` | Pose2D | m, rad | Soll-Pose (Regelungsziel)                     |
| `ctrl[].targetTwist` | Twist2D | m/s, rad/s | Soll-Twist (Feedforward)               |
| `ctrl[].output` | Twist2D | m/s, rad/s | Regelungsausgabe (v, omega)                |

---

## Beispiel (gekürzt, 2 Roboter pro Team)

```json
{
  "t": 1.234,
  "tVision": 1.228,
  "ball": {
    "p": {"x": 0.12, "y": -0.05},
    "v": {"x": 0, "y": 0},
    "valid": true
  },
  "blue": [
    {"id": 0, "pose": {"p": {"x": -0.5, "y": 0.3}, "theta": 0.1}, "twist": {"v": {"x": 0, "y": 0}, "omega": 0}, "present": true},
    {"id": 1, "pose": {"p": {"x": 0, "y": 0}, "theta": 0}, "twist": {"v": {"x": 0, "y": 0}, "omega": 0}, "present": false}
  ],
  "yellow": [
    {"id": 0, "pose": {"p": {"x": 0.5, "y": -0.2}, "theta": 3.14}, "twist": {"v": {"x": 0, "y": 0}, "omega": 0}, "present": true},
    {"id": 1, "pose": {"p": {"x": 0, "y": 0}, "theta": 0}, "twist": {"v": {"x": 0, "y": 0}, "omega": 0}, "present": false}
  ],
  "ctrl": [
    {
      "team": "yellow",
      "robotId": 0,
      "targetPose": {"p": {"x": 1.0, "y": 0.5}, "theta": 0},
      "targetTwist": {"v": {"x": 0, "y": 0}, "omega": 0},
      "output": {"v": {"x": 0.3, "y": 0.1}, "omega": 0.05}
    }
  ]
}
```

**Hinweise:**
- In der Datei sind immer 16 Roboter pro Team, auch wenn viele `present: false` haben.
- `ctrl` enthält die Se2PController-Ausgaben (aktuell: gelber Roboter 0 fährt zum Ziel (1.0, 0.5)).

---

## Verarbeitung (z.B. Visualisierung)

1. Datei zeilenweise lesen
2. Pro Zeile: `JSON.parse(line)` → ein Frame
3. Frames nach `t` sortieren (optional, falls nötig)
4. Ball zeichnen wenn `ball.valid`
5. Roboter zeichnen wenn `present: true`
6. `ctrl`: Soll-Pose und Ausgabe (v, omega) pro gesteuertem Roboter visualisieren (z.B. Pfeil für Soll-Richtung, Ausgabe als Geschwindigkeitsvektor)
7. Koordinaten: SSL-Feld, Meter (x: Längsachse, y: Querachse)
