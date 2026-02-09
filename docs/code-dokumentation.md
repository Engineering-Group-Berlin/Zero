# SSL Zero AI – Code-Dokumentation

Umfassende technische Dokumentation der Codebasis für die SSL (Small Size League) Roboter-Steuerung.

---

## Inhaltsverzeichnis

1. [Projektüberblick](#1-projektüberblick)
2. [Architektur](#2-architektur)
3. [Einstiegspunkt & Anwendung](#3-einstiegspunkt--anwendung)
4. [Controlcore – Geometrie & Steuerung](#4-controlcore--geometrie--steuerung)
5. [Vision](#5-vision)
6. [Debug-Stream](#6-debug-stream)
7. [Tests](#7-tests)
8. [Build & Abhängigkeiten](#8-build--abhängigkeiten)

---

## 1. Projektüberblick

**SSL Zero AI** ist eine Java-Anwendung für die Steuerung von Robotern in der RoboCup Small Size League. Sie empfängt Vision-Daten per Multicast, hält einen Weltzustand und kann Roboter mit P-Controllern ansteuern. Optional wird ein Debug-Export für Visualisierung bereitgestellt.

**Technologie-Stack:**
- Java 21
- Gradle mit Protobuf-Plugin
- Google Protobuf für SSL Vision/Referee-Protokolle
- Jackson für JSON-Serialisierung

---

## 2. Architektur

```
de.egb
├── Main.java                 # Einstiegspunkt
├── core/
│   └── App.java             # Zentrale Anwendung (Init, Loop, Shutdown)
├── controlcore/             # Geometrie, Weltzustand, Regler, Trajektorien, Path Planning
│   ├── control/             # PController, Se2PController
│   ├── planning/            # Grid2D, AStarPathfinder
│   └── tests/               # TrajectoryAndPControllerTest, TrajectoryDebugMain
├── vision/
│   └── VisionReceiver.java  # Multicast-Empfang SSL Vision
├── debugstream/             # DebugSnapshot, JSONFrameWriter
└── referee/
    └── RefereeReceiver.java # (Stub, aktuell leer)
```

**Datenfluss:**
1. **VisionReceiver** (Thread) → empfängt UDP-Multicast → parst Protobuf → aktualisiert **WorldStateProvider**
2. **App.mainLoop** → liest `WorldStateProvider.getCurrent()` → berechnet Se2PController-Outputs → schreibt **DebugSnapshot** (optional)
3. **WorldState** ist immutable und thread-sicher über `volatile` im Provider

---

## 3. Einstiegspunkt & Anwendung

### 3.1 Main.java

**Pfad:** `de.egb.Main`

Einstiegspunkt. Startet entweder:
- **`--traj-test`**: Führt `TrajectoryAndPControllerTest.run()` aus und beendet
- **Standard**: Erstellt `App`, initialisiert mit `args`, startet `run()`, registriert Shutdown-Hook

**Kommandozeilen-Argumente:**
| Argument       | Beschreibung                                           |
|----------------|--------------------------------------------------------|
| `--traj-test`  | Nur Trajektorien- und P-Controller-Test, dann Exit     |
| `--path-planning-test` | Path Planning + Trajektorien + Controller Integrationstest |
| `--debug-export` | Aktiviert Debug-Export nach `build/debug/recording.jsonl` |

### 3.2 App.java

**Pfad:** `de.egb.core.App`

Zentrale Anwendungsklasse. Zuständigkeiten:

| Methode / Bereich    | Beschreibung                                                      |
|----------------------|-------------------------------------------------------------------|
| `initialize(args)`   | Parst `--debug-export`, erstellt WorldStateProvider, VisionReceiver, ggf. JSONFrameWriter + Se2PController |
| `run()`              | Startet Vision-Thread, ruft `mainLoop()` auf                      |
| `mainLoop()`         | Schleife mit 100 ms Takt; bei Debug-Export: `performDebugExport()` |
| `performDebugExport()` | Erstellt `DebugSnapshot` aus WorldState + Se2PController-Ausgaben für alle Roboter |
| `shutdown()`         | Stoppt Vision-Thread (Interrupt), schließt Debug-Writer            |

**Konfiguration:**
- `stdDeltaTimeMs` / `actualDeltaTimeMs`: Taktzeit der Hauptschleife (Standard: 100 ms)
- Debug-Controller-Ziel: `Pose2D(1.0, 0.5), theta=0` mit `Twist2D(0, 0)`

---

## 4. Controlcore – Geometrie & Steuerung

Paket: `de.egb.controlcore`

### 4.1 Geometrische Typen

| Klasse    | Beschreibung                                              |
|-----------|-----------------------------------------------------------|
| **Vec2**  | 2D-Vektor `(x, y)`, Methoden: `add`, `sub`, `mul`         |
| **Pose2D**| Position + Orientierung: `Vec2 p`, `double theta` (rad)   |
| **Twist2D** | Geschwindigkeit: `Vec2 v`, `double omega` (rad/s)      |
| **State2D** | Vollständiger Zustand: Pose, Twist, Zeitstempel, Frame, Validitätsflags |
| **Frame2D** | Enum: `WORLD`, `ROBOT` (Referenzrahmen)                 |

### 4.2 Zusätzliche Datentypen

| Klasse         | Beschreibung                                              |
|----------------|-----------------------------------------------------------|
| **BallState2D**| Ball: `Vec2 p`, `Vec2 v`, `double t`, `boolean valid`     |
| **RobotId**    | Record `(int value)`, 0–255                               |
| **TeamColor**  | Enum: `BLUE`, `YELLOW`                                    |
| **Command2D**  | Roboterbefehl: v, omega, kickSpeed, dribblerRpm, valid    |
| **Goal2D**     | Ziel: p, theta, vEnd, omegaEnd, posTol, thetaTol          |
| **Limits**     | vMax, aMax, jerkMax, omegaMax, alphaMax, Deadbands        |

### 4.3 WorldState & WorldStateProvider

**WorldState** (immutable):
- `double t` – Zeitstempel
- `BallState2D ball`
- `TeamState blue`, `TeamState yellow` – je 16 Roboter

**WorldState.RobotState:** `RobotId`, `State2D`, `boolean present`

**WorldStateProvider:**
- Thread-sicherer Holder für aktuellen `WorldState`
- `getCurrent()` – aktueller Zustand
- `update(WorldState)` – Aktualisierung aus Vision

### 4.4 Math2D

Statische Hilfsfunktionen:
- `dot`, `norm`, `norm2`, `normalized`, `clampVecNorm` für Vec2
- `clamp(v, lo, hi)` – Skalar
- `wrapToPi(a)` – Winkel auf [-π, π)

### 4.5 PController

**Pfad:** `de.egb.controlcore.control.PController`

Proportional-Controller für einen SISO-Kanal:
- `update(setpoint, measurement, dt)` – Regelung auf Sollwert
- `updateError(error, dt)` – direkt mit Fehler
- Deadband, Slew-Rate-Limit, Ausgangs-Begrenzung

### 4.6 Se2PController

**Pfad:** `de.egb.controlcore.control.Se2PController`

SE(2)-P-Regler (Position + Orientierung):
- Drei interne PController: x, y, theta
- `update(poseMeas, poseRef, twistRef, dt)` → `Command(v, omega)`
- Vektor-Begrenzung für v, Skalar-Begrenzung für omega

### 4.7 Trajektorien

| Klasse                   | Beschreibung                                                   |
|--------------------------|----------------------------------------------------------------|
| **Trajectory2D**         | Record: t0, tf, valid                                          |
| **TrajectorySample2D**   | Ein Sample: Pose, Twist, t                                     |
| **TrapezoidalTrajectory2D** | Trapezoidprofil von Start zu Ziel, XY + Theta getrennt      |

**TrapezoidalTrajectory2D:**
- Konstruktor: `(State2D start, Goal2D goal, Limits limits)`
- `sample(t)` → `TrajectorySample2D`
- Getrennte Timing-Berechnung für Position und Orientierung

### 4.8 Limits

`Limits.defaults()` liefert:
- vMax: 4.0 m/s, aMax: 3.0, omegaMax: 8.0 rad/s, alphaMax: 20.0
- Deadbands: 0

### 4.9 Path Planning (Grid2D, AStarPathfinder)

**Paket:** `de.egb.controlcore.planning`

**Grid2D:** 2D-Raster mit Begehbarkeit (`walkable`) und Kosten pro Zelle (`cost`). Ermöglicht 4- oder 8-Nachbarschaft mit optionaler Corner-Cutting-Verhinderung. Siehe [grid2d-dokumentation.md](grid2d-dokumentation.md).

**AStarPathfinder:** A*-Implementierung für Grid2D.
- `findPath(start, goal)` → `List<Cell>` (Pfad inkl. Start und Ziel, oder leer wenn kein Pfad)
- Heuristik: `MANHATTAN` oder `EUCLIDEAN`
- Connectivity: `FOUR` oder `EIGHT`
- `preventCornerCutting` bei 8-Nachbarn (Standard: true)
- Bewegungs kosten: Kardinal = Zellkosten, Diagonal = Zellkosten × √2

---

## 5. Vision

### 5.1 VisionReceiver

**Pfad:** `de.egb.vision.VisionReceiver`

`Runnable`, läuft in eigenem Thread:
- Multicast: `224.5.23.2:10020` (SSL Vision Standard)
- Parst `SSL_WrapperPacket` (Protobuf)
- **Detection**: Ball + Roboter Blue/Yellow → `WorldState` → `WorldStateProvider.update()`
- **Geometry**: Feldmaße werden geloggt

Koordinatenumrechnung: Vision liefert mm → intern m (Division durch 1000).

---

## 6. Debug-Stream

### 6.1 DebugSnapshot

**Pfad:** `de.egb.debugstream.DebugSnapshot`

Record für einen Welt-Snapshot:
- `t` – Wall-Clock-Zeit (s)
- `tVision` – Vision-Zeitstempel
- `BallData` – p, v, valid
- `blue`, `yellow` – Listen von `RobotData` (id, pose, twist, present)
- `ctrl` – Liste von `CtrlData` (team, robotId, targetPose, targetTwist, output)

`DebugSnapshot.from(WorldState, tWall)` bzw. `.from(ws, tWall, ctrlList)` erstellt Snapshot.

### 6.2 JSONFrameWriter

**Pfad:** `de.egb.debugstream.JSONFrameWriter`

Schreibt Objekte als JSON-Zeilen (JSONL):
- Konstruktor: `(Path file, boolean append)`
- `write(obj)`, `flush()`, `close()`
- Legt übergeordnete Verzeichnisse an

### 6.3 DebugFrame

**Pfad:** `de.egb.debugstream.DebugFrame`

Einfacher Record: t, robotId, pose, vel, target, ctrl (für alternative Nutzung).

### 6.4 Debug-Export-Format

Detaillierte Beschreibung siehe: [docs/debug-export-format.md](debug-export-format.md)

---

## 7. Tests

### 7.1 PathPlanningIntegrationTest

**Pfad:** `de.egb.controlcore.tests.PathPlanningIntegrationTest`  
**Aufruf:** `./gradlew run --args="--path-planning-test"`

Integrations-Test für Path Planning, Trajektorien und Controller:
1. Erstellt Grid2D mit Hindernissen, A* findet Pfad
2. Konvertiert Zellen zu Weltkoordinaten (Vec2)
3. Baut Trajektorien-Kette (TrapezoidalTrajectory2D pro Segment)
4. Simuliert Roboter mit Se2PController (50 Hz, Euler-Integration)
5. Gibt Raster-Visualisierung, Positionsverlauf und Ergebnis aus

### 7.2 TrajectoryAndPControllerTest

**Pfad:** `de.egb.controlcore.tests.TrajectoryAndPControllerTest`

Integrationstest:
- Erzeugt `TrapezoidalTrajectory2D` von (0,0) nach (2, 1.2), theta 20° → 140°
- Simuliert P-Controller + kinematisches Modell (Euler-Integration)
- Prüft: Befehle innerhalb Limits, Endfehler unter Toleranz
- Aufruf: `./gradlew run --args="--traj-test"`

### 7.3 TrajectoryDebugMain

**Pfad:** `de.egb.controlcore.tests.TrajectoryDebugMain`

Standalone-Debug-Programm: gleiche Logik wie der Test, mit `Se2PController`, gibt Position/Fehler aus.

---

## 8. Build & Abhängigkeiten

### 8.1 Gradle

- **Hauptklasse:** `de.egb.Main`
- **Java:** 21
- **Plugins:** java, application, protobuf

### 8.2 Wichtige Abhängigkeiten

| Abhängigkeit              | Verwendung                               |
|---------------------------|------------------------------------------|
| protobuf-java             | SSL Vision / Referee Protobuf            |
| jackson-databind          | JSON für Debug-Export                    |
| slf4j, logback            | Logging                                  |
| commons-math3             | Mathematische Hilfen                     |

### 8.3 Protobuf

Proto-Dateien unter `src/main/proto/`:
- `vision/` – ssl_vision_detection, ssl_vision_geometry, ssl_vision_wrapper
- Weitere SSL GC / API / State / Referee-Definitionen

Generierter Code: `build/generated/source/proto/main/java`

### 8.4 Befehle

```bash
./gradlew build          # Kompilieren + Tests
./gradlew run            # App starten
./gradlew run --args="--debug-export"   # Mit Debug-Export
./gradlew run --args="--traj-test"      # Nur Trajektorien-Test
```

---

## Anhang: RefereeReceiver

**Pfad:** `de.egb.referee.RefereeReceiver`

Aktuell leerer Stub; vorgesehen für zukünftigen Referee-Multicast-Empfang.
