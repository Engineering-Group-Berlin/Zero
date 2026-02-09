package de.egb.controlcore.tests;

import de.egb.controlcore.*;
import de.egb.controlcore.control.Se2PController;
import de.egb.controlcore.planning.AStarPathfinder;
import de.egb.controlcore.planning.Grid2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Integrations-Test: Path Planning (A*) + Trajektorien + Se2PController.
 * Simuliert einen Roboter, der einen A*-Pfad abfährt.
 */
public final class PathPlanningIntegrationTest {

    private static final double CELL_SIZE = 0.5;  // Meter pro Rasterzelle
    private static final double DT = 0.02;        // 50 Hz Simulationsschritt

    public static void run() {
        Locale.setDefault(Locale.US);

        System.out.println("=== Path Planning Integration Test ===\n");

        // ---- 1. A* Path Planning ----
        System.out.println("1. Grid + A* Path Planning");
        Grid2D grid = createTestGrid();
        Grid2D.Cell startCell = grid.cell(1, 1);
        Grid2D.Cell goalCell = grid.cell(14, 10);

        AStarPathfinder pathfinder = new AStarPathfinder(grid, Grid2D.Connectivity.EIGHT);
        List<Grid2D.Cell> path = pathfinder.findPath(startCell, goalCell);

        if (path.isEmpty()) {
            System.err.println("FEHLER: Kein Pfad gefunden!");
            return;
        }
        System.out.println("   Pfad gefunden: " + path.size() + " Wegpunkte");
        printGridWithPath(grid, path);

        // ---- 2. Pfad zu Weltkoordinaten (Vec2) ----
        System.out.println("\n2. Umrechnung in Weltkoordinaten (m)");
        List<Vec2> waypoints = cellsToWaypoints(path);
        System.out.println("   Start: " + waypoints.get(0) + " m");
        System.out.println("   Ziel:  " + waypoints.getLast() + " m");

        // ---- 3. Trajektorie entlang des Pfades ----
        System.out.println("\n3. Trajektorien-Kette (TrapezoidalTrajectory2D pro Segment)");
        Limits limits = Limits.defaults();
        List<TrapezoidalTrajectory2D> trajectorySegments = buildTrajectoryChain(waypoints, limits);
        if (trajectorySegments.isEmpty()) {
            System.err.println("   FEHLER: Pfad hat nur einen Wegpunkt, keine Trajektorie möglich.");
            return;
        }
        double totalDuration = trajectorySegments.getLast().tf();
        System.out.println("   Segmente: " + trajectorySegments.size());
        System.out.println("   Gesamtdauer: " + String.format("%.2f", totalDuration) + " s");

        // ---- 4. Controller + Simulation ----
        System.out.println("\n4. Simulation: Se2PController folgt Trajektorie");
        Se2PController controller = new Se2PController(2.0, 4.0, limits);

        Pose2D robotPose = new Pose2D(waypoints.get(0), 0.0);
        double t = 0;
        int step = 0;
        double maxPosErr = 0;
        double finalPosErr = 0;

        while (t <= totalDuration + 1.0) {
            TrajectorySample2D ref = sampleChainedTrajectory(trajectorySegments, t);
            if (ref == null) ref = trajectorySegments.getLast().sample(totalDuration);

            Pose2D poseRef = ref.pose();
            Twist2D twistRef = ref.twist();

            Se2PController.Command cmd = controller.update(robotPose, poseRef, twistRef, DT);

            // Kinematik-Integration (Euler)
            Vec2 pNext = robotPose.p().add(cmd.v().mul(DT));
            double thNext = Math2D.wrapToPi(robotPose.theta() + cmd.omega() * DT);
            robotPose = new Pose2D(pNext, thNext);

            double posErr = Math2D.norm(poseRef.p().sub(robotPose.p()));
            maxPosErr = Math.max(maxPosErr, posErr);
            if (t >= totalDuration) finalPosErr = posErr;

            if (step % 25 == 0) {
                System.out.printf("   t=%.2f  pos=(%.2f, %.2f)  ref=(%.2f, %.2f)  ePos=%.3f m%n",
                        t, robotPose.p().x(), robotPose.p().y(),
                        poseRef.p().x(), poseRef.p().y(), posErr);
            }

            t += DT;
            step++;
        }

        // ---- 5. Ergebnis ----
        Vec2 goalPos = waypoints.getLast();
        double distToGoal = Math2D.norm(goalPos.sub(robotPose.p()));
        System.out.println("\n5. Ergebnis");
        System.out.println("   Endposition: " + robotPose.p());
        System.out.println("   Soll-Ziel:   " + goalPos);
        System.out.println("   Abstand zum Ziel: " + String.format("%.4f", distToGoal) + " m");
        System.out.println("   Max. Positionsfehler während Fahrt: " + String.format("%.4f", maxPosErr) + " m");
        System.out.println("   Endfehler (am Trajektorienende): " + String.format("%.4f", finalPosErr) + " m");

        if (distToGoal < 0.1) {
            System.out.println("\n   PASSED: Roboter erreicht Ziel.");
        } else {
            System.out.println("\n   WARNUNG: Roboter hat Ziel nicht genau erreicht (Toleranz 0.1 m).");
        }
    }

    private static Grid2D createTestGrid() {
        int w = 16;
        int h = 12;
        Grid2D grid = new Grid2D(w, h);

        // Hindernis-Wand in der Mitte
        for (int y = 3; y <= 8; y++) {
            grid.setWalkable(7, y, false);
            grid.setWalkable(8, y, false);
        }
        // Kleines Hindernis
        grid.setWalkable(4, 5, false);
        grid.setWalkable(4, 6, false);

        return grid;
    }

    private static List<Vec2> cellsToWaypoints(List<Grid2D.Cell> path) {
        List<Vec2> wp = new ArrayList<>(path.size());
        for (Grid2D.Cell c : path) {
            // Zellmittelpunkt in Meter
            double wx = (c.x + 0.5) * CELL_SIZE;
            double wy = (c.y + 0.5) * CELL_SIZE;
            wp.add(new Vec2(wx, wy));
        }
        return wp;
    }

    private static List<TrapezoidalTrajectory2D> buildTrajectoryChain(List<Vec2> waypoints, Limits limits) {
        List<TrapezoidalTrajectory2D> segments = new ArrayList<>();

        if (waypoints.size() < 2) return segments;

        double t = 0;
        double theta = 0;

        for (int i = 0; i < waypoints.size() - 1; i++) {
            Vec2 p0 = waypoints.get(i);
            Vec2 p1 = waypoints.get(i + 1);

            double dx = p1.x() - p0.x();
            double dy = p1.y() - p0.y();
            double nextTheta = (Math.abs(dx) < 1e-9 && Math.abs(dy) < 1e-9)
                    ? theta
                    : Math.atan2(dy, dx);

            State2D start = State2D.poseOnly(new Pose2D(p0, theta), t);
            Goal2D goal = Goal2D.defaultStop(p1, nextTheta);

            TrapezoidalTrajectory2D seg = new TrapezoidalTrajectory2D(start, goal, limits);
            segments.add(seg);

            t = seg.tf();
            theta = nextTheta;
        }
        return segments;
    }

    private static TrajectorySample2D sampleChainedTrajectory(List<TrapezoidalTrajectory2D> segments, double t) {
        for (TrapezoidalTrajectory2D seg : segments) {
            if (t <= seg.tf()) return seg.sample(t);
        }
        return null;
    }

    private static void printGridWithPath(Grid2D grid, List<Grid2D.Cell> path) {
        var pathSet = new java.util.HashSet<>(path);
        Grid2D.Cell startCell = path.getFirst();
        Grid2D.Cell goalCell = path.getLast();
        System.out.println("   Raster (S=Start, G=Ziel, #=Hindernis, .=Pfad):");
        for (int y = 0; y < grid.height(); y++) {
            StringBuilder row = new StringBuilder("   ");
            for (int x = 0; x < grid.width(); x++) {
                if (!grid.isWalkable(x, y)) row.append("#");
                else if (x == startCell.x && y == startCell.y) row.append("S");
                else if (x == goalCell.x && y == goalCell.y) row.append("G");
                else if (pathSet.contains(grid.cell(x, y))) row.append(".");
                else row.append(" ");
            }
            System.out.println(row);
        }
    }
}
