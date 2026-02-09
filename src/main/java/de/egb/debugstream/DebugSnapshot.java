package de.egb.debugstream;

import de.egb.controlcore.Pose2D;
import de.egb.controlcore.Twist2D;
import de.egb.controlcore.Vec2;
import de.egb.controlcore.WorldState;

import java.util.ArrayList;
import java.util.List;

/**
 * Ein vollständiger Snapshot der Welt für Debug-Export und Visualisierung.
 * Wird als JSON-Zeile geschrieben (JSONL). Externe Tools können damit
 * Trajektorien, Ball, Roboter und Controller-Ausgaben animieren.
 * Nutzt Pose2D, Twist2D, Vec2 aus dem controlcore.
 */
public record DebugSnapshot(
        double t,           // Wall-clock Zeit (s seit Start)
        double tVision,     // Vision-Capture-Zeit (s)
        BallData ball,
        List<RobotData> blue,
        List<RobotData> yellow,
        List<CtrlData> ctrl  // Se2PController-Ausgaben pro gesteuertem Roboter
) {
    /** Ball-Position (p), Geschwindigkeit (v), gültig wenn sichtbar. */
    public record BallData(Vec2 p, Vec2 v, boolean valid) {}
    /** Roboter: id, Pose2D, Twist2D, present. */
    public record RobotData(int id, Pose2D pose, Twist2D twist, boolean present) {}
    /** Controller: team, robotId, Soll-Pose, Soll-Twist, Ausgabe (v, omega). */
    public record CtrlData(String team, int robotId, Pose2D targetPose, Twist2D targetTwist, Twist2D output) {}

    public static DebugSnapshot from(WorldState ws, double tWall) {
        return from(ws, tWall, List.of());
    }

    public static DebugSnapshot from(WorldState ws, double tWall, List<CtrlData> ctrl) {
        BallData ball = new BallData(ws.ball.p(), ws.ball.v(), ws.ball.valid());
        List<RobotData> blue = robotList(ws.blue.robots);
        List<RobotData> yellow = robotList(ws.yellow.robots);
        return new DebugSnapshot(tWall, ws.t, ball, blue, yellow, ctrl != null ? ctrl : List.of());
    }

    private static List<RobotData> robotList(WorldState.RobotState[] robots) {
        List<RobotData> list = new ArrayList<>(robots.length);
        for (WorldState.RobotState rs : robots) {
            list.add(new RobotData(
                    rs.id.value(),
                    rs.state.pose(),
                    rs.state.twist(),
                    rs.present
            ));
        }
        return list;
    }
}
