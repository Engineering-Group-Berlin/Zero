package de.egb.vision;

import de.egb.controlcore.*;
import de.npe.proto.vision.SslVisionDetection;
import de.npe.proto.vision.SslVisionGeometry;
import de.npe.proto.vision.SslVisionWrapper;

import java.net.*;
import java.util.Arrays;

public class VisionReceiver implements Runnable {
    private static final String VISION_ADDRESS = "224.5.23.2";
    private static final int VISION_PORT = 10020;

    private final WorldStateProvider worldStateProvider;

    public VisionReceiver(WorldStateProvider worldStateProvider) {
        this.worldStateProvider = worldStateProvider;
    }

    @Override
    public void run() {
        try {
            MulticastSocket socket = new MulticastSocket(VISION_PORT);
            InetAddress group = InetAddress.getByName(VISION_ADDRESS);
            socket.joinGroup(group);

            byte[] buffer = new byte[65536];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);

            System.out.println("Vision receiver started on " + VISION_ADDRESS + ":" + VISION_PORT);

            while (!Thread.currentThread().isInterrupted()) {
                socket.receive(packet);

                parseVisionPacket(packet.getData(), packet.getLength());
            }

            socket.leaveGroup(group);
            socket.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void parseVisionPacket(byte[] data, int length) {
        try {
            byte[] actualData = Arrays.copyOf(data, length);
            SslVisionWrapper.SSL_WrapperPacket packet = SslVisionWrapper.SSL_WrapperPacket.parseFrom(actualData);

            // Hat das Paket Detection-Daten?
            if (packet.hasDetection()) {
                processDetection(packet.getDetection());
            }

            // Hat das Paket Geometry-Daten?
            if (packet.hasGeometry()) {
                processGeometry(packet.getGeometry());
            }

        } catch (Exception e) {
            System.err.println("Error parsing: " + e.getMessage());
        }
    }

    private void processDetection(SslVisionDetection.SSL_DetectionFrame detection) {
        double t = detection.getTCapture();

        // Ball: Vision liefert nur Position, keine Geschwindigkeit
        BallState2D ball;
        if (detection.getBallsCount() > 0) {
            SslVisionDetection.SSL_DetectionBall b = detection.getBalls(0);
            Vec2 p = new Vec2(b.getX() / 1000.0, b.getY() / 1000.0);
            ball = new BallState2D(p, Vec2.ZERO, t, true);
        } else {
            ball = new BallState2D(Vec2.ZERO, Vec2.ZERO, t, false);
        }

        // Roboter pro Team
        WorldState.TeamState blue = buildTeamState(TeamColor.BLUE, detection.getRobotsBlueList(), t);
        WorldState.TeamState yellow = buildTeamState(TeamColor.YELLOW, detection.getRobotsYellowList(), t);

        WorldState state = new WorldState(t, ball, blue, yellow);
        worldStateProvider.update(state);
    }

    private static WorldState.TeamState buildTeamState(
            TeamColor color,
            java.util.List<SslVisionDetection.SSL_DetectionRobot> detected,
            double t) {
        WorldState.RobotState[] robots = new WorldState.RobotState[WorldState.MAX_ROBOTS_PER_TEAM];
        State2D emptyState = State2D.poseOnly(new Pose2D(Vec2.ZERO, 0.0), t);

        for (int i = 0; i < robots.length; i++) {
            robots[i] = new WorldState.RobotState(new RobotId(i), emptyState, false);
        }

        for (SslVisionDetection.SSL_DetectionRobot r : detected) {
            int id = r.getRobotId();
            if (id < 0 || id >= WorldState.MAX_ROBOTS_PER_TEAM) continue;

            Vec2 p = new Vec2(r.getX() / 1000.0, r.getY() / 1000.0);
            double theta = r.getOrientation();
            Pose2D pose = new Pose2D(p, theta);
            State2D state = State2D.poseOnly(pose, t);

            robots[id] = new WorldState.RobotState(new RobotId(id), state, true);
        }

        return new WorldState.TeamState(color, robots);
    }

    private void processGeometry(SslVisionGeometry.SSL_GeometryData geometry) {
        SslVisionGeometry.SSL_GeometryFieldSize field = geometry.getField();

        System.out.println("Field dimensions:");
        System.out.println("  Length: " + field.getFieldLength() + " mm");
        System.out.println("  Width: " + field.getFieldWidth() + " mm");
        System.out.println("  Goal width: " + field.getGoalWidth() + " mm");
        System.out.println("  Goal depth: " + field.getGoalDepth() + " mm");
    }
}
