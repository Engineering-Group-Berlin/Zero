package de.npe.vision;

import de.npe.proto.vision.SslVisionDetection;
import de.npe.proto.vision.SslVisionGeometry;
import de.npe.proto.vision.SslVisionWrapper;

import java.net.*;
import java.util.Arrays;

public class VisionReceiver implements Runnable {
    private static final String VISION_ADDRESS = "224.5.23.2";
    private static final int VISION_PORT = 10020;

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
        // Frame-Info
        System.out.println("Frame #" + detection.getFrameNumber() +
                " @ " + detection.getTCapture() + "s");

        // Baelle
        for (SslVisionDetection.SSL_DetectionBall ball : detection.getBallsList()) {
            double x = ball.getX() / 1000.0;  // mm -> m
            double y = ball.getY() / 1000.0;
            System.out.printf("  Ball: (%.2f, %.2f) m%n", x, y);
        }

        // Gelbe Roboter
        System.out.println("  Yellow robots: " + detection.getRobotsYellowCount());
        for (SslVisionDetection.SSL_DetectionRobot robot : detection.getRobotsYellowList()) {
            double x = robot.getX() / 1000.0;
            double y = robot.getY() / 1000.0;
            double angle = Math.toDegrees(robot.getOrientation());
            System.out.printf("    Robot %d: (%.2f, %.2f) @ %.0f°%n",
                    robot.getRobotId(), x, y, angle);
        }

        // Blaue Roboter
        System.out.println("  Blue robots: " + detection.getRobotsBlueCount());
        for (SslVisionDetection.SSL_DetectionRobot robot : detection.getRobotsBlueList()) {
            double x = robot.getX() / 1000.0;
            double y = robot.getY() / 1000.0;
            double angle = Math.toDegrees(robot.getOrientation());
            System.out.printf("    Robot %d: (%.2f, %.2f) @ %.0f°%n",
                    robot.getRobotId(), x, y, angle);
        }
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
