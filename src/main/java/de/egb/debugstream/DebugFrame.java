package de.egb.debugstream;

import de.egb.controlcore.Pose2D;
import de.egb.controlcore.Twist2D;

public record DebugFrame(
        double t,
        int robotId,
        Pose2D pose,
        Twist2D vel,
        Pose2D target,
        Twist2D ctrl
) {}
