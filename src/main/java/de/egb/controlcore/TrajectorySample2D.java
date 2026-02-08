package de.egb.controlcore;

public record TrajectorySample2D(Pose2D pose, Twist2D twist, double t)
{
    public TrajectorySample2D
    {
        if(pose == null) throw new IllegalArgumentException("pose must not be null");
        twist = (twist == null) ? new Twist2D(Vec2.ZERO, 0.0) : twist;
    }
}
