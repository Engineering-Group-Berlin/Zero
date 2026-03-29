package de.egb.controlcore;

public record Pose2D(Vec2 p, double theta)
{
    public Pose2D
    {
        p = (p == null) ? Vec2.ZERO : p;
        theta = Math2D.wrapToPi(theta);
    }
}
