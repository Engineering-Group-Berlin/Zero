package de.egb.controlcore;

public record BallState2D(Vec2 p, Vec2 v, double t, boolean valid)
{
    public BallState2D
    {
        p = (p == null) ? Vec2.ZERO : p;
        v = (v == null) ? Vec2.ZERO : v;
    }
}
