package de.egb.controlcore;

public record Twist2D(Vec2 v, double omega)
{
    public Twist2D
    {
        v = (v == null) ? Vec2.ZERO : v;
    }
}
