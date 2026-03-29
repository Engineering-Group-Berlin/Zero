package de.egb.controlcore;

public final class Math2D
{
    private Math2D() {}

    public static double dot(Vec2 a, Vec2 b)
    {
        return a.x() * b.x() + a.y() * b.y();
    }

    public static double norm2(Vec2 v)
    {
        return dot(v, v);
    }

    public static double norm(Vec2 v)
    {
        return Math.sqrt(norm2(v));
    }

    public static Vec2 normalized(Vec2 v)
    {
        double n = norm(v);
        if(n < 1e-12) return Vec2.ZERO;
        return new Vec2(v.x() / n, v.y() /n );
    }

    public static double clamp(double v, double lo, double hi)
    {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    public static double wrapToPi(double a)
    {
        double twoPi = 2.0 * Math.PI;
        while(a <= -Math.PI) a += twoPi;
        while(a > Math.PI) a -= twoPi;

        return a;
    }

    public static Vec2 clampVecNorm(Vec2 v, double maxNorm)
    {
        double n = norm(v);
        if(n < 1e-12 || n <= maxNorm) return v;
        double s = maxNorm / n;
        return v.mul(s);
    }
}
