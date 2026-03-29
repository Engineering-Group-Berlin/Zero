package de.egb.controlcore;

public record Goal2D(
        Vec2 p,
        double theta,
        double vEnd,
        double omegaEnd,
        double posTol,
        double thetaTol
)
{
    public Goal2D
    {
        p = (p == null) ? Vec2.ZERO : p;
        theta = Math2D.wrapToPi(theta);

        if(posTol <= 0) throw new IllegalArgumentException("posTol must be > 0");
        if(thetaTol <= 0) throw new IllegalArgumentException("thetaTol must be > 0");
    }

    public static Goal2D defaultStop(Vec2 p, double theta)
    {
        return new Goal2D(
                p, theta,
                0.0, 0.0,
                0.002,
                Math.toRadians(0.5)
        );
    }
}
