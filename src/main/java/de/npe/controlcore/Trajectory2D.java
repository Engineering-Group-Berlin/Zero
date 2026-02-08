package controlcore;

public record Trajectory2D(double t0, double tf, boolean valid)
{
    public static Trajectory2D invalid()
    {
        return new Trajectory2D(0.0, 0.0, false);
    }
}
