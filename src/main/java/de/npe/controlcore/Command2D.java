package controlcore;

public record Command2D(
        Vec2 v,
        double omega,
        double kickSpeed,
        double dribblerRpm,
        boolean valid
)
{
    public Command2D
    {
        v = (v == null) ? Vec2.ZERO : v;
    }

    public static Command2D stop()
    {
        return new Command2D(Vec2.ZERO, 0.0, 0.0, 0.0, true);
    }
}
