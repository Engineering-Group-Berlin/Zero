package de.egb.controlcore;

public record Limits(
        double vMax, double aMax, double jerkMax,
        double omegaMax, double alphaMax, double angJerMax,
        double vDeadband, double omegaDeadband
)
{
    public static Limits defaults() {
        return new Limits(
                4.0, 3.0, 0.0,
            8.0, 20.0, 0.0,
                0.0, 0.0
        );
    }
}
