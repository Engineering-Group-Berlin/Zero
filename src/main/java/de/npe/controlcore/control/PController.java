package controlcore.control;

import controlcore.Math2D;

public final class PController
{
    private final double kp;
    private final double uMin;
    private final double uMax;

    private final double deadband;
    private final double maxDeltaPerSec;

    private double lastU = 0.0;

    public PController(double kp, double uMin, double uMax, double deadband, double maxDeltaPerSec)
    {
        if(uMin > uMax)
            throw new IllegalArgumentException("uMin > uMax");
        if(deadband < 0)
            throw new IllegalArgumentException("deadband < 0");
        if(maxDeltaPerSec < 0)
            throw new IllegalArgumentException("maxDeltaPerSec < 0");

        this.kp = kp;
        this.uMin = uMin;
        this.uMax = uMax;
        this.deadband = deadband;
        this.maxDeltaPerSec = maxDeltaPerSec;
    }

    public double update(double setpoint, double measurement, double dtSec)
    {
        if(dtSec <= 0)
            throw new IllegalArgumentException("dtSec must be > 0");

        double error = setpoint - measurement;

        //Deadband:
        if(Math.abs(error) <= deadband)
            return applySlewAndClamp(0.0, dtSec);
        double uRaw = kp * error;
        return applySlewAndClamp(uRaw, dtSec);
    }

    public double updateError(double error, double dtSec)
    {
        if(dtSec <= 0)
            throw new IllegalArgumentException("dtSec must be > 0");

        if(Math.abs(error) <= deadband)
            return applySlewAndClamp(0.0, dtSec);

        double uRaw = kp * error;
        return applySlewAndClamp(uRaw, dtSec);
    }

    private double applySlewAndClamp(double targetU, double dtSec)
    {
        double u = targetU;

        u = Math2D.clamp(u, uMin, uMax);

        if(maxDeltaPerSec > 0)
        {
            double maxDelta = maxDeltaPerSec * dtSec;
            double delta = u - lastU;
            if(delta > maxDelta) u = lastU + maxDelta;
            else if(delta < -maxDelta) u = lastU - maxDelta;
        }

        lastU = u;
        return u;
    }
}
