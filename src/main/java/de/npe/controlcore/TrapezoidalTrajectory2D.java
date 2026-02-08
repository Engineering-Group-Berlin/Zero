package controlcore;

public final class TrapezoidalTrajectory2D {

    private final Vec2 p0;
    private final Vec2 p1;
    private final Vec2 dir;   // unit direction vector
    private final double L;   // path length

    private final double th0;
    private final double th1;
    private final double dth;     // shortest signed angle difference
    private final double sgnTh;   // sign(dth)
    private final double Ath;     // abs(dth)

    private final double t0;
    private final double tf;

    // XY timing
    private final double taXY, tcXY, tdXY;
    private final double vMax, aMax;

    // Theta timing
    private final double taTh, tcTh, tdTh;
    private final double wMax, alphaMax;

    public TrapezoidalTrajectory2D(State2D start, Goal2D goal, Limits limits) {
        this.p0 = start.pose().p();
        this.p1 = goal.p();          // <- Goal2D sollte Pose2D liefern
        this.th0 = start.pose().theta();
        this.th1 = goal.theta();

        // --- XY geometry ---
        Vec2 delta = p1.sub(p0);
        this.L = Math2D.norm(delta);
        this.dir = (L > 1e-9) ? Math2D.normalized(delta) : Vec2.ZERO;

        // --- theta geometry ---
        this.dth = wrapToPi(th1 - th0);     // [-pi, pi)
        this.Ath = Math.abs(dth);
        this.sgnTh = (dth >= 0.0) ? 1.0 : -1.0;

        // limits
        this.vMax = limits.vMax();
        this.aMax = limits.aMax();
        this.wMax = limits.omegaMax();
        this.alphaMax = limits.alphaMax();

        // compute timings for XY and theta independently
        Timing xy = computeTrapezoidTiming(L, vMax, aMax);
        this.taXY = xy.ta;
        this.tcXY = xy.tc;
        this.tdXY = xy.td;
        double tfXY = xy.totalTime;

        Timing th = computeTrapezoidTiming(Ath, wMax, alphaMax);
        this.taTh = th.ta;
        this.tcTh = th.tc;
        this.tdTh = th.td;
        double tfTh = th.totalTime;

        this.t0 = start.t();
        this.tf = t0 + Math.max(tfXY, tfTh);
    }

    public double t0() { return t0; }
    public double tf() { return tf; }

    public TrajectorySample2D sample(double t) {
        double tau = t - t0;
        if (tau <= 0.0) return startSample(t);
        if (t >= tf) return endSample(t);

        // --- XY sample (clamped to its own profile duration) ---
        double tXY = Math.min(tau, taXY + tcXY + tdXY);
        DistVel xy = sampleTrapezoid(tXY, taXY, tcXY, aMax, vMax);

        Vec2 pos = p0.add(dir.mul(xy.s));
        Vec2 vel = dir.mul(xy.v);

        // --- theta sample (clamped to its own profile duration) ---
        double tTh = Math.min(tau, taTh + tcTh + tdTh);
        DistVel th = sampleTrapezoid(tTh, taTh, tcTh, alphaMax, wMax);

        double theta = wrapToPi(th0 + sgnTh * th.s);
        double omega = sgnTh * th.v;

        return new TrajectorySample2D(
                new Pose2D(pos, theta),
                new Twist2D(vel, omega),
                t
        );
    }

    private TrajectorySample2D startSample(double t) {
        return new TrajectorySample2D(
                new Pose2D(p0, th0),
                new Twist2D(Vec2.ZERO, 0.0),
                t
        );
    }

    private TrajectorySample2D endSample(double t) {
        return new TrajectorySample2D(
                new Pose2D(p1, th1),
                new Twist2D(Vec2.ZERO, 0.0),
                t
        );
    }

    // ===== helpers =====

    private static final class Timing {
        final double ta, tc, td, totalTime;
        Timing(double ta, double tc, double td) {
            this.ta = ta; this.tc = tc; this.td = td;
            this.totalTime = ta + tc + td;
        }
    }

    private static Timing computeTrapezoidTiming(double distance, double vMax, double aMax) {
        if (distance <= 1e-12) return new Timing(0.0, 0.0, 0.0);

        double taTmp = vMax / aMax;
        double da = 0.5 * aMax * taTmp * taTmp;

        if (2.0 * da <= distance) {
            double ta = taTmp;
            double tc = (distance - 2.0 * da) / vMax;
            double td = ta;
            return new Timing(ta, tc, td);
        } else {
            // triangular
            double ta = Math.sqrt(distance / aMax);
            double tc = 0.0;
            double td = ta;
            return new Timing(ta, tc, td);
        }
    }

    private static final class DistVel {
        final double s; // distance progressed
        final double v; // speed
        DistVel(double s, double v) { this.s = s; this.v = v; }
    }

    /**
     * Samples a trapezoid/triangle profile parameterized by (ta, tc, aMax, vMax).
     * Assumes td = ta.
     */
    private static DistVel sampleTrapezoid(double tau, double ta, double tc, double aMax, double vMax) {
        double td = ta;
        double total = ta + tc + td;

        if (tau <= 0.0) return new DistVel(0.0, 0.0);
        if (tau >= total) {
            // end state: full distance is not known here, but we can reconstruct it:
            double vPeak = (tc > 0.0) ? vMax : aMax * ta;
            double sEnd = 0.5 * aMax * ta * ta + vPeak * tc + vPeak * td - 0.5 * aMax * td * td;
            return new DistVel(sEnd, 0.0);
        }

        if (tau < ta) {
            double s = 0.5 * aMax * tau * tau;
            double v = aMax * tau;
            return new DistVel(s, v);
        }

        if (tau < ta + tc) {
            double tCruise = tau - ta;
            double s = 0.5 * aMax * ta * ta + vMax * tCruise;
            double v = vMax;
            return new DistVel(s, v);
        }

        double tDec = tau - ta - tc;
        double vPeak = (tc > 0.0) ? vMax : aMax * ta;
        double s = 0.5 * aMax * ta * ta
                + vPeak * tc
                + vPeak * tDec
                - 0.5 * aMax * tDec * tDec;
        double v = vPeak - aMax * tDec;
        return new DistVel(s, v);
    }

    /** Wrap angle to [-pi, pi). */
    private static double wrapToPi(double a) {
        double twoPi = 2.0 * Math.PI;
        a = (a + Math.PI) % twoPi;
        if (a < 0) a += twoPi;
        return a - Math.PI;
    }
}
