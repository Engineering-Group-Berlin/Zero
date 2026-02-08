package controlcore.tests;

import controlcore.*;
import controlcore.control.*;

import java.util.Locale;

public final class TrajectoryAndPControllerTest {

    // ---------------- util ----------------
    private static double clamp(double x, double lo, double hi) {
        return Math.max(lo, Math.min(hi, x));
    }

    private static double norm(Vec2 v) {
        return Math.sqrt(v.x() * v.x() + v.y() * v.y());
    }

    private static Vec2 clampNorm(Vec2 v, double maxNorm) {
        double n = norm(v);
        if (n <= maxNorm || n < 1e-12) return v;
        double s = maxNorm / n;
        return new Vec2(v.x() * s, v.y() * s);
    }

    private static void require(boolean cond, String msg) {
        if (!cond) throw new AssertionError(msg);
    }

    // ---------------- controller ----------------
    /**
     * Very simple P controller in WORLD frame:
     * vCmd = vFF + Kp_pos * (pRef - p)
     * wCmd = wFF + Kp_theta * wrap(thetaRef - theta)
     *
     * Outputs are clamped to limits.
     */
    private static final class PSe2Controller {
        private final double kpPos;     // 1/s if [m] -> [m/s]
        private final double kpTheta;   // 1/s if [rad] -> [rad/s]
        private final Limits limits;

        PSe2Controller(double kpPos, double kpTheta, Limits limits) {
            this.kpPos = kpPos;
            this.kpTheta = kpTheta;
            this.limits = limits;
        }

        Command update(Pose2D poseMeas, Pose2D poseRef, Twist2D twistRef) {
            // position
            Vec2 ePos = poseRef.p().sub(poseMeas.p());
            Vec2 vCmd = twistRef.v().add(ePos.mul(kpPos));
            vCmd = clampNorm(vCmd, limits.vMax());

            // theta
            double eTheta = Math2D.wrapToPi(poseRef.theta() - poseMeas.theta());
            double omegaCmd = twistRef.omega() + kpTheta * eTheta;
            omegaCmd = clamp(omegaCmd, -limits.omegaMax(), limits.omegaMax());

            // deadbands (optional)
            if (norm(ePos) < limits.vDeadband()) {
                // keep feedforward, nothing to do
            }
            if (Math.abs(eTheta) < limits.omegaDeadband()) {
                // keep feedforward, nothing to do
            }

            return new Command(vCmd, omegaCmd);
        }
    }

    private record Command(Vec2 v, double omega) {}

    // ---------------- test main ----------------
    public static void run() {
        Locale.setDefault(Locale.US);

        // Limits
        Limits limits = Limits.defaults();

        // Start / Goal
        Pose2D startPose = new Pose2D(new Vec2(0.0, 0.0), Math.toRadians(20.0));
        Pose2D goalPose  = new Pose2D(new Vec2(2.0, 1.2), Math.toRadians(140.0));

        State2D start = State2D.poseOnly(startPose, 0.0);
        Goal2D goal = Goal2D.defaultStop(goalPose.p(), goalPose.theta());

        // Trajectory (currently XY only in your implementation)
        TrapezoidalTrajectory2D traj = new TrapezoidalTrajectory2D(start, goal, limits);

        // Controller gains: start stable, then tune up
        PSe2Controller ctrl = new PSe2Controller(
                2.0,  // kpPos
                4.0,  // kpTheta
                limits
        );

        // Simulation params
        double dt = 0.01;                 // 100 Hz
        double t = traj.t0();
        double tEnd = traj.tf() + 1.0;    // settle

        Pose2D x = startPose;

        double minPosErr = Double.POSITIVE_INFINITY;
        double minThetaErr = Double.POSITIVE_INFINITY;

        int steps = 0;
        while (t <= tEnd) {
            TrajectorySample2D ref = traj.sample(t);

            // ---- IMPORTANT ADAPTER ----
            // Your trajectory currently outputs theta=0.
            // For combined test, use goal.theta() as orientation reference.
            Pose2D poseRef = new Pose2D(ref.pose().p(), goal.theta());

            // For now: use ref linear velocity; omega FF = 0.
            Twist2D twistRef = new Twist2D(ref.twist().v(), 0.0);

            // Controller output
            Command u = ctrl.update(x, poseRef, twistRef);

            // Assertions: commands within limits
            require(norm(u.v()) <= limits.vMax() + 1e-6,
                    "vCmd exceeded vMax: " + norm(u.v()) + " > " + limits.vMax());
            require(Math.abs(u.omega()) <= limits.omegaMax() + 1e-6,
                    "omegaCmd exceeded omegaMax: " + Math.abs(u.omega()) + " > " + limits.omegaMax());

            // Integrate a simple SE2 kinematic plant in WORLD frame
            Vec2 pNext = x.p().add(u.v().mul(dt));
            double thNext = Math2D.wrapToPi(x.theta() + u.omega() * dt);
            x = new Pose2D(pNext, thNext);

            // Track errors vs goal
            double ePos = norm(goal.p().sub(x.p()));
            double eTheta = Math.abs(Math2D.wrapToPi(goal.theta() - x.theta()));
            minPosErr = Math.min(minPosErr, ePos);
            minThetaErr = Math.min(minThetaErr, eTheta);

            // Log every 0.2s
            if (steps % 20 == 0) {
                System.out.printf(
                        "t=%.2f  pos=(%.3f, %.3f) th=%.1fdeg  ePos=%.4f  eTh=%.2fdeg%n",
                        t,
                        x.p().x(), x.p().y(),
                        Math.toDegrees(x.theta()),
                        ePos,
                        Math.toDegrees(eTheta)
                );
            }

            t += dt;
            steps++;
        }

        // Final checks
        double finalPosErr = norm(goal.p().sub(x.p()));
        double finalThetaErr = Math.abs(Math2D.wrapToPi(goal.theta() - x.theta()));

        require(finalPosErr <= goal.posTol() * 5.0,
                "Final position error too large: " + finalPosErr + " (tol=" + goal.posTol() + ")");
        require(finalThetaErr <= goal.thetaTol() * 5.0,
                "Final theta error too large: " + Math.toDegrees(finalThetaErr) + " deg (tol=" + Math.toDegrees(goal.thetaTol()) + " deg)");

        System.out.println("✅ Trajectory + P controller integration test PASSED.");
        System.out.printf("Final errors: ePos=%.6f m, eTheta=%.3f deg%n",
                finalPosErr, Math.toDegrees(finalThetaErr));
    }
}
