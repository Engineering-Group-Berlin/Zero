package de.egb.controlcore.control;

import de.egb.controlcore.*;

public final class Se2PController {

    private final PController px;
    private final PController py;
    private final PController pTheta;
    private final Limits limits;

    public Se2PController(double kpPos, double kpTheta, Limits limits) {
        this.limits = limits;

        // x/y controllers: output is m/s, clamp via vMax later as vector
        this.px = new PController(kpPos, -limits.vMax(), limits.vMax(),
                limits.vDeadband(), 0.0);
        this.py = new PController(kpPos, -limits.vMax(), limits.vMax(),
                limits.vDeadband(), 0.0);

        // theta controller: output is rad/s, clamp by omegaMax
        this.pTheta = new PController(kpTheta, -limits.omegaMax(), limits.omegaMax(),
                limits.omegaDeadband(), 0.0);
    }

    public Command update(Pose2D poseMeas, Pose2D poseRef, Twist2D twistRef, double dt) {

        // --- Position (world frame) ---
        double vxP = px.update(poseRef.p().x(), poseMeas.p().x(), dt);
        double vyP = py.update(poseRef.p().y(), poseMeas.p().y(), dt);

        Vec2 vCmd = new Vec2(vxP, vyP).add(twistRef.v());

        // vector clamp (important!)
        vCmd = clampNorm(vCmd, limits.vMax());

        // --- Theta ---
        double eTheta = Math2D.wrapToPi(poseRef.theta() - poseMeas.theta());
        double omegaP = pTheta.updateError(eTheta, dt); // needs updateError(...) method
        double omegaCmd = omegaP + twistRef.omega();
        omegaCmd = Math2D.clamp(omegaCmd, -limits.omegaMax(), limits.omegaMax());

        return new Command(vCmd, omegaCmd);
    }

    public record Command(Vec2 v, double omega) {}

    private static double norm(Vec2 v) {
        return Math.sqrt(v.x() * v.x() + v.y() * v.y());
    }

    private static Vec2 clampNorm(Vec2 v, double maxNorm) {
        double n = norm(v);
        if (n <= maxNorm || n < 1e-12) return v;
        double s = maxNorm / n;
        return new Vec2(v.x() * s, v.y() * s);
    }
}
