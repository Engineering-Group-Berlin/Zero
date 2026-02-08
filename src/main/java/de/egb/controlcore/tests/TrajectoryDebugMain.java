package controlcore.tests;

import controlcore.*;
import controlcore.control.Se2PController;

import java.util.Locale;

public final class TrajectoryDebugMain {

    public static void main(String[] args) {
        Locale.setDefault(Locale.US);

        Limits limits = Limits.defaults();

        Pose2D startPose = new Pose2D(new Vec2(0.0, 0.0), Math.toRadians(20.0));
        Pose2D goalPose  = new Pose2D(new Vec2(2.0, 1.2), Math.toRadians(140.0));

        State2D start = State2D.poseOnly(startPose, 0.0);
        Goal2D goal = Goal2D.defaultStop(goalPose.p(), goalPose.theta());

        TrapezoidalTrajectory2D traj = new TrapezoidalTrajectory2D(start, goal, limits);

        Se2PController ctrl = new Se2PController(
                2.0,  // kpPos
                4.0,  // kpTheta
                limits
        );

        double dt = 0.01;
        double t = traj.t0();
        double tEnd = traj.tf() + 1.0;

        Pose2D x = startPose;

        int steps = 0;
        while (t <= tEnd) {
            TrajectorySample2D ref = traj.sample(t);

            // Adapter solange Trajectory theta noch 0 liefert:
            Pose2D poseRef = new Pose2D(ref.pose().p(), goal.theta());
            Twist2D twistRef = new Twist2D(ref.twist().v(), 0.0);

            Se2PController.Command u = ctrl.update(x, poseRef, twistRef, dt);

            // integrate plant
            Vec2 pNext = x.p().add(u.v().mul(dt));
            double thNext = Math2D.wrapToPi(x.theta() + u.omega() * dt);
            x = new Pose2D(pNext, thNext);

            double ePos = Math2D.norm(goal.p().sub(x.p()));
            double eTheta = Math.abs(Math2D.wrapToPi(goal.theta() - x.theta()));

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

        double finalPosErr = Math2D.norm(goal.p().sub(x.p()));
        double finalThetaErr = Math.abs(Math2D.wrapToPi(goal.theta() - x.theta()));

        System.out.println("Final errors: ePos=" + finalPosErr +
                " m, eTheta=" + Math.toDegrees(finalThetaErr) + " deg");
    }
}
