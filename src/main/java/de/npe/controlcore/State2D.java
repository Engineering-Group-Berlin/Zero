package controlcore;

public record State2D(
        Pose2D pose,
        Twist2D twist,
        double t,
        Frame2D twistFrame,
        boolean poseValid,
        boolean twistValid
)
{
    public State2D
    {
        if(pose == null) throw new IllegalArgumentException("pose must not be null");
        twist = (twist == null) ? new Twist2D(Vec2.ZERO, 0.0) : twist;
        twistFrame = (twistFrame == null) ? Frame2D.WORLD : twistFrame;
    }

    public static State2D poseOnly(Pose2D pose, double t)
    {
        return new State2D(pose, new Twist2D(Vec2.ZERO, 0.0), t, Frame2D.WORLD, true, false);
    }
}
