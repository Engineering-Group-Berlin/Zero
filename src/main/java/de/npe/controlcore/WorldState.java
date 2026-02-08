package controlcore;

import java.util.Arrays;

public final class WorldState {
    public static final int MAX_ROBOTS_PER_TEAM = 16;

    public static final class RobotState {
        public final RobotId id;
        public final State2D state;
        public final boolean present;

        public RobotState(RobotId id, State2D state, boolean present) {
            this.id = id;
            this.state = state;
            this.present = present;
        }
    }

    public static final class TeamState {
        public final TeamColor color;
        public final RobotState[] robots; // size MAX_ROBOTS_PER_TEAM

        public TeamState(TeamColor color, RobotState[] robots) {
            this.color = color;
            this.robots = Arrays.copyOf(robots, MAX_ROBOTS_PER_TEAM);
        }
    }

    public final double t;
    public final BallState2D ball;
    public final TeamState blue;
    public final TeamState yellow;

    public WorldState(double t, BallState2D ball, TeamState blue, TeamState yellow) {
        this.t = t;
        this.ball = ball;
        this.blue = blue;
        this.yellow = yellow;
    }
}