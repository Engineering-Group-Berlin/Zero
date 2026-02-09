package de.egb.controlcore;

/**
 * Thread-sicherer Holder für den aktuellen WorldState.
 * Vision aktualisiert, Control/Planning liest.
 */
public final class WorldStateProvider {

    private volatile WorldState current;

    public WorldStateProvider() {
        this.current = createEmpty();
    }

    /** Liefert eine Kopie des aktuellen WorldState (immutable, kein Defensive Copy nötig). */
    public WorldState getCurrent() {
        return current;
    }

    /** Aktualisiert den WorldState (z.B. aus Vision). */
    public void update(WorldState next) {
        if (next == null) throw new IllegalArgumentException("WorldState must not be null");
        this.current = next;
    }

    private static WorldState createEmpty() {
        WorldState.RobotState[] emptyBlue = emptyTeam(TeamColor.BLUE);
        WorldState.RobotState[] emptyYellow = emptyTeam(TeamColor.YELLOW);
        return new WorldState(
                0.0,
                new BallState2D(Vec2.ZERO, Vec2.ZERO, 0.0, false),
                new WorldState.TeamState(TeamColor.BLUE, emptyBlue),
                new WorldState.TeamState(TeamColor.YELLOW, emptyYellow)
        );
    }

    private static WorldState.RobotState[] emptyTeam(TeamColor color) {
        WorldState.RobotState[] arr = new WorldState.RobotState[WorldState.MAX_ROBOTS_PER_TEAM];
        State2D emptyState = State2D.poseOnly(new Pose2D(Vec2.ZERO, 0.0), 0.0);
        for (int i = 0; i < arr.length; i++) {
            arr[i] = new WorldState.RobotState(new RobotId(i), emptyState, false);
        }
        return arr;
    }
}
