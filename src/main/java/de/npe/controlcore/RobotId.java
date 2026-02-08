package controlcore;

public record RobotId(int value) {
    public RobotId {
        if(value < 0 || value > 255) throw new IllegalArgumentException("Robot out of Range: " + value);
    }
}
