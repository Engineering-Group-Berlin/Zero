package de.egb.controlcore;

public record Vec2(double x, double y) {
    public static final Vec2 ZERO = new Vec2(0.0, 0.0);

    public Vec2 add(Vec2 o) { return new Vec2(x + o.x, y + o.y); }
    public Vec2 sub(Vec2 o) { return new Vec2(x - o.x, y - o.y); }
    public Vec2 mul(double s) { return new Vec2(x * s, y * s); }
}
