package de.egb.controlcore.planning;

import java.util.Arrays;

public class Grid2D {
    public enum Connectivity 
    {
        FOUR, EIGHT
    }

    public static final class Cell 
    {
        public final int x; 
        public final int y;

        public Cell(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        @Override public boolean equals(Object o)
        {
            if(this == o) return true;
            if(!(o instanceof Cell)) return false;
            Cell c = (Cell) o;
            return x == c.x && y == c.y;
        }

        @Override public int hashCode()
        {
            return (x * 73856093) ^ (y * 19349663);
        }

        @Override public String toString()
        {
            return "(" + x + ", " + y + ")";
        }
    }

    private final int width;
    private final int height;

    private final boolean[][] walkable; 

    private final double[][] cost; 

    public Grid2D(int width, int height)
    {
        if(width <= 0 || height <= 0) throw new IllegalArgumentException("Width and height must be > 0");
        this.width = width;
        this.height = height;
        this.walkable = new boolean[height][width];
        this.cost = new double[height][width];
        for(int y = 0; y < height; y++) 
        {
            Arrays.fill(this.walkable[y], true);
            Arrays.fill(this.cost[y], 1.0);
        }
    }

    public int width()
    {
        return width;
    }

    public int height()
    {
        return height;
    }
    
    public boolean inBounds(int x, int y)
    {
        return (x >= 0 && x < width && y >= 0 && y < height);
    }
}
