package de.egb.controlcore.planning;

import java.util.Arrays;
import java.util.List;
import java.util.Collections;
import java.util.ArrayList;

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

    public boolean isWalkable(int x, int y)
    {
        requireInBounds(x, y); 
        return walkable[y][x];
    }

    public void setWalkable(int x, int y, boolean value)
    {
        requireInBounds(x, y); 
        walkable[y][x] = value;
    }

    public double getCost(int x, int y)
    {
        requireInBounds(x, y); 
        return cost[y][x];
    }

    public void setCost(int x, int y, double value)
    {
        requireInBounds(x, y); 
        if(value < 0.0 || Double.isNaN(value) || Double.isInfinite(value)) throw new IllegalArgumentException("Cost must be >= 0");
        cost[y][x] = value;
    }
    
    public Cell cell(int x, int y) 
    {
        requireInBounds(x, y); 
        return new Cell(x, y);
    }
    

    public List<Cell> neighbors(int x, int y, Connectivity conn, boolean preventCornerCutting) {
        requireInBounds(x, y);
        if (!isWalkable(x, y)) return Collections.emptyList();

        ArrayList<Cell> out = new ArrayList<>(conn == Connectivity.FOUR ? 4 : 8);

        // 4-connected
        tryAdd(out, x + 1, y);
        tryAdd(out, x - 1, y);
        tryAdd(out, x, y + 1);
        tryAdd(out, x, y - 1);

        if (conn == Connectivity.EIGHT) {
            // diagonals: (dx,dy)
            addDiagonal(out, x, y, +1, +1, preventCornerCutting);
            addDiagonal(out, x, y, +1, -1, preventCornerCutting);
            addDiagonal(out, x, y, -1, +1, preventCornerCutting);
            addDiagonal(out, x, y, -1, -1, preventCornerCutting);
        }

        return out;
    }

    public List<Cell> neighbors(int x, int y, Connectivity conn) 
    {
        return neighbors(x, y, conn, false);
    }

    private void tryAdd(List<Cell> out, int nx, int ny) 
    {
        if(!inBounds(nx, ny)) return; 
        if(!walkable[ny][nx]) return; 
        out.add(new Cell(nx, ny));
    }

    private void addDiagonal(List<Cell> out, int x, int y, int dx, int dy, boolean preventCornerCutting) {
        int nx = x + dx;
        int ny = y + dy;
        if (!inBounds(nx, ny)) return;
        if (!walkable[ny][nx]) return;

        if (preventCornerCutting) {
            // If moving diagonally, require both adjacent cardinal cells to be free
            int ax = x + dx; // adjacent in x direction
            int ay = y;      // same y
            int bx = x;      // same x
            int by = y + dy; // adjacent in y direction
            if (!inBounds(ax, ay) || !inBounds(bx, by)) return;
            if (!walkable[ay][ax] || !walkable[by][bx]) return;
        }

        out.add(new Cell(nx, ny));
    }

    private void requireInBounds(int x, int y) 
    {
        if(!inBounds(x, y)) throw new IllegalArgumentException("Cell (" + x + ", " + y + ") is out of bounds");
    }
}
