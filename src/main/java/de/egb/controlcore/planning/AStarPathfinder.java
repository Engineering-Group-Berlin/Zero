package de.egb.controlcore.planning;

import java.util.*;

/**
 * A*-Pathfinder für Grid2D-Raster.
 * Findet einen kürzesten Pfad von Start zu Ziel unter Berücksichtigung
 * von Hindernissen und variablen Zellkosten.
 */
public final class AStarPathfinder {

    private final Grid2D grid;
    private final Grid2D.Connectivity connectivity;
    private final boolean preventCornerCutting;
    private final Heuristic heuristic;

    public AStarPathfinder(Grid2D grid, Grid2D.Connectivity connectivity) {
        this(grid, connectivity, true, Heuristic.EUCLIDEAN);
    }

    public AStarPathfinder(Grid2D grid, Grid2D.Connectivity connectivity,
                          boolean preventCornerCutting, Heuristic heuristic) {
        if (grid == null) throw new IllegalArgumentException("grid must not be null");
        this.grid = grid;
        this.connectivity = connectivity;
        this.preventCornerCutting = preventCornerCutting && connectivity == Grid2D.Connectivity.EIGHT;
        this.heuristic = heuristic != null ? heuristic : Heuristic.EUCLIDEAN;
    }

    public enum Heuristic {
        /** Summe |dx| + |dy| – zulässig für 4- und 8-Nachbarn */
        MANHATTAN,
        /** √(dx² + dy²) – zulässig, oft weniger Expansionen bei 8-Nachbarn */
        EUCLIDEAN
    }

    /**
     * Sucht einen Pfad von start nach goal.
     * @return Pfad von start (inkl.) bis goal (inkl.), oder leere Liste wenn kein Pfad existiert
     */
    public List<Grid2D.Cell> findPath(Grid2D.Cell start, Grid2D.Cell goal) {
        if (start == null || goal == null) throw new IllegalArgumentException("start and goal must not be null");
        if (!grid.inBounds(start.x, start.y) || !grid.inBounds(goal.x, goal.y)) return List.of();
        if (!grid.isWalkable(start.x, start.y) || !grid.isWalkable(goal.x, goal.y)) return List.of();
        if (start.equals(goal)) return List.of(start);

        // g(cell) = tatsächliche Kosten von start bis cell
        Map<Grid2D.Cell, Double> g = new HashMap<>();
        g.put(start, 0.0);

        // parent(cell) = Vorgänger auf dem besten Pfad
        Map<Grid2D.Cell, Grid2D.Cell> parent = new HashMap<>();

        // Open: (cell, f) nach f sortiert
        PriorityQueue<Entry> open = new PriorityQueue<>(Comparator.comparingDouble(e -> e.f));
        open.add(new Entry(start, heuristic(start, goal)));

        Set<Grid2D.Cell> closed = new HashSet<>();

        while (!open.isEmpty()) {
            Entry current = open.poll();
            Grid2D.Cell c = current.cell;

            if (closed.contains(c)) continue;
            if (c.equals(goal)) return reconstructPath(parent, start, goal);

            closed.add(c);
            double gCurrent = g.getOrDefault(c, Double.POSITIVE_INFINITY);

            for (Grid2D.Cell neighbor : grid.neighbors(c.x, c.y, connectivity, preventCornerCutting)) {
                if (closed.contains(neighbor)) continue;

                double moveCost = movementCost(c, neighbor);
                double tentativeG = gCurrent + moveCost;

                if (tentativeG >= g.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) continue;

                parent.put(neighbor, c);
                g.put(neighbor, tentativeG);
                double f = tentativeG + heuristic(neighbor, goal);
                open.add(new Entry(neighbor, f));
            }
        }

        return List.of();
    }

    private double heuristic(Grid2D.Cell from, Grid2D.Cell to) {
        int dx = Math.abs(to.x - from.x);
        int dy = Math.abs(to.y - from.y);
        return switch (heuristic) {
            case MANHATTAN -> dx + dy;
            case EUCLIDEAN -> Math.sqrt(dx * dx + dy * dy);
        };
    }

    private double movementCost(Grid2D.Cell from, Grid2D.Cell to) {
        double cost = grid.getCost(to.x, to.y);
        boolean diagonal = (Math.abs(to.x - from.x) + Math.abs(to.y - from.y)) == 2;
        return diagonal ? cost * Math.sqrt(2) : cost;
    }

    private List<Grid2D.Cell> reconstructPath(Map<Grid2D.Cell, Grid2D.Cell> parent,
                                              Grid2D.Cell start, Grid2D.Cell goal) {
        List<Grid2D.Cell> path = new ArrayList<>();
        Grid2D.Cell curr = goal;
        while (curr != null) {
            path.add(curr);
            curr = parent.get(curr);
        }
        Collections.reverse(path);
        return path;
    }

    private record Entry(Grid2D.Cell cell, double f) {}
}
