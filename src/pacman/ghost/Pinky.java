package pacman.ghost;

import pacman.board.BoardItem;
import pacman.board.PacmanBoard;
import pacman.game.PacmanGame;
import pacman.util.Direction;
import pacman.util.Position;

import java.util.PriorityQueue;
import java.util.Comparator;

/**
 * Pinky is a cunning ghost that tries to ambush the hunter.
 * In this version, during CHASE she uses A* to follow a shortest path
 * towards a tile up to 4 squares in front of the hunter (ambush target).
 *
 * Other phases (SCATTER/FRIGHTENED) are handled by the base Ghost class:
 *  - SCATTER uses home(game)
 *  - FRIGHTENED uses frightenedPosition(game)
 */
public class Pinky extends Ghost {

    @Override
    public String getColour() {
        return "#c397d8";
    }

    @Override
    public GhostType getType() {
        return GhostType.PINKY;
    }

    /**
     * CHASE behaviour:
     *  - Compute a tile up to 4 squares ahead of the hunter (ambush target)
     *  - Use A* to find a path from Pinky to that tile
     *  - Return the NEXT STEP on that path as the chase target
     *
     * IMPORTANT:
     *  - The base Ghost class only calls chaseTarget(...) during CHASE.
     *  - In SCATTER, it uses home(game).
     *  - In FRIGHTENED, it uses frightenedPosition(game).
     */
    @Override
    public Position chaseTarget(PacmanGame game) {
        PacmanBoard board = game.getBoard();
        Position start     = getPosition();

        if (start == null || game.getHunter() == null
                || game.getHunter().getPosition() == null) {
            // Fallback: stay where we are
            return start;
        }

        // 1. Compute ambush tile (up to 4 tiles ahead of hunter)
        Position goal = computeAmbushTarget(game, board);

        // Already at the goal.
        if (start.equals(goal)) {
            return goal;
        }

        // 2. Run A* to get the next step toward that goal.
        Position nextStep = aStarNextStepTowards(board, start, goal);

        // If A* fails, fall back to the goal tile (engine will pick some direction).
        return (nextStep != null) ? nextStep : goal;
    }

    /**
     * Pinky's home position is one block outside of the top left of the board.
     * Where the top left position of the board is (0, 0).
     * Used by the SCATTER phase.
     */
    @Override
    public Position home(PacmanGame game) {
        return new Position(-1, -1);
    }

    /* ==========================================================
     * Ambush target: up to 4 tiles ahead of hunter
     * ========================================================== */

    private Position computeAmbushTarget(PacmanGame game, PacmanBoard board) {
        Position hunterPos = game.getHunter().getPosition();
        Direction dir      = game.getHunter().getDirection();

        int hx = hunterPos.getX();
        int hy = hunterPos.getY();

        int dx = 0;
        int dy = 0;

        if (dir != null) {
            switch (dir) {
                case UP:
                    dy = -1;
                    break;
                case DOWN:
                    dy = 1;
                    break;
                case LEFT:
                    dx = -1;
                    break;
                case RIGHT:
                    dx = 1;
                    break;
                default:
                    break;
            }
        }

        int width  = board.getWidth();
        int height = board.getHeight();

        // If Pacman isn't moving, just target his tile.
        if (dx == 0 && dy == 0) {
            return hunterPos;
        }

        Position lastValid = hunterPos; // fallback
        int x = hx;
        int y = hy;

        // Walk at most 4 tiles ahead, stopping if out of bounds or not walkable.
        for (int i = 0; i < 4; i++) {
            x += dx;
            y += dy;

            if (!inBounds(x, y, width, height)) {
                break;
            }
            if (!isWalkable(board, x, y)) {
                break;
            }

            lastValid = new Position(x, y);
        }

        return lastValid;
    }

    /* ==========================================================
     * A* pathfinding: next step towards goal
     * ========================================================== */

    private Position aStarNextStepTowards(PacmanBoard board, Position start, Position goal) {
        int width  = board.getWidth();
        int height = board.getHeight();

        int sx = start.getX();
        int sy = start.getY();
        int gx = goal.getX();
        int gy = goal.getY();

        if (!inBounds(sx, sy, width, height) || !inBounds(gx, gy, width, height)) {
            return null;
        }

        // Ensure the goal is walkable; if it's not, bail out.
        if (!isWalkable(board, gx, gy)) {
            return null;
        }

        final int INF = Integer.MAX_VALUE;

        int[][] gScore     = new int[width][height];
        boolean[][] closed = new boolean[width][height];
        Position[][] parent = new Position[width][height];

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                gScore[x][y]  = INF;
                closed[x][y]  = false;
                parent[x][y]  = null;
            }
        }

        class Node {
            final int x, y;
            final int f; // f = g + h

            Node(int x, int y, int f) {
                this.x = x;
                this.y = y;
                this.f = f;
            }
        }

        PriorityQueue<Node> open =
                new PriorityQueue<>(Comparator.comparingInt(n -> n.f));

        gScore[sx][sy] = 0;
        int hStart = heuristic(sx, sy, gx, gy);
        open.add(new Node(sx, sy, hStart));

        // 4-direction neighbors: right, left, down, up
        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0,  0, 1,-1 };

        while (!open.isEmpty()) {
            Node node = open.poll();
            int cx = node.x;
            int cy = node.y;

            if (closed[cx][cy]) {
                continue;
            }
            closed[cx][cy] = true;

            // Reached the goal
            if (cx == gx && cy == gy) {
                break;
            }

            int currentG = gScore[cx][cy];
            if (currentG == INF) {
                continue;
            }

            for (int i = 0; i < 4; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (!inBounds(nx, ny, width, height)) {
                    continue;
                }
                if (!isWalkable(board, nx, ny)) {
                    continue;
                }
                if (closed[nx][ny]) {
                    continue;
                }

                int tentativeG = currentG + 1;

                if (tentativeG < gScore[nx][ny]) {
                    gScore[nx][ny] = tentativeG;
                    parent[nx][ny] = new Position(cx, cy);

                    int f = tentativeG + heuristic(nx, ny, gx, gy);
                    open.add(new Node(nx, ny, f));
                }
            }
        }

        // No path
        if (gScore[gx][gy] == INF) {
            return null;
        }

        // Reconstruct path backwards from goal to start.
        Position step = new Position(gx, gy);
        Position prev = parent[gx][gy];

        while (prev != null && !(prev.getX() == sx && prev.getY() == sy)) {
            step = prev;
            prev = parent[step.getX()][step.getY()];
        }

        return step;
    }

    /* ==========================================================
     * Helpers (same style as Blinky/Inky)
     * ========================================================== */

    private boolean inBounds(int x, int y, int width, int height) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    private boolean isWalkable(PacmanBoard board, int x, int y) {
        BoardItem item = board.getEntry(new Position(x, y));
        if (item == null) {
            return false;
        }
        return item.getPathable();
    }

    private int heuristic(int x1, int y1, int x2, int y2) {
        // Manhattan distance
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }
}
