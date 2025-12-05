package pacman.ghost;

import pacman.board.BoardItem;
import pacman.board.PacmanBoard;
import pacman.game.PacmanGame;
import pacman.util.Position;

import java.util.ArrayDeque;
import java.util.Queue;

/**
 * Blinky is a ghost that behaves in a very aggressive manner towards the hunter.
 * In this version, during CHASE he uses BFS to pick the next step along the
 * shortest path to the hunter. Other phases (SCATTER/FRIGHTENED) are handled
 * by the base Ghost class via home(...) and frightenedPosition(...).
 */
public class Blinky extends Ghost {

    @Override
    public String getColour() {
        return "#d54e53";
    }

    @Override
    public GhostType getType() {
        return GhostType.BLINKY;
    }

    /**
     * CHASE behaviour:
     *  - Compute a BFS path from Blinky to the hunter
     *  - Return the NEXT STEP along that path as the target tile
     *
     * IMPORTANT:
     *  - The base Ghost class only calls chaseTarget(...) in CHASE phase.
     *  - In SCATTER, it will use home(game).
     *  - In FRIGHTENED, it will use frightenedPosition(game).
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

        Position hunterPos = game.getHunter().getPosition();

        // If we're already on the hunter, just return that tile.
        if (start.equals(hunterPos)) {
            return hunterPos;
        }

        Position nextStep = bfsNextStepTowards(board, start, hunterPos);

        // If BFS found a path, return the neighbor tile as the target.
        // If not, just aim directly at the hunter.
        return (nextStep != null) ? nextStep : hunterPos;
    }

    /**
     * Blinky's home position is one block outside of the top right of the board.
     * Used by the SCATTER phase.
     */
    @Override
    public Position home(PacmanGame game) {
        return new Position(game.getBoard().getWidth(), -1);
    }

    /**
     * Internal helper for a "corner inside" tile (if you ever want it):
     * a tile inside the maze near the top-right corner.
     * Assumes outer border (x=0, x=width-1, y=0, y=height-1) are walls.
     */
    @SuppressWarnings("unused")
    private Position getCornerInside(PacmanGame game) {
        PacmanBoard board = game.getBoard();
        int width  = board.getWidth();
        int height = board.getHeight();

        // One in from right wall, one down from top wall: (width-2, 1)
        return new Position(width - 2, 1);
    }

    /**
     * Run BFS on the board to find the shortest path from start to goal,
     * and return the *next* position along that path (one tile away from start).
     *
     * If no path exists, returns null.
     */
    private Position bfsNextStepTowards(PacmanBoard board, Position start, Position goal) {
        int width  = board.getWidth();
        int height = board.getHeight();

        int sx = start.getX();
        int sy = start.getY();
        int gx = goal.getX();
        int gy = goal.getY();

        if (!inBounds(sx, sy, width, height) || !inBounds(gx, gy, width, height)) {
            return null;
        }

        // If either tile is not walkable, bail out.
        if (!isWalkable(board, sx, sy) || !isWalkable(board, gx, gy)) {
            return null;
        }

        // If we're already at the goal, no next step exists.
        if (sx == gx && sy == gy) {
            return goal;
        }

        boolean[][] visited = new boolean[width][height];
        Position[][] parent = new Position[width][height];

        Queue<Position> queue = new ArrayDeque<>();
        queue.add(start);
        visited[sx][sy] = true;
        parent[sx][sy]  = null;

        // 4-direction movement: right, left, down, up
        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0,  0, 1,-1 };

        while (!queue.isEmpty()) {
            Position current = queue.remove();
            int cx = current.getX();
            int cy = current.getY();

            if (cx == gx && cy == gy) {
                // Reached the goal
                break;
            }

            for (int i = 0; i < 4; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (!inBounds(nx, ny, width, height)) {
                    continue;
                }
                if (visited[nx][ny]) {
                    continue;
                }
                if (!isWalkable(board, nx, ny)) {
                    continue;
                }

                visited[nx][ny] = true;
                parent[nx][ny]  = current;
                queue.add(new Position(nx, ny));
            }
        }

        // If we never visited the goal, no path exists
        if (!visited[gx][gy]) {
            return null;
        }

        // Reconstruct path backwards: goal -> ... -> start
        Position step = new Position(gx, gy);
        Position prev = parent[gx][gy];

        // Walk back until the tile directly after 'start'
        while (prev != null && !(prev.getX() == sx && prev.getY() == sy)) {
            step = prev;
            prev = parent[step.getX()][step.getY()];
        }

        // 'step' is now the tile directly after 'start' along the shortest path
        return step;
    }

    /** Simple bounds helper. */
    private boolean inBounds(int x, int y, int width, int height) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    /**
     * A tile is walkable if its BoardItem is pathable (i.e., not a wall).
     */
    private boolean isWalkable(PacmanBoard board, int x, int y) {
        BoardItem item = board.getEntry(new Position(x, y));
        if (item == null) {
            return false;
        }
        return item.getPathable();
    }
}
