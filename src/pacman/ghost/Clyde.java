package pacman.ghost;

import pacman.board.BoardItem;
import pacman.board.PacmanBoard;
import pacman.game.PacmanGame;
import pacman.util.Position;

import java.util.ArrayDeque;
import java.util.Queue;

/**
 * Clyde is a ghost that behaves in a very scared manner when close to the hunter.
 *
 * Classic-style behaviour:
 *  - CHASE (handled here via chaseTarget):
 *      * if distance to hunter >= 8 tiles → chase hunter (BFS next step)
 *      * if distance to hunter <  8 tiles → run toward his corner (bottom-left inside the maze)
 *
 * Other phases (handled by base Ghost class):
 *  - SCATTER: target home(game) (off-board bottom-left)
 *  - FRIGHTENED: target frightenedPosition(game)
 *
 * The engine's ghost-movement logic then moves one tile toward the
 * target returned by chaseTarget/home/frightenedPosition.
 */
public class Clyde extends Ghost {

    @Override
    public String getColour() {
        return "#e78c45";
    }

    @Override
    public GhostType getType() {
        return GhostType.CLYDE;
    }

    /**
     * CHASE behaviour:
     *  - If Clyde is far (distance >= 8) from the hunter:
     *      -> BFS toward the hunter (aggressive)
     *  - If Clyde is close (distance < 8) to the hunter:
     *      -> BFS toward a bottom-left "corner inside" tile (cowardly)
     *
     * IMPORTANT:
     *  - The base Ghost class only calls chaseTarget(...) during CHASE.
     *  - In SCATTER, it uses home(game) (off-board bottom-left).
     *  - In FRIGHTENED, it uses frightenedPosition(game).
     */
    @Override
    public Position chaseTarget(PacmanGame game) {
        PacmanBoard board = game.getBoard();
        Position start    = getPosition();

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

        // Distance between Clyde and the hunter. (Uses Position.distance from the project.)
        double dToHunter = start.distance(hunterPos);

        // Choose goal based on classic Clyde rule:
        //  - far (>= 8)  → hunter
        //  - close (< 8) → corner inside bottom-left
        Position goal;
        if (dToHunter >= 8.0) {
            goal = hunterPos;
        } else {
            goal = getCornerInside(game);
        }

        Position nextStep = bfsNextStepTowards(board, start, goal);

        // If BFS found a path, return the neighbor tile as the target.
        // If not, just aim directly at the chosen goal.
        return (nextStep != null) ? nextStep : goal;
    }

    /**
     * Clyde's home position is one block outside of the bottom left of the board.
     * Where the top left position of the board is (0, 0).
     * Used by SCATTER.
     */
    @Override
    public Position home(PacmanGame game) {
        return new Position(-1, game.getBoard().getHeight());
    }

    /**
     * A tile inside the maze near the bottom-left corner.
     * Assumes outer border (x=0, x=width-1, y=0, y=height-1) are walls.
     * This is Clyde's "coward corner" when he gets too close to the hunter.
     */
    private Position getCornerInside(PacmanGame game) {
        PacmanBoard board = game.getBoard();
        int width  = board.getWidth();
        int height = board.getHeight();

        // One in from left wall, one up from bottom wall: (1, height-2)
        return new Position(1, height - 2);
    }

    /* ==========================================================
     *  BFS path: next step toward goal
     * ========================================================== */

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

    /* ==========================================================
     *  Helpers
     * ========================================================== */

    private boolean inBounds(int x, int y, int width, int height) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    private boolean isWalkable(PacmanBoard board, int x, int y) {
        Position pos = new Position(x, y);
        BoardItem item = board.getEntry(pos);
        if (item == null) {
            return false;
        }
        return item.getPathable();
    }
}
