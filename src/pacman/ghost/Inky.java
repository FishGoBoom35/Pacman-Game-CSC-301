package pacman.ghost;

import pacman.board.BoardItem;
import pacman.board.PacmanBoard;
import pacman.game.PacmanGame;
import pacman.util.Direction;
import pacman.util.Position;

import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;

/**
 * Inky is a hybrid ghost:
 * - Uses BFS from the hunter to predict where the hunter CAN go soon
 * - Uses A* from Inky to those candidate tiles to find interception points
 * - Chooses an intercept tile where Inky can cut Pacman off
 * - Falls back to a simpler hybrid (BFS → hunter vs A* → 4-ahead ambush)
 * - Returns the NEXT STEP on the chosen path.
 *
 * The engine's ghost-movement logic then moves one tile toward this target.
 */
public class Inky extends Ghost {

    @Override
    public String getColour() {
        return "#7fd0ff"; // light cyan-ish
    }

    @Override
    public GhostType getType() {
        return GhostType.INKY;
    }

    /**
     * CHASE behaviour:
     *  1. Predict an intercept tile:
     *     - BFS from Pacman to tiles within a small radius (e.g. 6 steps),
     *       focusing on tiles in front of his current direction.
     *     - For those tiles, use A* from Inky to see how fast Inky can get there.
     *     - Choose an intercept where Inky can arrive ~as fast as Pacman.
     *  2. If intercept found:
     *       - A* path from Inky to that tile, return NEXT STEP.
     *     Else:
     *       - Fall back to choosing between:
     *         - BFS path directly to the hunter (Blinky-style)
     *         - A* path to a 4-ahead ambush tile (Pinky-style)
     *       - Return NEXT STEP on the shorter path (ties → hunter).
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

        // First try to find a true intercept tile.
        Position interceptTarget = computeInterceptTarget(game, board, start);

        if (interceptTarget != null && !interceptTarget.equals(start)) {
            PathResult interceptPath = aStarPath(board, start, interceptTarget);
            if (interceptPath != null && interceptPath.nextStep != null) {
                return interceptPath.nextStep;
            }
        }

        // If we can't get a good intercept, fall back to simple hybrid:
        // - BFS to hunter
        // - A* to 4-ahead ambush
        Position ambushTarget = computeAmbushTarget(game, board);

        PathResult bfsPath   = bfsPath(board, start, hunterPos);
        PathResult aStarPath = aStarPath(board, start, ambushTarget);

        // If both failed, just aim directly for hunter
        if (bfsPath == null && aStarPath == null) {
            return hunterPos;
        }

        // If only one worked, use that one
        if (bfsPath != null && aStarPath == null) {
            return (bfsPath.nextStep != null) ? bfsPath.nextStep : hunterPos;
        }
        if (aStarPath != null && bfsPath == null) {
            return (aStarPath.nextStep != null) ? aStarPath.nextStep : ambushTarget;
        }

        // Both paths exist: choose the shorter distance (ties → BFS / hunter)
        if (bfsPath.distance <= aStarPath.distance) {
            return (bfsPath.nextStep != null) ? bfsPath.nextStep : hunterPos;
        } else {
            return (aStarPath.nextStep != null) ? aStarPath.nextStep : ambushTarget;
        }
    }

    /**
     * Inky's home position – outside bottom-right corner.
     */
    @Override
    public Position home(PacmanGame game) {
        PacmanBoard board = game.getBoard();
        return new Position(board.getWidth(), board.getHeight());
    }

    /* ==========================================================
     *  Intercept logic
     * ========================================================== */

    /**
     * Try to find an intercept tile:
     *  - BFS from Pacman up to maxPacDepth tiles
     *  - Consider only tiles "in front of" Pacman (based on direction)
     *  - For each candidate, compute A* distance from Inky
     *  - Choose a tile where Inky can arrive about as fast as Pacman
     */
    private Position computeInterceptTarget(PacmanGame game,
                                            PacmanBoard board,
                                            Position inkyPos) {
        Position pacPos = game.getHunter().getPosition();
        Direction dir   = game.getHunter().getDirection();

        int hx = pacPos.getX();
        int hy = pacPos.getY();

        // If we don't know Pacman's direction, just fall back later.
        if (dir == null) {
            return null;
        }

        int facingDx = 0;
        int facingDy = 0;
        switch (dir) {
            case UP:    facingDy = -1; break;
            case DOWN:  facingDy =  1; break;
            case LEFT:  facingDx = -1; break;
            case RIGHT: facingDx =  1; break;
            default:    break;
        }

        int width  = board.getWidth();
        int height = board.getHeight();
        final int INF = Integer.MAX_VALUE;

        // 1. BFS from Pacman to get distances to nearby tiles
        int maxPacDepth = 6; // how far ahead we predict
        int[][] pacDist = bfsDistances(board, pacPos, maxPacDepth);

        Position bestTile  = null;
        int bestScore      = Integer.MAX_VALUE; // lower is better (dInky - dPac)
        int interceptSlack = 2; // allow Inky to arrive up to 2 steps after Pacman

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                int dPac = pacDist[x][y];
                if (dPac == INF || dPac == 0) {
                    continue; // unreachable or the starting tile itself
                }

                // Only consider tiles roughly "in front" of Pacman
                int vx = x - hx;
                int vy = y - hy;
                int dot = vx * facingDx + vy * facingDy;
                if (dot < 0) {
                    continue; // behind Pacman
                }

                Position candidate = new Position(x, y);

                int dInky = aStarDistance(board, inkyPos, candidate);
                if (dInky == INF) {
                    continue; // Inky can't reach this tile
                }

                int score = dInky - dPac; // negative = Inky arrives first

                // We want tiles where Inky is not too late.
                if (score <= interceptSlack && score < bestScore) {
                    bestScore = score;
                    bestTile  = candidate;
                }
            }
        }

        // If we found a plausible intercept, use it. Otherwise, null (fallback).
        return bestTile;
    }

    /**
     * BFS from a start tile (Pacman) with an upper depth limit.
     * Returns a distance grid; unreachable tiles are Integer.MAX_VALUE.
     */
    private int[][] bfsDistances(PacmanBoard board, Position start, int maxDepth) {
        int width  = board.getWidth();
        int height = board.getHeight();
        final int INF = Integer.MAX_VALUE;

        int[][] dist = new int[width][height];
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                dist[x][y] = INF;
            }
        }

        int sx = start.getX();
        int sy = start.getY();
        if (!inBounds(sx, sy, width, height) || !isWalkable(board, sx, sy)) {
            return dist;
        }

        Queue<Position> queue = new ArrayDeque<>();
        queue.add(start);
        dist[sx][sy] = 0;

        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0,  0, 1,-1 };

        while (!queue.isEmpty()) {
            Position current = queue.remove();
            int cx = current.getX();
            int cy = current.getY();
            int d  = dist[cx][cy];

            if (d >= maxDepth) {
                continue; // don't expand beyond maxDepth
            }

            for (int i = 0; i < 4; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (!inBounds(nx, ny, width, height)) continue;
                if (!isWalkable(board, nx, ny))       continue;
                if (dist[nx][ny] != INF)              continue;

                dist[nx][ny] = d + 1;
                queue.add(new Position(nx, ny));
            }
        }

        return dist;
    }

    /* ==========================================================
     *  Ambush target (Pinky-style, but clamped to valid tiles)
     * ========================================================== */

    /**
     * Returns the furthest valid walkable tile up to 4 steps
     * in front of the hunter in its current direction.
     * If none are valid, returns the hunter's current tile.
     */
    private Position computeAmbushTarget(PacmanGame game, PacmanBoard board) {
        Position hunterPos = game.getHunter().getPosition();
        Direction dir      = game.getHunter().getDirection();

        int hx = hunterPos.getX();
        int hy = hunterPos.getY();

        int dx = 0;
        int dy = 0;

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
                return hunterPos;
        }

        int width  = board.getWidth();
        int height = board.getHeight();

        Position lastValid = hunterPos;

        // Step 1 to 4 tiles ahead, but stop at walls / edges
        for (int i = 1; i <= 4; i++) {
            int nx = hx + dx * i;
            int ny = hy + dy * i;

            if (!inBounds(nx, ny, width, height)) {
                break;
            }

            if (!isWalkable(board, nx, ny)) {
                break;
            }

            lastValid = new Position(nx, ny);
        }

        return lastValid;
    }

    /* ==========================================================
     *  Small struct for path results
     * ========================================================== */

    private static class PathResult {
        final Position nextStep; // first tile to move to from start
        final int distance;      // number of steps from start to goal

        PathResult(Position nextStep, int distance) {
            this.nextStep = nextStep;
            this.distance = distance;
        }
    }

    /* ==========================================================
     *  BFS path (Blinky-style)
     * ========================================================== */

    /**
     * Returns the first step and distance from start to goal using BFS,
     * or null if goal is unreachable.
     */
    private PathResult bfsPath(PacmanBoard board, Position start, Position goal) {
        int width  = board.getWidth();
        int height = board.getHeight();

        int sx = start.getX();
        int sy = start.getY();
        int gx = goal.getX();
        int gy = goal.getY();

        if (!inBounds(sx, sy, width, height) || !inBounds(gx, gy, width, height)) {
            return null;
        }

        if (!isWalkable(board, sx, sy) || !isWalkable(board, gx, gy)) {
            return null;
        }

        if (sx == gx && sy == gy) {
            return new PathResult(goal, 0);
        }

        final int INF = Integer.MAX_VALUE;

        boolean[][] visited = new boolean[width][height];
        Position[][] parent = new Position[width][height];
        int[][] dist        = new int[width][height];

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                visited[x][y] = false;
                parent[x][y]  = null;
                dist[x][y]    = INF;
            }
        }

        Queue<Position> queue = new ArrayDeque<>();
        queue.add(start);
        visited[sx][sy] = true;
        dist[sx][sy]    = 0;

        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0,  0, 1,-1 };

        while (!queue.isEmpty()) {
            Position current = queue.remove();
            int cx = current.getX();
            int cy = current.getY();

            if (cx == gx && cy == gy) {
                break;
            }

            for (int i = 0; i < 4; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (!inBounds(nx, ny, width, height)) continue;
                if (visited[nx][ny])                  continue;
                if (!isWalkable(board, nx, ny))       continue;

                visited[nx][ny] = true;
                parent[nx][ny]  = current;
                dist[nx][ny]    = dist[cx][cy] + 1;
                queue.add(new Position(nx, ny));
            }
        }

        if (!visited[gx][gy]) {
            return null; // unreachable
        }

        // Reconstruct path: goal -> ... -> start
        Position step = new Position(gx, gy);
        Position prev = parent[gx][gy];

        // Walk back until the tile directly after the start
        while (prev != null && !(prev.getX() == sx && prev.getY() == sy)) {
            step = prev;
            prev = parent[step.getX()][step.getY()];
        }

        int distance = dist[gx][gy];
        return new PathResult(step, distance);
    }

    /* ==========================================================
     *  A* path (Pinky-style, but to arbitrary goal)
     * ========================================================== */

    private static class Node {
        final int x, y;
        final int f; // f = g + h

        Node(int x, int y, int f) {
            this.x = x;
            this.y = y;
            this.f = f;
        }
    }

    /**
     * Returns the first step and distance from start to goal using A* with
     * Manhattan distance heuristic, or null if unreachable.
     */
    private PathResult aStarPath(PacmanBoard board, Position start, Position goal) {
        int width  = board.getWidth();
        int height = board.getHeight();

        int sx = start.getX();
        int sy = start.getY();
        int gx = goal.getX();
        int gy = goal.getY();

        if (!inBounds(sx, sy, width, height) || !inBounds(gx, gy, width, height)) {
            return null;
        }

        if (!isWalkable(board, sx, sy) || !isWalkable(board, gx, gy)) {
            return null;
        }

        if (sx == gx && sy == gy) {
            return new PathResult(goal, 0);
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

        PriorityQueue<Node> open =
                new PriorityQueue<>(Comparator.comparingInt(n -> n.f));

        gScore[sx][sy] = 0;
        int hStart = heuristic(sx, sy, gx, gy);
        open.add(new Node(sx, sy, hStart));

        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0,  0, 1,-1 };

        while (!open.isEmpty()) {
            Node node = open.poll();
            int cx = node.x;
            int cy = node.y;

            if (closed[cx][cy]) continue;
            closed[cx][cy] = true;

            if (cx == gx && cy == gy) {
                break;
            }

            if (gScore[cx][cy] == INF) continue;

            for (int i = 0; i < 4; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (!inBounds(nx, ny, width, height)) continue;
                if (!isWalkable(board, nx, ny))       continue;
                if (closed[nx][ny])                   continue;

                int tentativeG = gScore[cx][cy] + 1;

                if (tentativeG < gScore[nx][ny]) {
                    gScore[nx][ny] = tentativeG;
                    parent[nx][ny] = new Position(cx, cy);
                    int f = tentativeG + heuristic(nx, ny, gx, gy);
                    open.add(new Node(nx, ny, f));
                }
            }
        }

        if (gScore[gx][gy] == INF) {
            return null; // unreachable
        }

        // Reconstruct path: goal -> ... -> start
        Position step = new Position(gx, gy);
        Position prev = parent[gx][gy];

        while (prev != null && !(prev.getX() == sx && prev.getY() == sy)) {
            step = prev;
            prev = parent[step.getX()][step.getY()];
        }

        int distance = gScore[gx][gy];
        return new PathResult(step, distance);
    }

    /**
     * A* distance only (for scoring intercept candidates).
     * Returns Integer.MAX_VALUE if unreachable.
     */
    private int aStarDistance(PacmanBoard board, Position start, Position goal) {
        int width  = board.getWidth();
        int height = board.getHeight();

        int sx = start.getX();
        int sy = start.getY();
        int gx = goal.getX();
        int gy = goal.getY();

        if (!inBounds(sx, sy, width, height) || !inBounds(gx, gy, width, height)) {
            return Integer.MAX_VALUE;
        }

        if (!isWalkable(board, sx, sy) || !isWalkable(board, gx, gy)) {
            return Integer.MAX_VALUE;
        }

        final int INF = Integer.MAX_VALUE;

        int[][] gScore     = new int[width][height];
        boolean[][] closed = new boolean[width][height];

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                gScore[x][y]  = INF;
                closed[x][y]  = false;
            }
        }

        PriorityQueue<Node> open =
                new PriorityQueue<>(Comparator.comparingInt(n -> n.f));

        gScore[sx][sy] = 0;
        int hStart = heuristic(sx, sy, gx, gy);
        open.add(new Node(sx, sy, hStart));

        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0,  0, 1,-1 };

        while (!open.isEmpty()) {
            Node node = open.poll();
            int cx = node.x;
            int cy = node.y;

            if (closed[cx][cy]) continue;
            closed[cx][cy] = true;

            if (cx == gx && cy == gy) {
                break;
            }

            if (gScore[cx][cy] == INF) continue;

            for (int i = 0; i < 4; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                if (!inBounds(nx, ny, width, height)) continue;
                if (!isWalkable(board, nx, ny))       continue;
                if (closed[nx][ny])                   continue;

                int tentativeG = gScore[cx][cy] + 1;

                if (tentativeG < gScore[nx][ny]) {
                    gScore[nx][ny] = tentativeG;
                    int f = tentativeG + heuristic(nx, ny, gx, gy);
                    open.add(new Node(nx, ny, f));
                }
            }
        }

        return gScore[gx][gy];
    }

    /* ==========================================================
     *  Shared helpers
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

    private int heuristic(int x1, int y1, int x2, int y2) {
        // Manhattan distance
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }
}
