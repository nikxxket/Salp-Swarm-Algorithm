package algorithm;

import geometry.GeometryUtils;
import model.Cube;
import model.Point3D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SSA {
    private final int numWaypoints = 60;
    private final int dim = numWaypoints * 3;
    private final int popSize = 60;
    private final int maxIter = 800;

    private final List<Cube> obstacles;
    private final List<List<Point3D>> previousPaths;
    private final Random rand = new Random();

    private final double[] lb = {0, 0, 0};
    private final double[] ub = {16, 5, 6};
    private final double safetyMargin = 0.15;

    public SSA(List<Cube> obstacles, List<List<Point3D>> previousPaths) {
        this.obstacles = obstacles;
        this.previousPaths = previousPaths;
    }

    public List<Point3D> optimize(Point3D start, Point3D goal) {
        double[][] pop = new double[popSize][dim];

        for (int i = 0; i < popSize; i++) {
            for (int w = 0; w < numWaypoints; w++) {
                double t = (double) w / (numWaypoints - 1);
                double x = start.x + t * (goal.x - start.x);

                if (w == 0) {
                    pop[i][w * 3]     = start.x;
                    pop[i][w * 3 + 1] = start.y;
                    pop[i][w * 3 + 2] = start.z;
                } else if (w == numWaypoints - 1) {
                    pop[i][w * 3]     = goal.x;
                    pop[i][w * 3 + 1] = goal.y;
                    pop[i][w * 3 + 2] = goal.z;
                } else {
                    double safeY = findSafeY(x);
                    double y = safeY + (rand.nextDouble() - 0.5) * 0.6;
                    double z = 1.5 + rand.nextDouble() * 2.5;

                    pop[i][w * 3]     = x + (rand.nextDouble() - 0.5) * 0.3;
                    pop[i][w * 3 + 1] = Math.max(lb[1], Math.min(ub[1], y));
                    pop[i][w * 3 + 2] = z;
                }
            }
            repairArray(pop[i], start, goal);
        }

        double[] bestPath = pop[0].clone();
        double bestFitness = calcFitness(bestPath);

        for (int iter = 0; iter < maxIter; iter++) {
            for (int i = 1; i < popSize; i++) {
                double fit = calcFitness(pop[i]);
                if (fit < bestFitness) { bestFitness = fit; bestPath = pop[i].clone(); }
            }

            double c1 = 2 * Math.exp(-Math.pow(4.0 * iter / maxIter, 2));

            for (int i = 0; i < popSize; i++) {
                if (i == 0) {
                    for (int j = 3; j < dim - 3; j++) {
                        double c2 = rand.nextDouble();
                        double c3 = rand.nextDouble();
                        if (c3 >= 0.5)
                            pop[i][j] = bestPath[j] + c1 * ((ub[j % 3] - lb[j % 3]) * c2 + lb[j % 3]);
                        else
                            pop[i][j] = bestPath[j] - c1 * ((ub[j % 3] - lb[j % 3]) * c2 + lb[j % 3]);
                    }
                } else {
                    for (int j = 3; j < dim - 3; j++) {
                        pop[i][j] = 0.5 * (pop[i][j] + pop[i - 1][j]);
                    }
                }
            }

            for (int i = 0; i < popSize; i++) {
                for (int j = 0; j < dim; j++) {
                    pop[i][j] = Math.max(lb[j % 3], Math.min(ub[j % 3], pop[i][j]));
                }
                repairArray(pop[i], start, goal);
            }
        }

        List<Point3D> res = new ArrayList<>();
        for (int w = 0; w < numWaypoints; w++) {
            res.add(new Point3D(bestPath[w * 3], bestPath[w * 3 + 1], bestPath[w * 3 + 2]));
        }
        return res;
    }

    private double findSafeY(double x) {
        List<double[]> intervals = new ArrayList<>();
        intervals.add(new double[]{lb[1], ub[1]});

        for (Cube c : obstacles) {
            if (x >= c.minX - 0.1 && x <= c.maxX + 0.1) {
                List<double[]> newIntervals = new ArrayList<>();
                for (double[] interval : intervals) {
                    double a = interval[0], b = interval[1];
                    if (c.maxY <= a || c.minY >= b) newIntervals.add(interval);
                    else {
                        if (c.minY > a) newIntervals.add(new double[]{a, c.minY});
                        if (c.maxY < b) newIntervals.add(new double[]{c.maxY, b});
                    }
                }
                intervals = newIntervals;
            }
        }
        if (intervals.isEmpty()) return (lb[1] + ub[1]) / 2;
        double[] best = intervals.get(0);
        for (double[] interval : intervals) {
            if (interval[1] - interval[0] > best[1] - best[0]) best = interval;
        }
        return (best[0] + best[1]) / 2;
    }

    private Point3D repairPoint(Point3D p) {
        for (int iter = 0; iter < 5; iter++) {
            boolean moved = false;
            for (Cube cube : obstacles) {
                if (cube.isInside(p)) {
                    double xmin = cube.minX - 0.3, xmax = cube.maxX + 0.3;
                    double ymin = cube.minY - 0.3, ymax = cube.maxY + 0.3;
                    double zmin = cube.minZ - 0.3, zmax = cube.maxZ + 0.3;

                    double[] dists = {
                            p.x - xmin, xmax - p.x,
                            p.y - ymin, ymax - p.y,
                            p.z - zmin, zmax - p.z
                    };
                    int idx = 0;
                    for (int i = 1; i < 6; i++) if (dists[i] < dists[idx]) idx = i;

                    switch (idx) {
                        case 0: p.x = xmin; break;
                        case 1: p.x = xmax; break;
                        case 2: p.y = ymin; break;
                        case 3: p.y = ymax; break;
                        case 4: p.z = zmin; break;
                        case 5: p.z = zmax; break;
                    }
                    moved = true;
                }
            }
            if (!moved) break;
        }
        p.x = Math.max(lb[0], Math.min(ub[0], p.x));
        p.y = Math.max(lb[1], Math.min(ub[1], p.y));
        p.z = Math.max(lb[2], Math.min(ub[2], p.z));
        return p;
    }

    private void repairArray(double[] path, Point3D start, Point3D goal) {
        for (int w = 1; w < numWaypoints - 1; w++) {
            Point3D p = new Point3D(path[w * 3], path[w * 3 + 1], path[w * 3 + 2]);
            p = repairPoint(p);
            path[w * 3]     = p.x;
            path[w * 3 + 1] = p.y;
            path[w * 3 + 2] = p.z;
        }
    }

    private double calcFitness(double[] path) {
        double cfuel = 0, ccollision = 0, cuav = 0, cdivergence = 0;
        Point3D[] pts = new Point3D[numWaypoints];

        for (int w = 0; w < numWaypoints; w++) {
            pts[w] = new Point3D(path[w * 3], path[w * 3 + 1], path[w * 3 + 2]);
            if (pts[w].x < lb[0] || pts[w].x > ub[0] ||
                    pts[w].y < lb[1] || pts[w].y > ub[1] ||
                    pts[w].z < lb[2] || pts[w].z > ub[2]) ccollision += 1e7;
        }

        for (int i = 0; i < numWaypoints - 1; i++) {
            cfuel += pts[i].distTo(pts[i + 1]);

            for (Cube c : obstacles) {
                if (GeometryUtils.segmentIntersectsCube(pts[i], pts[i + 1], c, safetyMargin)) {
                    ccollision += 1e9;
                }
            }

            if (i > 0) {
                Point3D v1 = new Point3D(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y, pts[i].z - pts[i - 1].z);
                Point3D v2 = new Point3D(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y, pts[i + 1].z - pts[i].z);
                double dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
                double mag1 = Math.sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
                double mag2 = Math.sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
                double angle = Math.acos(Math.max(-1, Math.min(1, dot / (mag1 * mag2 + 1e-6)))) * 180 / Math.PI;
                if (angle > 40) cdivergence += angle * 5.0;
            }
        }

        if (previousPaths != null) {
            for (List<Point3D> other : previousPaths) {
                for (int i = 0; i < numWaypoints; i++) {
                    if (pts[i].distTo(other.get(i)) < 1.5) cuav += 1e6;
                }
            }
        }
        return cfuel + cdivergence + ccollision + cuav;
    }
}