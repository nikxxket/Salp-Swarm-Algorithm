package view;

import algorithm.SSA;
import geometry.GeometryUtils;
import model.Cube;
import model.Point3D;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;

public class MapPanel extends JPanel {

    double yaw = Math.toRadians(-45);
    double pitch = Math.toRadians(25);
    double scale = 60.0;
    int lastMouseX, lastMouseY;

    List<Cube> obstacles = new ArrayList<>();
    List<List<Point3D>> uavPaths = new ArrayList<>();
    boolean[] showUav = {true, true, true};

    Timer animationTimer;
    int[] currentWaypoint = new int[3];
    double[] progress = new double[3];
    Point3D[] currentPositions = new Point3D[3];

    Point3D[] starts = {new Point3D(1, 1, 5), new Point3D(1, 2.5, 3), new Point3D(1, 4, 1)};
    Point3D[] goals  = {new Point3D(15, 4, 5), new Point3D(15, 2.5, 3), new Point3D(15, 1, 1)};

    public MapPanel() {
        setBackground(new Color(40, 44, 52));

        Color blue = new Color(50, 100, 255);
        Color red = new Color(255, 80, 80);

        obstacles.add(new Cube(2.0, 3.5, 0, 3, 0, 6, blue));
        obstacles.add(new Cube(4.2, 6.0, 2, 5, 0, 6, red));
        obstacles.add(new Cube(6.7, 8.2, 0, 3, 0, 6, blue));
        obstacles.add(new Cube(8.9, 10.4, 2, 5, 0, 6, red));
        obstacles.add(new Cube(11.1, 12.6, 0, 3, 0, 6, blue));
        obstacles.add(new Cube(13.3, 14.8, 2, 5, 0, 6, red));

        MouseAdapter mouseAdapter = new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                lastMouseX = e.getX();
                lastMouseY = e.getY();
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                int dx = e.getX() - lastMouseX;
                int dy = e.getY() - lastMouseY;
                yaw += dx * 0.01;
                pitch -= dy * 0.01;
                pitch = Math.max(-Math.PI / 2 + 0.1, Math.min(Math.PI / 2 - 0.1, pitch));
                lastMouseX = e.getX();
                lastMouseY = e.getY();
                repaint();
            }
        };
        addMouseListener(mouseAdapter);
        addMouseMotionListener(mouseAdapter);

        animationTimer = new Timer(30, e -> {
            boolean allFinished = true;
            for (int i = 0; i < 3; i++) {
                if (uavPaths.isEmpty() || uavPaths.get(i).isEmpty()) continue;
                if (currentWaypoint[i] < uavPaths.get(i).size() - 1) {
                    progress[i] += 0.1;
                    if (progress[i] >= 1.0) {
                        progress[i] = 0.0;
                        currentWaypoint[i]++;
                    }

                    if (currentWaypoint[i] < uavPaths.get(i).size() - 1) {
                        Point3D p1 = uavPaths.get(i).get(currentWaypoint[i]);
                        Point3D p2 = uavPaths.get(i).get(currentWaypoint[i] + 1);
                        currentPositions[i] = new Point3D(
                                p1.x + progress[i] * (p2.x - p1.x),
                                p1.y + progress[i] * (p2.y - p1.y),
                                p1.z + progress[i] * (p2.z - p1.z)
                        );
                        allFinished = false;
                    } else {
                        currentPositions[i] = uavPaths.get(i).get(currentWaypoint[i]);
                    }
                }
            }
            repaint();
            if (allFinished) animationTimer.stop();
        });
    }

    public void runSSA() {
        uavPaths.clear();
        List<List<Point3D>> previousPaths = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            SSA ssa = new SSA(obstacles, previousPaths);
            List<Point3D> bestPath = ssa.optimize(starts[i], goals[i]);
            uavPaths.add(bestPath);
            previousPaths.add(bestPath);
        }
    }

    public void startAnimation() {
        if (uavPaths.size() < 3) return;
        for (int i = 0; i < 3; i++) {
            currentWaypoint[i] = 0;
            progress[i] = 0.0;
            currentPositions[i] = uavPaths.get(i).get(0);
        }
        animationTimer.start();
    }

    public void stopAnimation() {
        if (animationTimer != null) animationTimer.stop();
    }

    Point3D projectWithDepth(double x, double y, double z) {
        double cx = 8, cy = 2.5, cz = 3;
        double dx = x - cx, dy = y - cy, dz = z - cz;
        double x1 = dx * Math.cos(yaw) - dy * Math.sin(yaw);
        double y1 = dx * Math.sin(yaw) + dy * Math.cos(yaw);
        double z1 = dz;
        double y2 = y1 * Math.cos(pitch) - z1 * Math.sin(pitch);
        double z2 = y1 * Math.sin(pitch) + z1 * Math.cos(pitch);
        return new Point3D(getWidth() / 2.0 + x1 * scale, getHeight() / 2.0 - z2 * scale, y2);
    }

    // ==== Построение 3D-граней куба (визуализация) ====
    private List<Polygon3D> cubeFaces(Cube c) {
        List<Polygon3D> faces = new ArrayList<>();
        Point3D[] v = new Point3D[]{
                new Point3D(c.minX, c.minY, c.minZ), new Point3D(c.maxX, c.minY, c.minZ),
                new Point3D(c.maxX, c.maxY, c.minZ), new Point3D(c.minX, c.maxY, c.minZ),
                new Point3D(c.minX, c.minY, c.maxZ), new Point3D(c.maxX, c.minY, c.maxZ),
                new Point3D(c.maxX, c.maxY, c.maxZ), new Point3D(c.minX, c.maxY, c.maxZ)
        };
        int[][] faceIndices = {
                {0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4},
                {2, 3, 7, 6}, {1, 2, 6, 5}, {0, 3, 7, 4}
        };
        double[] lightDir = {-0.5, -0.5, -1.0};
        for (int[] idx : faceIndices) {
            Point3D[] p3d = new Point3D[4];
            Point3D[] screenPts = new Point3D[4];
            double depth = 0;
            for (int i = 0; i < 4; i++) {
                p3d[i] = v[idx[i]];
                screenPts[i] = projectWithDepth(p3d[i].x, p3d[i].y, p3d[i].z);
                depth += screenPts[i].z;
            }
            depth /= 4.0;
            double nx = p3d[1].y - p3d[0].y;
            double ny = p3d[1].z - p3d[0].z;
            double nz = p3d[1].x - p3d[0].x;
            double dot = nx * lightDir[0] + ny * lightDir[1] + nz * lightDir[2];
            double shade = Math.max(0.4, Math.min(1.0, Math.abs(dot)));
            faces.add(new Polygon3D(screenPts, c.color, depth, shade));
        }
        return faces;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawGridAndAxes(g2d);
        drawBoundingBox(g2d);

        List<Polygon3D> polygons = new ArrayList<>();
        for (Cube cube : obstacles) polygons.addAll(cubeFaces(cube));
        polygons.sort((p1, p2) -> Double.compare(p2.depth, p1.depth));

        Composite originalComposite = g2d.getComposite();
        g2d.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.7f));
        for (Polygon3D p : polygons) p.draw(g2d);
        g2d.setComposite(originalComposite);

        Color[] colors = {new Color(0, 255, 0), new Color(255, 255, 0), new Color(0, 255, 255)};

        for (int i = 0; i < uavPaths.size(); i++) {
            if (!showUav[i]) continue;
            List<Point3D> path = uavPaths.get(i);

            boolean hasCollision = false;
            for (int j = 0; j < path.size() - 1; j++) {
                for (Cube c : obstacles) {
                    if (GeometryUtils.segmentIntersectsCube(path.get(j), path.get(j + 1), c, 0)) {
                        hasCollision = true;
                        break;
                    }
                }
                if (hasCollision) break;
            }

            g2d.setColor(hasCollision ? Color.RED : colors[i]);
            g2d.setStroke(new BasicStroke(hasCollision ? 4.0f : 3.0f));

            for (int j = 0; j < path.size() - 1; j++) {
                Point3D p1 = projectWithDepth(path.get(j).x, path.get(j).y, path.get(j).z);
                Point3D p2 = projectWithDepth(path.get(j + 1).x, path.get(j + 1).y, path.get(j + 1).z);
                g2d.drawLine((int) p1.x, (int) p1.y, (int) p2.x, (int) p2.y);
            }

            if (currentPositions[i] != null) {
                Point3D pt = projectWithDepth(currentPositions[i].x, currentPositions[i].y, currentPositions[i].z);
                g2d.setColor(hasCollision ? Color.RED : colors[i]);
                g2d.fillOval((int) pt.x - 6, (int) pt.y - 6, 12, 12);
                g2d.setColor(Color.WHITE);
                g2d.drawOval((int) pt.x - 6, (int) pt.y - 6, 12, 12);
            }
        }

        for (int i = 0; i < starts.length; i++) {
            Point3D s = projectWithDepth(starts[i].x, starts[i].y, starts[i].z);
            g2d.setColor(new Color(0, 200, 0));
            g2d.fillOval((int) s.x - 8, (int) s.y - 8, 16, 16);
            Point3D e = projectWithDepth(goals[i].x, goals[i].y, goals[i].z);
            g2d.setColor(new Color(200, 0, 0));
            g2d.fillOval((int) e.x - 8, (int) e.y - 8, 16, 16);
        }
    }

    private void drawGridAndAxes(Graphics2D g2d) {
        g2d.setColor(new Color(70, 70, 70));
        g2d.setStroke(new BasicStroke(1f));
        for (int i = 0; i <= 16; i += 2) {
            drawLine3D(g2d, new Point3D(i, 0, 0), new Point3D(i, 5, 0));
            drawLine3D(g2d, new Point3D(0, i / 3.2, 0), new Point3D(16, i / 3.2, 0));
        }
        g2d.setStroke(new BasicStroke(2f));
        g2d.setColor(Color.RED);
        drawLine3D(g2d, new Point3D(0, 0, 0), new Point3D(17, 0, 0));
        g2d.setColor(Color.GREEN);
        drawLine3D(g2d, new Point3D(0, 0, 0), new Point3D(0, 6, 0));
        g2d.setColor(Color.BLUE);
        drawLine3D(g2d, new Point3D(0, 0, 0), new Point3D(0, 0, 7));
    }

    private void drawBoundingBox(Graphics2D g2d) {
        g2d.setColor(new Color(255, 255, 255, 30));
        g2d.setStroke(new BasicStroke(1f));
        Point3D[] c = {
                new Point3D(0, 0, 0), new Point3D(16, 0, 0), new Point3D(16, 5, 0), new Point3D(0, 5, 0),
                new Point3D(0, 0, 6), new Point3D(16, 0, 6), new Point3D(16, 5, 6), new Point3D(0, 5, 6)
        };
        drawLine3D(g2d, c[0], c[1]); drawLine3D(g2d, c[1], c[2]);
        drawLine3D(g2d, c[2], c[3]); drawLine3D(g2d, c[3], c[0]);
        drawLine3D(g2d, c[4], c[5]); drawLine3D(g2d, c[5], c[6]);
        drawLine3D(g2d, c[6], c[7]); drawLine3D(g2d, c[7], c[4]);
        drawLine3D(g2d, c[0], c[4]); drawLine3D(g2d, c[1], c[5]);
        drawLine3D(g2d, c[2], c[6]); drawLine3D(g2d, c[3], c[7]);
    }

    private void drawLine3D(Graphics2D g2d, Point3D p1, Point3D p2) {
        Point3D s1 = projectWithDepth(p1.x, p1.y, p1.z);
        Point3D s2 = projectWithDepth(p2.x, p2.y, p2.z);
        g2d.drawLine((int) s1.x, (int) s1.y, (int) s2.x, (int) s2.y);
    }
}