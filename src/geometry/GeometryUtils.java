package geometry;

import model.Cube;
import model.Point3D;

public final class GeometryUtils {

    private GeometryUtils() {}

    /** Slab-метод: проверяет пересечение отрезка [a,b] с кубом (с учётом margin). */
    public static boolean segmentIntersectsCube(Point3D a, Point3D b, Cube c, double margin) {
        double xmin = c.minX - margin, xmax = c.maxX + margin;
        double ymin = c.minY - margin, ymax = c.maxY + margin;
        double zmin = c.minZ - margin, zmax = c.maxZ + margin;

        double tmin = 0.0, tmax = 1.0;

        // X
        if (Math.abs(b.x - a.x) < 1e-9) {
            if (a.x < xmin || a.x > xmax) return false;
        } else {
            double t1 = (xmin - a.x) / (b.x - a.x), t2 = (xmax - a.x) / (b.x - a.x);
            if (t1 > t2) { double tmp = t1; t1 = t2; t2 = tmp; }
            tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
        }
        // Y
        if (Math.abs(b.y - a.y) < 1e-9) {
            if (a.y < ymin || a.y > ymax) return false;
        } else {
            double t1 = (ymin - a.y) / (b.y - a.y), t2 = (ymax - a.y) / (b.y - a.y);
            if (t1 > t2) { double tmp = t1; t1 = t2; t2 = tmp; }
            tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
        }
        // Z
        if (Math.abs(b.z - a.z) < 1e-9) {
            if (a.z < zmin || a.z > zmax) return false;
        } else {
            double t1 = (zmin - a.z) / (b.z - a.z), t2 = (zmax - a.z) / (b.z - a.z);
            if (t1 > t2) { double tmp = t1; t1 = t2; t2 = tmp; }
            tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
            if (tmin > tmax) return false;
        }
        return true;
    }
}