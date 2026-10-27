package model;

import java.awt.Color;

public class Cube {
    public final double minX, maxX, minY, maxY, minZ, maxZ;
    public final Color color;

    public Cube(double minX, double maxX, double minY, double maxY,
                double minZ, double maxZ, Color color) {
        this.minX = minX; this.maxX = maxX;
        this.minY = minY; this.maxY = maxY;
        this.minZ = minZ; this.maxZ = maxZ;
        this.color = color;
    }

    public boolean isInside(Point3D p) {
        double buf = 0.15;
        return p.x >= minX - buf && p.x <= maxX + buf &&
                p.y >= minY - buf && p.y <= maxY + buf &&
                p.z >= minZ - buf && p.z <= maxZ + buf;
    }
}