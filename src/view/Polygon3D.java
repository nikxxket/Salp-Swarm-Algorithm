package view;

import model.Point3D;

import java.awt.*;

public class Polygon3D {
    Point3D[] pts;
    Color color;
    double depth;
    double shade;

    Polygon3D(Point3D[] pts, Color color, double depth, double shade) {
        this.pts = pts;
        this.color = color;
        this.depth = depth;
        this.shade = shade;
    }

    void draw(Graphics2D g2d) {
        int[] x = new int[4], y = new int[4];
        for (int i = 0; i < 4; i++) {
            x[i] = (int) pts[i].x;
            y[i] = (int) pts[i].y;
        }
        int r = (int) (color.getRed() * shade);
        int g = (int) (color.getGreen() * shade);
        int b = (int) (color.getBlue() * shade);
        g2d.setColor(new Color(r, g, b));
        g2d.fillPolygon(x, y, 4);
        g2d.setColor(new Color(0, 0, 0, 100));
        g2d.drawPolygon(x, y, 4);
    }
}