package org.example;

import java.awt.*;

public abstract class Entity {
    public double x, y;
    public double width, height;
    // unit circle angle
    public double angle;
    private double vel = 0, accel = 0;

    public Entity() {
        x = 100;
        y = 100;
        width = 50;
        height = 50;
    }

    public Entity(double x, double y, double width, double height) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }

    public Entity(double x, double y, double width, double height, double angle) {
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.angle = angle;
    }

    public boolean isTouching(Entity e) {
        return !(x > e.x + e.width || x + width < e.x || y > e.y + e.height || y + height < e.y);
    }

    public abstract void update();

    public abstract  void draw(Graphics2D g);
}
