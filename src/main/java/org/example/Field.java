package org.example;


import java.awt.*;

public class Field extends Entity {
    public static double targetX = 0.4572 / 2, targetY = 8.001 - 2.063394 - (1.05 / 2), targetZ = 2.05;
    @Override
    public void update() {

    }

    @Override
    public void draw(Graphics2D g) {
        g.setColor(Color.WHITE);
        // top left to top right
        g.drawLine(fixX(0), fixY(8.001), fixX(16.5354), fixY(8.001));
        // top left to bottom left
        g.drawLine(fixX(0), fixY(8.001), fixX(0), fixY(0 + 1.54305));
        // bottom left to bottom right
        g.drawLine(fixX(0 + 1.93294), fixY(0), fixX(16.5354 - 1.93294), fixY(0));
        // bottom right to top right
        g.drawLine(fixX(16.5354), fixY(0 + 1.54305), fixX(16.5354), fixY(8.001));
        // bottom left upper to bottom left lower
        g.drawLine(fixX(0), fixY(0 + 1.54305), fixX(0 + 1.93294), fixY(0));
        // bottom right upper to bottom right lower
        g.drawLine(fixX(16.5354), fixY(0 + 1.54305), fixX(16.5354 - 1.93294), fixY(0));

        // draw blue stage
        g.setColor(new Color(0, 100, 255));
        g.drawLine(fixX(5.87248), fixY((8.001 / 2) + (3.114548 / 2)), fixX(5.87248), fixY((8.001 / 2) - (3.114584 / 2)));
        g.drawLine(fixX(5.87248), fixY((8.001 / 2) + (3.114548 / 2)), fixX(5.87248 + Math.cos(Math.toRadians(180+30)) * 3.114584), fixY((8.001 / 2) + (3.114548 / 2) + Math.sin(Math.toRadians(180+30)) * 3.114584));
        g.drawLine(fixX(5.87248), fixY((8.001 / 2) - (3.114548 / 2)), fixX(5.87248 + Math.cos(Math.toRadians(180-30)) * 3.114584), fixY((8.001 / 2) - (3.114548 / 2) + Math.sin(Math.toRadians(180-30)) * 3.114584));

        // draw blue speaker
        g.drawLine(fixX(0), fixY(8.001 - 2.063394), fixX(0.4572), fixY(8.001 - 2.063394));
        g.drawLine(fixX(0.4572), fixY(8.001 - 2.063394), fixX(0.4572), fixY(8.001 - 2.063394 - 1.05));
        g.drawLine(fixX(0.4572), fixY(8.001 - 2.063394 - 1.05), fixX(0), fixY(8.001 - 2.063394 - 1.05));
        // draw blue target
        g.setColor(Color.GREEN);
        g.drawLine(fixX(0.4572 / 2), fixY(8.001 - 2.063394 - (1.05 / 2) - 0.05), fixX(0.4572 / 2), fixY(8.001 - 2.063394 - (1.05 / 2) + 0.05));
        g.drawLine(fixX(0.4572 / 2 - 0.05), fixY(8.001 - 2.063394 - (1.05 / 2)), fixX(0.4572 / 2 + 0.05), fixY(8.001 - 2.063394 - (1.05 / 2)));

        // draw red stage
        g.setColor(Color.RED);
        g.drawLine(fixX(16.5354 - 5.87248), fixY((8.001 / 2) + (3.114548 / 2)), fixX(16.5354 - 5.87248), fixY((8.001 / 2) - (3.114584 / 2)));
        g.drawLine(fixX(16.5354 - 5.87248), fixY((8.001 / 2) + (3.114548 / 2)), fixX(16.5354 - (5.87248 + Math.cos(Math.toRadians(180+30)) * 3.114584)), fixY((8.001 / 2) + (3.114548 / 2) + Math.sin(Math.toRadians(180+30)) * 3.114584));
        g.drawLine(fixX(16.5354 - 5.87248), fixY((8.001 / 2) - (3.114548 / 2)), fixX(16.5354 - (5.87248 + Math.cos(Math.toRadians(180-30)) * 3.114584)), fixY((8.001 / 2) - (3.114548 / 2) + Math.sin(Math.toRadians(180-30)) * 3.114584));

        // draw blue speaker
        g.drawLine(fixX(16.5354 - 0), fixY(8.001 - 2.063394), fixX(16.5354 - 0.4572), fixY(8.001 - 2.063394));
        g.drawLine(fixX(16.5354 - 0.4572), fixY(8.001 - 2.063394), fixX(16.5354 - 0.4572), fixY(8.001 - 2.063394 - 1.05));
        g.drawLine(fixX(16.5354 - 0.4572), fixY(8.001 - 2.063394 - 1.05), fixX(16.5354 - 0), fixY(8.001 - 2.063394 - 1.05));
        // draw blue target
        g.setColor(Color.GREEN);
        g.drawLine(fixX(16.5354 - 0.4572 / 2), fixY(8.001 - 2.063394 - (1.05 / 2) - 0.05), fixX(16.5354 - 0.4572 / 2), fixY(8.001 - 2.063394 - (1.05 / 2) + 0.05));
        g.drawLine(fixX(16.5354 - (0.4572 / 2 - 0.05)), fixY(8.001 - 2.063394 - (1.05 / 2)), fixX(16.5354 - (0.4572 / 2 + 0.05)), fixY(8.001 - 2.063394 - (1.05 / 2)));

        g.setColor(Color.WHITE);
        g.drawLine(1800 - 14, 200 - 83, 1800 - 14, 200 - 83 - 7);
        g.drawLine(1800 - 14, 200 - 83 - 7, 1800 - 14 + 23, 200 - 83 - 7 - 8);
        g.drawLine(1800 - 14 + 23, 200 - 83 - 7 - 8, 1800 - 14 + 23 + 18, 200 - 83 - 7 - 8);
        g.drawLine(1800 - 14 + 23 + 18, 200 - 83 - 7 - 8, 1800 - 14 + 23 + 18 + 8, 200 - 83 - 7 - 8 + 8);
        g.drawLine(1800 - 14 + 23 + 18 + 8, 200 - 83 - 7 - 8 + 8, 1800 - 14 + 23 + 18 + 8, 200 - 83 - 7 - 8 + 8 + 14);
        g.drawLine(1800 - 14 + 23 + 18 + 8, 200 - 83 - 7 - 8 + 8 + 14, 1800 - 14 + 23 + 18 + 8 - 21, 200 - 83 - 7 - 8 + 8 + 14);
    }

    public static int fixX(double x) {
        return 100 + (int) (x * 100);
    }

    public static int fixY(double y) {
        return Simulator.frame.getHeight() - 150 - (int) (y * 100);
    }

    public static double screenXtoField(double x) {
        return (x - 100) / 100.0;
    }

    public static double screenYtoField(double y) {
        return (y - Simulator.frame.getHeight() + 150) / -100.0;
    }
}
