package org.example;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N5;

import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class Robot extends Entity {
    double shooterTheta = 0.5;
    double shooterLength = 37.5;
    double trajectoryTime = 0;
    ArrayList<Vector<N5>> trajectory;
    Shooter shooter = new Shooter();
    private final Translation2d SHOOTER_PIVOT_ROBOT_REL = new Translation2d(-0.2757, 0.5972);
    private Translation2d shooterExit = new Translation2d(0, 0);
    double robotPose = 0;
    double distance = 0;

    public Robot(double width, double height) {
        super(50, 800, 71, 5);
        trajectory = new ArrayList<>();
    }

    @Override
    public void update() {
        trajectory.clear();
        if (Mouse.state.equals(MouseState.CLICKED) || Mouse.state.equals(MouseState.DRAGGED)) {
            this.x = Mouse.x;
        }



        robotPose = ((x + width / 2) / 100);
        distance = 8 - robotPose;

        shooterTheta = shooter.thetaFromDistance2(distance);
        shooterExit = shooter.shooterExitRobotRelative(shooterTheta);
        trajectoryTime = shooter.timeFromDistance(distance);

        Vector<N4> in = VecBuilder.fill(shooterExit.getX(), shooterExit.getY(), 20 * Math.cos(shooterTheta), 20 * Math.sin(shooterTheta));

        trajectory = shooter.propagateWholeTrajectory(in, 2, 500);
    }

    @Override
    public void draw(Graphics2D g) {
        Rectangle2D rect = new Rectangle2D.Double(x, y - height, width, height);
        g.setColor(Color.white);
        g.draw(rect);

        g.setColor(Color.green);
        g.draw(new Line2D.Double(790, y - 205, 810, y - 205));
        g.drawLine(800, (int) y - 205 - 10, 800, (int) y - 205 + 10);

        // draw speaker
        // g.drawLine(800 + 23, (int)y - 198, 800 - 23, (int)y - 211);
        g.drawLine(800 - 23, (int) y - 211, 800 - 23, (int) y - 211 - 17);
        g.drawLine(800 - 23 + 122, (int) y - 198, 800 - 23 + 122, (int) y - 198 - 35);
        g.drawLine(800 - 23 + 122, (int) y - 198 - 35, 800 - 23 + 122 - 65,(int) y - 198 - 35 - 19);
        g.drawLine(800 - 23, (int) y - 211 - 17, 800 - 23 + 122 - 65,(int) y - 198 - 35 - 19);
        g.drawLine(800 - 23 + 122, (int) y - 198, 800 + 23, (int) y - 198);

        g.setColor(Color.WHITE);
        g.draw(new Line2D.Double((x + width / 2) + SHOOTER_PIVOT_ROBOT_REL.getX() * 100, y - SHOOTER_PIVOT_ROBOT_REL.getY() * 100, (x + width / 2) + (shooterExit.getX() * 100), y - (shooterExit.getY() * 100)));

        g.setColor(Color.ORANGE);


        ArrayList<Point> trajectoryScreenPoints = new ArrayList<>();
        for (Vector v : trajectory) {
            int screenX = (int) ((x + width / 2) + (v.get(0, 0) * 100));
            int screenY = (int) (y - v.get(1, 0) * 100);
            trajectoryScreenPoints.add(new Point(screenX, screenY));

        }

        if (!trajectoryScreenPoints.isEmpty()) {
            Point prev = trajectoryScreenPoints.get(0);
            for (Point p : trajectoryScreenPoints) {
                g.drawLine(prev.x, prev.y, p.x, p.y);
                g.drawLine(prev.x, prev.y + 5, p.x, p.y + 5);
                g.drawLine(prev.x, prev.y - 5, p.x, p.y - 5);
                prev = p;
            }
        }

        g.setColor(Color.WHITE);
        g.drawString("RobotX     " + robotPose, 10, 40);
        g.drawString("Distance   " + (800 - (x + width / 2) + (shooterExit.getX() * 100)) / 100, 10, 50);
        g.drawString("Theta        " + shooterTheta, 10, 60);
        g.drawString("Time         " + trajectoryTime, 10, 70);
    }
}
