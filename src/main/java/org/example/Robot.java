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
import java.text.DecimalFormat;
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
    boolean spacePrev = false;
    DecimalFormat format = new DecimalFormat("#.#####");

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



        robotPose = ((x + width / 2) / 100.0);
        distance = 8 - robotPose;

        shooterTheta = shooter.thetaFromDistance2(distance);
        shooterExit = shooter.shooterExitRobotRelative(shooterTheta);
        trajectoryTime = shooter.timeFromDistance(distance);

        Vector<N4> in = VecBuilder.fill(shooterExit.getX(), shooterExit.getY(), 20 * Math.cos(shooterTheta), 20 * Math.sin(shooterTheta));

        trajectory = shooter.propagateWholeTrajectory(in, 0.5, 200);



        if (!spacePrev && Key.isKeyPressed(KeyEvent.VK_SPACE)) {
            spacePrev = true;

            ArrayList<Vector<N5>> temp = new ArrayList<>();
            for (Vector v : trajectory) {
                temp.add(VecBuilder.fill(v.get(0, 0), v.get(1, 0), v.get(2, 0), v.get(3, 0), v.get(4, 0)));
            }
            EntityHandler.addEntity(new Note(temp));
            System.out.println("!!!");
        } else if (spacePrev && !Key.isKeyPressed(KeyEvent.VK_SPACE)) {
            spacePrev = false;
        }
    }

    @Override
    public void draw(Graphics2D g) {
        Rectangle2D rect = new Rectangle2D.Double(x, y - height, width, height);
        g.setColor(new Color(0, 80, 255));
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

        g.setColor(Color.GRAY);

        try {
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
                    //g.drawLine(prev.x, prev.y + 5, p.x, p.y + 5);
                    //g.drawLine(prev.x, prev.y - 5, p.x, p.y - 5);
                    prev = p;
                }
            }
        } catch (Exception e) {
            System.out.println("concurrent modification");
        }

        g.setColor(Color.WHITE);
        Font f = new Font("Courier New", 0, 20);
        g.setFont(f);
        g.drawString("RobotX    " + format.format(robotPose) + "   meters", 10, 50);
        g.drawString("Distance  " + format.format((800 - (x + width / 2) + (shooterExit.getX() * 100)) / 100) + " meters", 10, 70);
        g.drawString("Theta     " + format.format(shooterTheta) + " radians", 10, 90);
        g.drawString("Time      " + format.format(trajectoryTime) + " seconds", 10, 110);
    }
}
