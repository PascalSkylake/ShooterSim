package org.example;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N5;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class Note extends Entity {
    ArrayList<Vector<N5>> trajectory;
    Vector<N5> pose;
    double x, y, width, height;
    int screenX, screenY;
    double initialX, initialY, initialTheta;
    int counter = 0;

    public Note(ArrayList<Vector<N5>> trajectory) {
        this.trajectory = trajectory;
        pose = trajectory.get(0);

        x = (int) (pose.get(0, 0) * 100);
        y = (int) (pose.get(1, 0) * 100);
        width = 36;
        height = 5;

        screenX = (int) ((Main.robot.x + width / 2) + (pose.get(0, 0) * 100));
        screenY = (int) (Main.robot.y - (pose.get(1, 0) * 100));
        initialX = Main.robot.x;
        initialY = Main.robot.y;
        initialTheta = Main.robot.shooterTheta;

    }
    @Override
    public void update() {
        if (counter == trajectory.size()) {
            return;
        }
        pose = trajectory.get(counter);

        x = (int) (pose.get(0, 0) * 100);
        y = (int) (pose.get(1, 0) * 100);

        width = 36;
        height = 5;

        screenX = (int) ((initialX + width / 2) + (pose.get(0, 0) * 100));
        screenY = (int) (initialY - (pose.get(1, 0) * 100));

        //System.out.println(screenX + " " + screenY);
        counter++;
    }

    @Override
    public void draw(Graphics2D g) {
        if (counter == trajectory.size()) {
            EntityHandler.removeEntity(this);
            return;
        }
        if (trajectory.size() > 0) {
            g.setColor(Color.ORANGE);
            AffineTransform transform = new AffineTransform();
            transform.translate(screenX, screenY);
            transform.rotate(-initialTheta);
            g.transform(transform);

            Rectangle2D.Double pose = new Rectangle2D.Double(-width / 2.0, height / 2.0, width, height);
            g.draw(pose);

        }


    }
}
