package org.example;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N5;

import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.text.DecimalFormat;
import java.util.ArrayList;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.*;
import org.apache.commons.math3.optim.linear.NonNegativeConstraint;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.BOBYQAOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.MultiDirectionalSimplex;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;
import org.example.math.Shooter;

public class Robot extends Entity {
    double shooterTheta = 0.5;
    double shooterLength = 37.5;
    double trajectoryTime = 0;
    double[][] trajectory;

    Shooter shooter = new Shooter();
    private final Translation2d SHOOTER_PIVOT_ROBOT_REL = new Translation2d(-0.2757, 0.5972);
    private Translation2d shooterExit = new Translation2d(0, 0);
    double robotPose = 0;
    double distance = 0;
    boolean spacePrev = false;
    DecimalFormat format = new DecimalFormat("#.#####");
    double vTheta = 0;
    double vX = 0;
    double vY = 0;
    double v = 0;
    double z = 0;
    double v0 = 20;
    double phi;
    double theta;

    public Robot(double width, double height) {
        super(2, 2, 71.12, 71.12);

    }

    @Override
    public void update() {
        if (Key.isKeyPressed(KeyEvent.VK_Z)) {
            this.x = Field.screenXtoField(Mouse.x);
            this.y = Field.screenYtoField(Mouse.y);
        }

        if (Mouse.state.equals(MouseState.CLICKED) || Mouse.state.equals(MouseState.DRAGGED)) {
            double vectorX = Mouse.x;
            double vectorY = Mouse.y;
            double tx = Field.screenXtoField(Mouse.x) - this.x;
            double ty = Field.screenYtoField(Mouse.y) - this.y;
            this.vX = tx;
            this.vY = ty;
            this.v = Math.sqrt((tx * tx) + (ty * ty));
            this.vTheta = Math.atan2(ty, tx);
        }

        double dx = Field.targetX - x;
        double dy = Field.targetY - y;
        distance = Math.sqrt((dx * dx) + (dy * dy));
        double[] optimal = optimizeShooterOrientation(1, Math.atan2(Field.targetY - y, Field.targetX - x), 0.001, Field.targetX, Field.targetY, Field.targetZ);
        optimal[1] = normalizeAngle(optimal[1]);
        double[] in = {x, y, 0,
                 vX + (v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.cos(optimal[1])),
                 vY + (v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.sin(optimal[1])), v0 * Math.sin(Math.PI / 2 - optimal[0])};
        trajectory = shooter.propagateWholeTrajectory3d(in, optimal[2], 5);
        angle = optimal[1];
        phi = optimal[1];
        theta = optimal[0];

        if (trajectory[trajectory.length - 1][5] < 0 || Math.abs(trajectory[trajectory.length - 1][2] - Field.targetZ) > 0.1) {
            trajectory = null;
        }

        x += vX / 100;
        y += vY / 100;





    }

    public static double normalizeAngle(double angle) {
        return angle - (Math.ceil((angle + Math.PI)/(2*Math.PI))-1)*2*Math.PI;
    }

    @Override
    public void draw(Graphics2D g) {
        g.setColor(Color.WHITE);
        // Create a new transform for the group
        AffineTransform transform = new AffineTransform();
        transform.translate(Field.fixX(x), Field.fixY(y));
        transform.rotate(-angle);
        transform.translate(-width / 2, -height / 2);

        // Apply the transform to the graphics context
        g.transform(transform);

        // Draw the rectangle and line as part of the group
        Rectangle2D r = new Rectangle2D.Double(0, 0, width, height);
        Line2D line = new Line2D.Double(width / 2, height / 2, width / 2 + 30, height / 2);
        g.draw(r);
        g.draw(line);

        // Reset the transform on the graphics context
        g.setTransform(new AffineTransform());

        g.setColor(Color.RED);
        g.drawLine(Field.fixX(x), Field.fixY(y), Field.fixX(vX + x), Field.fixY(vY + y));

        g.setColor(Color.ORANGE);
        if (trajectory != null && !(trajectory.length == 0)) {
            double prevx = x;
            double prevy = y;

            for (double[] d : trajectory) {
                g.drawLine(Field.fixX(prevx), Field.fixY(prevy), Field.fixX(d[0]), Field.fixY(d[1]));
                prevx = d[0];
                prevy = d[1];
            }

            try {
                g.setColor(Color.WHITE);
                Font f = new Font("Courier New", 0, 20);
                g.setFont(f);
                g.drawString("RobotX    " + format.format(x) + "   meters", 10, 50);
                g.drawString("RobotY    " + format.format(y) + "   meters", 10, 70);
                g.drawString("Distance  " + format.format(distance) + " meters", 10, 90);
                g.drawString("Speed     " + format.format(Math.sqrt((vX * vX) + (vY * vY))) + " m/s", 10, 110);
                g.drawString("Theta     " + format.format(90 - Math.toDegrees(theta)) + " degrees", 10, 130);
                g.drawString("Phi      " + format.format(Math.toDegrees(phi)) + " degrees", 10, 150);
                g.drawString("Time      " + format.format(trajectory[trajectory.length - 1][6]) + " seconds", 10, 170);
            } catch (NullPointerException e) {

            }
        } else {
            g.setColor(Color.WHITE);
            Font f = new Font("Courier New", 0, 20);
            g.setFont(f);
            g.drawString("RobotX    " + format.format(x) + "   meters", 10, 50);
            g.drawString("RobotY    " + format.format(y) + "   meters", 10, 70);
            g.drawString("Distance  " + format.format(distance) + " meters", 10, 90);
            g.drawString("Speed     " + format.format(Math.sqrt((vX * vX) + (vY * vY))) + " m/s", 10, 110);
            g.drawString("Theta     " + format.format(90 - Math.toDegrees(theta)) + " degrees", 10, 130);
            g.drawString("Phi      " + format.format(Math.toDegrees(phi)) + " degrees", 10, 150);
            g.drawString("Time      " + format.format(0) + " seconds", 10, 170);
            g.setColor(Color.RED);
            g.drawString("CANNOT SHOOT HERE", 10, 190);
        }


    }

    private double[] optimizeShooterOrientation(double initialTheta, double initialPhi, double initialTime, double targetX, double targetY, double targetZ) {

        MultivariateFunction f = point -> {
            // calculate trajectory given theta phi and t

            double[] in = {x, y, z,
                    vX + (v0 * Math.sin(Math.PI / 2 - point[0]) * Math.cos(point[1])),
                    vY + (v0 * Math.sin(Math.PI / 2 - point[0]) * Math.sin(point[1])), Main.v0 * Math.sin(Math.PI / 2 - point[0])};
            double[][] trajectory = Shooter.propagateWholeTrajectory3d(in, point[2], 1);
            double[] finalPosition = trajectory[trajectory.length - 1];

            double xdiff = targetX - finalPosition[0];
            double ydiff = targetY - finalPosition[1];
            double zdiff = targetZ - finalPosition[2];

            double distance = Math.sqrt((xdiff * xdiff) + (ydiff * ydiff) + (zdiff * zdiff));
//            if (finalPosition[5] < 0) {
//                distance += 5;
//            }

            return distance;
        };

        ObjectiveFunction objective = new ObjectiveFunction(f);

        double[] initialGuess = new double[] { initialTheta, initialPhi, initialTime };
        InitialGuess guess = new InitialGuess(initialGuess);
        MaxIter maxIter = new MaxIter(20000);
//        // look at my lawyer dawg I'm goin to jail!!!
        MaxEval maxEval = new MaxEval(100000);
        MultiDirectionalSimplex simplex = new MultiDirectionalSimplex(3, 0.001);
        SimplexOptimizer optimizer = new SimplexOptimizer(new SimpleValueChecker(0.000001, 0.000001));

        BOBYQAOptimizer powell = new BOBYQAOptimizer(5);

        // Set bounds
        double[] lowerBound = {0, 0, 0};
        double[] upperBound = {Math.PI / 2, 2 * Math.PI, 1};
        SimpleBounds bounds = new SimpleBounds(new double[] {0, -Math.PI, 0}, new double[] {Math.PI/2, Math.PI, 1});
        NonNegativeConstraint constraint = new NonNegativeConstraint(true);

        PointValuePair optimum = optimizer.optimize(
                maxIter,
                maxEval,
                objective,
                GoalType.MINIMIZE,
                //bounds,
                guess,
                simplex,
                constraint

        );


        return optimum.getPoint();
    }
}
