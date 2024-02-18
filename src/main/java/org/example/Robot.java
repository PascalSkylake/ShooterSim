package org.example;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

import edu.wpi.first.math.util.Units;
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
    private final double SHOOTER_PIVOT_TO_END = 0.2413;
    private final Translation3d SHOOTER_PIVOT_ROBOT_REL = new Translation3d(-0.2921, 0, 0.5588);
    double distance = 0;
    DecimalFormat format = new DecimalFormat("#.#####");
    double vTheta = 0;
    double vX = 0;
    double vY = 0;
    double v = 0;
    double z = 0;
    double v0 = 15;
    double phi;
    double theta;
    boolean goodTrajectory = false;
    Translation3d shooterPose;
    Translation3d shooterPoseRobotRelative;

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
        double[] optimal = optimizeShooterOrientation(1, Math.atan2(Field.targetY - y, Field.targetX - x), 0.1, Field.targetX, Field.targetY, Field.targetZ);
        optimal[1] = normalizeAngle(optimal[1]);


        angle = optimal[1];
        phi = optimal[1];
        theta = optimal[0];


        Pose2d pose = new Pose2d(x, y, new Rotation2d(optimal[1]));
        shooterPoseRobotRelative = shooterExitRobotRelative(optimal[0]);
        shooterPose = shooterExitFieldRelative(pose, shooterPoseRobotRelative);
        //System.out.println(shooterPose);

        double[] in = {shooterPose.getX(), shooterPose.getY(), shooterPose.getZ(),
                vX + (v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.cos(optimal[1])),
                vY + (v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.sin(optimal[1])), v0 * Math.cos(Math.PI / 2 - optimal[0])};
        trajectory = shooter.propagateWholeTrajectory3d(in, optimal[2], 100);


        if (trajectory[trajectory.length - 1][5] < 0 || Math.abs(trajectory[trajectory.length - 1][2] - Field.targetZ) > 0.2) {
            goodTrajectory = false;
        } else {
            goodTrajectory = true;
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
        if (goodTrajectory && !(trajectory.length == 0)) {
            double prevx = shooterPose.getX();
            double prevy = shooterPose.getY();

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
                g.drawString("Theta     " + format.format(Math.toDegrees(theta)) + " degrees", 10, 130);
                g.drawString("Phi       " + format.format(Math.toDegrees(phi)) + " degrees", 10, 150);
                g.drawString("Time      " + format.format(trajectory[trajectory.length - 1][6]) + " seconds", 10, 170);
                g.drawString("Error     " + format.format((Math.abs(trajectory[trajectory.length - 1][2] - Field.targetZ)) * 100) + " cm", 10, 190);
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
            g.drawString("Theta     " + format.format(Math.toDegrees(theta)) + " degrees", 10, 130);
            g.drawString("Phi      " + format.format(Math.toDegrees(phi)) + " degrees", 10, 150);
            g.drawString("Time      " + format.format(0) + " seconds", 10, 170);
            g.setColor(Color.RED);
            g.drawString("CANNOT SHOOT HERE", 10, 190);
        }

        g.setColor(Color.WHITE);
        int robotCenter = 1800 - (int) Units.metersToInches(distance);
        g.drawRect(robotCenter - (14), 200 - 3, 28, 3);
        //vertical pole
        g.drawLine(robotCenter + 6, 200, robotCenter + 6, 200 - 22);
        //arm horizontal
        g.drawLine(robotCenter + 6, 200 - 22, robotCenter + (int) Units.metersToInches(SHOOTER_PIVOT_ROBOT_REL.getX()), 200 - (int) Units.metersToInches(SHOOTER_PIVOT_ROBOT_REL.getZ()));
        //shooter?
        g.drawLine(robotCenter + (int) Units.metersToInches(SHOOTER_PIVOT_ROBOT_REL.getX()), 200 - (int) Units.metersToInches(SHOOTER_PIVOT_ROBOT_REL.getZ()), robotCenter + (int) Units.metersToInches(shooterPoseRobotRelative.getX()), 200 - (int) Units.metersToInches(shooterPoseRobotRelative.getZ()));
        //System.out.println(shooterPoseRobotRelative);
        g.setColor(Color.GREEN);
        g.drawLine(1800, 200 - 81 - 3, 1800, 200 - 81 + 3);
        g.drawLine(1800 - 3, 200 - 81, 1800 + 3, 200 - 81);

        int robotX = robotCenter;
        int robotY = 200;

        double prevDist = 0;
        double prevZ = shooterPoseRobotRelative.getZ();

        if (goodTrajectory) {
            g.setColor(Color.GREEN);
        } else {
            g.setColor(Color.RED);
        }
            for (double[] d : trajectory) {
                double dx = d[0] - x;
                double dy = d[1] - y;

                double dist = Math.sqrt((dx * dx) + (dy * dy));

                g.drawLine(robotX + (int)  Units.metersToInches(prevDist), robotY - (int) Units.metersToInches(prevZ), robotX + (int) Units.metersToInches(dist), robotY - (int) Units.metersToInches(d[2]));
                prevDist = Math.sqrt((dx * dx) + (dy * dy));
                prevZ = d[2];
            }
    }

    private double[] optimizeShooterOrientation(double initialTheta, double initialPhi, double initialTime, double targetX, double targetY, double targetZ) {

        MultivariateFunction f = point -> {
            // calculate trajectory given theta phi and t
            Pose2d pose = new Pose2d(x, y, new Rotation2d(initialPhi));
            Translation3d shooterPose = shooterExitFieldRelative(pose, shooterExitRobotRelative(initialTheta));

            double[] in = {shooterPose.getX(), shooterPose.getY(), shooterPose.getZ(),
                    vX + (v0 * Math.sin((Math.PI / 2) - point[0]) * Math.cos(point[1])),
                    vY + (v0 * Math.sin((Math.PI / 2) - point[0]) * Math.sin(point[1])),
                    v0 * Math.cos(Math.PI / 2 - point[0])};
            double[][] trajectory = Shooter.propagateWholeTrajectory3d(in, point[2], 1);
            double[] finalPosition = trajectory[trajectory.length - 1];

            double xdiff = targetX - finalPosition[0];
            double ydiff = targetY - finalPosition[1];
            double zdiff = (targetZ + 0.1) - finalPosition[2];

            double distance = Math.sqrt((xdiff * xdiff) + (ydiff * ydiff) + (zdiff * zdiff));

            return distance * 2;
        };

        ObjectiveFunction objective = new ObjectiveFunction(f);

        double[] initialGuess = new double[] { initialTheta, initialPhi, initialTime };
        InitialGuess guess = new InitialGuess(initialGuess);
        MaxIter maxIter = new MaxIter(20000);
        // look at my lawyer dawg I'm goin to jail!!!
        MaxEval maxEval = new MaxEval(100000);
        MultiDirectionalSimplex simplex = new MultiDirectionalSimplex(3, 0.1);
        SimplexOptimizer optimizer = new SimplexOptimizer(new SimpleValueChecker(0.0001, 0.0001));

        NonNegativeConstraint constraint = new NonNegativeConstraint(true);

        PointValuePair optimum = optimizer.optimize(
                maxIter,
                maxEval,
                objective,
                GoalType.MINIMIZE,
                guess,
                simplex,
                constraint

        );


        return optimum.getPoint();
    }

    public Translation3d shooterExitRobotRelative(double theta) {
        double tx = SHOOTER_PIVOT_TO_END * Math.cos(theta);
        double ty = 0;
        double tz = SHOOTER_PIVOT_TO_END * Math.sin(theta);

        return SHOOTER_PIVOT_ROBOT_REL.plus(new Translation3d(tx, ty, tz));
    }

    public Translation3d shooterExitFieldRelative(Pose2d robotPose, Translation3d shooterRobotRelative) {
        double diffX = shooterRobotRelative.getX();

        Translation3d shooterFieldRelative = new Translation3d(robotPose.getX() + (diffX * robotPose.getRotation().getCos()), robotPose.getY() + (diffX * robotPose.getRotation().getSin()), shooterRobotRelative.getZ());
        return shooterFieldRelative;
    }
}
