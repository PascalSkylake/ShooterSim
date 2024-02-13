package org.example;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.numbers.N8;
import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.MaxIter;
import org.apache.commons.math3.optim.OptimizationData;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.PowellOptimizer;
import org.example.math.Shooter;

import java.util.ArrayList;

public class Main {
//    public static Simulator sim;
    public static Robot robot;
    static Shooter shooter = new Shooter();
    static double height = 0.5;
    static double v0 = 20;
    static double x = 0;
    static double y = 0;
    static double z = 0;
    static double vx = 0;
    static double vy = 0;
    static double targetx = 0;
    static double targety = 3;
    static double targetz = 2;

    public static void main(String[] args) {
//        sim = new Simulator("Arm Simulator", false);
        //robot = new Robot(10, 10);
//
//        sim.createMouseListener();
//        sim.createKeyboardListener();
//
//
//        EntityHandler.addEntity(robot);
        double[] optimal = new double[3];
        long start = System.nanoTime();

        optimal = optimizeShooterOrientation(1, Math.atan2(targety - y, targetx - x), 0.05, targetx, targety, targetz);
        System.out.println(Math.atan2(targety - y, targetx - x));
        System.out.println(optimal[0]);


        //System.out.println(optimal[0] + "," + optimal[1] + "," + optimal[2]);

        //Translation2d shooterExit = shooter.shooterExitRobotRelative(optimal[0]);
//         double[] in = {Main.x, Main.y, Main.z,
//                 Main.vx + (Main.v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.cos(optimal[1])),
//                 Main.vy + (Main.v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.sin(optimal[1])), Main.v0 * Math.sin(Math.PI / 2 - optimal[0])};
//         double[][] trajectory = shooter.propagateWholeTrajectory3d(in, optimal[2], 100);
//         double[] d = trajectory[trajectory.length - 1];


//        double[] in = {Main.x, Main.y, Main.z,
//                Main.vx + (Main.v0 * Math.sin(Math.PI / 2 - 1) * Math.cos(1)),
//                Main.vy + (Main.v0 * Math.sin(Math.PI / 2 - 1) * Math.sin(1)), Main.v0 * Math.sin(Math.PI / 2 - 1)};
//
//        //ArrayList<Vector<N8>> trajectory = shooter.propagateWholeTrajectory3d(in, 1, 20);
//        shooter.propagateWholeTrajectory3d(in, 1, 1000000);
        long end = System.nanoTime();
        double[] in = {Main.x, Main.y, Main.z,
                 Main.vx + (Main.v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.cos(optimal[1])),
                 Main.vy + (Main.v0 * Math.sin(Math.PI / 2 - optimal[0]) * Math.sin(optimal[1])), Main.v0 * Math.sin(Math.PI / 2 - optimal[0])};
        double[][] trajectory = shooter.propagateWholeTrajectory3d(in, optimal[2], 50);

        for (int i = 0; i < trajectory.length; i++) {
            double[] d = trajectory[i];
            //System.out.println(d[0] + "," + d[1] + "," + d[2]);
        }


        System.out.println((end - start) / 1000000.0);


    }



    private static double[] optimizeShooterOrientation(double initialTheta, double initialPhi, double initialTime, double targetX, double targetY, double targetZ) {

        MultivariateFunction f = point -> {
            // calculate trajectory given theta phi and t

            double[] in = {Main.x, Main.y, Main.z,
                    Main.vx + (Main.v0 * Math.sin(Math.PI / 2 - point[0]) * Math.cos(point[1])),
                    Main.vy + (Main.v0 * Math.sin(Math.PI / 2 - point[0]) * Math.sin(point[1])), Main.v0 * Math.sin(Math.PI / 2 - point[0])};
            double[][] trajectory = shooter.propagateWholeTrajectory3d(in, point[2], 50);
            double[] finalPosition = trajectory[trajectory.length - 1];

            double xdiff = targetX - finalPosition[0];
            double ydiff = targetY - finalPosition[1];
            double zdiff = targetZ - finalPosition[2];

            double distance = Math.sqrt((xdiff * xdiff) + (ydiff * ydiff) + (zdiff * zdiff));

            return distance;
        };

        ObjectiveFunction objective = new ObjectiveFunction(f);



        PowellOptimizer optimizer = new PowellOptimizer(0.000000000000001, 0.000000000000001);
        double[] initialGuess = new double[] { initialTheta, initialPhi, initialTime };
        InitialGuess guess = new InitialGuess(initialGuess);
        MaxIter maxIter = new MaxIter(20000);
        // look at my lawyer dawg I'm goin to jail!!!
        MaxEval maxEval = new MaxEval(100000);
        GoalType goalType = GoalType.MINIMIZE;

        double[] optimalParams = optimizer.optimize(objective, guess, maxIter, maxEval, goalType).getPoint();

        return optimalParams;
    }
}