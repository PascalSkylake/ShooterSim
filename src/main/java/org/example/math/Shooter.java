package org.example.math;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.DoubleStream;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.numbers.N8;

public class Shooter {
    Function<Vector<N6>, Vector<N6>> projectileEquation;
    Function<double[], double[]> projectileEquation3d;

    private final double SPEAKER_HEIGHT = 1.9812;
    private final double DRAG_COEFFICIENT = 0.5;
    private final double AIR_DENSITY = 1.225;
    private final double CROSS_SECTIONAL_AREA = 0.018;
    private final double NOTE_MASS = 0.2353;
    private final double MU = (DRAG_COEFFICIENT * AIR_DENSITY * CROSS_SECTIONAL_AREA) / (2 * NOTE_MASS);

    private final double SHOOTER_PIVOT_TO_END = 0.37516;
    private final Translation2d SHOOTER_PIVOT_ROBOT_REL = new Translation2d(-0.2757, 0.5972);

    public Shooter() {
        projectileEquation3d = (double[] x) -> {
            double vx = x[3];
            double vy = x[4];
            double vz = x[5];
            double v = Math.sqrt((vx * vx) + (vy * vy) + (vz * vz));
            double ax = -MU * vx * v;
            double ay = -MU * vy * v;
            double az = -9.8 - (MU * vz * v);

            return new double[]{vx, vy, vz, ax, ay, az, 0, 0};
        };
    }

    public double[] rkFour(double[] x, Function<double[], double[]> f) {
        double h = x[x.length - 1];

        double[] k_1 = f.apply(x);
        double[] k_2 = f.apply(addVectors(x, multiplyByScalar(k_1, h / 2)));
        double[] k_3 = f.apply(addVectors(x, multiplyByScalar(k_2, h / 2)));
        double[] k_4 = f.apply(addVectors(x, multiplyByScalar(k_3, h)));

        double[] out = addVectors(multiplyByScalar(addVectors(addVectors(addVectors(k_1, multiplyByScalar(k_2, 2)), multiplyByScalar(k_3, 2)), k_4), h / 6), x);
        out[x.length - 2] += h;
        return out;
    }

    public double[] addVectors(double[] a, double[] b) {
        double[] out = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            out[i] = a[i] + b[i];
        }
        return out;
    }

    public double[] multiplyVectors(double[] a, double[] b) {
        double[] out = new double[8];
        for (int i = 0; i < a.length; i++) {
            out[i] = a[i] * b[i];
        }
        return out;
    }

    public double[] multiplyByScalar(double[] a, double b) {
        double[] out = new double[8];
        for (int i = 0; i < a.length; i++) {
            out[i] = a[i] * b;
        }
        return out;
    }

//    public Vector<N6> propagateState(Vector<N4> x, double t, int intervals) {
//        double dt = t / intervals;
//        Vector<N6> state = VecBuilder.fill(x.get(0, 0), x.get(1, 0), x.get(2, 0), x.get(3, 0), dt, 0);
//        for (int i = 0; i < intervals; i++) {
//            state = rkFour(state, projectileEquation);
//            System.out.println("" + (i * dt) + "," + state.get(0, 0) + "," + state.get(1, 0) + "," + state.get(2, 0) + ","
//                    + state.get(3, 0));
//
//        }
//
//        return state;
//    }

//    public Optional<Double> calcTrajectoryIntersectWithSpeakerHeightPlane(Vector<N4> k, double dt) {
//        double x = k.get(0, 0);
//        double y = k.get(1, 0);
//        double vx = k.get(3, 0);
//        double vy = k.get(4, 0);
//        Vector<N6> state = VecBuilder.fill(x, y, vx, vy, 0, dt);
//
//        while (state.get(3, 0) > 0) {
//
//            state = rkFour(state, projectileEquation);
//
//            if (state.get(1, 0) > SPEAKER_HEIGHT) {
//                double timeAgo = (state.get(1, 0) - SPEAKER_HEIGHT) / state.get(3, 0);
//                double intersectX = state.get(0, 0) - (timeAgo * state.get(2, 0));
//                //System.out.println(state.get(4, 0) + "," + intersectX);
//                return Optional.of(intersectX);
//            }
//
//        }
//
//        return Optional.empty();
//    }

//    public Optional<Double> calcTrajectoryIntersectWithSpeakerHeightPlane3d(Vector<N6> k, double dt) {
//        double x = k.get(0, 0);
//        double y = k.get(1, 0);
//        double z = k.get(2, 0);
//        double vx = k.get(3, 0);
//        double vy = k.get(4, 0);
//        double vz = k.get(5, 0);
//
//        Vector<N8> state = VecBuilder.fill(x, y, z, vx, vy, vz, 0, dt);
//
//        while (state.get(5, 0) > 0) {
//
//            state = rkFour(state, projectileEquation3d);
//            // System.out.println(state.get(2, 0) + "," + state.get(5, 0));
//
//            if (state.get(2, 0) > SPEAKER_HEIGHT) {
//                double timeAgo = (state.get(2, 0) - SPEAKER_HEIGHT) / state.get(5, 0);
//                double intersectX = state.get(0, 0) - (timeAgo * state.get(3, 0));
//                double intersectY = state.get(1, 0) - (timeAgo * state.get(4, 0));
//                System.out.println(state.get(6, 0) + "," + intersectX + "," + intersectY);
//                return Optional.of(intersectX);
//            }
//
//        }
//
//        return Optional.empty();
//    }

    public double[][] propagateWholeTrajectory3d(double[] k, double t, int intervals) {
        double x = k[0];
        double y = k[1];
        double z = k[2];
        double vx = k[3];
        double vy = k[4];
        double vz = k[5];

        double[][] out = new double[intervals][k.length + 2];
        double dt = t / intervals;

        double[] state = {x, y, z, vx, vy, vz, 0, dt};

        for (int i = 0; i < intervals; i++) {
            state = rkFour(state, projectileEquation3d);
            out[i] = state;
        }

        return out;
    }

    public Translation2d shooterExitRobotRelative(double theta) {
        double x = SHOOTER_PIVOT_TO_END * Math.cos(theta);
        double y = SHOOTER_PIVOT_TO_END * Math.sin(theta);

        return SHOOTER_PIVOT_ROBOT_REL.plus(new Translation2d(x, y));
    }

}
