package org.example;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.optim.*;
import org.apache.commons.math3.optim.linear.NonNegativeConstraint;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.*;
import org.example.math.Shooter;

import java.util.ArrayList;

public class Main {
    public static Simulator sim;
    public static Robot robot;
    static Shooter shooter = new Shooter();
    public static Field field = new Field();
    static double v0 = 20;

    public static void main(String[] args) {
        sim = new Simulator("Arm Simulator", false);
        field = new Field();
        robot = new Robot(71.12, 71.12);

        sim.createKeyboardListener();
        sim.createMouseListener();
        EntityHandler.addEntity(field);
        EntityHandler.addEntity(robot);

        double x = v0 * Math.sin(1) * Math.cos(1);
        double y = v0 * Math.sin(1) * Math.sin(1);
        double z = v0 * Math.sin(1);
    }
}