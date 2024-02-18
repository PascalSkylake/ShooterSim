package org.example;

public class Main {
    public static Simulator sim;
    public static Robot robot;
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
    }
}