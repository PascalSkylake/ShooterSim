package org.example;

public class Main {
    public static Simulator sim;
    public static Robot robot;

    public static void main(String[] args) {
        sim = new Simulator("Arm Simulator", false);
        robot = new Robot(10, 10);

        sim.createMouseListener();
        sim.createKeyboardListener();


        EntityHandler.addEntity(robot);
    }
}