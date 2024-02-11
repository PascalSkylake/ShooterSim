package org.example;

import javax.swing.*;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.HashMap;

public class Key extends KeyAdapter {
    private static final HashMap<Integer, Boolean> currentlyPressed = new HashMap<>();

    public Key(JFrame frame) {
        frame.addKeyListener(this);
    }

    @Override
    public void keyPressed(KeyEvent e) {
        currentlyPressed.put(e.getKeyCode(), true);
    }

    @Override
    public void keyReleased(KeyEvent e) {
        currentlyPressed.put(e.getKeyCode(), false);
    }

    public static boolean isKeyPressed(int key) {
        if (currentlyPressed.get(key) == null) {
            System.out.println("Unexpected key (" + key + ") pressed!");
            currentlyPressed.put(key, false);
            return false;
        } else if (currentlyPressed.get(key)){
            System.out.println("pressed key" + (char) key);
            return currentlyPressed.get(key);
        } else {
            return false;
        }
    }

    public static boolean isKeyPressed(char key) {
        return isKeyPressed((int) key);
    }

    public static void addExpectedKey(int key) {
        currentlyPressed.put(key, false);
    }

    public static void addExpectedKey(char key) {
        addExpectedKey((int) key);
    }

    public static void update() {

    }
}
