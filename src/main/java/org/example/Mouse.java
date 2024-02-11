package org.example;

import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;

public class Mouse extends MouseAdapter {
    public static MouseState state = MouseState.RELEASED;
    public static int amountScrolled = 1;
    public static int x, y;

    @Override
    public void mouseClicked(MouseEvent e) {
        state = MouseState.CLICKED;
        x = e.getX();
        y = e.getY();
    }

    @Override
    public void mousePressed(MouseEvent e) {
        state = MouseState.PRESSED;
        x = e.getX();
        y = e.getY();
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        state = MouseState.DRAGGED;
        x = e.getX();
        y = e.getY();
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        state = MouseState.RELEASED;
        x = e.getX();
        y = e.getY();
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        state = MouseState.MOVED;
        x = e.getX();
        y = e.getY();
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        System.out.println("A");
        if (e.getWheelRotation() < 1) {
            Mouse.amountScrolled = 1;
        } else {
            Mouse.amountScrolled += e.getWheelRotation();
            System.out.println(e.getWheelRotation());
        }
    }
}
