package org.example;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

public class Window {
    public static int WIDTH, HEIGHT;
    public final JFrame frame;

    public Window(String title, boolean fullscreen) {
        frame = new JFrame();

        Dimension dimension = Toolkit.getDefaultToolkit().getScreenSize();
        WIDTH = (int) 1280;
        HEIGHT = (int) 720;
        frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
        frame.setSize(WIDTH, HEIGHT);
        frame.setUndecorated(fullscreen);
        frame.setLocation(100, 400);
        frame.setVisible(true);
        frame.setTitle(title);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.addWindowFocusListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                Simulator.drawThread.interrupt();
                System.exit(0);
            }
        });
    }

    public void addMouseListener(MouseListener mouseListener) {
        frame.addMouseListener(mouseListener);
    }

    public JFrame getFrame() {
        return frame;
    }
}
