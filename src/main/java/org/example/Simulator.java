package org.example;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferStrategy;

public class Simulator {
    static JFrame frame;
    public static Thread tickThread, drawThread;
    boolean running;
    public final Window window;
    Color background;
    private final int tickRate = 60;

    public Simulator(String title, boolean fullscreen) {
        window = new Window(title, fullscreen);
        frame = window.frame;
        start();
    }

    public Simulator(String title, boolean fullscreen, Color color) {
        window = new Window(title, fullscreen);
        frame = window.frame;
        background = color;
        start();
    }

    public synchronized void start() {
        running = true;
        tickThread = new Thread(() -> {
            long lastime = System.nanoTime();
            double ns = 1000000000 / tickRate;
            double delta = 0;


            while (running) {
                long now = System.nanoTime();
                delta += (now - lastime) / ns;
                lastime = now;

                if (delta >= 1) {
                    Key.update();
                    update();
                    delta--;
                }
            }
        });

        drawThread = new Thread(() -> {
            long lastime = System.nanoTime();
            double ns = 1000000000 / tickRate;
            double delta = 0;


            while (running) {
                long now = System.nanoTime();
                delta += (now - lastime) / ns;
                lastime = now;

                if (delta >= 1) {
                    draw();
                    delta--;
                }
            }
        });
        tickThread.setName("TickThread");
        drawThread.setName("DrawThread");
        tickThread.start();
        drawThread.start();
    }

    public void update() {
        EntityHandler.update();
    }

    public void draw() {
        BufferStrategy bs = window.frame.getBufferStrategy();
        if (bs == null) {
            window.frame.createBufferStrategy(2);
            return;
        }
        Graphics2D g = (Graphics2D) bs.getDrawGraphics();
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        //g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);
        g.setColor(background);
        g.fillRect(frame.getX(), frame.getY(), frame.getWidth(), frame.getHeight());
        EntityHandler.draw(g);
        g.dispose();
        bs.show();
    }

    public void createKeyboardListener() {
        try {
            new Key(frame);
            boolean key = true;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void createMouseListener() {
        try {
            Mouse m = new Mouse();
            window.getFrame().addMouseListener(m);
            window.getFrame().addMouseMotionListener(m);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public int getTickRate() {
        return tickRate;
    }
}
