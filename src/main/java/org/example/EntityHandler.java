package org.example;

import java.awt.*;
import java.util.ArrayList;

public class EntityHandler {
    static ArrayList<Entity> entities = new ArrayList<>();
    static ArrayList<Entity> toRemove = new ArrayList<>();
    static ArrayList<Entity> toAdd = new ArrayList<>();
    private static boolean displayHitbox = false;

    public static void update() {
        for (Entity entity : entities) {
            entity.update();
        }

        if (!toRemove.isEmpty() || !toAdd.isEmpty()) {
            entities.removeAll(toRemove);
            entities.addAll(toAdd);
            toRemove.clear();
            toAdd.clear();
        }
    }

    public static void draw(Graphics2D g) {
        for (Entity entity : entities) {
            entity.draw(g);
            if (displayHitbox) {
                g.setColor(Color.GREEN);
                g.drawRect((int)entity.x, (int)entity.y, (int)entity.width, (int)entity.height);
            }
        }
    }

    public static void addEntity(Entity entity) {
        toAdd.add(entity);
    }

    public static void removeEntity(Entity entity) {
        toRemove.add(entity);
    }

    public static void setDisplayHitbox(boolean display) {
        displayHitbox = display;
    }
}