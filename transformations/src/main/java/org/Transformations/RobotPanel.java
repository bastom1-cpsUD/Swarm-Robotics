package org.transformations;

import java.awt.Dimension;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.List;

public class RobotPanel extends javax.swing.JPanel {

    private List<Robot> robots;

    public RobotPanel() {
        robots = new ArrayList<>();
        this.setPreferredSize(new Dimension(300, 300));
    }

    public void addRobot(Robot robot) {
        robots.add(robot);
    }

    public void paintComponent(java.awt.Graphics g) {
        super.paintComponent(g);
        for (Robot robot : robots) {
            robot.draw((Graphics2D) g);
        }
    }

    public List<Robot> getRobots() {
        return robots;
    }
}
