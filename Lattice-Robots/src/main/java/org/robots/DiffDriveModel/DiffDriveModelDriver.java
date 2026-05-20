package org.robots.DiffDriveModel;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.KeyListener;
import java.awt.event.KeyEvent;

import javax.swing.JPanel;
import javax.swing.Timer;

import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class DiffDriveModelDriver extends JPanel {

    private DiffDriveRobot robot;
    private ArrayList<Point2D> trace;
    private boolean traceOn;
    
    public DiffDriveModelDriver() {
        this.setPreferredSize(new Dimension(1000, 1000));
        this.setBackground(java.awt.Color.WHITE);
        this.setFocusable(true);
        this.requestFocusInWindow();

        robot = new DiffDriveRobot();
        trace = new ArrayList<>();
        traceOn = false;

        addKeyListener( new KeyListener() {
            @Override
            public void keyTyped(KeyEvent e) {}

            @Override
            public void keyPressed(KeyEvent e) {
                if(e.getKeyCode() == KeyEvent.VK_SPACE) {
                    beginSimulation();
                }

                if(e.getKeyCode() == KeyEvent.VK_T) {
                    traceOn = !traceOn;
                }
            }

            @Override
            public void keyReleased(KeyEvent e) {}
        });
    }

    private void beginSimulation() {
        //Move robot 100 meters, with v = 2m/s
        long startTime = System.nanoTime();

        final long[] lastMovementTime = {System.nanoTime()};

        Timer start = new Timer(1000 / 30, e -> {

            long currentTime = System.nanoTime();
            double dt = (currentTime - lastMovementTime[0]) / 1_000_000_000.0;

            lastMovementTime[0] = currentTime;
            //Robot get position; add to trace arraylist
            trace.add(robot.getPosition());
            robot.move(20, Math.PI / 8, dt);
            System.out.println(robot);
            System.out.println((currentTime - startTime) / 1_000_000_000 + " seconds");
            repaint();
        });

        start.start(); 
    }

    private void tracePath(Graphics2D g2d) {
        Path2D.Double path = new Path2D.Double();
        
        for(Point2D point : trace) {
            path.moveTo(point.getX(), point.getY());
            path.lineTo(point.getX(), point.getY());
        }
        path.closePath();
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setColor(Color.BLACK);
        g2d.draw(path);
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2d = (Graphics2D) g;

        if(traceOn) {
            tracePath(g2d);
        }

        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setColor(Color.BLACK);
        g2d.fill(robot.draw());
    }

    public static void main(String[] args) {

        javax.swing.JFrame frame = new javax.swing.JFrame("Lattice Robots Panel");
        DiffDriveModelDriver panel = new DiffDriveModelDriver();

        frame.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
        frame.add(panel);
        frame.pack();
        frame.setVisible(true);
        
    }
}
