package org.DiffDriveModel;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.event.KeyListener;
import java.awt.event.KeyEvent;

import javax.swing.JPanel;
import javax.swing.Timer;

import org.graphs.OrientedPoint;
import org.robots.LatticeRobot;

import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class DiffDriveModelDriver extends JPanel {

    private LatticeRobot robot;
    private ArrayList<Point2D> trace;
    private boolean traceOn;
    private boolean targetOutlineOn;
    private final OrientedPoint ASSIGNED_POINT = new OrientedPoint(400, 600, 3*Math.PI/2);
    
    public DiffDriveModelDriver() {
        this.setPreferredSize(new Dimension(1000, 1000));
        this.setBackground(java.awt.Color.WHITE);
        this.setFocusable(true);
        this.requestFocusInWindow();

        robot = new LatticeRobot(1, new OrientedPoint(500, 500, 0));
        trace = new ArrayList<>();
        traceOn = false;

        addKeyListener( new KeyListener() {
            @Override
            public void keyTyped(KeyEvent e) {}

            @Override
            public void keyPressed(KeyEvent e) {

                if(e.getKeyCode() == KeyEvent.VK_T) {
                    traceOn = !traceOn;
                }

                if(e.getKeyCode() == KeyEvent.VK_A) {
                    beginMoveToPointPose();
                }
            }

            @Override
            public void keyReleased(KeyEvent e) {}
        });
    }

    private void beginMoveToPointPose() {
        long startTime = System.nanoTime();
        final long[] lastMovementTime = {System.nanoTime()};
        targetOutlineOn = !targetOutlineOn;

        robot.startMoving();

        Timer start = new Timer(1000 / 30, e -> {

            long currentTime = System.nanoTime();
            double dt = (currentTime - lastMovementTime[0]) / 1_000_000_000.0;

            lastMovementTime[0] = currentTime;
            //Robot get position; add to trace arraylist
            trace.add(robot.getPosition());
            boolean done = robot.moveTo(ASSIGNED_POINT, dt);
            System.out.println(robot);
            System.out.println((currentTime - startTime) / 1_000_000_000 + " seconds");
            repaint();
            if(done) {
                System.exit(0);
            }
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

    private void drawTargetOutline(Graphics2D g2d) {
        Polygon p = new Polygon();
        double s = 50;
        double R = s / Math.sqrt(3.0);
        double theta0 = ASSIGNED_POINT.orientation;
        int[] xcoords = new int[4];
        int[] ycoords = new int[4]; 
        //Calculate first triangle vertices
        double radius = R * 1.20;
        double angle = theta0; // 0 angle for the first vertex 
        xcoords[0] =  (int) Math.round(ASSIGNED_POINT.x + radius * Math.cos(angle));
        ycoords[0] =  (int) Math.round(ASSIGNED_POINT.y + radius * Math.sin(angle)); 
        //Calculate second triangle vertices
        angle = theta0 + 2 * Math.PI / 3.0; // 120 degrees or 2π/3 radians
        xcoords[1] =  (int) Math.round(ASSIGNED_POINT.x + R * Math.cos(angle));
        ycoords[1] =  (int) Math.round(ASSIGNED_POINT.y + R * Math.sin(angle)); 
        //Calculate third vertice (for flag like tail)
        radius = R * 0.1;
        angle = theta0 + Math.PI; /// 180 degrees or π radians 
        xcoords[2] =  (int) Math.round(ASSIGNED_POINT.x + radius * Math.cos(angle));
        ycoords[2] =  (int) Math.round(ASSIGNED_POINT.y + radius * Math.sin(angle)); 
        //Calculate fourth triangle vertices
        angle = theta0 + 4 * Math.PI / 3.0; // 240 degrees or 4π/3 radians
        xcoords[3] =  (int) Math.round(ASSIGNED_POINT.x + R * Math.cos(angle));
        ycoords[3] =  (int) Math.round(ASSIGNED_POINT.y + R * Math.sin(angle));

        for(int i = 0; i < xcoords.length; i++) {
            p.addPoint(xcoords[i], ycoords[i]);
        }
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setColor(Color.GREEN);
        g2d.fill(p);
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2d = (Graphics2D) g;

        if(traceOn) {
            tracePath(g2d);
        }

        if(targetOutlineOn) {
            drawTargetOutline(g2d);
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
