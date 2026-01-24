package org.robots;

import javax.swing.JPanel;

import org.transformations.OrientedPoint;

import java.util.ArrayList;
import java.util.List;
import java.awt.event.KeyEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.awt.Graphics2D;
import java.time.LocalDateTime;

public class RobotPanel extends JPanel {
    
    private List<LatticeRobot> robots;
    private LatticeRobot selectedRobot = null;
    private boolean dragging = false;

    public RobotPanel() {
        robots = new ArrayList<>();
        this.setPreferredSize(new java.awt.Dimension(900, 900));
        this.setBackground(java.awt.Color.WHITE);
        this.setFocusable(true);
        this.requestFocusInWindow();

        //Create 4 sample robots and connect them
        LatticeRobot robot1 = new LatticeRobot(1, new OrientedPoint(100, 100, 0));
        LatticeRobot robot2 = new LatticeRobot(2, new OrientedPoint(250, 150, Math.PI / 4));
        LatticeRobot robot3 = new LatticeRobot(3, new OrientedPoint(150, 250, Math.PI / 2));
        LatticeRobot robot4 = new LatticeRobot(4, new OrientedPoint(300, 200, Math.PI));

        //Add robots to panel
        robots.add(robot1);
        robots.add(robot2);
        robots.add(robot3);
        robots.add(robot4);

        //Add neighbors for robot 1
        robot1.addNeighbor(robot2);
        robot1.addNeighbor(robot3);
        robot1.addNeighbor(robot4);

        //Add neighbors for robot 2
        robot2.addNeighbor(robot4);

        //Add neighbors for robot 3
        robot3.addNeighbor(robot4); //Note: bidirectional connection ensured in addNeighbor

        //Set different trust levels
        robot4.setTrustLevel(TrustLevel.Hostile);

        //Add listner to robots for moving them by mouse drag
        addMouseListener( new MouseAdapter() {
            
            @Override
            public void mousePressed(MouseEvent e) {
                
                for(int i = robots.size() - 1; i >= 0; i--) {
                    LatticeRobot robot = robots.get(i);
                    if(robot.contains(e.getX(), e.getY())) {
                        selectedRobot = robot;
                        dragging = true;
                        robot.offsetX = e.getX() - robot.getPosition().x;
                        robot.offsetY = e.getY() - robot.getPosition().y;
                        
                        robots.remove(i);
                        robots.add(robot); //Bring to front
                        repaint();
                        return;
                    }
                }
                selectedRobot = null;
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                if(selectedRobot != null) {
                    dragging = false;
                    selectedRobot = null;
                    repaint();
                }
            }
        });

        //Allow for dragging robots
        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e) {
                if(selectedRobot != null && dragging) {
                    double nx = e.getX() - selectedRobot.offsetX;
                    double ny = e.getY() - selectedRobot.offsetY;
                    selectedRobot.setPosition(new OrientedPoint(nx, ny, selectedRobot.getPosition().getOrientation()));
                    repaint();
                }
            }
        });

        //Add key listener for exporting panel image
        this.addKeyListener( new KeyListener() {
            @Override
            public void keyTyped(KeyEvent e) {}

            @Override
            public void keyPressed(KeyEvent e) {
                //Export JPEG image on 'S' key press
                if(e.getKeyCode() == KeyEvent.VK_S) {
                    LocalDateTime now = LocalDateTime.now();
                    java.time.format.DateTimeFormatter formatter = java.time.format.DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss");
                    if(savePanelImageAsJPEG("build/robot_panel_snapshot_" + now.format(formatter) + ".png")) {
                        System.out.println("Panel image saved as robot_panel_snapshot_" + now.format(formatter) + ".png");
                    }
                }
            }

            @Override
            public void keyReleased(java.awt.event.KeyEvent e) {}
        });
    }

    protected void paintComponent(java.awt.Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        for(LatticeRobot robot : robots) {
            robot.getEdges().forEach(edge -> edge.draw(g2d));
        }
        for (LatticeRobot robot : robots) {
            robot.draw(g2d);
        }
    }

    public boolean savePanelImageAsJPEG(String filePath) {
        //Create a buffered image
        BufferedImage image = new BufferedImage(this.getWidth(), this.getHeight(), BufferedImage.TYPE_INT_ARGB);
        //Create a graphics context
        Graphics2D g2d = image.createGraphics();
        this.paintAll(g2d);
        g2d.dispose();
        
        try{
            boolean test = javax.imageio.ImageIO.write(image, "png", new java.io.File(filePath));
            return test;
        } catch (java.io.IOException e) {
            System.err.println("Error saving panel image: " + e.getMessage());
            e.printStackTrace();
            return false;
        }   
    }

    public static void main(String[] args) {
        javax.swing.JFrame frame = new javax.swing.JFrame("Lattice Robots Panel");
        RobotPanel panel = new RobotPanel();

        frame.setDefaultCloseOperation(javax.swing.JFrame.EXIT_ON_CLOSE);
        frame.add(panel);
        frame.pack();
        frame.setVisible(true);
    }
}

