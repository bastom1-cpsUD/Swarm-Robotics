package org.robots;

import javax.swing.JPanel;

import org.transformations.OrientedPoint;

import java.awt.event.MouseMotionAdapter;
import java.util.ArrayList;
import java.util.List;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

public class RobotPanel extends JPanel {
    
    private List<LatticeRobot> robots;
    private LatticeRobot selectedRobot = null;
    private boolean dragging = false;

    public RobotPanel() {
        robots = new ArrayList<>();
        this.setPreferredSize(new java.awt.Dimension(900, 900));
        this.setBackground(java.awt.Color.WHITE);

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
    }

    protected void paintComponent(java.awt.Graphics g) {
        super.paintComponent(g);
        java.awt.Graphics2D g2d = (java.awt.Graphics2D) g;
        for(LatticeRobot robot : robots) {
            robot.getEdges().forEach(edge -> edge.draw(g2d));
        }
        for (LatticeRobot robot : robots) {
            robot.draw(g2d);
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

