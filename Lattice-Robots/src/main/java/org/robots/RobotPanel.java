package org.robots;

import javax.swing.JPanel;

import org.transformations.OrientedPoint;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Scanner;
import java.awt.event.KeyEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.awt.Graphics2D;
import java.time.LocalDateTime;

public class RobotPanel extends JPanel {
    
    private static Map<Integer, LatticeRobot> robots;
    private LatticeRobot selectedRobot = null;
    private boolean dragging = false;

    public RobotPanel() {
        robots = new LinkedHashMap<Integer, LatticeRobot>();
        this.setPreferredSize(new java.awt.Dimension(900, 900));
        this.setBackground(java.awt.Color.WHITE);
        this.setFocusable(true);
        this.requestFocusInWindow();

        //Add listner to robots for moving them by mouse drag
        addMouseListener( new MouseAdapter() {
            
            @Override
            public void mousePressed(MouseEvent e) {

                LatticeRobot hitRobot = null;

                //Check if a robot was clicked
                for (LatticeRobot robot : robots.values()) {
                    if (robot.contains(e.getX(), e.getY())) {
                        hitRobot = robot;
                        break;
                    }
                }
                //If a robot was clicked, prepare for dragging
                if (hitRobot != null) {
                    dragging = true;

                    hitRobot.offsetX = e.getX() - hitRobot.getPosition().x;
                    hitRobot.offsetY = e.getY() - hitRobot.getPosition().y;

                    // Bring to front by re-inserting into the map
                    robots.remove(hitRobot.getAuthorityId());
                    robots.put(hitRobot.getAuthorityId(), hitRobot);

                    selectedRobot = hitRobot;
                    repaint();

                } else {
                    selectedRobot = null;
                }
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
                    if(savePanelImageAsJPEG()) {
                        System.out.println("Panel image saved to build/robot_panel_images!");
                    }
                }

                //Export robot data on 'E' key press
                if(e.getKeyCode() == KeyEvent.VK_E) {
                    if(exportDataToCSV()) {
                        System.out.println("Robot data exported to build/robot_data!");
                    }
                }

                //Import robot data on 'I' key press
                if(e.getKeyCode() == KeyEvent.VK_I) {
                    if(readDataFromCSV()) {
                        System.out.println("Robot data imported from build/robot_data!");
                        repaint();
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
        for(LatticeRobot robot : robots.values()) {
            //Draw edges
            robot.getEdges().forEach(edge -> {
                //Retrieve the 'to' robot
                LatticeRobot to = robots.get(edge.getToId());
                //Draw the edge
                edge.draw(g2d, robot, to);
            });
        }
        for(LatticeRobot robot : robots.values()) {
            robot.draw(g2d);
        }
    }

    public boolean savePanelImageAsJPEG() {
        //Create a buffered image
        BufferedImage image = new BufferedImage(this.getWidth(), this.getHeight(), BufferedImage.TYPE_INT_ARGB);
        //Create a graphics context
        Graphics2D g2d = image.createGraphics();
        this.paintAll(g2d);
        g2d.dispose();
        
        try{
            File outputDir = new File("Output/robot_panel_images");
            if(!outputDir.exists()) {
                outputDir.mkdirs();
            }
            LocalDateTime now = LocalDateTime.now();
            java.time.format.DateTimeFormatter formatter = java.time.format.DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss");
            
            String filePath = "Output/robot_panel_images/robot_panel_snapshot_" + now.format(formatter) + ".png";

            boolean test = javax.imageio.ImageIO.write(image, "png", new java.io.File(filePath));
            return test;
        } catch (java.io.IOException e) {
            System.err.println("Error saving panel image: " + e.getMessage());
            e.printStackTrace();
            return false;
        }   
    }

    public static boolean exportDataToCSV() {
        //Create pose_info.txt
        File outputDir = new File("Output/robot_data");
        if(!outputDir.exists()) {
            outputDir.mkdirs();
        }
        
        File poseFile = new File(outputDir, "pose_info.txt");

        //Write robot positions to pose_info.txt
        try {
             FileWriter poseWriter = new FileWriter(poseFile);
            for(LatticeRobot robot : robots.values()) {
                OrientedPoint pos = robot.getPosition();
                String line = String.format("%d,%.2f,%.2f,%.2f\n", robot.getAuthorityId(), pos.x, pos.y, pos.getOrientation());
                poseWriter.write(line);
            }
            poseWriter.close();
        } catch (IOException e) {
            System.err.println("Error writing pose_info.txt: " + e.getMessage());
            e.printStackTrace();
        }

        //Create comm_info.txt
        File commFile = new File(outputDir, "comm_info.txt");

        //Write communication edges to comm_info.txt
        try {
            FileWriter commWriter = new FileWriter(commFile);
            for(LatticeRobot robot : robots.values()) {
                for(Edge edge : robot.getEdges()) {
                    if(edge.getFromId() < edge.getToId()) { //Avoid duplicate edges
                        String line = String.format("%d,%d\n", edge.getFromId(), edge.getToId());
                        commWriter.write(line);
                    }
                }
            }
            commWriter.close();
        } catch (IOException e) {
            System.err.println("Error writing comm_info.txt: " + e.getMessage());
            e.printStackTrace();
        }

        //Create trust_info.txt
        File trustFile = new File(outputDir, "trust_info.txt");

        //Write robot trust levels to trust_info.txt
        try {
            FileWriter trustWriter = new FileWriter(trustFile);
            for(LatticeRobot robot : robots.values()) {
                String line = String.format("%d,%s\n", robot.getAuthorityId(), robot.getTrustLevel().toString());
                trustWriter.write(line);
            }
            trustWriter.close();
        } catch (IOException e) {
            System.err.println("Error writing trust_info.txt: " + e.getMessage());
            e.printStackTrace();
        }
        return true;
    }

    public static boolean readDataFromCSV() {
        //Create robots from pose_info.txt
        File inputDir = new File("Output/robot_data");
        if(!inputDir.exists()) {
            System.out.println("Data does not exist. Cannot read in robot data.");
        }
        File pose_file = new File(inputDir, "pose_info.txt");
        
        //Read robot positions from pose_info.txt and create robots
        try(Scanner poseScanner = new Scanner(pose_file)) {
            poseScanner.useDelimiter(",");
            while(poseScanner.hasNextLine()) {
                String[] robotInfo = poseScanner.nextLine().split(",");
                int robotId = Integer.valueOf(robotInfo[0]);
                OrientedPoint robotPosition = new OrientedPoint(Double.valueOf(robotInfo[1]), Double.valueOf(robotInfo[2]), Double.valueOf(robotInfo[3]));
            
                LatticeRobot importedRobot = new LatticeRobot(robotId, robotPosition);
                
                robots.put(robotId, importedRobot);
            }
        } catch (FileNotFoundException e) {
            System.err.println("Error reading pose_info.txt: " + e.getMessage());
            e.printStackTrace();
        }

        //Add edges from comm_info.txt
        File comm_file = new File(inputDir, "comm_info.txt");
        
        //Read communication edges from comm_info.txt and add to robots
        try(Scanner commmScanner = new Scanner(comm_file)) {
            commmScanner.useDelimiter(",");
            while(commmScanner.hasNextLine()) {
                String[] EdgeInfo = commmScanner.nextLine().split(",");
                
                int fromId = Integer.valueOf(EdgeInfo[0]);
                int toId = Integer.valueOf(EdgeInfo[1]);

                LatticeRobot fromRobot = robots.get(fromId);
                LatticeRobot toRobot = robots.get(toId);
                if (fromRobot != null && toRobot != null) {
                    fromRobot.addNeighbor(toRobot);
                }
            }
        } catch (FileNotFoundException e) {
            System.err.println("Error reading comm_info.txt: " + e.getMessage());
            e.printStackTrace();
        }
    
        //Get trust levels from trust_info.txt
        File trust_file = new File(inputDir, "trust_info.txt");

        //Read robot trust levels from trust_info.txt and set them
        try(Scanner trustScanner = new Scanner(trust_file)) {
            trustScanner.useDelimiter(",");
            while(trustScanner.hasNextLine()) {
                String[] TrustInfo = trustScanner.nextLine().split(",");
                
                int robotId = Integer.valueOf(TrustInfo[0]);
            TrustLevel trustLevel = TrustLevel.valueOf(TrustInfo[1]);

            LatticeRobot robot = robots.get(robotId);
            if (robot != null) {
                robot.setTrustLevel(trustLevel);
            }
        }
        } catch (FileNotFoundException e) {
            System.err.println("Error reading trust_info.txt: " + e.getMessage());
            e.printStackTrace();
        }
        
        return true;
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

