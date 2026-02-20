package org.robots;

import javax.swing.JPanel;

import org.transformations.OrientedPoint;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.util.LinkedHashMap;
import java.util.Map;
import java.awt.event.KeyEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.awt.Graphics2D;
import java.time.LocalDateTime;

public class RobotPanel extends JPanel {
    
    private static Map<Integer, LatticeRobot> robots;
    private LatticeRobot selectedRobot = null;
    private boolean dragging = false;
    private double offsetX;
    private double offsetY;

    public RobotPanel() {
        robots = new LinkedHashMap<Integer, LatticeRobot>();
        this.setPreferredSize(new java.awt.Dimension(900, 900));
        this.setBackground(java.awt.Color.WHITE);
        this.setFocusable(true);
        this.requestFocusInWindow();

        //Add listener to robots for moving them by mouse drag
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

                    offsetX = e.getX() - hitRobot.getPosition().x;
                    offsetY = e.getY() - hitRobot.getPosition().y;

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
                    double nx = e.getX() - offsetX;
                    double ny = e.getY() - offsetY;
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
                //Export PNG image on 'S' key press
                if(e.getKeyCode() == KeyEvent.VK_S) {
                    if(savePanelImageAsPNG()) {
                        System.out.println("Panel image saved to output/robot_panel_images!");
                    }
                }

                //Export robot data on 'J' key press
                if(e.getKeyCode() == KeyEvent.VK_J) {
                    if(exportDatatoJSON()) {
                        System.out.println("Robot data exported to output/robot_data!");
                    }
                }

                //Import robot data on 'K' key press
                if(e.getKeyCode() == KeyEvent.VK_K) {
                    if(readDataFromJSON()) {
                        System.out.println("Robot data imported from output/robot_data!");
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

    public boolean savePanelImageAsPNG() {
        //Create a buffered image
        BufferedImage image = new BufferedImage(this.getWidth(), this.getHeight(), BufferedImage.TYPE_INT_ARGB);
        //Create a graphics context
        Graphics2D g2d = image.createGraphics();
        this.paintAll(g2d);
        g2d.dispose();
        
        try{
            File outputDir = new File("output/robot_panel_images");
            if(!outputDir.exists()) {
                outputDir.mkdirs();
            }
            LocalDateTime now = LocalDateTime.now();
            java.time.format.DateTimeFormatter formatter = java.time.format.DateTimeFormatter.ofPattern("yyyy_MM_dd_HH_mm_ss");
            
            String filePath = "output/robot_panel_images/robot_panel_snapshot_" + now.format(formatter) + ".png";

            boolean test = javax.imageio.ImageIO.write(image, "png", new java.io.File(filePath));
            return test;
        } catch (java.io.IOException e) {
            System.err.println("Error saving panel image: " + e.getMessage());
            e.printStackTrace();
            return false;
        }   
    }

    public static boolean exportDatatoJSON() {

        //Create output directory
        File outputDir = new File("output/robot_data");
        if(!outputDir.exists()) {
            outputDir.mkdirs();
        }

        File jsonFile = new File(outputDir, "robot_data.json");

        try {
            //Create Jackson ObjectMapper
            ObjectMapper mapper = new ObjectMapper();

            //Create root JSON object (create file base)
            ObjectNode rootNode = mapper.createObjectNode();

            /*****************************
                1. Write robots into file
            ******************************/

            //Create array to store robots in
            ArrayNode robotsArray = mapper.createArrayNode();
            
            //Create a node for each robot's data and add to array
            for(LatticeRobot robot : robots.values()) {
                ObjectNode robotNode = mapper.createObjectNode();
                OrientedPoint robotPose = robot.getPosition();

                //Add data to node for single robot
                robotNode.put("id", robot.getAuthorityId());
                robotNode.put("x", robotPose.x);
                robotNode.put("y", robotPose.y);
                robotNode.put("orientation", robotPose.getOrientation());

                //Add robot to robotsArray
                robotsArray.add(robotNode);
            }

            //Add list of robots to JSON file
            rootNode.set("robots", robotsArray);


            /*****************************
                2. Write edge links
            ******************************/
          
            //Create array to stores edges in
            ArrayNode edgesArray = mapper.createArrayNode();
            
            //Create a node for each edge to store data
            for(LatticeRobot robot : robots.values()) {
                
                //Create a node for each edge's data
                for(Edge edge : robot.getEdges()) {
                    //Avoid duplicate edges by only writing when fromID < toID
                    if(edge.getFromId() < edge.getToId()) {

                        //Add data to node
                        ObjectNode edgeNode = mapper.createObjectNode();
                        edgeNode.put("fromId", edge.getFromId());
                        edgeNode.put("toId", edge.getToId());

                        //Add edge node to edge array
                        edgesArray.add(edgeNode);
                    }
                }
            }

            //Add list of edges to JSON file
            rootNode.set("edges", edgesArray);

            /*****************************
                3. Write trust levels
            ******************************/

            //Create array to store trust levels in
            ArrayNode trustArray = mapper.createArrayNode();

            //Create a node for each robot's trust level
            for(LatticeRobot robot : robots.values()) {
                ObjectNode trustNode = mapper.createObjectNode();

                //Add data to node
                trustNode.put("id", robot.getAuthorityId());
                trustNode.put("trustLevel", robot.getTrustLevel().toString());

                //Add trust node to trust array
                trustArray.add(trustNode);
            }

            //Add list of trust levels to JSON file
            rootNode.set("trust_levels", trustArray);

            //Write JSON data to file
            mapper.writerWithDefaultPrettyPrinter().writeValue(jsonFile, rootNode);
        
            return true;

        } catch(IOException e) {
            System.err.println("Error writing JSON file: " + e.getMessage());
            e.printStackTrace();
            return false;
        }
    }

    public static boolean readDataFromJSON() {
        //clear existing robots
        robots.clear();

        //Create file path for input file
        File inputDir = new File("output/robot_data");
        if(!inputDir.exists()) { 
            System.out.println("Data does not exist. Cannot read in robot data");
            return false;
        }

        //Create json file
        File jsonFile = new File(inputDir, "robot_data.json");
        if(!jsonFile.exists()) {
            System.err.println("Json file does not exist: " + jsonFile.getPath());
            return false;
        }

        try {
            //create Jackson ObjectMapper
            ObjectMapper mapper = new ObjectMapper();

            //Read JSON file
            JsonNode rootNode = mapper.readTree(jsonFile);

            /*****************************
                1. Read in robot data
            ******************************/

            //Access Robot array for robot data
            JsonNode robotsNode = rootNode.get("robots");

            //If array contains data, read and create robots
            if(robotsNode != null && robotsNode.isArray()) {

                //Create individual robots from objectNode informations
                for(JsonNode robotNode : robotsNode) {

                    //Retrieve data
                    int robotId = robotNode.get("id").asInt();
                    double x = robotNode.get("x").asDouble();
                    double y = robotNode.get("y").asDouble();
                    double orientation = robotNode.get("orientation").asDouble();

                    //Create robot and add to panel map
                    OrientedPoint robotPosition = new OrientedPoint(x, y, orientation);
                    LatticeRobot importedRobot = new LatticeRobot(robotId, robotPosition);
                    robots.put(robotId, importedRobot);
                }
            }

            /*****************************
                2. Read in edge data
            ******************************/

            //Access Edge array for edge data
            JsonNode edgesNode = rootNode.get("edges");

            //If array contains data, read and create edge
            if(edgesNode != null && edgesNode.isArray()){
                for(JsonNode edgeNode : edgesNode) {
                    //Retrieve data
                    int fromId = edgeNode.get("fromId").asInt();
                    int toId = edgeNode.get("toId").asInt();

                    //Create edge between robots
                    LatticeRobot fromRobot = robots.get(fromId);
                    LatticeRobot toRobot = robots.get(toId);

                    if(fromRobot != null && toRobot != null) {
                        fromRobot.addNeighbor(toRobot);
                    }
                }
            }

            /*****************************
                3. Read in trust data
            ******************************/

            //Access trust array for trust data
            JsonNode trustNode = rootNode.get("trust_levels");
            
            //If array contains data, read and assign trust levels
            if(trustNode != null && trustNode.isArray()){
                for(JsonNode trustInfo : trustNode) {
                    //retrieve data
                    int robotId = trustInfo.get("id").asInt();
                    String trustLevelStr = trustInfo.get("trustLevel").asText();
                    TrustLevel trust = TrustLevel.valueOf(trustLevelStr);

                    //Assign trust level
                    LatticeRobot robot = robots.get(robotId);
                    if(robot != null) {
                        robot.setTrustLevel(trust);
                    }
                }
            }

            return true;

        } catch(IOException e) {
            System.err.println("Error reading JSON file: " + e.getMessage());
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

