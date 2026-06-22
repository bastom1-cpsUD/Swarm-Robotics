package org.simulation;

import javax.swing.JPanel;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.LinkedHashMap;
import java.util.Map;
import javax.swing.Timer;
import java.awt.event.KeyEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.geom.Ellipse2D;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.time.LocalDateTime;

import org.robots.GeometricCycleLatticeRobot;
import org.communicationModels.TrustLevel;
import org.graphs.OrientedPoint;

public class RobotPanel extends JPanel {
    private static Map<Integer, GeometricCycleLatticeRobot> robots;
    private static GeometricCycleLatticeRobot selectedRobot = null;
    private static boolean dragging = false;
    private static boolean displayRobotProximity = false;
    private static double offsetX;
    private static double offsetY;

    public RobotPanel() {
        robots = new LinkedHashMap<Integer, GeometricCycleLatticeRobot>();
        this.setPreferredSize(new java.awt.Dimension(900, 900));
        this.setBackground(java.awt.Color.WHITE);
        this.setFocusable(true);
        this.requestFocusInWindow();

        //Add listener to robots for moving them by mouse drag
        addMouseListener( new MouseAdapter() {           
            @Override
            public void mousePressed(MouseEvent e) {

                GeometricCycleLatticeRobot hitRobot = null;

                //Check if a robot was clicked
                for (GeometricCycleLatticeRobot robot : robots.values()) {
                    if (robot.contains(e.getX(), e.getY())) {
                        hitRobot = robot;
                        break;
                    }
                }
                //If a robot was clicked, prepare for dragging
                if (hitRobot != null) {
                    dragging = true;

                    //FOR TESTING ONLY
                    hitRobot.dataDump();
                    hitRobot.promoteToPrimaryRoot();

                    offsetX = e.getX() - hitRobot.getPosition().x;
                    offsetY = e.getY() - hitRobot.getPosition().y;

                    // Bring to front by re-inserting into the map
                    robots.remove(hitRobot.getRobotId());
                    robots.put(hitRobot.getRobotId(), hitRobot);

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

        //Check proximity of robots to update edges when they are moved by mouse drag
        Timer proximityTimer = new Timer(100, e -> {
            if(!dragging) {
                return; // Skip proximity check while dragging to improve performance
            }
            proximityCheckForAllRobots();
            repaint();
        });
        proximityTimer.start();

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

        //KEY LISTENERS: for exporting panel image and robot data, and importing robot data
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
                        proximityCheckForAllRobots();
                        System.out.println("Robot data imported from output/robot_data!");
                        repaint();
                    }
                }

                if(e.getKeyCode() == KeyEvent.VK_D) {
                    displayRobotProximity = !displayRobotProximity;
                    repaint();
                }

                if(e.getKeyCode() == KeyEvent.VK_SPACE) {
                    beginSimulation();
                }
            }

            @Override
            public void keyReleased(java.awt.event.KeyEvent e) {}
        });
    }

    private void beginSimulation() {
    final double[] timeSinceStart = {0.0};
    final long[] lastFrameTime = {System.nanoTime()};
    final long[] lastStateTime = {System.nanoTime()};
    final boolean[] firstStateUpdated = {false};
    Timer simLoop = new Timer(1000 / 30, e -> {
        long current = System.nanoTime();
        double dt = (current - lastFrameTime[0]) / 1_000_000_000.0;
        timeSinceStart[0] += dt;
        lastFrameTime[0] = current;

        // State update — only when a full second has elapsed
        if (current - lastStateTime[0] >= 1_000_000_000L) {
            double timeStep = (current-lastStateTime[0]) / 1_000_000_000.0;
            System.out.println("Time elapsed: " + timeSinceStart[0]);
            firstStateUpdated[0] = true;
            lastStateTime[0] = current;

            proximityCheckForAllRobots();

            for (GeometricCycleLatticeRobot robot : robots.values()) {
                proximityCheckForAllRobots();
                robot.executeTimeStep(timeStep);
            }

             proximityCheckForAllRobots();
        }

        // Movement — always runs after the state block above
        if(firstStateUpdated[0]) {
            for (GeometricCycleLatticeRobot robot : robots.values()) {
                robot.move(dt);
            }
        }   
        proximityCheckForAllRobots();
        repaint();
    });

    simLoop.start();
}

    protected void paintComponent(java.awt.Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        if(displayRobotProximity) {
            drawRobotProximity(g2d);
        }
        for(GeometricCycleLatticeRobot robot : robots.values()) {
            //Draw edges
            robot.getEdges().forEach(edge -> {
                //Retrieve the 'to' robot
                GeometricCycleLatticeRobot to = robots.get(edge.getToId());
                GeometricCycleLatticeRobot from = robots.get(edge.getFromId());
                //Draw the edge
                edge.draw(g2d, from, to);
            });
        }
        
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setColor(Color.BLACK);
        for(GeometricCycleLatticeRobot robot : robots.values()) {
            g2d.fill(robot.draw());
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
            for(GeometricCycleLatticeRobot robot : robots.values()) {
                ObjectNode robotNode = mapper.createObjectNode();
                OrientedPoint robotPose = robot.getPosition();

                //Add data to node for single robot
                robotNode.put("id", robot.getRobotId());
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
            for(GeometricCycleLatticeRobot robot : robots.values()) {
                
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
            for(GeometricCycleLatticeRobot robot : robots.values()) {
                ObjectNode trustNode = mapper.createObjectNode();

                //Add data to node
                trustNode.put("id", robot.getRobotId());
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
                    GeometricCycleLatticeRobot importedRobot = new GeometricCycleLatticeRobot(robotId, robotPosition);
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
                    GeometricCycleLatticeRobot fromRobot = robots.get(fromId);
                    GeometricCycleLatticeRobot toRobot = robots.get(toId);

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
                    GeometricCycleLatticeRobot robot = robots.get(robotId);
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
    
    public static void proximityCheckForAllRobots() {
        for(GeometricCycleLatticeRobot robot : robots.values()) {
            robot.clearNeighbors();
            for(GeometricCycleLatticeRobot other : robots.values()) {
                if(robot.getRobotId() != other.getRobotId()) {
                    double distance = robot.getPosition().distance(other.getPosition());
                    if(distance <= GeometricCycleLatticeRobot.COMM_RANGE) {
                        robot.addNeighbor(other);
                    } else {
                        robot.removeNeighbor(other);
                    }
                }
             }
        }
    }
    
    public static void drawRobotProximity(Graphics2D g) {
        if(!displayRobotProximity) {
            return;
        }
        Graphics2D g2d = (Graphics2D) g;
        for(GeometricCycleLatticeRobot robot: robots.values()) {
            double x = robot.getPosition().x;
            double y = robot.getPosition().y;

            double proximityThreshold = GeometricCycleLatticeRobot.COMM_RANGE;

            Ellipse2D.Double proximityCircle = new Ellipse2D.Double(x - proximityThreshold, y - proximityThreshold, proximityThreshold * 2, proximityThreshold * 2);

            g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2d.draw(proximityCircle);
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


