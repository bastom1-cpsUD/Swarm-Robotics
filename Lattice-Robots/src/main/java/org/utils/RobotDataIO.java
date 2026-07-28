package org.utils;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import org.communicationModels.TrustLevel;
import org.graphs.util.OrientedPoint;
import org.robots.GeometricCycleLatticeRobot;
import org.simulation.Edge;

import java.io.File;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Utility class for importing and exporting robot simulation state to/from JSON.
 * Extracted from RobotPanel so both the synchronous and asynchronous panels share
 * the same serialisation logic without either owning it.
 */
public final class RobotDataIO {

    private static final String OUTPUT_DIR  = "output/robot_data";
    private static final String JSON_FILE   = "robot_data.json";

    private RobotDataIO() { /* utility class */ }

    // ------------------------------------------------------------------
    // Export
    // ------------------------------------------------------------------

    /**
     * Serialises the provided robot map to {@code output/robot_data/robot_data.json}.
     *
     * @param robots snapshot of the current robot map (caller is responsible for
     *               holding any necessary locks before passing this in)
     * @return {@code true} on success, {@code false} on any IO error
     */
    public static boolean exportToJSON(Map<Integer, GeometricCycleLatticeRobot> robots) {
        File outputDir = new File(OUTPUT_DIR);
        if (!outputDir.exists()) outputDir.mkdirs();

        File jsonFile = new File(outputDir, JSON_FILE);

        try {
            ObjectMapper mapper = new ObjectMapper();
            ObjectNode root = mapper.createObjectNode();

            // 1. Robots
            ArrayNode robotsArray = mapper.createArrayNode();
            for (GeometricCycleLatticeRobot robot : robots.values()) {
                OrientedPoint p = robot.getPosition();
                ObjectNode node = mapper.createObjectNode();
                node.put("id",          robot.getRobotId());
                node.put("x",           p.x);
                node.put("y",           p.y);
                node.put("orientation", p.getOrientation());
                robotsArray.add(node);
            }
            root.set("robots", robotsArray);

            // 2. Edges (deduplicated: fromId < toId)
            ArrayNode edgesArray = mapper.createArrayNode();
            for (GeometricCycleLatticeRobot robot : robots.values()) {
                for (Edge edge : robot.getEdges()) {
                    if (edge.getFromId() < edge.getToId()) {
                        ObjectNode node = mapper.createObjectNode();
                        node.put("fromId", edge.getFromId());
                        node.put("toId",   edge.getToId());
                        edgesArray.add(node);
                    }
                }
            }
            root.set("edges", edgesArray);

            // 3. Trust levels
            ArrayNode trustArray = mapper.createArrayNode();
            for (GeometricCycleLatticeRobot robot : robots.values()) {
                ObjectNode node = mapper.createObjectNode();
                node.put("id",         robot.getRobotId());
                node.put("trustLevel", robot.getTrustLevel().toString());
                trustArray.add(node);
            }
            root.set("trust_levels", trustArray);

            mapper.writerWithDefaultPrettyPrinter().writeValue(jsonFile, root);
            return true;

        } catch (IOException e) {
            System.err.println("[RobotDataIO] Export failed: " + e.getMessage());
            return false;
        }
    }

    // ------------------------------------------------------------------
    // Import
    // ------------------------------------------------------------------

    /**
     * Deserialises {@code output/robot_data/robot_data.json} into a fresh
     * {@link LinkedHashMap} of robots.  Returns an empty map (not null) on any
     * error so callers can always iterate safely.
     *
     * <p>Neighbour lists are intentionally <em>not</em> restored from the edge
     * array — those edges represent a past visualisation state.  The simulation
     * panel should call its proximity check immediately after import to rebuild
     * live neighbour relationships from current positions.
     */
    public static Map<Integer, GeometricCycleLatticeRobot> importFromJSON() {
        Map<Integer, GeometricCycleLatticeRobot> robots = new LinkedHashMap<>();

        File jsonFile = new File(OUTPUT_DIR, JSON_FILE);
        if (!jsonFile.exists()) {
            System.err.println("[RobotDataIO] No JSON file found at: " + jsonFile.getPath());
            return robots;
        }

        try {
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(jsonFile);

            // 1. Robots
            JsonNode robotsNode = root.get("robots");
            if (robotsNode != null && robotsNode.isArray()) {
                for (JsonNode n : robotsNode) {
                    int    id          = n.get("id").asInt();
                    double x           = n.get("x").asDouble();
                    double y           = n.get("y").asDouble();
                    double orientation = n.get("orientation").asDouble();
                    GeometricCycleLatticeRobot robot =
                            new GeometricCycleLatticeRobot(id, new OrientedPoint(x, y, orientation));
                    robots.put(id, robot);
                }
            }

            // 2. Edges — restore visual edges between known robots only
            JsonNode edgesNode = root.get("edges");
            if (edgesNode != null && edgesNode.isArray()) {
                for (JsonNode n : edgesNode) {
                    int fromId = n.get("fromId").asInt();
                    int toId   = n.get("toId").asInt();
                    GeometricCycleLatticeRobot from = robots.get(fromId);
                    GeometricCycleLatticeRobot to   = robots.get(toId);
                    if (from != null && to != null) {
                        from.addNeighbor(to);
                    }
                }
            }

            // 3. Trust levels
            JsonNode trustNode = root.get("trust_levels");
            if (trustNode != null && trustNode.isArray()) {
                for (JsonNode n : trustNode) {
                    int        id    = n.get("id").asInt();
                    TrustLevel trust = TrustLevel.valueOf(n.get("trustLevel").asText());
                    GeometricCycleLatticeRobot robot = robots.get(id);
                    if (robot != null) robot.setTrustLevel(trust);
                }
            }

        } catch (IOException e) {
            System.err.println("[RobotDataIO] Import failed: " + e.getMessage());
            robots.clear();
        }

        return robots;
    }
}