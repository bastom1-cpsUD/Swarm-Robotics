package org.transformations;

import javax.swing.JFrame;
import javax.swing.JPanel;

/**
 * Class representing a 2D transformation including translation and rotation.
 */
public class Transformation2D {

    /** The x-coordinate of the translation. */
    private double x;
    /** The y-coordinate of the translation. */
    private double y;
    /** The rotation angle in radians. */
    private double theta;
    /** The transformation matrix. */
    private double[][] matrix;

    /**Constructor to initialize the transformation.
     * @param x The x-coordinate of the translation.
     * @param y The y-coordinate of the translation.
     * @param theta The rotation angle in radians.
     */
    public Transformation2D(double x, double y, double theta) {
        
        this.x = x;    
        this.y = y;
        this.theta = theta;

        matrix = new double[3][3];

        double sin = Math.sin(theta);
        double cos = Math.cos(theta);

        matrix[0][0] = cos;
        matrix[0][1] = -sin;
        matrix[0][2] = x;

        matrix[1][0] = sin;
        matrix[1][1] = cos;
        matrix[1][2] = y;

        matrix[2][0] = 0;
        matrix[2][1] = 0;
        matrix[2][2] = 1;

    }

    /**
     * Applies the transformation to an OrientedPoint.
     * @param point The point to transform.
     * @return The transformed point.
     */
    public OrientedPoint applyTransformation(OrientedPoint point) {
        double x = point.x;
        double y = point.y;
        double orientation = point.getOrientation();

        if(this.x == 0 && this.y == 0) {
            return new OrientedPoint(x, y, orientation + theta);
        }

        double[][] homogenousCoordinates = {{x}, {y}, {1}};
        // Matrix multiplication
        for(int row = 0; row < matrix.length; row++) {
            homogenousCoordinates[row][0] = (matrix[row][0] * x + matrix[row][1] * y + matrix[row][2] * 1);
        }
        return new OrientedPoint(homogenousCoordinates[0][0], homogenousCoordinates[1][0], orientation + theta);
    }

    @Override
    public String toString() {
        return "2DTransformation[x: " + x + ", y: " + y + ", theta: " + theta + "]\n"
            + String.format("%.2f, %.2f, %d", matrix[0][0], matrix[0][1], (int)matrix[0][2]) + "\n"
            + String.format("%.2f, %.2f, %d", matrix[1][0], matrix[1][1], (int)matrix[1][2]) + "\n"
            + String.format("%d, %d, %d", (int)matrix[2][0], (int)matrix[2][1], (int)matrix[2][2]);
    }


    public static void main(String[] args) {
        // Create a simple GUI to visualize the transformation
        JFrame frame = new JFrame("2D Transformation Visualization");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(600, 600);
        frame.setLayout(new java.awt.BorderLayout());

        //Create the robot panel
        RobotPanel panel = new RobotPanel();
        panel.setBackground(java.awt.Color.WHITE);
        frame.add(panel, java.awt.BorderLayout.CENTER);

        // Create control buttons
        JPanel buttonPanel = new JPanel();

        // Translate up, down, left, right by 2 unit and rotate by 90 degrees
        java.awt.Button upOne = new java.awt.Button("Translate (2,0)");
        upOne.addActionListener(e -> {
            for(Robot r : panel.getRobots()) {
                Transformation2D up1 = new Transformation2D(2, 0, 0);
                OrientedPoint transformedPosition = up1.applyTransformation(r.getPosition());
                r.setPosition(transformedPosition);
            }
            panel.repaint();
        });
        buttonPanel.add(upOne); 

        java.awt.Button rightOne = new java.awt.Button("Translate (0,-2)");
        rightOne.addActionListener(e -> {
            for(Robot r : panel.getRobots()) {
                Transformation2D right1 = new Transformation2D(0, 2, 0);
                OrientedPoint transformedPosition = right1.applyTransformation(r.getPosition());
                r.setPosition(transformedPosition);
            }
            panel.repaint();
        });
        buttonPanel.add(rightOne);

        java.awt.Button downOne = new java.awt.Button("Translate (0,2)");
        downOne.addActionListener(e -> {
            for(Robot r : panel.getRobots()) {
                Transformation2D down1 = new Transformation2D(0, -2, 0);
                OrientedPoint transformedPosition = down1.applyTransformation(r.getPosition());
                r.setPosition(transformedPosition);
            }
            panel.repaint();
        });
        buttonPanel.add(downOne);

        java.awt.Button leftOne = new java.awt.Button("Translate (-2,0)");
        leftOne.addActionListener(e -> {
            for(Robot r : panel.getRobots()) {
                Transformation2D left1 = new Transformation2D(-2, 0, 0);
                OrientedPoint transformedPosition = left1.applyTransformation(r.getPosition());
                r.setPosition(transformedPosition);
            }
            panel.repaint();
        });
        buttonPanel.add(leftOne);

        java.awt.Button rotate90Button = new java.awt.Button("Rotate 90°");
        rotate90Button.addActionListener(e -> {
            for(Robot r : panel.getRobots()) {
                Transformation2D rotate90 = new Transformation2D(0, 0, Math.PI / 2);
                OrientedPoint transformedPosition = rotate90.applyTransformation(r.getPosition());
                r.setPosition(transformedPosition);
            }
            panel.repaint();
        });
        buttonPanel.add(rotate90Button);


        frame.add(buttonPanel, java.awt.BorderLayout.SOUTH);
        frame.setVisible(true);

        // Add a robot at the center
        Robot robot = new Robot(panel.getWidth() / 2.0, panel.getHeight() / 2.0, 0);
        panel.addRobot(robot);
        panel.repaint();
    }
}
