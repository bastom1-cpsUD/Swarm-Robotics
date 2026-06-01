package org.communicationModels.HungarianAlgo;

/**
 * Utility package for matrix padding and the definition of infinity in the context of the Hungarian assignment problem
 */
public final class HungarianMatrixUtils {

    /**
     * the defined infinity amount, larger than any possible euclidean distance for the assignment problem
     */
    public static final double INFINITY = 1000;
   
    private HungarianMatrixUtils() {
        //Utility class
    }

    /**
     * A method for paddign a matrix into a square matrix
     * @param cost an unsquare cost matrix
     * @return a square version of the cost matrix with fake rows or columns and dummy values for proper algorithm execution
     */
    public static double[][] padToSquare(double[][] cost) {
        int rows = cost.length;
        int cols = cost[0].length;
        int size = Math.max(rows, cols);

        double dummyValue;

        // Decide if it is feasible to assign a robot or edge to null
        if(rows > cols) {
            dummyValue = INFINITY;
        } else {
            dummyValue = 0.0;
        }

        double[][] padded = new double[size][size];

        for(int row = 0; row < size; row++) {
            for(int col = 0; col < size; col++) {
                if(row < rows && col < cols) {
                    padded[row][col] = cost[row][col];
                } else {
                    padded[row][col] = dummyValue;
                }
            }
        }

        return padded;
    }
}
