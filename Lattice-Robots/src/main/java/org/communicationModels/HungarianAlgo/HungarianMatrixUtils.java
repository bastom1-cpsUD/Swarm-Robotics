package org.communicationModels.HungarianAlgo;

public final class HungarianMatrixUtils {

    public static final double INFINITY = 1000;
    private HungarianMatrixUtils() {
        //Utility class
    }

    public static double[][] padToSquare(double[][] cost) {
        int rows = cost.length;
        int cols = cost[0].length;
        int size = Math.max(rows, cols);

        double dummyValue;

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
