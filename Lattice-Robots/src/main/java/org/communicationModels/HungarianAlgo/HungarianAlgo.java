 package org.communicationModels.HungarianAlgo;

import java.util.ArrayList;

/**
 * A class recreating the hungarian algorithm, with pseudocode laid out by Duke University
 */
public class HungarianAlgo {
     private final double[][] costMatrix;

    private final int numRows;
    private final int numCols;

    // starsCol[row] = col of starred zero in row
    private final int[] starsCol;

    // starsRow[col] = row of starred zero in col
    private final int[] starsRow;

    // primes[row] = col of primed zero in row
    private final int[] primes;

    private final boolean[] coveredRows;
    private final boolean[] coveredCols;

    /**
     * Public constructor that builds a Hungarian algorithm object
     * @param costMatrix the cost matrix of the assignment problem
     */
    public HungarianAlgo(double[][] costMatrix) {
        if (costMatrix == null || costMatrix.length == 0 || costMatrix[0].length == 0) {
            throw new IllegalArgumentException("Cost matrix cannot be null or empty");
        }

        this.numRows = costMatrix.length;
        this.numCols = costMatrix[0].length;

        if(numRows > numCols) {
            throw new IllegalArgumentException("Cost matrix must be square for the Hungarian Algorithm");
        }

        if(numCols < numRows) {
            throw new IllegalArgumentException("Cost matrix must be square for the Hungarian Algorithm");
        }

        this.primes = new int[numRows];
        this.starsCol = new int[numRows];
        this.starsRow = new int[numCols];

        this.coveredRows = new boolean[numRows];
        this.coveredCols = new boolean[numCols];
    
        this.costMatrix = costMatrix;

        for(int i = 0; i < numRows; i++) {
            starsCol[i] = -1;
            primes[i] = -1;
        }

        for(int j = 0; j < numCols; j++) {
            starsRow[j] = -1;
        }
    }

    /**
     * Solves the assignment problem by calling each step of the Hungarian algorithm
     * @return optimal assignment for task problem
    */
    public int[][] solve() {
        // Step 1: Reduce the rows of the cost matrix
        reduceMatrix();
        // Step 2: find a zero in the resulting matrix and star
        starZeros();
        
        while(true) {
            // Step 3: Cover each column containing a starred zero
            coverColumnsWithStarredZeroes();

            // Step 4: Test for optimality
            if(allColCovered()) {
                break; // Optimal assignment is found
            }
            // Step 5: Find a non-covered zero and prime it
            int[] primedZero = primeZeroes();

            augmentPath(primedZero);
        }
        
        // Construct the assignment result based on the starred zeroes
        int[][] assignment = new int[numRows][2];

        for(int i = 0; i < numRows; i++) {
            assignment[i][0] = i;
            assignment[i][1] = starsCol[i];
        }

        return assignment;
    }

    private void coverColumnsWithStarredZeroes() {
    for (int i = 0; i < numRows; i++) coveredRows[i] = false;
    for (int j = 0; j < numCols; j++) coveredCols[j] = (starsRow[j] != -1);
}
    
    private boolean allColCovered() {
        System.out.println("Checking if all columns are covered...");
        for(int j = 0; j < numCols; j++) {
            if(!coveredCols[j]) {
                return false;
            }
        }
        return true;
    }

    /**
     * Reduces the matrix by subtracting the minimum value from each row
     */
    private void reduceMatrix() {

    System.out.println("Reducing matrix...");

    // Row reduction
    for(int i = 0; i < numRows; i++) {

        double minValue = Double.MAX_VALUE;

        for(int j = 0; j < numCols; j++) {
            minValue = Math.min(minValue, costMatrix[i][j]);
        }

        for(int j = 0; j < numCols; j++) {
            costMatrix[i][j] -= minValue;
        }
    }
}

    /**
     * stars zeros once per row & column; acts as drawing lines to eliminate all zeros
     */
    private void starZeros() {
        System.out.println("Starring zeroes...");
        
        for(int i = 0; i < numRows; i++) {
            for(int j = 0; j < numCols; j++) {
                if(isZero(costMatrix[i][j]) && starsCol[i] == -1 && starsRow[j] == -1) {
                    starsCol[i] = j; // Store the column index of the starred zero for this row
                    starsRow[j] = i; // Store the row index of the starred zero for this column
                }
            }
        }

        // Print out the starred zeroes for debugging
        System.out.println("Starred zeroes:");
        for(int i = 0; i < numRows; i++) {
            if(starsCol[i] != -1) {
                System.out.println("Starred zero at: (" + i + ", " + starsCol[i] + ")");
            }
        }
    }

    /**
     * Primes zeros that are not covered by the starred rows/columns
     * @return position of uncovered prime zero for the construction of the alternating primed and starred zero path
     */
    private int[] primeZeroes() {

    System.out.println("Priming Zeroes...");

    while(true) {

        int[] zeroPos = findNonCoveredZero();

        // No uncovered zero exists -> adjust matrix
        if(zeroPos == null) {
            double minValue = Double.MAX_VALUE;

            for(int i = 0; i < numRows; i++) {
                for(int j = 0; j < numCols; j++) {

                    if(!coveredRows[i] && !coveredCols[j]) {
                        minValue = Math.min(minValue, costMatrix[i][j]);
                    }
                }
            }

            adjustMatrixBySmallest(minValue);
            continue;
        }

        int row = zeroPos[0];
        int col = zeroPos[1];

        // Prime the zero
        primes[row] = col;

        System.out.println("Primed zero at: (" + row + ", " + col + ")");

        int starCol = starsCol[row];
        if (starCol == -1) {
            // No starred zero in this row → go to Step 5
            return new int[]{row, col};
        }

        // Starred zero exists in this row:
        // cover the row, uncover the starred zero's column, keep going
        coveredRows[row] = true;
        coveredCols[starCol] = false;
    }
}

    /**
     * Constructs the alternating path that unstars starred zeros and stars primed zeros
     * @param startPrime the position of the uncovered primed zero 
     */
    private void augmentPath(int[] startPrime) {
        ArrayList<int[]> primedZeros = new ArrayList<>();
        ArrayList<int[]> starredZeros = new ArrayList<>();

        primedZeros.add(startPrime);

        while (true) {
            // Find starred zero in the column of the last primed zero
            int col = primedZeros.get(primedZeros.size() - 1)[1];
            int starredRow = starsRow[col];

            if (starredRow == -1) break; // no starred zero in this col → path ends

            starredZeros.add(new int[]{starredRow, col});

            // Find primed zero in the row of that starred zero
            int primedCol = primes[starredRow];
            primedZeros.add(new int[]{starredRow, primedCol});
        }

        // Unstar all starred zeros in the path
        for (int[] s : starredZeros) {
            starsCol[s[0]] = -1;
            starsRow[s[1]] = -1;
        }

        // Star all primed zeros in the path
        for (int[] p : primedZeros) {
            starsCol[p[0]] = p[1];
            starsRow[p[1]] = p[0];
        }

        // Clear primes and covers
        for (int i = 0; i < numRows; i++) primes[i]      = -1;
        for (int i = 0; i < numRows; i++) coveredRows[i] = false;
        for (int j = 0; j < numCols; j++) coveredCols[j] = false;
    }  
    
    private int[] findNonCoveredZero() {
        for(int i = 0; i < numRows; i++) {
            for(int j = 0; j < numCols; j++) {

                if(!coveredCols[j] && !coveredRows[i] && isZero(costMatrix[i][j])) {
                    return new int[]{i, j};
                }
            }
        }
        return null;
    }

    private boolean isZero(double value) {
        return Math.abs(value) < 1e-9;
    }
   
    private void adjustMatrixBySmallest(double minValue) {
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            if (coveredRows[i] && coveredCols[j]) {
                costMatrix[i][j] += minValue;   // doubly covered → add
            } else if (!coveredRows[i] && !coveredCols[j]) {
                costMatrix[i][j] -= minValue;   // uncovered → subtract
            }
        }
    }
    }

    public static void main(String[] args) {
        double[][] costMatrix = {
            {2500, 4000, 3500},
            {4000, 6000, 3500},
            {2000, 4000, 2500}
        };

        HungarianAlgo hungarian = new HungarianAlgo(costMatrix);
        int[][] assignment = hungarian.solve();

        System.out.println("Optimal Assignment:");
        for(int i = 0; i < assignment.length; i++) {
            System.out.println("Robot " + (assignment[i][0] + 1) + " assigned to Edge " + (assignment[i][1] + 1));
        }
    }
}