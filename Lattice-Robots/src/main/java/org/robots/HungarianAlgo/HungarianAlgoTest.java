package org.robots.HungarianAlgo;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Comprehensive JUnit 5 test suite for HungarianAlgo.
 *
 * All expected optimal assignments were independently verified by
 * brute-force enumeration of every permutation.
 *
 * Test strategy:
 *  - Input validation  : null, empty, and non-square matrices
 *  - Correctness (x5)  : 1x1, 2x2, 3x3, 4x4, and 5x5 problems with
 *                        known, unique optimal assignments
 *  - Edge cases        : uniform cost matrix, zero-diagonal matrix
 */
class HungarianAlgoTest {

    // -----------------------------------------------------------------------
    //  Helper: build a row→col mapping from the raw int[][] returned by solve()
    // -----------------------------------------------------------------------
    private int[] toAssignmentMap(int[][] result) {
        int[] map = new int[result.length];
        for (int[] pair : result) {
            map[pair[0]] = pair[1];
        }
        return map;
    }

    /** Compute the total cost of an assignment against the original cost matrix. */
    private double assignmentCost(double[][] original, int[] assignmentMap) {
        double cost = 0;
        for (int i = 0; i < assignmentMap.length; i++) {
            cost += original[i][assignmentMap[i]];
        }
        return cost;
    }

    // -----------------------------------------------------------------------
    //  Input-validation tests
    // -----------------------------------------------------------------------

    @Test
    @DisplayName("Null cost matrix throws IllegalArgumentException")
    void testNullMatrix() {
        assertThrows(IllegalArgumentException.class,
                () -> new HungarianAlgo(null),
                "Constructor should reject a null cost matrix");
    }

    @Test
    @DisplayName("Empty cost matrix (zero rows) throws IllegalArgumentException")
    void testEmptyMatrix() {
        assertThrows(IllegalArgumentException.class,
                () -> new HungarianAlgo(new double[0][]),
                "Constructor should reject a zero-row cost matrix");
    }

    @Test
    @DisplayName("Matrix with more rows than columns throws IllegalArgumentException")
    void testMoreRowsThanColumns_3x2() {
        double[][] matrix = {
            {1, 2},
            {3, 4},
            {5, 6}
        };
        assertThrows(IllegalArgumentException.class,
                () -> new HungarianAlgo(matrix),
                "Constructor should reject a cost matrix that has more rows than columns");
    }

    @Test
    @DisplayName("Matrix with more rows than columns (4x1) throws IllegalArgumentException")
    void testMoreRowsThanColumns_4x1() {
        double[][] matrix = {{1}, {2}, {3}, {4}};
        assertThrows(IllegalArgumentException.class,
                () -> new HungarianAlgo(matrix),
                "Constructor should reject a 4x1 cost matrix");
    }

    // -----------------------------------------------------------------------
    //  Correctness test 1 — 1x1 trivial case
    //
    //  Cost matrix:
    //    [ 42 ]
    //
    //  Only possible assignment: Robot 0 → Task 0, cost = 42
    // -----------------------------------------------------------------------
    @Test
    @DisplayName("Correctness 1 — 1x1 matrix: only assignment is (0→0)")
    void testAssignment_1x1() {
        double[][] cost = {{42}};
        double[][] original = {{42}};

        int[] assignment = toAssignmentMap(new HungarianAlgo(cost).solve());

        assertEquals(1, assignment.length, "Should produce exactly one assignment");
        assertEquals(0, assignment[0], "Row 0 must be assigned to column 0");
        assertEquals(42.0, assignmentCost(original, assignment), 1e-9);
    }

    // -----------------------------------------------------------------------
    //  Correctness test 2 — 2x2 simple
    //
    //  Cost matrix:
    //    [ 1  2 ]
    //    [ 4  3 ]
    //
    //  Optimal: row 0 → col 0 (cost 1), row 1 → col 1 (cost 3), total = 4
    //  Alternative: row 0 → col 1 (2) + row 1 → col 0 (4) = 6  ← suboptimal
    // -----------------------------------------------------------------------
    @Test
    @DisplayName("Correctness 2 — 2x2: clear unique optimal (0→0, 1→1), cost = 4")
    void testAssignment_2x2_simple() {
        double[][] original = {
            {1, 2},
            {4, 3}
        };
        double[][] cost = {
            {1, 2},
            {4, 3}
        };

        int[] assignment = toAssignmentMap(new HungarianAlgo(cost).solve());

        assertEquals(0, assignment[0], "Row 0 should be assigned to column 0");
        assertEquals(1, assignment[1], "Row 1 should be assigned to column 1");
        assertEquals(4.0, assignmentCost(original, assignment), 1e-9,
                "Total cost must equal 4");
    }

    // -----------------------------------------------------------------------
    //  Correctness test 3 — 3x3 from HungarianAlgo.main()
    //
    //  Cost matrix:
    //    [ 2500  4000  3500 ]
    //    [ 4000  6000  3500 ]
    //    [ 2000  4000  2500 ]
    //
    //  Optimal: row 0 → col 1 (4000), row 1 → col 2 (3500), row 2 → col 0 (2000)
    //  Total = 9500  (next-best is 10000)
    // -----------------------------------------------------------------------
    @Test
    @DisplayName("Correctness 3 — 3x3 (from main): assignment (0→1, 1→2, 2→0), cost = 9500")
    void testAssignment_3x3_fromMain() {
        double[][] original = {
            {2500, 4000, 3500},
            {4000, 6000, 3500},
            {2000, 4000, 2500}
        };
        double[][] cost = {
            {2500, 4000, 3500},
            {4000, 6000, 3500},
            {2000, 4000, 2500}
        };

        int[] assignment = toAssignmentMap(new HungarianAlgo(cost).solve());

        assertEquals(1, assignment[0], "Row 0 should map to column 1");
        assertEquals(2, assignment[1], "Row 1 should map to column 2");
        assertEquals(0, assignment[2], "Row 2 should map to column 0");
        assertEquals(9500.0, assignmentCost(original, assignment), 1e-9,
                "Total cost must equal 9500");
    }

    // -----------------------------------------------------------------------
    //  Correctness test 4 — 4x4 classic textbook problem
    //
    //  Cost matrix:
    //    [ 9  2  7  8 ]
    //    [ 6  4  3  7 ]
    //    [ 5  8  1  8 ]
    //    [ 7  6  9  4 ]
    //
    //  Optimal: row 0 → col 1 (2), row 1 → col 0 (6), row 2 → col 2 (1), row 3 → col 3 (4)
    //  Total = 13
    // -----------------------------------------------------------------------
    @Test
    @DisplayName("Correctness 4 — 4x4 classic: assignment (0→1, 1→0, 2→2, 3→3), cost = 13")
    void testAssignment_4x4_classic() {
        double[][] original = {
            {9, 2, 7, 8},
            {6, 4, 3, 7},
            {5, 8, 1, 8},
            {7, 6, 9, 4}
        };
        double[][] cost = {
            {9, 2, 7, 8},
            {6, 4, 3, 7},
            {5, 8, 1, 8},
            {7, 6, 9, 4}
        };

        int[] assignment = toAssignmentMap(new HungarianAlgo(cost).solve());

        assertEquals(1, assignment[0], "Row 0 should map to column 1");
        assertEquals(0, assignment[1], "Row 1 should map to column 0");
        assertEquals(2, assignment[2], "Row 2 should map to column 2");
        assertEquals(3, assignment[3], "Row 3 should map to column 3");
        assertEquals(13.0, assignmentCost(original, assignment), 1e-9,
                "Total cost must equal 13");
    }

    // -----------------------------------------------------------------------
    //  Correctness test 5 — 5x5 problem
    //
    //  Cost matrix:
    //    [ 12   7   9   7   9 ]
    //    [  8   6   6   2   7 ]
    //    [  7  11  11  11   1 ]
    //    [ 14   6   5   7   6 ]
    //    [  8   5   4   4   3 ]
    //
    //  Optimal: row 0 → col 1 (7), row 1 → col 3 (2), row 2 → col 4 (1),
    //           row 3 → col 2 (5), row 4 → col 0 (8)
    //  Total = 23
    // -----------------------------------------------------------------------
    @Test
    @DisplayName("Correctness 5 — 5x5: assignment (0→1, 1→3, 2→4, 3→2, 4→0), cost = 23")
    void testAssignment_5x5() {
        double[][] original = {
            {12,  7,  9,  7,  9},
            { 8,  6,  6,  2,  7},
            { 7, 11, 11, 11,  1},
            {14,  6,  5,  7,  6},
            { 8,  5,  4,  4,  3}
        };
        double[][] cost = {
            {12,  7,  9,  7,  9},
            { 8,  6,  6,  2,  7},
            { 7, 11, 11, 11,  1},
            {14,  6,  5,  7,  6},
            { 8,  5,  4,  4,  3}
        };

        int[] assignment = toAssignmentMap(new HungarianAlgo(cost).solve());

        assertEquals(1, assignment[0], "Row 0 should map to column 1");
        assertEquals(3, assignment[1], "Row 1 should map to column 3");
        assertEquals(4, assignment[2], "Row 2 should map to column 4");
        assertEquals(2, assignment[3], "Row 3 should map to column 2");
        assertEquals(0, assignment[4], "Row 4 should map to column 0");
        assertEquals(23.0, assignmentCost(original, assignment), 1e-9,
                "Total cost must equal 23");
    }

    // -----------------------------------------------------------------------
    //  Edge case: uniform cost matrix
    //
    //  Every entry is identical — any bijection is optimal, so we check that
    //  (a) the algorithm terminates, (b) produces a valid bijection, and
    //  (c) the cost equals n * uniform_value.
    // -----------------------------------------------------------------------
    @Test
    @DisplayName("Edge case — 3x3 uniform costs: any bijection accepted, total cost = 15")
    void testAssignment_3x3_allEqual() {
        final double VALUE = 5.0;
        double[][] original = {
            {VALUE, VALUE, VALUE},
            {VALUE, VALUE, VALUE},
            {VALUE, VALUE, VALUE}
        };
        double[][] cost = {
            {VALUE, VALUE, VALUE},
            {VALUE, VALUE, VALUE},
            {VALUE, VALUE, VALUE}
        };

        int[] assignment = toAssignmentMap(new HungarianAlgo(cost).solve());

        // Verify bijection: every column appears exactly once
        boolean[] colUsed = new boolean[3];
        for (int col : assignment) {
            assertFalse(colUsed[col], "Each column must appear at most once in a valid assignment");
            colUsed[col] = true;
        }
        assertEquals(3 * VALUE, assignmentCost(original, assignment), 1e-9,
                "Total cost of any bijection must equal 15");
    }

    // -----------------------------------------------------------------------
    //  Edge case: zero-diagonal matrix
    //
    //  Cost matrix:
    //    [ 0  1  2 ]
    //    [ 1  0  3 ]
    //    [ 2  3  0 ]
    //
    //  Optimal: row 0 → col 0, row 1 → col 1, row 2 → col 2, total = 0
    // -----------------------------------------------------------------------
    @Test
    @DisplayName("Edge case — 3x3 zero diagonal: assignment (0→0, 1→1, 2→2), cost = 0")
    void testAssignment_3x3_zeroDiagonal() {
        double[][] original = {
            {0, 1, 2},
            {1, 0, 3},
            {2, 3, 0}
        };
        double[][] cost = {
            {0, 1, 2},
            {1, 0, 3},
            {2, 3, 0}
        };

        int[] assignment = toAssignmentMap(new HungarianAlgo(cost).solve());

        assertEquals(0, assignment[0], "Row 0 should map to column 0");
        assertEquals(1, assignment[1], "Row 1 should map to column 1");
        assertEquals(2, assignment[2], "Row 2 should map to column 2");
        assertEquals(0.0, assignmentCost(original, assignment), 1e-9,
                "Total cost must equal 0");
    }

    // -----------------------------------------------------------------------
    //  Structure test: result has the correct shape
    // -----------------------------------------------------------------------
    @Test
    @DisplayName("Result shape: solve() returns numRows pairs, each containing [row, col]")
    void testResultShape() {
        double[][] cost = {
            {3, 1, 4},
            {1, 5, 9},
            {2, 6, 5}
        };
        int[][] result = new HungarianAlgo(cost).solve();

        assertEquals(3, result.length, "Result should have one pair per row");
        for (int i = 0; i < result.length; i++) {
            assertEquals(2, result[i].length,
                    "Each pair must contain exactly two elements [row, col]");
            assertEquals(i, result[i][0],
                    "First element of each pair must be the row index");
            assertTrue(result[i][1] >= 0 && result[i][1] < 3,
                    "Column index must be within [0, numCols)");
        }
    }

    public static void main(String[] args) {
        
    }
}