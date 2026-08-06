package org.communicationModels.HungarianAlgo;

import java.util.ArrayList;
import java.util.List;

/**
 * Standalone test runner for HungarianAlgoTest.
 *
 * Instantiates HungarianAlgoTest directly and calls every test method in turn.
 * Each test is wrapped in a try/catch so a single failure never aborts the run.
 *
 * Output format
 * ─────────────
 *   [PASS]  testName — display name
 *   [FAIL]  testName — display name
 *           └─ ExceptionType: message
 *
 * Exit codes: 0 = all passed, 1 = one or more failed.
 *
 * Usage (from project root after building):
 *   java -cp build/libs/<jar> org.robots.HungarianAlgoTestRunner
 */
public class HungarianAlgoTestRunner {

    // -----------------------------------------------------------------------
    //  Internal result record
    // -----------------------------------------------------------------------
    private record TestResult(String methodName, String displayName,
                               boolean passed, Throwable failure) {}

    // -----------------------------------------------------------------------
    //  Test registry — one entry per test method, preserving declaration order
    // -----------------------------------------------------------------------
    @FunctionalInterface
    private interface TestMethod { void run(HungarianAlgoTest t) throws Throwable; }

    private record TestEntry(String methodName, String displayName, TestMethod method) {}

    private static final List<TestEntry> TESTS = List.of(

        // ── Input validation ──────────────────────────────────────────────
        new TestEntry(
            "testNullMatrix",
            "Null cost matrix throws IllegalArgumentException",
            HungarianAlgoTest::testNullMatrix
        ),
        new TestEntry(
            "testEmptyMatrix",
            "Empty cost matrix (zero rows) throws IllegalArgumentException",
            HungarianAlgoTest::testEmptyMatrix
        ),
        new TestEntry(
            "testMoreRowsThanColumns_3x2",
            "Matrix with more rows than columns (3x2) throws IllegalArgumentException",
            HungarianAlgoTest::testMoreRowsThanColumns_3x2
        ),
        new TestEntry(
            "testMoreRowsThanColumns_4x1",
            "Matrix with more rows than columns (4x1) throws IllegalArgumentException",
            HungarianAlgoTest::testMoreRowsThanColumns_4x1
        ),

        // ── Correctness ───────────────────────────────────────────────────
        new TestEntry(
            "testAssignment_1x1",
            "Correctness 1 — 1x1 matrix: only assignment is (0→0)",
            HungarianAlgoTest::testAssignment_1x1
        ),
        new TestEntry(
            "testAssignment_2x2_simple",
            "Correctness 2 — 2x2: clear unique optimal (0→0, 1→1), cost = 4",
            HungarianAlgoTest::testAssignment_2x2_simple
        ),
        new TestEntry(
            "testAssignment_3x3_fromMain",
            "Correctness 3 — 3x3 (from main): assignment (0→1, 1→2, 2→0), cost = 9500",
            HungarianAlgoTest::testAssignment_3x3_fromMain
        ),
        new TestEntry(
            "testAssignment_4x4_classic",
            "Correctness 4 — 4x4 classic: assignment (0→1, 1→0, 2→2, 3→3), cost = 13",
            HungarianAlgoTest::testAssignment_4x4_classic
        ),
        new TestEntry(
            "testAssignment_5x5",
            "Correctness 5 — 5x5: assignment (0→1, 1→3, 2→4, 3→2, 4→0), cost = 23",
            HungarianAlgoTest::testAssignment_5x5
        ),

        // ── Edge cases ────────────────────────────────────────────────────
        new TestEntry(
            "testAssignment_3x3_allEqual",
            "Edge case — 3x3 uniform costs: any bijection accepted, total cost = 15",
            HungarianAlgoTest::testAssignment_3x3_allEqual
        ),
        new TestEntry(
            "testAssignment_3x3_zeroDiagonal",
            "Edge case — 3x3 zero diagonal: assignment (0→0, 1→1, 2→2), cost = 0",
            HungarianAlgoTest::testAssignment_3x3_zeroDiagonal
        ),

        // ── Structure ─────────────────────────────────────────────────────
        new TestEntry(
            "testResultShape",
            "Result shape: solve() returns numRows pairs, each containing [row, col]",
            HungarianAlgoTest::testResultShape
        )
    );

    // -----------------------------------------------------------------------
    //  Runner
    // -----------------------------------------------------------------------
    public static void main(String[] args) {

        List<TestResult> results = new ArrayList<>();

        // Header
        System.out.println("═".repeat(72));
        System.out.println(" HungarianAlgo — Test Runner");
        System.out.println(" Total: " + TESTS.size() + " tests");
        System.out.println("═".repeat(72));

        // Run every test against a fresh instance of the test class
        for (TestEntry entry : TESTS) {
            HungarianAlgoTest instance = new HungarianAlgoTest();
            try {
                entry.method().run(instance);
                results.add(new TestResult(entry.methodName(), entry.displayName(), true, null));
                System.out.printf("[PASS]  %s%n", entry.methodName());
                System.out.printf("        %s%n%n", entry.displayName());
            } catch (Throwable t) {
                results.add(new TestResult(entry.methodName(), entry.displayName(), false, t));
                System.out.printf("[FAIL]  %s%n", entry.methodName());
                System.out.printf("        %s%n", entry.displayName());
                System.out.printf("        └─ %s: %s%n%n",
                        t.getClass().getSimpleName(), t.getMessage());
            }
        }

        // Summary
        long passed = results.stream().filter(TestResult::passed).count();
        long failed = results.size() - passed;

        System.out.println("═".repeat(72));
        System.out.printf(" Results:  %d passed  |  %d failed  |  %d total%n",
                passed, failed, results.size());
        System.out.println("═".repeat(72));

        // Detail block for failures only
        if (failed > 0) {
            System.out.println();
            System.out.println("── Failure detail ──────────────────────────────────────────────────");
            for (TestResult r : results) {
                if (!r.passed()) {
                    System.out.printf("%n[FAIL]  %s%n", r.methodName());
                    System.out.printf("        %s%n", r.displayName());
                    Throwable t = r.failure();
                    // Print the first few stack frames for quick diagnosis
                    System.out.printf("        %s: %s%n",
                            t.getClass().getSimpleName(), t.getMessage());
                    StackTraceElement[] frames = t.getStackTrace();
                    int framesToShow = Math.min(frames.length, 5);
                    for (int i = 0; i < framesToShow; i++) {
                        System.out.printf("            at %s%n", frames[i]);
                    }
                }
            }
            System.out.println();
        }

        System.exit(failed > 0 ? 1 : 0);
    }
}