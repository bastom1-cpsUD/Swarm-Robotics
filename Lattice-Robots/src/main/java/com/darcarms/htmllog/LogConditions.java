package com.darcarms.htmllog;

import java.util.List;
import java.util.Objects;

/**
 * Factory methods for building {@link LogCondition} and {@link FrameCondition}
 * instances.
 *
 * <p>{@code LogConditions} provides reusable predicates for HtmlLog's
 * stack-aware configuration system. These predicates let callers enable,
 * disable, or modify logging behavior depending on where a logging call
 * originated.</p>
 *
 * <p>For example, a rule can disable serializable snapshots from methods named
 * {@code show}:</p>
 *
 * <pre>{@code
 * log.addConfigRule(
 *     LogConditions.inMethod("show"),
 *     "Disable snapshots from show methods",
 *     LogConfigPatch.builder()
 *         .serializables(false)
 *         .build()
 * );
 * }</pre>
 *
 * <p>The {@code in...} methods test the most immediate application frame. The
 * {@link #andBelow(FrameCondition)} method tests whether a frame-level
 * condition appears anywhere in the current application call stack.</p>
 */
public final class LogConditions {

    private LogConditions() {
    }

    /**
     * Returns a condition that always matches.
     *
     * @return a condition that always returns {@code true}
     */
    public static LogCondition everywhere() {
        return stack -> true;
    }

    /**
     * Returns a condition that matches when the most immediate application
     * frame has the given method name.
     *
     * @param methodName the method name to match
     * @return a stack condition matching the top application frame's method
     * @throws NullPointerException if {@code methodName} is null
     */
    public static LogCondition inMethod(String methodName) {
        Objects.requireNonNull(methodName, "methodName");
        return stack -> !stack.isEmpty() && methodName.equals(stack.get(0).getMethodName());
    }

    /**
     * Returns a condition that matches when the most immediate application
     * frame's class name contains the supplied class-name fragment.
     *
     * <p>The match uses {@link String#contains(CharSequence)} so callers may
     * pass either a fully qualified class name or a short class-name fragment.</p>
     *
     * @param classNameFragment the class-name fragment to search for
     * @return a stack condition matching the top application frame's class name
     * @throws NullPointerException if {@code classNameFragment} is null
     */
    public static LogCondition inClass(String classNameFragment) {
        Objects.requireNonNull(classNameFragment, "classNameFragment");
        return stack -> !stack.isEmpty()
                && stack.get(0).getClassName().contains(classNameFragment);
    }

    /**
     * Returns a condition that matches when the most immediate application
     * frame's source filename contains the supplied filename fragment.
     *
     * <p>Stack frames may not always contain source filenames, depending on
     * compiler options and runtime environment. Frames with a null filename do
     * not match.</p>
     *
     * @param filenameFragment the filename fragment to search for
     * @return a stack condition matching the top application frame's filename
     * @throws NullPointerException if {@code filenameFragment} is null
     */
    public static LogCondition inFile(String filenameFragment) {
        Objects.requireNonNull(filenameFragment, "filenameFragment");
        return stack -> !stack.isEmpty()
                && stack.get(0).getFileName() != null
                && stack.get(0).getFileName().contains(filenameFragment);
    }

    /**
     * Returns a frame condition that matches a specific method name.
     *
     * @param methodName the method name to match
     * @return a frame condition matching {@link StackTraceElement#getMethodName()}
     * @throws NullPointerException if {@code methodName} is null
     */
    public static FrameCondition methodFrame(String methodName) {
        Objects.requireNonNull(methodName, "methodName");
        return frame -> methodName.equals(frame.getMethodName());
    }

    /**
     * Returns a frame condition that matches a class-name fragment.
     *
     * <p>The match uses {@link String#contains(CharSequence)} so callers may
     * pass either a fully qualified class name or a short class-name fragment.</p>
     *
     * @param classNameFragment the class-name fragment to search for
     * @return a frame condition matching class names containing the fragment
     * @throws NullPointerException if {@code classNameFragment} is null
     */
    public static FrameCondition classFrame(String classNameFragment) {
        Objects.requireNonNull(classNameFragment, "classNameFragment");
        return frame -> frame.getClassName().contains(classNameFragment);
    }

    /**
     * Returns a frame condition that matches a source filename fragment.
     *
     * <p>Frames with null source filenames do not match.</p>
     *
     * @param filenameFragment the filename fragment to search for
     * @return a frame condition matching filenames containing the fragment
     * @throws NullPointerException if {@code filenameFragment} is null
     */
    public static FrameCondition fileFrame(String filenameFragment) {
        Objects.requireNonNull(filenameFragment, "filenameFragment");
        return frame -> frame.getFileName() != null
                && frame.getFileName().contains(filenameFragment);
    }

    /**
     * Returns a stack condition that matches when the supplied frame condition
     * matches any frame in the current application call stack.
     *
     * <p>This is useful for rules that should apply not only inside a method,
     * but also in helper methods called below it.</p>
     *
     * <pre>{@code
     * log.addConfigRule(
     *     LogConditions.andBelow(LogConditions.methodFrame("solve")),
     *     "Disable images below solve()",
     *     LogConfigPatch.builder()
     *         .images(false)
     *         .build()
     * );
     * }</pre>
     *
     * @param frameCondition the frame-level condition to search for
     * @return a stack condition that matches if any stack frame matches
     * @throws NullPointerException if {@code frameCondition} is null
     */
    public static LogCondition andBelow(FrameCondition frameCondition) {
        Objects.requireNonNull(frameCondition, "frameCondition");
        return stack -> stack.stream().anyMatch(frameCondition::test);
    }

    /**
     * Returns the logical negation of a condition.
     *
     * @param condition the condition to negate
     * @return a condition that returns {@code true} exactly when
     *         {@code condition} returns {@code false}
     * @throws NullPointerException if {@code condition} is null
     */
    public static LogCondition not(LogCondition condition) {
        Objects.requireNonNull(condition, "condition");
        return stack -> !condition.test(stack);
    }

    /**
     * Returns the logical conjunction of two conditions.
     *
     * @param left the first condition
     * @param right the second condition
     * @return a condition that matches only when both inputs match
     * @throws NullPointerException if either condition is null
     */
    public static LogCondition and(LogCondition left, LogCondition right) {
        Objects.requireNonNull(left, "left");
        Objects.requireNonNull(right, "right");
        return stack -> left.test(stack) && right.test(stack);
    }

    /**
     * Returns the logical disjunction of two conditions.
     *
     * @param left the first condition
     * @param right the second condition
     * @return a condition that matches when either input condition matches
     * @throws NullPointerException if either condition is null
     */
    public static LogCondition or(LogCondition left, LogCondition right) {
        Objects.requireNonNull(left, "left");
        Objects.requireNonNull(right, "right");
        return stack -> left.test(stack) || right.test(stack);
    }

    /**
     * Returns a debug string for a stack.
     *
     * <p>This helper is useful when diagnosing why a stack-aware configuration
     * rule did or did not match. It is not used by normal logging output unless
     * called explicitly.</p>
     *
     * @param stack the stack to describe
     * @return a multiline string containing each stack frame
     * @throws NullPointerException if {@code stack} is null
     */
    public static String describeStack(List<StackTraceElement> stack) {
        Objects.requireNonNull(stack, "stack");

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < stack.size(); i++) {
            sb.append(i)
                    .append(": ")
                    .append(stack.get(i))
                    .append(System.lineSeparator());
        }
        return sb.toString();
    }
}