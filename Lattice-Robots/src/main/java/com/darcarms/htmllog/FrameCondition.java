package com.darcarms.htmllog;

/**
 * Predicate over a single Java stack frame.
 *
 * <p>{@code FrameCondition} is used by {@link LogConditions} when building
 * stack-aware configuration rules. A frame condition answers whether one
 * {@link StackTraceElement} matches some criterion, such as a method name,
 * class name, or source filename.</p>
 *
 * <p>Most users do not need to implement this interface directly. Instead,
 * use helper methods such as {@link LogConditions#methodFrame(String)},
 * {@link LogConditions#classFrame(String)}, and
 * {@link LogConditions#fileFrame(String)}.</p>
 *
 * <p>The most common use is with {@link LogConditions#andBelow(FrameCondition)},
 * which turns a condition on one frame into a {@link LogCondition} that applies
 * when the frame appears anywhere in the current call stack.</p>
 *
 * <pre>{@code
 * log.addConfigRule(
 *     LogConditions.andBelow(LogConditions.methodFrame("show")),
 *     "Disable serialization below show()",
 *     LogConfigPatch.builder().serializables(false).build()
 * );
 * }</pre>
 */
@FunctionalInterface
public interface FrameCondition {

    /**
     * Tests whether the supplied stack frame matches this condition.
     *
     * @param frame the stack frame to test
     * @return {@code true} if the frame matches; otherwise {@code false}
     */
    boolean test(StackTraceElement frame);
}