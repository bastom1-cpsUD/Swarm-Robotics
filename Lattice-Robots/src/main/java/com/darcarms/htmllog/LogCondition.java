package com.darcarms.htmllog;

import java.util.List;

/**
 * Predicate over a filtered Java call stack.
 *
 * <p>{@code LogCondition} is used by {@link ConfigRule} to decide whether a
 * configuration override should apply at the current logging call site. The
 * logger builds a stack trace, removes HtmlLog's own internal frames, and then
 * passes the remaining application frames to this predicate.</p>
 *
 * <p>Most users should create conditions with the static helpers in
 * {@link LogConditions}, such as {@link LogConditions#inMethod(String)},
 * {@link LogConditions#inClass(String)}, {@link LogConditions#inFile(String)},
 * {@link LogConditions#and(LogCondition, LogCondition)}, and
 * {@link LogConditions#not(LogCondition)}.</p>
 *
 * <pre>{@code
 * log.addConfigRule(
 *     LogConditions.inMethod("show"),
 *     "Disable serialized snapshots from show methods",
 *     LogConfigPatch.builder().serializables(false).build()
 * );
 * }</pre>
 *
 * <p>A {@code LogCondition} receives the whole stack, unlike
 * {@link FrameCondition}, which receives only a single {@link StackTraceElement}.
 * Use {@link LogConditions#andBelow(FrameCondition)} when you want to apply a
 * frame-level condition anywhere in the call stack.</p>
 */
@FunctionalInterface
public interface LogCondition {

    /**
     * Tests whether this condition matches the supplied call stack.
     *
     * <p>The stack is ordered with the most immediate application frame first.
     * It should not include HtmlLog's own internal implementation frames.</p>
     *
     * @param stack the filtered application call stack
     * @return {@code true} if the condition matches; otherwise {@code false}
     */
    boolean test(List<StackTraceElement> stack);
}