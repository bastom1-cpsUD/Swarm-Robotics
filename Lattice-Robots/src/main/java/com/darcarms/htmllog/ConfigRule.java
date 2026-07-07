package com.darcarms.htmllog;

import java.util.List;
import java.util.Objects;

/**
 * A stack-aware configuration rule for {@link HtmlLog}.
 *
 * <p>A config rule combines a {@link LogCondition}, a human-readable
 * description, and a {@link LogConfigPatch}. When {@link HtmlLog#getConfig()}
 * is called, the logger evaluates its rules in order. For each rule whose
 * condition matches the current call stack, the rule's patch is applied to the
 * effective {@link LogConfig}.</p>
 *
 * <p>This makes it possible to selectively enable or disable features such as
 * image output or serializable snapshots depending on where the logging call
 * originated.</p>
 *
 * <pre>{@code
 * log.addConfigRule(
 *     LogConditions.inMethod("show"),
 *     "Disable serializable snapshots from show methods",
 *     LogConfigPatch.builder()
 *         .serializables(false)
 *         .build()
 * );
 * }</pre>
 *
 * @param condition the condition that decides whether this rule applies
 * @param description a human-readable description used for debugging and
 *                    diagnostics
 * @param patch the partial configuration override to apply when the condition
 *              matches
 */
public record ConfigRule(
        LogCondition condition,
        String description,
        LogConfigPatch patch
) {

    /**
     * Creates a configuration rule.
     *
     * @throws NullPointerException if {@code condition}, {@code description},
     *                              or {@code patch} is null
     */
    public ConfigRule {
        Objects.requireNonNull(condition, "condition");
        Objects.requireNonNull(description, "description");
        Objects.requireNonNull(patch, "patch");
    }

    /**
     * Returns whether this rule applies to the supplied call stack.
     *
     * @param stack the filtered application call stack
     * @return {@code true} if this rule's condition matches; otherwise
     *         {@code false}
     */
    public boolean matches(List<StackTraceElement> stack) {
        return condition.test(stack);
    }

    /**
     * Applies this rule's patch to a base configuration.
     *
     * <p>This method does not check whether the rule matches the current call
     * stack. Call {@link #matches(List)} first if conditional application is
     * needed.</p>
     *
     * @param base the configuration to patch
     * @return the patched configuration
     * @throws NullPointerException if {@code base} is null
     */
    public LogConfig applyTo(LogConfig base) {
        Objects.requireNonNull(base, "base");
        return base.withPatch(patch);
    }
}