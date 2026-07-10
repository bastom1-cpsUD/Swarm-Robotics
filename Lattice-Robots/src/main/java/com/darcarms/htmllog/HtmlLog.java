package com.darcarms.htmllog;

import javax.imageio.ImageIO;
import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.PrintWriter;
import java.io.Serializable;
import java.io.StringWriter;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.UUID;

/**
 * HTML visual logger for Java programs.
 *
 * <p>{@code HtmlLog} writes a timestamped log directory containing an
 * {@code index.html} file and any generated asset files. It is intended for
 * rich debugging and inspection output, including headings, paragraphs,
 * preformatted blocks, nested groups, Java2D images, exception traces,
 * object-rendering hooks, serialized object snapshots, and stack-aware
 * configuration rules.</p>
 *
 * <p>The most common use is to create a log with {@link #createDefault()}, add
 * content, and close the log with try-with-resources:</p>
 *
 * <pre>{@code
 * try (HtmlLog log = HtmlLog.createDefault()) {
 *     log.heading("Demo");
 *     log.text("Hello from HtmlLog.");
 *
 *     try (HtmlLog.Group group = log.grouped("Drawing")) {
 *         try (LogGraphics canvas = log.graphics("drawing-", 600, 400)) {
 *             canvas.graphics().drawLine(50, 50, 550, 350);
 *         }
 *     }
 * }
 * }</pre>
 *
 * <p>{@code HtmlLog} is not intended to be a general application logging
 * framework. It is intended for producing human-readable HTML debug reports
 * with optional visual artifacts.</p>
 */
public final class HtmlLog implements AutoCloseable {
    private final Path directory;
    private final Path indexFile;
    private final BrowserLauncher browserLauncher;
    private final List<ConfigRule> configRules = new ArrayList<>();

    private BufferedWriter writer;
    private int headingLevel = 1;
    private boolean closed = false;
    private boolean verboseRules = false;

    private HtmlLog(Path directory, LogConfig initialConfig) {
        this(directory, initialConfig,  DesktopBrowserLauncher.INSTANCE);
    }

    /**
     * Creates a log with an injectable browser launcher.
     *
     * <p>This constructor is package-private so tests in
     * {@code com.darcarms.htmllog} can verify browser-opening behavior without
     * opening a real browser. Public callers should use one of the static
     * factory methods.</p>
     *
     * @param directory the directory that should contain {@code index.html} and
     *                  generated assets
     * @param initialConfig the initial logging configuration
     * @param browserLauncher strategy used to open the generated log in a
     *                        browser
     * @throws NullPointerException if any argument is null
     */
    HtmlLog(Path directory, LogConfig initialConfig, BrowserLauncher browserLauncher) {
        this.directory = Objects.requireNonNull(directory, "directory");
        Objects.requireNonNull(initialConfig, "initialConfig");
        this.browserLauncher = Objects.requireNonNull(browserLauncher, "browserLauncher");
        this.indexFile = directory.resolve("index.html");
        ensureDirectoryExists();

        addConfigRule(
                LogConditions.everywhere(),
                "default",
                LogConfigPatch.builder()
                        .active(initialConfig.active())
                        .images(initialConfig.images())
                        .serializables(initialConfig.serializables())
                        .browser(initialConfig.browser())
                        .browserNew(initialConfig.browserNew())
                        .build()
        );
    }

    /**
     * Creates a log in a timestamped directory under {@code logs/}.
     *
     * <p>The directory name uses the current local date and time. The generated
     * HTML file is named {@code index.html}.</p>
     *
     * @return a new log using {@link LogConfig#defaults()}
     */
    public static HtmlLog createDefault() {
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyy-MM-dd'T'HH-mm-ss"));
        return new HtmlLog(Path.of("logs", stamp), LogConfig.defaults());
    }

    /**
     * Creates a log in the supplied directory using the default configuration.
     *
     * @param directory the directory that should contain {@code index.html} and
     *                  generated assets
     * @return a new log using {@link LogConfig#defaults()}
     * @throws NullPointerException if {@code directory} is null
     */
    public static HtmlLog create(Path directory) {
        return new HtmlLog(directory, LogConfig.defaults());
    }

    /**
     * Creates a log in the supplied directory with common configuration
     * options.
     *
     * @param directory the directory that should contain {@code index.html} and
     *                  generated assets
     * @param imagesEnabled whether image assets should be written and embedded
     * @param openBrowser whether the generated log should be opened in a
     *                    browser when first written
     * @return a new log using the supplied options
     * @throws NullPointerException if {@code directory} is null
     */
    public static HtmlLog create(Path directory, boolean imagesEnabled, boolean openBrowser) {
        LogConfig config = new LogConfig(true, imagesEnabled, true, openBrowser, 2);
        return new HtmlLog(directory, config);
    }

    /**
     * Creates a log in the supplied directory using the supplied configuration.
     *
     * @param directory the directory that should contain {@code index.html} and
     *                  generated assets
     * @param config the initial logging configuration
     * @return a new log using the supplied configuration
     * @throws NullPointerException if {@code directory} or {@code config} is
     *                              null
     */
    public static HtmlLog create(Path directory, LogConfig config) {
        return new HtmlLog(directory, config);
    }

    /**
     * Creates a log with an injectable browser launcher for package-level tests.
     *
     * <p>This method is intentionally package-private. It is not part of the
     * public HtmlLog API.</p>
     *
     * @param directory the directory that should contain {@code index.html} and
     *                  generated assets
     * @param config the initial logging configuration
     * @param browserLauncher strategy used to open the generated log in a
     *                        browser
     * @return a new log using the supplied configuration and browser launcher
     */
    static HtmlLog createForTesting(
            Path directory,
            LogConfig config,
            BrowserLauncher browserLauncher
    ) {
        return new HtmlLog(directory, config, browserLauncher);
    }

    /**
     * Returns the directory containing this log's HTML file and generated
     * assets.
     *
     * @return the log directory
     */
    public Path directory() {
        return directory;
    }

    /**
     * Returns the path to this log's generated {@code index.html} file.
     *
     * @return the HTML index file path
     */
    public Path indexFile() {
        return indexFile;
    }

    /**
     * Returns whether logging is currently active at the calling location.
     *
     * <p>This value is resolved through {@link #getConfig()}, so stack-aware
     * configuration rules may affect the answer.</p>
     *
     * @return {@code true} if logging output should be written; otherwise
     *         {@code false}
     */
    public boolean isActive() {
        return getConfig().active();
    }

    /**
     * Returns whether image output is currently enabled at the calling
     * location.
     *
     * @return {@code true} if image assets should be written and embedded;
     *         otherwise {@code false}
     */
    public boolean imagesEnabled() {
        return getConfig().images();
    }

    /**
     * Returns whether serializable snapshots are currently enabled at the
     * calling location.
     *
     * @return {@code true} if {@link Serializable} objects should be written as
     *         linked {@code .ser} files; otherwise {@code false}
     */
    public boolean serializablesEnabled() {
        return getConfig().serializables();
    }

    /**
     * Enables or disables diagnostic output for configuration-rule evaluation.
     *
     * <p>When enabled, calls to {@link #getConfig()} print the filtered call
     * stack, each rule match result, and the final resolved configuration to
     * standard error.</p>
     *
     * @param enabled {@code true} to print rule diagnostics; {@code false} to
     *                disable them
     */
    public void debugConfigRules(boolean enabled) {
        this.verboseRules = enabled;
    }

    /**
     * Adds a stack-aware configuration rule.
     *
     * <p>Rules are evaluated in the order they are added. When a rule matches,
     * its patch is applied to the current effective configuration.</p>
     *
     * @param condition the condition deciding whether the rule applies
     * @param description a human-readable rule description
     * @param patch the partial configuration override to apply
     * @throws NullPointerException if any argument is null
     */
    public void addConfigRule(LogCondition condition, String description, LogConfigPatch patch) {
        configRules.add(new ConfigRule(condition, description, patch));
    }

    /**
     * Adds a stack-aware configuration rule with a generic description.
     *
     * @param condition the condition deciding whether the rule applies
     * @param patch the partial configuration override to apply
     * @throws NullPointerException if either argument is null
     */
    public void addConfigRule(LogCondition condition, LogConfigPatch patch) {
        addConfigRule(condition, "custom", patch);
    }

    /**
     * Returns the configuration rules currently registered with this log.
     *
     * @return an unmodifiable view of the registered rules
     */
    public List<ConfigRule> configRules() {
        return Collections.unmodifiableList(configRules);
    }

    /**
     * Resolves the effective configuration for the current calling location.
     *
     * <p>The logger builds a filtered call stack, starts from an inactive base
     * configuration, and applies each matching {@link ConfigRule} in order.
     * The default rule added during construction provides the normal base
     * behavior.</p>
     *
     * @return the effective logging configuration for the current call site
     */
    public LogConfig getConfig() {
        List<StackTraceElement> stack = filteredStack();
        LogConfig config = new LogConfig(false, false, false, false, 2);

        if (verboseRules) {
            System.err.println("\nHtmlLog config check");
            for (StackTraceElement frame : stack) {
                System.err.println("  frame " + frame);
            }
        }

        for (ConfigRule rule : configRules) {
            boolean matches = rule.matches(stack);
            if (matches) {
                config = rule.applyTo(config);
            }
            if (verboseRules) {
                System.err.println((matches ? "  [*] " : "  [ ] ") + rule.description() + " " + rule.patch());
            }
        }

        if (verboseRules) {
            System.err.println("  => " + config);
        }
        return config;
    }

    /**
     * Writes escaped paragraph text to the log.
     *
     * @param text the text to write
     */
    public void text(String text) {
        writeIfActive("<p>" + escape(text) + "</p>\n");
    }

    /**
     * Writes escaped preformatted text to the log.
     *
     * <p>The content is HTML-escaped, so arbitrary text is safe to pass.</p>
     *
     * @param text the preformatted text to write
     */
    public void pre(String text) {
        writeIfActive("<pre>" + escape(text) + "</pre>\n");
    }

    /**
     * Writes preformatted content without escaping it.
     *
     * <p>Use this method only with trusted content. Untrusted input may inject
     * arbitrary HTML into the generated log.</p>
     *
     * @param html trusted preformatted HTML content
     */
    public void preRaw(String html) {
        writeIfActive("<pre>" + html + "</pre>\n");
    }

    /**
     * Writes a heading at the current nesting level.
     *
     * <p>The heading level is clamped to the HTML range {@code h1} through
     * {@code h6}. Nested groups increase the heading level.</p>
     *
     * @param text the heading text
     */
    public void heading(String text) {
        int level = Math.max(1, Math.min(6, headingLevel));
        writeIfActive("<h" + level + ">" + escape(text) + "</h" + level + ">\n");
    }

    /**
     * Writes raw HTML directly to the log.
     *
     * <p>Use this method only with trusted content. Untrusted input may inject
     * arbitrary HTML into the generated log.</p>
     *
     * @param html trusted HTML content
     */
    public void rawHtml(String html) {
        writeIfActive(html + "\n");
    }

    /**
     * Writes a horizontal rule to the log.
     */
    public void horizontalRule() {
        writeIfActive("<hr>\n");
    }

    /**
     * Starts a nested visual group.
     *
     * <p>The returned group must be closed to write the closing {@code div}.
     * Prefer try-with-resources so the group is closed even if an exception is
     * thrown.</p>
     *
     * @return an auto-closeable group handle
     */
    public Group grouped() {
        pushGroup();
        return new Group(this);
    }

    /**
     * Starts a nested visual group and immediately writes a heading inside it.
     *
     * @param heading the heading to write inside the group
     * @return an auto-closeable group handle
     */
    public Group grouped(String heading) {
        Group group = grouped();
        heading(heading);
        return group;
    }

    private void pushGroup() {
        if (!isActive()) {
            return;
        }
        double percent = Math.max(70.0, 100.0 - headingLevel * 7.5);
        writeRaw(String.format(
                "<div style=\"padding:20px; margin:5px; background-color:rgb(%.1f%%,%.1f%%,%.1f%%)\">\n",
                percent, percent, percent));
        headingLevel++;
    }

    private void popGroup() {
        if (!isActive()) {
            return;
        }
        headingLevel = Math.max(1, headingLevel - 1);
        writeRaw("</div>\n");
    }

    /**
     * Writes an exception trace to the log inside a nested group.
     *
     * @param throwable the exception or error to display
     */
    public void exception(Throwable throwable) {
        if (!isActive()) {
            return;
        }
        try (Group ignored = grouped()) {
            heading("Unhandled exception");
            pre(stackTraceToString(throwable));
        }
    }

    /**
     * Displays an object in the log.
     *
     * <p>If the object implements {@link Loggable}, rendering is delegated to
     * {@link Loggable#show(HtmlLog)}. Otherwise, {@link String#valueOf(Object)}
     * is written as an escaped preformatted block.</p>
     *
     * <p>If serializable snapshots are enabled and the object implements
     * {@link Serializable}, the object is also written as a linked {@code .ser}
     * file.</p>
     *
     * @param object the object to display
     */
    public void show(Object object) {
        if (!isActive()) {
            return;
        }

        if (object instanceof Loggable loggable) {
            try {
                loggable.show(this);
            } catch (Exception e) {
                exception(e);
            }
        } else {
            pre(String.valueOf(object));
        }

        if (getConfig().serializables() && object instanceof Serializable serializable) {
            serialize(serializable, "object-");
        }
    }

    /**
     * Creates a unique asset file descriptor in this log's directory.
     *
     * <p>The returned file is not created automatically. Callers are
     * responsible for writing to {@link AssetFile#path()} and may reference the
     * asset from the HTML using {@link AssetFile#filename()}.</p>
     *
     * @param prefix preferred filename prefix; unsafe characters are replaced
     *               with underscores
     * @param suffix filename suffix, such as {@code ".png"} or {@code ".ser"}
     * @return a unique asset file descriptor
     */
    public AssetFile anonymousFile(String prefix, String suffix) {
        String safePrefix = prefix == null || prefix.isBlank() ? "asset-" : sanitizePrefix(prefix);
        String safeSuffix = suffix == null ? "" : suffix;
        String name = safePrefix + UUID.randomUUID().toString().replace("-", "") + safeSuffix;
        return new AssetFile(directory.resolve(name), name);
    }

    /**
     * Creates a unique asset path in this log's directory.
     *
     * @param prefix preferred filename prefix
     * @param suffix filename suffix
     * @return a unique filesystem path
     */
    public Path anonymousPath(String prefix, String suffix) {
        return anonymousFile(prefix, suffix).path();
    }

    /**
     * Creates a Java2D PNG drawing surface with a white background.
     *
     * @param prefix filename prefix for the generated PNG
     * @param width image width in pixels
     * @param height image height in pixels
     * @return an auto-closeable drawing surface
     */
    public LogGraphics graphics(String prefix, int width, int height) {
        return graphics(prefix, width, height, Color.WHITE, "");
    }

    /**
     * Creates a Java2D PNG drawing surface with a white background and custom
     * image CSS.
     *
     * @param prefix filename prefix for the generated PNG
     * @param width image width in pixels
     * @param height image height in pixels
     * @param style CSS style string for the generated {@code img} tag
     * @return an auto-closeable drawing surface
     */
    public LogGraphics graphics(String prefix, int width, int height, String style) {
        return graphics(prefix, width, height, Color.WHITE, style);
    }

    /**
     * Creates a Java2D PNG drawing surface.
     *
     * <p>When the returned {@link LogGraphics} is closed, the image is written
     * to disk and embedded in the log.</p>
     *
     * @param prefix filename prefix for the generated PNG
     * @param width image width in pixels
     * @param height image height in pixels
     * @param background background color; pass null for a transparent image
     * @param style CSS style string for the generated {@code img} tag
     * @return an auto-closeable drawing surface
     */
    public LogGraphics graphics(String prefix, int width, int height, Color background, String style) {
        AssetFile asset = anonymousFile(prefix, ".png");
        return new LogGraphics(this, asset, width, height, null, 0.0, background, style);
    }

    /**
     * Creates a mapped Java2D PNG drawing surface with default padding and a
     * white background.
     *
     * <p>The supplied user-space rectangle is fitted into the output image.
     * Drawing commands can then use logical coordinates rather than pixel
     * coordinates.</p>
     *
     * @param prefix filename prefix for the generated PNG
     * @param userAabb user-space rectangle to map into the image
     * @param width image width in pixels
     * @param height image height in pixels
     * @return an auto-closeable mapped drawing surface
     */
    public LogGraphics mappedGraphics(String prefix, Rect userAabb, int width, int height) {
        return mappedGraphics(prefix, userAabb, width, height, 0.1, Color.WHITE, "");
    }

    /**
     * Creates a mapped Java2D PNG drawing surface with default padding, a white
     * background, and custom image CSS.
     *
     * @param prefix filename prefix for the generated PNG
     * @param userAabb user-space rectangle to map into the image
     * @param width image width in pixels
     * @param height image height in pixels
     * @param style CSS style string for the generated {@code img} tag
     * @return an auto-closeable mapped drawing surface
     */
    public LogGraphics mappedGraphics(String prefix, Rect userAabb, int width, int height, String style) {
        return mappedGraphics(prefix, userAabb, width, height, 0.1, Color.WHITE, style);
    }

    /**
     * Creates a mapped Java2D PNG drawing surface.
     *
     * <p>The drawing transform maps {@code userAabb} into the output image
     * while preserving aspect ratio and applying the supplied padding.</p>
     *
     * @param prefix filename prefix for the generated PNG
     * @param userAabb user-space rectangle to map into the image
     * @param width image width in pixels
     * @param height image height in pixels
     * @param padding fractional padding relative to the larger user-space
     *                dimension
     * @param background background color; pass null for a transparent image
     * @param style CSS style string for the generated {@code img} tag
     * @return an auto-closeable mapped drawing surface
     */
    public LogGraphics mappedGraphics(
            String prefix,
            Rect userAabb,
            int width,
            int height,
            double padding,
            Color background,
            String style
    ) {
        AssetFile asset = anonymousFile(prefix, ".png");
        return new LogGraphics(this, asset, width, height, userAabb, padding, background, style);
    }

    /**
     * Writes and embeds an existing image.
     *
     * @param image the image to write as PNG
     * @param prefix filename prefix for the generated PNG
     */
    public void image(BufferedImage image, String prefix) {
        image(image, prefix, "");
    }

    /**
     * Writes and embeds an existing image with custom image CSS.
     *
     * @param image the image to write as PNG
     * @param prefix filename prefix for the generated PNG
     * @param style CSS style string for the generated {@code img} tag
     * @throws NullPointerException if {@code image} is null
     * @throws UncheckedIOException if the image cannot be written
     */
    public void image(BufferedImage image, String prefix, String style) {
        Objects.requireNonNull(image, "image");

        if (!getConfig().images()) {
            return;
        }

        AssetFile asset = anonymousFile(prefix, ".png");
        try {
            ImageIO.write(image, "png", asset.path().toFile());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        embedImage(asset.filename(), style);
    }

    /**
     * Writes a link to an existing log asset.
     *
     * @param asset the asset to link
     * @param label the link label
     * @throws NullPointerException if {@code asset} is null
     */
    public void linkAsset(AssetFile asset, String label) {
        Objects.requireNonNull(asset, "asset");

        writeIfActive("<div style=\"text-align:right\"><a href=\""
                + escapeAttribute(asset.filename()) + "\">"
                + escape(label) + "</a></div>\n");
    }

    /**
     * Serializes an object to a linked {@code .ser} asset.
     *
     * <p>If serializable snapshots are currently disabled, this method returns
     * null and writes nothing. If serialization fails, the error is ignored and
     * null is returned so that diagnostic logging does not interrupt the program
     * being inspected.</p>
     *
     * @param object the object to serialize
     * @param prefix filename prefix for the generated serialized asset
     * @return the generated asset, or null if no asset was written
     */
    public AssetFile serialize(Serializable object, String prefix) {
        if (!getConfig().serializables()) {
            return null;
        }

        AssetFile asset = anonymousFile(prefix, ".ser");
        try (ObjectOutputStream out = new ObjectOutputStream(Files.newOutputStream(asset.path()))) {
            out.writeObject(object);
        } catch (IOException e) {
            return null;
        }

        linkAsset(asset, "[serialized]");
        return asset;
    }

    /**
     * Embeds an image asset in the generated HTML.
     *
     * <p>This method is package-private so {@link LogGraphics} can embed the
     * PNG it writes when closed.</p>
     *
     * @param filename relative image filename
     * @param style CSS style string for the generated {@code img} tag
     */
    void embedImage(String filename, String style) {
        if (getConfig().images()) {
            writeRaw("<img src=\"" + escapeAttribute(filename) + "\" style=\""
                    + escapeAttribute(style) + "\">\n");
        }
    }

    private void writeIfActive(String html) {
        if (isActive()) {
            writeRaw(html);
        }
    }

    private void writeRaw(String html) {
        if (closed) {
            throw new IllegalStateException("HtmlLog is already closed");
        }

        try {
            openIfNeeded();
            writer.write(html);
            writer.flush();
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private void openIfNeeded() throws IOException {
        if (writer != null) {
            return;
        }

        ensureDirectoryExists();
        writer = Files.newBufferedWriter(indexFile, StandardCharsets.UTF_8);
        writer.write("<!doctype html>\n");
        writer.write("<html>\n<head>\n<meta charset=\"utf-8\">\n<title>HtmlLog</title>\n");
        writer.write("<style>\n");
        writer.write("body{font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;line-height:1.35;}\n");
        writer.write("pre{background:#f6f6f6;padding:10px;overflow:auto;border-left:4px solid #ddd;}\n");
        writer.write("img{max-width:100%;height:auto;border:1px solid #ddd;margin:4px 0;}\n");
        writer.write("a{color:#0645ad;}\n");
        writer.write("</style>\n");
        writer.write("</head>\n<body>\n");
        writer.write("<div style=\"position:fixed;bottom:0;right:0;font-size:200%;background:white;padding:0 0.2em\"><a href=\"\">&#x27F3;</a></div>\n");
        writer.flush();

        maybeOpenBrowser();
    }

    private void maybeOpenBrowser() {
        LogConfig config = getConfig();

        if (!config.browser()) {
            return;
        }

        try {
            browserLauncher.open(indexFile);
        } catch (IOException | UnsupportedOperationException | SecurityException ignored) {
            // Browser opening is best-effort only.
        }
    }

    private void ensureDirectoryExists() {
        try {
            Files.createDirectories(directory);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private List<StackTraceElement> filteredStack() {
        return filterStack(Thread.currentThread().getStackTrace());
    }

    /**
     * Filters internal and runtime frames out of a stack trace.
     *
     * <p>This package-private helper exists so stack filtering can be tested with
     * synthetic stack frames. Normal callers should not use it directly.</p>
     *
     * @param raw the raw stack trace to filter
     * @return application-level stack frames, preserving original order
     */
    static List<StackTraceElement> filterStack(StackTraceElement[] raw) {
        List<StackTraceElement> result = new ArrayList<>();

        for (StackTraceElement element : raw) {
            String className = element.getClassName();

            if (className.equals(Thread.class.getName())) {
                continue;
            }
            if (className.equals(HtmlLog.class.getName())
                    || className.equals(LogGraphics.class.getName())
                    || className.equals(LogConditions.class.getName())
                    || className.equals(ConfigRule.class.getName())
                    || className.equals(LogConfig.class.getName())
                    || className.equals(LogConfigPatch.class.getName())) {
                continue;
            }
            if (className.startsWith("java.")
                    || className.startsWith("jdk.")
                    || className.startsWith("sun.")) {
                continue;
            }

            result.add(element);
        }

        return result;
    }

    /**
     * Closes the log.
     *
     * <p>If the HTML file has been opened, this method writes a small end
     * marker and closes the document. Closing is idempotent.</p>
     *
     * @throws UncheckedIOException if the HTML file cannot be closed
     */
    @Override
    public void close() {
        if (closed) {
            return;
        }

        if (writer != null) {
            try {
                writer.write("<br>[end]\n</body>\n</html>\n");
                writer.close();
            } catch (IOException e) {
                throw new UncheckedIOException(e);
            } finally {
                closed = true;
            }
        } else {
            closed = true;
        }
    }

    /**
     * Escapes text for safe insertion into HTML element content.
     *
     * @param text the text to escape
     * @return HTML-escaped text, or {@code "null"} if {@code text} is null
     */
    public static String escape(String text) {
        if (text == null) {
            return "null";
        }

        return text
                .replace("&", "&amp;")
                .replace("<", "&lt;")
                .replace(">", "&gt;");
    }

    /**
     * Escapes text for safe insertion into an HTML attribute value.
     *
     * @param text the text to escape
     * @return HTML-escaped attribute text
     */
    public static String escapeAttribute(String text) {
        return escape(text).replace("\"", "&quot;");
    }

    /**
     * Converts a throwable's stack trace to a string.
     *
     * @param throwable the throwable to render
     * @return the throwable's stack trace as text
     */
    public static String stackTraceToString(Throwable throwable) {
        StringWriter sw = new StringWriter();
        throwable.printStackTrace(new PrintWriter(sw));
        return sw.toString();
    }

    private static String sanitizePrefix(String prefix) {
        return prefix.replaceAll("[^A-Za-z0-9._-]", "_");
    }

    /**
     * Auto-closeable handle for a nested visual group in the HTML log.
     *
     * <p>Instances are created by {@link HtmlLog#grouped()} and
     * {@link HtmlLog#grouped(String)}. Use try-with-resources to ensure the
     * group is closed correctly.</p>
     */
    public static final class Group implements AutoCloseable {
        private final HtmlLog log;
        private boolean closed;

        private Group(HtmlLog log) {
            this.log = log;
        }

        /**
         * Closes the group and writes the closing HTML tag.
         *
         * <p>Closing is idempotent.</p>
         */
        @Override
        public void close() {
            if (!closed) {
                closed = true;
                log.popGroup();
            }
        }
    }
}