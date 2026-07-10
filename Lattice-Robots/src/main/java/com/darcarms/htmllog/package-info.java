/**
 * HTML visual logging utilities for Java programs.
 *
 * <p>The {@code com.darcarms.htmllog} package provides a lightweight,
 * Java2D-backed logging system for generating rich HTML debug reports.
 * Logs are written to timestamped directories and may contain headings,
 * paragraphs, preformatted text, nested visual groups, PNG images,
 * exception traces, serialized object snapshots, and Java2D drawings.</p>
 *
 * <p>The central class is {@link com.darcarms.htmllog.HtmlLog}. Most users
 * create a log with {@link com.darcarms.htmllog.HtmlLog#createDefault()},
 * then write content using methods such as {@code heading}, {@code text},
 * {@code pre}, {@code grouped}, {@code graphics}, and
 * {@code mappedGraphics}.</p>
 *
 * <p>This package is intended for visual debugging and exploratory program
 * inspection. It is not intended to replace ordinary application logging
 * frameworks.</p>
 */
package com.darcarms.htmllog;