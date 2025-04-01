const std = @import("std");

/// LogLevel defines the severity of the message
pub const LogLevel = enum {
    debug, // Detailed info for debugging
    info, // General information
    warning, // Warnings
    err, // Errors (renamed from 'error' which is a keyword)
    none, // No logging
};

/// Convert level to string for display
pub fn levelToString(level: LogLevel) []const u8 {
    return switch (level) {
        .debug => "[DEBUG]",
        .info => "[INFO]",
        .warning => "[WARN]",
        .err => "[ERROR]",
        .none => "[NONE]",
    };
}

/// LogCategory defines the subsystem the log message belongs to
pub const LogCategory = enum {
    general, // General physics engine messages
    performance, // Performance metrics
    collision, // Collision detection and resolution
    integration, // Physics integration
    sleep, // Sleep state management
    stability, // Stability and sanitization
    broadphase, // Broad phase collision detection

    /// Convert category to color for nicer terminal output
    pub fn toColor(self: LogCategory) []const u8 {
        return switch (self) {
            .general => "\x1b[37m", // White
            .performance => "\x1b[36m", // Cyan
            .collision => "\x1b[31m", // Red
            .integration => "\x1b[32m", // Green
            .sleep => "\x1b[35m", // Magenta
            .stability => "\x1b[33m", // Yellow
            .broadphase => "\x1b[34m", // Blue
        };
    }

    /// Convert category to emoji prefix for nicer terminal output
    pub fn toEmoji(self: LogCategory) []const u8 {
        return switch (self) {
            .general => "🔧 ", // Wrench
            .performance => "⏱️ ", // Stopwatch
            .collision => "💥 ", // Collision
            .integration => "🧮 ", // Calculation
            .sleep => "💤 ", // Sleep
            .stability => "⚠️ ", // Warning
            .broadphase => "🔍 ", // Magnifying glass
        };
    }
};

/// Logger manages logging configuration for the physics engine
pub const Logger = struct {
    // Hardcoded count of LogCategory fields - update if enum changes
    const CATEGORY_COUNT = 7; // general, performance, collision, integration, sleep, stability, broadphase

    enabled: bool = true,
    min_level: LogLevel = .info,
    category_filters: [CATEGORY_COUNT]bool = [_]bool{true} ** CATEGORY_COUNT,
    color_enabled: bool = true,
    emoji_enabled: bool = true,

    /// Initialize a new logger with default settings
    pub fn init() Logger {
        return .{};
    }

    /// Set the minimum log level (messages below this level are suppressed)
    pub fn setMinLevel(self: *Logger, level: LogLevel) void {
        self.min_level = level;
    }

    /// Enable or disable all logging
    pub fn setEnabled(self: *Logger, enabled: bool) void {
        self.enabled = enabled;
    }

    /// Enable or disable a specific category
    pub fn setCategoryEnabled(self: *Logger, category: LogCategory, enabled: bool) void {
        self.category_filters[@intFromEnum(category)] = enabled;
    }

    /// Enable or disable colored output
    pub fn setColorEnabled(self: *Logger, enabled: bool) void {
        self.color_enabled = enabled;
    }

    /// Enable or disable emoji prefixes
    pub fn setEmojiEnabled(self: *Logger, enabled: bool) void {
        self.emoji_enabled = enabled;
    }

    /// Check if a message should be logged based on level and category
    pub fn shouldLog(self: Logger, level: LogLevel, category: LogCategory) bool {
        if (!self.enabled) return false;
        if (@intFromEnum(level) < @intFromEnum(self.min_level)) return false;
        if (!self.category_filters[@intFromEnum(category)]) return false;
        return true;
    }

    /// Log a message if it passes the filters
    pub fn log(self: Logger, level: LogLevel, category: LogCategory, comptime fmt: []const u8, args: anytype) void {
        if (!self.shouldLog(level, category)) return;

        // Format prefix with category and level indicators
        const reset = if (self.color_enabled) "\x1b[0m" else "";
        const color = if (self.color_enabled) category.toColor() else "";
        const emoji = if (self.emoji_enabled) category.toEmoji() else "";
        const level_str = switch (level) {
            .debug => "[DEBUG]",
            .info => "[INFO]",
            .warning => "[WARN]",
            .err => "[ERROR]",
            .none => unreachable, // This shouldn't be logged
        };

        // Format and output the message
        std.debug.print("{s}{s}{s}{s} ", .{ color, emoji, level_str, reset });
        std.debug.print(fmt, args);
        std.debug.print("\n", .{});
    }

    /// Helper methods for different log levels
    pub fn debug(self: Logger, category: LogCategory, comptime fmt: []const u8, args: anytype) void {
        self.log(.debug, category, fmt, args);
    }

    pub fn info(self: Logger, category: LogCategory, comptime fmt: []const u8, args: anytype) void {
        self.log(.info, category, fmt, args);
    }

    pub fn warning(self: Logger, category: LogCategory, comptime fmt: []const u8, args: anytype) void {
        self.log(.warning, category, fmt, args);
    }

    pub fn err(self: Logger, category: LogCategory, comptime fmt: []const u8, args: anytype) void {
        self.log(.err, category, fmt, args);
    }

    /// Log performance metrics with timing info
    pub fn logPerformance(self: Logger, operation: []const u8, time_ns: i64, additional_info: anytype) void {
        if (!self.shouldLog(.info, .performance)) return;

        const time_ms = @as(f64, @floatFromInt(time_ns)) / 1_000_000.0;
        self.info(.performance, "{s}: {d:.2}ms {any}", .{ operation, time_ms, additional_info });
    }
};

/// Create a global logger instance
pub var global = Logger.init();

/// Convenience exports as functions
pub fn debug(category: LogCategory, comptime fmt: []const u8, args: anytype) void {
    global.debug(category, fmt, args);
}

pub fn info(category: LogCategory, comptime fmt: []const u8, args: anytype) void {
    global.info(category, fmt, args);
}

pub fn warning(category: LogCategory, comptime fmt: []const u8, args: anytype) void {
    global.warning(category, fmt, args);
}

pub fn logPerformance(operation: []const u8, time_ns: i64, additional_info: anytype) void {
    global.logPerformance(operation, time_ns, additional_info);
}

pub fn err(category: LogCategory, comptime fmt: []const u8, args: anytype) void {
    global.err(category, fmt, args);
}
