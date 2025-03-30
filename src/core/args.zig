const std = @import("std");

/// Options that can be configured via command-line arguments
pub const EngineOptions = struct {
    debug_mode: bool = false,
    collision_logging: bool = false,
    draw_forces: bool = false,
    draw_velocities: bool = true,
    draw_normals: bool = false,
    target_fps: i32 = 60,
};

/// Parse command-line arguments and return engine options
pub fn parseArgs(allocator: std.mem.Allocator) !EngineOptions {
    // Get command line args
    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    var options = EngineOptions{};

    // Skip the program name (first argument)
    var i: usize = 1;
    while (i < args.len) : (i += 1) {
        const arg = args[i];

        if (std.mem.eql(u8, arg, "--debug") or std.mem.eql(u8, arg, "-d")) {
            options.debug_mode = true;
            std.debug.print("Debug mode enabled\n", .{});
        } else if (std.mem.eql(u8, arg, "--collision-logging") or std.mem.eql(u8, arg, "-c")) {
            options.collision_logging = true;
            std.debug.print("Collision logging enabled\n", .{});
        } else if (std.mem.eql(u8, arg, "--draw-forces") or std.mem.eql(u8, arg, "-f")) {
            options.draw_forces = true;
            std.debug.print("Force vectors enabled\n", .{});
        } else if (std.mem.eql(u8, arg, "--draw-normals") or std.mem.eql(u8, arg, "-n")) {
            options.draw_normals = true;
            std.debug.print("Normal vectors enabled\n", .{});
        } else if (std.mem.eql(u8, arg, "--no-velocities")) {
            options.draw_velocities = false;
            std.debug.print("Velocity vectors disabled\n", .{});
        } else if (std.mem.eql(u8, arg, "--fps") or std.mem.eql(u8, arg, "-r")) {
            // Make sure there's a next argument for the FPS value
            if (i + 1 < args.len) {
                i += 1;
                options.target_fps = try std.fmt.parseInt(i32, args[i], 10);
                std.debug.print("Target FPS set to {d}\n", .{options.target_fps});
            }
        } else if (std.mem.eql(u8, arg, "--help") or std.mem.eql(u8, arg, "-h")) {
            printHelp();
        }
    }

    return options;
}

/// Print help information about command-line arguments
pub fn printHelp() void {
    std.debug.print(
        \\ZigPhysics Engine Command Line Options:
        \\  --debug, -d            Enable debug mode (shows forces and vectors)
        \\  --collision-logging, -c Enable collision logging
        \\  --draw-forces, -f      Enable force vector visualization
        \\  --draw-normals, -n     Enable normal vector visualization
        \\  --no-velocities        Disable velocity vector visualization
        \\  --fps, -r <number>     Set target framerate
        \\  --help, -h             Display this help text
        \\
    , .{});
}
