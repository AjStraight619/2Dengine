const std = @import("std");
const rl = @import("raylib");
const ze = @import("zigengine_lib");
const detector = @import("../../src/physics/collision/detector.zig");

// Performance test for physics simulation
// This file helps identify performance bottlenecks in the physics engine

// Structure to hold performance metrics
const PerformanceMetrics = struct {
    frame_time: f64 = 0,
    physics_update_time: f64 = 0,
    broadphase_time: f64 = 0,
    narrowphase_time: f64 = 0,
    integration_time: f64 = 0,
    collision_count: usize = 0,
    body_count: usize = 0,
    sleeping_bodies: usize = 0,
};

// Global counters to track physics statistics
var g_total_circles: usize = 0;
var g_sleeping_bodies: usize = 0;

pub fn main() !void {
    // Initialize allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Setup signal handler to catch SIGINT for clean shutdown
    const print_stats = std.posix.Sigaction{
        .handler = .{ .handler = sigHandler },
        .mask = std.posix.empty_sigset,
        .flags = 0,
    };
    _ = std.posix.sigaction(std.posix.SIG.INT, &print_stats, null);

    // Parse command-line arguments
    const options = try ze.core.args.parseArgs(allocator);

    // Initialize the engine with the parsed options
    var game_engine = try ze.core.Engine.init(allocator, 800, 600, "Physics Performance Test", options);
    defer {
        // Print collision stats on exit
        std.debug.print("\n\n--- FINAL COLLISION STATISTICS ---\n", .{});
        std.debug.print("Total bodies: {d}\n", .{game_engine.physics_world.bodies.items.len});
        std.debug.print("Last frame collisions: {d}\n", .{game_engine.physics_world.collision_count});
        std.debug.print("Dynamic bodies: {d}\n", .{g_total_circles});
        std.debug.print("Sleeping bodies: {d}\n", .{countSleepingBodies(&game_engine)});
        std.debug.print("Total contact events: {d}\n", .{detector.total_collision_count});
        std.debug.print("Significant impacts: {d}\n", .{detector.significant_collision_count});
        std.debug.print("Resting contacts skipped: {d}\n", .{detector.resting_contact_count});
        if (detector.unstable_friction_count > 0) {
            std.debug.print("⚠️ Unstable friction events: {d}\n", .{detector.unstable_friction_count});
        }
        std.debug.print("-----------------------------\n\n", .{});
        game_engine.deinit();
    }

    // Configure physics world
    game_engine.physics_world.position_iterations = 4;
    game_engine.physics_world.velocity_iterations = 8;
    game_engine.physics_world.setCollisionLogging(false); // Disable logging for performance

    // Create the floor and walls
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(400, 550),
        .width = 700,
        .height = 20,
        .restitution = 0.3,
        .friction = 0.5,
    });

    // Left wall
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(50, 300),
        .width = 20,
        .height = 500,
        .restitution = 0.3,
        .friction = 0.5,
    });

    // Right wall
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(750, 300),
        .width = 20,
        .height = 500,
        .restitution = 0.3,
        .friction = 0.5,
    });

    // Add some initial objects for testing
    try addBatchOfObjects(&game_engine, 10);

    // Run the engine with our performance monitoring handlers
    try game_engine.run(@ptrCast(&game_engine), handleInput, update);
}

// Signal handler for clean shutdown
fn sigHandler(sig: c_int) callconv(.C) void {
    std.debug.print("\nReceived signal {d}, exiting cleanly.\n", .{sig});

    // Print collision stats
    std.debug.print("\n\n--- FINAL COLLISION STATISTICS ---\n", .{});
    std.debug.print("Dynamic bodies: {d}\n", .{g_total_circles});
    std.debug.print("Sleeping bodies: {d}\n", .{g_sleeping_bodies});
    std.debug.print("Total contact events: {d}\n", .{detector.total_collision_count});
    std.debug.print("Significant impacts: {d}\n", .{detector.significant_collision_count});
    std.debug.print("Resting contacts skipped: {d}\n", .{detector.resting_contact_count});
    if (detector.unstable_friction_count > 0) {
        std.debug.print("⚠️ Unstable friction events: {d}\n", .{detector.unstable_friction_count});
    }
    std.debug.print("-----------------------------\n\n", .{});

    std.posix.exit(0);
}

// Input handler callback
fn handleInput(ctx: *anyopaque, eng: *ze.core.Engine) !void {
    _ = eng;
    const self = @as(*ze.core.Engine, @alignCast(@ptrCast(ctx)));

    // Add a batch of objects with 1 key
    if (rl.isKeyPressed(rl.KeyboardKey.one)) {
        try addBatchOfObjects(self, 10); // Add 10 objects
        std.debug.print("Added 10 objects\n", .{});
    }

    // Add a larger batch of objects with 2 key
    if (rl.isKeyPressed(rl.KeyboardKey.two)) {
        try addBatchOfObjects(self, 50); // Add 50 objects
        std.debug.print("Added 50 objects\n", .{});
    }

    // Add a stress test amount with 3 key
    if (rl.isKeyPressed(rl.KeyboardKey.three)) {
        try addBatchOfObjects(self, 200); // Add 200 objects
        std.debug.print("Added 200 objects\n", .{});
    }

    // Toggle grid-based optimization with G key
    if (rl.isKeyPressed(rl.KeyboardKey.g)) {}

    // Toggle object sleeping with S key
    if (rl.isKeyPressed(rl.KeyboardKey.s)) {}

    // Change substeps with - and + keys
    if (rl.isKeyPressed(rl.KeyboardKey.minus)) {}

    if (rl.isKeyPressed(rl.KeyboardKey.equal)) {}

    // Reset simulation with R key
    if (rl.isKeyPressed(rl.KeyboardKey.r)) {
        clearAllDynamicBodies(self);
    }

    // Toggle debug visualization with D key
    if (rl.isKeyPressed(rl.KeyboardKey.d)) {
        self.physics_renderer.debug_mode = !self.physics_renderer.debug_mode;
        self.physics_world.setDebugDrawCollisions(self.physics_renderer.debug_mode);
        self.physics_world.setDebugDrawContacts(self.physics_renderer.debug_mode);
    }

    // Display performance information with P key
    if (rl.isKeyPressed(rl.KeyboardKey.p)) {
        const object_count = self.physics_world.bodies.items.len;
        const sleeping_count = countSleepingBodies(self);
        std.debug.print("\n--- Performance Info ---\n", .{});
        std.debug.print("Objects: {d}\n", .{object_count});
        std.debug.print("Sleeping: {d}/{d} dynamic objects\n", .{ sleeping_count, g_total_circles });
        std.debug.print("FPS: {d}\n", .{rl.getFPS()});
        std.debug.print("Collisions per frame: {d}\n", .{self.physics_world.collision_count});
        std.debug.print("Contacts skipped: {d}\n", .{detector.resting_contact_count});
    }
}

// Update callback with performance metrics
fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
    _ = eng;
    _ = dt;
    _ = @as(*ze.core.Engine, @alignCast(@ptrCast(ctx)));
}

// Helper function to count sleeping bodies
fn countSleepingBodies(engine: *ze.core.Engine) usize {
    var count: usize = 0;
    for (engine.physics_world.bodies.items) |body| {
        if (body.body_type == .dynamic and body.is_sleeping) {
            count += 1;
        }
    }
    g_sleeping_bodies = count; // Update global counter
    return count;
}

// Add a batch of random objects to the simulation
fn addBatchOfObjects(engine: *ze.core.Engine, count: usize) !void {
    for (0..count) |_| {
        // Randomize initial position in a reasonable area
        const x = @as(f32, @floatFromInt(rl.getRandomValue(100, 700)));
        const y = @as(f32, @floatFromInt(rl.getRandomValue(50, 200)));

        // Random size between 15 and 40
        const size = 15.0 + @as(f32, @floatFromInt(rl.getRandomValue(0, 25)));

        // Add circle
        _ = try engine.physics_world.dynamic().circle(.{
            .position = ze.math.Vector2.init(x, y),
            .radius = size / 2.0, // Half the rectangle size
            .mass = 1.0,
            .restitution = 0.3 + @as(f32, @floatFromInt(rl.getRandomValue(0, 6))) / 10.0, // 0.3-0.9
            .friction = 0.1 + @as(f32, @floatFromInt(rl.getRandomValue(0, 8))) / 10.0, // 0.1-0.9
        });

        // Increment our counter for statistics
        g_total_circles += 1;
    }
}

// Clear all dynamic bodies from the simulation
fn clearAllDynamicBodies(engine: *ze.core.Engine) void {
    var i: usize = 0;
    while (i < engine.physics_world.bodies.items.len) {
        const body = engine.physics_world.bodies.items[i];
        if (body.body_type == .dynamic) {
            _ = engine.physics_world.bodies.swapRemove(i);
            engine.physics_world.allocator.destroy(body);
        } else {
            i += 1;
        }
    }
}
