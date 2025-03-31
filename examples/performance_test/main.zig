const std = @import("std");
const rl = @import("raylib");
const ze = @import("zigengine_lib");

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
};

pub fn main() !void {
    // Initialize allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Parse command-line arguments
    const options = try ze.core.args.parseArgs(allocator);

    // Initialize the engine with the parsed options
    var game_engine = try ze.core.Engine.init(allocator, 800, 600, "Physics Performance Test", options);
    defer game_engine.deinit();

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
    if (rl.isKeyPressed(rl.KeyboardKey.g)) {
        const grid_enabled = !self.physics_world.broadphase.use_grid;
        self.physics_world.broadphase.setUseGrid(grid_enabled);
        std.debug.print("Grid optimization: {}\n", .{grid_enabled});
    }

    // Toggle object sleeping with S key
    if (rl.isKeyPressed(rl.KeyboardKey.s)) {
        const sleeping_enabled = !self.physics_world.enable_sleeping;
        self.physics_world.setEnableSleeping(sleeping_enabled);
        std.debug.print("Object sleeping: {}\n", .{sleeping_enabled});
    }

    // Change substeps with - and + keys
    if (rl.isKeyPressed(rl.KeyboardKey.minus)) {
        if (self.physics_world.substeps > 1) {
            self.physics_world.setSubsteps(self.physics_world.substeps - 1);
            std.debug.print("Substeps: {}\n", .{self.physics_world.substeps});
        }
    }

    if (rl.isKeyPressed(rl.KeyboardKey.equal)) {
        self.physics_world.setSubsteps(self.physics_world.substeps + 1);
        std.debug.print("Substeps: {}\n", .{self.physics_world.substeps});
    }

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
        std.debug.print("\n--- Performance Info ---\n", .{});
        std.debug.print("Objects: {d}\n", .{object_count});
        std.debug.print("FPS: {d}\n", .{rl.getFPS()});
        std.debug.print("Collisions per frame: {d}\n", .{self.physics_world.collision_count});
    }
}

// Update callback with performance metrics
fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
    _ = eng;
    _ = dt;
    const self = @as(*ze.core.Engine, @alignCast(@ptrCast(ctx)));

    // Display object count and FPS
    const object_count = self.physics_world.bodies.items.len;
    const fps = rl.getFPS();
    const collisions = self.physics_world.collision_count;
    const sleeping_count = countSleepingBodies(self);
    const grid_mode = if (self.physics_world.broadphase.use_grid) "Grid" else "Brute-force";
    const sleeping_enabled = if (self.physics_world.enable_sleeping) "On" else "Off";

    // Draw the performance info
    var info_buf: [100]u8 = undefined;
    const info_text = std.fmt.bufPrintZ(&info_buf, "Objects: {d} | FPS: {d} | Collisions: {d} | Sleeping: {d}", .{ object_count, fps, collisions, sleeping_count }) catch "Error";
    rl.drawText(info_text, 10, 10, 20, rl.Color.white);

    // Draw optimization status
    var opt_buf: [100]u8 = undefined;
    const opt_text = std.fmt.bufPrintZ(&opt_buf, "Grid: {s} | Sleeping: {s} | Substeps: {d}", .{ grid_mode, sleeping_enabled, self.physics_world.substeps }) catch "Error";
    rl.drawText(opt_text, 10, 35, 20, rl.Color.white);

    // Draw controls
    rl.drawText("1/2/3: Add objects | G: Toggle grid | S: Toggle sleeping | +/-: Change substeps | R: Reset", 10, 60, 18, rl.Color.light_gray);

    // Color code the FPS
    var fps_color = rl.Color.green;
    if (fps < 30) fps_color = rl.Color.red else if (fps < 60) fps_color = rl.Color.yellow;

    var fps_buf: [20]u8 = undefined;
    const fps_text = std.fmt.bufPrintZ(&fps_buf, "FPS: {d}", .{fps}) catch "Error";
    rl.drawText(fps_text, 700, 10, 30, fps_color);
}

// Helper function to count sleeping bodies
fn countSleepingBodies(engine: *ze.core.Engine) usize {
    var count: usize = 0;
    for (engine.physics_world.bodies.items) |body| {
        if (body.is_sleeping) count += 1;
    }
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

        // Add a random shape - 50% chance of circle, 50% chance of rectangle
        if (rl.getRandomValue(0, 1) == 0) {
            // Add rectangle
            _ = try engine.physics_world.dynamic().rectangle(.{
                .position = ze.math.Vector2.init(x, y),
                .width = size,
                .height = size,
                .mass = 1.0,
                .restitution = 0.3 + @as(f32, @floatFromInt(rl.getRandomValue(0, 6))) / 10.0, // 0.3-0.9
                .friction = 0.1 + @as(f32, @floatFromInt(rl.getRandomValue(0, 8))) / 10.0, // 0.1-0.9
            });
        } else {
            // Add circle
            _ = try engine.physics_world.dynamic().circle(.{
                .position = ze.math.Vector2.init(x, y),
                .radius = size / 2.0, // Half the rectangle size
                .mass = 1.0,
                .restitution = 0.3 + @as(f32, @floatFromInt(rl.getRandomValue(0, 6))) / 10.0, // 0.3-0.9
                .friction = 0.1 + @as(f32, @floatFromInt(rl.getRandomValue(0, 8))) / 10.0, // 0.1-0.9
            });
        }
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
