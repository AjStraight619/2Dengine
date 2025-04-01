const std = @import("std");
const rl = @import("raylib");
const ze = @import("zigengine_lib");

// Example 1: Basic physics demonstration
// This example shows how to create a simple physics scene with a bouncing circle

pub fn main() !void {
    // Initialize allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Parse command-line arguments
    const options = try ze.core.args.parseArgs(allocator);

    // Initialize the engine with the parsed options
    var game_engine = try ze.core.Engine.init(allocator, 800, 600, "ZigEngine Basic Example", options);
    defer game_engine.deinit();

    // Create a bouncing circle with the unified options struct
    const circle = try game_engine.physics_world.addCircle(.{
        .type = .dynamic,
        .position = ze.math.Vector2.init(400, 100),
        .radius = 20,
        .mass = 1.0,
        .restitution = 0.7,
        .friction = 0.2,
        .initial_force = ze.math.Vector2.init(0, 300),
    });
    _ = circle; // Use the circle variable to avoid unused warning

    // Add static floor and walls with the unified options structs
    _ = try game_engine.physics_world.addRectangle(.{
        .type = .static,
        .position = ze.math.Vector2.init(400, 200),
        .width = 200,
        .height = 30,
        .angle_degrees = 30.0,
        .restitution = 0.7,
        .friction = 0.1,
    });

    _ = try game_engine.physics_world.addRectangle(.{
        .type = .static,
        .position = ze.math.Vector2.init(50, 300),
        .width = 20,
        .height = 600,
        .restitution = 0.5,
        .friction = 0.1,
    });

    _ = try game_engine.physics_world.addRectangle(.{
        .type = .static,
        .position = ze.math.Vector2.init(750, 300),
        .width = 20,
        .height = 600,
        .restitution = 0.5,
        .friction = 0.1,
    });

    // Run the engine with our custom input and update handlers
    try game_engine.run(@ptrCast(&game_engine), handleInput, update);
}

// Input handler callback
fn handleInput(ctx: *anyopaque, eng: *ze.core.Engine) !void {
    _ = eng;
    const self = @as(*ze.core.Engine, @alignCast(@ptrCast(ctx)));

    // Check for keyboard input to modify circle behavior
    if (rl.isKeyPressed(rl.KeyboardKey.space)) {
        // Get the first body which should be our circle
        if (self.physics_world.bodies.items.len > 0) {
            const circle = self.physics_world.bodies.items[0];

            // Apply an upward impulse
            circle.applyImpulse(ze.math.Vector2.init(0, -500), null);
            std.debug.print("Jump impulse applied!\n", .{});
        }
    }

    // Add horizontal force with left/right arrow keys
    const horizontal_force = 200.0;
    if (rl.isKeyDown(rl.KeyboardKey.right)) {
        if (self.physics_world.bodies.items.len > 0) {
            const circle = self.physics_world.bodies.items[0];
            circle.applyForce(ze.math.Vector2.init(horizontal_force, 0));
        }
    }

    if (rl.isKeyDown(rl.KeyboardKey.left)) {
        if (self.physics_world.bodies.items.len > 0) {
            const circle = self.physics_world.bodies.items[0];
            circle.applyForce(ze.math.Vector2.init(-horizontal_force, 0));
        }
    }
}

// Update callback
fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
    _ = ctx;
    _ = eng;
    _ = dt;
    // Additional custom update logic could go here
}
