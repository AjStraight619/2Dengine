const std = @import("std");
const rl = @import("raylib");
const ze = @import("zigengine_lib");

// Example: Horizontal Rectangle Collision
// This example demonstrates horizontal collision detection and resolution between rectangles

pub fn main() !void {
    // Initialize allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Parse command-line arguments
    const options = try ze.core.args.parseArgs(allocator);

    // Initialize the engine with the parsed options
    var game_engine = try ze.core.Engine.init(allocator, 800, 600, "Horizontal Rectangle Collision Test", options);
    defer game_engine.deinit();

    // Enable collision debugging for better visualization
    game_engine.physics_world.setDebugDrawCollisions(true);
    game_engine.physics_world.setDebugDrawContacts(true);
    game_engine.physics_world.setCollisionLogging(true);

    // Set zero gravity to test pure horizontal movement
    game_engine.physics_world.gravity = ze.math.Vector2.init(0, 0);

    // Increase position iterations for more stable collisions
    game_engine.physics_world.position_iterations = 6;
    game_engine.physics_world.velocity_iterations = 8;

    // Create the floor (very thin)
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(400, 590),
        .width = 800,
        .height = 10,
        .restitution = 0.3,
        .friction = 0.2,
    });

    // Left wall
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(10, 300),
        .width = 20,
        .height = 600,
        .restitution = 0.5,
        .friction = 0.2,
    });

    // Right wall
    _ = try game_engine.physics_world.static().rectangle(.{
        .position = ze.math.Vector2.init(790, 300),
        .width = 20,
        .height = 600,
        .restitution = 0.5,
        .friction = 0.2,
    });

    // Create a static rectangle in the middle
    // _ = try game_engine.physics_world.static().rectangle(.{
    //     .position = ze.math.Vector2.init(400, 300),
    //     .width = 40,
    //     .height = 40,
    //     .restitution = 0.5,
    //     .friction = 0.2,
    // });

    // Add a dynamic rectangle that will collide horizontally
    _ = try game_engine.physics_world.dynamic().rectangle(.{
        .position = ze.math.Vector2.init(200, 300),
        .width = 40,
        .height = 40,
        .mass = 1.0,
        .restitution = 0.9, // Higher restitution for better bounce
        .friction = 0.1, // Low friction
        .velocity = ze.math.Vector2.init(200, 0), // Initial velocity to the right
    });

    // Add another dynamic rectangle on the other side
    _ = try game_engine.physics_world.dynamic().rectangle(.{
        .position = ze.math.Vector2.init(600, 300),
        .width = 40,
        .height = 40,
        .mass = 1.0,
        .restitution = 0.9,
        .friction = 0.1,
        .velocity = ze.math.Vector2.init(-200, 0), // Initial velocity to the left
    });

    // Run the engine with our custom input and update handlers
    try game_engine.run(@ptrCast(&game_engine), handleInput, update);
}

// Input handler callback
fn handleInput(ctx: *anyopaque, eng: *ze.core.Engine) !void {
    _ = eng;
    const self = @as(*ze.core.Engine, @alignCast(@ptrCast(ctx)));

    // Add horizontal forces with left/right arrows
    if (rl.isKeyDown(rl.KeyboardKey.right)) {
        for (self.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyForce(ze.math.Vector2.init(500, 0));
            }
        }
    }

    if (rl.isKeyDown(rl.KeyboardKey.left)) {
        for (self.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyForce(ze.math.Vector2.init(-500, 0));
            }
        }
    }

    // Launch a new rectangle from the right or left with A/D keys
    if (rl.isKeyPressed(rl.KeyboardKey.a)) {
        // Add a new rectangle from the left
        _ = try self.physics_world.dynamic().rectangle(.{
            .position = ze.math.Vector2.init(50, 300),
            .width = 40,
            .height = 40,
            .mass = 1.0,
            .restitution = 0.9,
            .friction = 0.1,
            .velocity = ze.math.Vector2.init(300, 0), // Initial velocity to the right
        });
    }

    if (rl.isKeyPressed(rl.KeyboardKey.d)) {
        // Add a new rectangle from the right
        _ = try self.physics_world.dynamic().rectangle(.{
            .position = ze.math.Vector2.init(750, 300),
            .width = 40,
            .height = 40,
            .mass = 1.0,
            .restitution = 0.9,
            .friction = 0.1,
            .velocity = ze.math.Vector2.init(-300, 0), // Initial velocity to the left
        });
    }

    // Reset simulation with R key
    if (rl.isKeyPressed(rl.KeyboardKey.r)) {
        // Clear all dynamic bodies
        var i: usize = 0;
        while (i < self.physics_world.bodies.items.len) {
            const body = self.physics_world.bodies.items[i];
            if (body.body_type == .dynamic) {
                _ = self.physics_world.bodies.swapRemove(i);
                self.physics_world.allocator.destroy(body);
            } else {
                i += 1;
            }
        }

        // Add two new rectangles
        _ = try self.physics_world.dynamic().rectangle(.{
            .position = ze.math.Vector2.init(200, 300),
            .width = 40,
            .height = 40,
            .mass = 1.0,
            .restitution = 0.9,
            .friction = 0.1,
            .velocity = ze.math.Vector2.init(200, 0),
        });

        _ = try self.physics_world.dynamic().rectangle(.{
            .position = ze.math.Vector2.init(600, 300),
            .width = 40,
            .height = 40,
            .mass = 1.0,
            .restitution = 0.9,
            .friction = 0.1,
            .velocity = ze.math.Vector2.init(-200, 0),
        });
    }
}

// Update callback
fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
    _ = ctx;
    _ = eng;
    _ = dt;

    // Draw instructions
    rl.drawText("LEFT/RIGHT: Apply force | A: Add box from left | D: Add box from right | R: Reset", 10, 10, 20, rl.Color.white);
    rl.drawText("HORIZONTAL COLLISION TEST (NO GRAVITY)", 200, 570, 20, rl.Color.white);
}
