const std = @import("std");
const rl = @import("raylib");
const ze = @import("zigengine_lib");

// Example: Vertical Rectangle Collision
// This example demonstrates vertical collision detection and resolution between rectangles

pub fn main() !void {
    // Initialize allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Parse command-line arguments
    const options = try ze.core.args.parseArgs(allocator);

    // Initialize the engine with the parsed options
    var game_engine = try ze.core.Engine.init(allocator, 800, 600, "Vertical Rectangle Collision Test", options);
    defer game_engine.deinit();

    // Enable collision debugging for better visualization
    game_engine.physics_world.setDebugDrawCollisions(true);
    game_engine.physics_world.setDebugDrawContacts(true);
    game_engine.physics_world.setCollisionLogging(true);

    // Set normal gravity for vertical testing
    game_engine.physics_world.gravity = ze.math.Vector2.init(0, 980);

    // Increase position iterations for more stable collisions
    game_engine.physics_world.position_iterations = 6;
    game_engine.physics_world.velocity_iterations = 8;

    // Create the floor
    _ = try game_engine.physics_world.addRectangle(.{
        .type = .static,
        .position = ze.math.Vector2.init(400, 550),
        .width = 800,
        .height = 20,
        .restitution = 0.3,
        .friction = 0.2,
    });

    // Left wall
    _ = try game_engine.physics_world.addRectangle(.{
        .type = .static,
        .position = ze.math.Vector2.init(10, 300),
        .width = 20,
        .height = 600,
        .restitution = 0.3,
        .friction = 0.2,
    });

    // Right wall
    _ = try game_engine.physics_world.addRectangle(.{
        .type = .static,
        .position = ze.math.Vector2.init(790, 300),
        .width = 20,
        .height = 600,
        .restitution = 0.3,
        .friction = 0.2,
    });

    // Create horizontal platforms at different heights
    _ = try game_engine.physics_world.addRectangle(.{
        .type = .static,
        .position = ze.math.Vector2.init(250, 400),
        .width = 200,
        .height = 20,
        .restitution = 0.3,
        .friction = 0.2,
    });

    _ = try game_engine.physics_world.addRectangle(.{
        .type = .static,
        .position = ze.math.Vector2.init(550, 250),
        .width = 200,
        .height = 20,
        .restitution = 0.3,
        .friction = 0.2,
    });

    // Create a dynamic rectangle that will fall and collide vertically
    _ = try game_engine.physics_world.addCircle(.{
        .type = .dynamic,
        .position = ze.math.Vector2.init(250, 100),
        .radius = 20,
        .mass = 1.0,
        .restitution = 0.3,
        .friction = 0.2,
    });

    // Create another dynamic rectangle on a higher platform
    _ = try game_engine.physics_world.addCircle(.{
        .type = .dynamic,
        .position = ze.math.Vector2.init(550, 100),
        .radius = 20,
        .mass = 1.0,
        .restitution = 0.3,
        .friction = 0.2,
    });

    // Run the engine with our custom input and update handlers
    try game_engine.run(@ptrCast(&game_engine), handleInput, update);
}

// Input handler callback
fn handleInput(ctx: *anyopaque, eng: *ze.core.Engine) !void {
    _ = eng;
    const self = @as(*ze.core.Engine, @alignCast(@ptrCast(ctx)));

    // Create a new rectangle at mouse position on click
    if (rl.isMouseButtonPressed(rl.MouseButton.left)) {
        const mouse_pos = rl.getMousePosition();
        _ = try self.physics_world.addRectangle(.{
            .type = .dynamic,
            .position = ze.math.Vector2.init(mouse_pos.x, mouse_pos.y),
            .width = 40,
            .height = 40,
            .mass = 1.0,
            .restitution = 0.3,
            .friction = 0.2,
        });
    }

    // Add vertical force (jump) with spacebar
    if (rl.isKeyPressed(rl.KeyboardKey.space)) {
        for (self.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyImpulse(ze.math.Vector2.init(0, -300), null);
            }
        }
    }

    // Add horizontal movements with left/right arrows
    if (rl.isKeyDown(rl.KeyboardKey.right)) {
        for (self.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyForce(ze.math.Vector2.init(100, 0));
            }
        }
    }

    if (rl.isKeyDown(rl.KeyboardKey.left)) {
        for (self.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyForce(ze.math.Vector2.init(-100, 0));
            }
        }
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

        // Add two new rectangles at starting positions
        _ = try self.physics_world.addRectangle(.{
            .type = .dynamic,
            .position = ze.math.Vector2.init(250, 100),
            .width = 40,
            .height = 40,
            .mass = 1.0,
            .restitution = 0.3,
            .friction = 0.2,
        });

        _ = try self.physics_world.addRectangle(.{
            .type = .dynamic,
            .position = ze.math.Vector2.init(550, 100),
            .width = 40,
            .height = 40,
            .mass = 1.0,
            .restitution = 0.3,
            .friction = 0.2,
        });
    }
}

// Update callback
fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
    _ = ctx;
    _ = eng;
    _ = dt;

    // Draw instructions
    rl.drawText("SPACE: Jump | LEFT/RIGHT: Move | CLICK: Create box | R: Reset", 10, 10, 20, rl.Color.white);
    rl.drawText("VERTICAL COLLISION TEST (WITH GRAVITY)", 200, 570, 20, rl.Color.white);
}
