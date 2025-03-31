const std = @import("std");
const rl = @import("raylib");
const ze = @import("zigengine_lib");

// Example: Circle vs Rectangle Collision
// This example demonstrates proper collision detection and resolution between circles and rectangles

pub fn main() !void {
    // Initialize allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Parse command-line arguments
    const options = try ze.core.args.parseArgs(allocator);

    // Initialize the engine with the parsed options
    var game_engine = try ze.core.Engine.init(allocator, 800, 600, "ZigEngine Circle vs Rectangle Collision Example", options);
    defer game_engine.deinit();

    // Enable collision debugging for better visualization
    game_engine.physics_world.setDebugDrawCollisions(true);
    game_engine.physics_world.setDebugDrawContacts(true);
    game_engine.physics_world.setCollisionLogging(true);
    game_engine.physics_world.gravity = ze.math.Vector2.init(0, 980);

    // Increase position iterations for more stable collisions
    game_engine.physics_world.position_iterations = 4;
    game_engine.physics_world.velocity_iterations = 8;

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

    // Add some obstacles (static rectangles) to test collision
    // _ = try game_engine.physics_world.static().rectangle(.{
    //     .position = ze.math.Vector2.init(300, 400),
    //     .width = 100,
    //     .height = 20,
    //     .restitution = 0.5,
    //     .friction = 0.2,
    // });

    // _ = try game_engine.physics_world.static().rectangle(.{
    //     .position = ze.math.Vector2.init(500, 300),
    //     .width = 100,
    //     .height = 20,
    //     .restitution = 0.5,
    //     .friction = 0.2,
    // });

    // // Add a large test rectangle to clearly show circle collisions
    // _ = try game_engine.physics_world.static().rectangle(.{
    //     .position = ze.math.Vector2.init(400, 200),
    //     .width = 200,
    //     .height = 30,
    //     .restitution = 0.7,
    //     .friction = 0.1,
    // });

    // Add a bouncing circle
    _ = try game_engine.physics_world.dynamic().circle(.{
        .position = ze.math.Vector2.init(200, 200),
        .radius = 20,
        .mass = 1.0,
        .restitution = 0.7,
        .friction = 0.2,
        .velocity = ze.math.Vector2.init(0, 0),
    });

    // Run the engine with our custom input and update handlers
    try game_engine.run(@ptrCast(&game_engine), handleInput, update);
}

// Input handler callback
fn handleInput(ctx: *anyopaque, eng: *ze.core.Engine) !void {
    _ = eng;
    const self = @as(*ze.core.Engine, @alignCast(@ptrCast(ctx)));

    // Check for keyboard input to modify circles
    if (rl.isKeyPressed(rl.KeyboardKey.space)) {
        // Launch a new circle from the mouse position
        const mouse_pos = rl.getMousePosition();
        _ = try self.physics_world.dynamic().circle(.{
            .position = ze.math.Vector2.init(mouse_pos.x, mouse_pos.y),
            .radius = 15 + @as(f32, @floatFromInt(rl.getRandomValue(0, 10))),
            .mass = 1.0,
            .restitution = 0.7,
            .friction = 0.2,
        });
    }

    // Add vertical force (jump) with up arrow
    if (rl.isKeyDown(rl.KeyboardKey.up)) {
        for (self.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyForce(ze.math.Vector2.init(0, -1000));
            }
        }
    }

    // Add horizontal forces with left/right arrows
    if (rl.isKeyDown(rl.KeyboardKey.right)) {
        for (self.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyForce(ze.math.Vector2.init(300, 0));
            }
        }
    }

    if (rl.isKeyDown(rl.KeyboardKey.left)) {
        for (self.physics_world.bodies.items) |body| {
            if (body.body_type == .dynamic) {
                body.applyForce(ze.math.Vector2.init(-300, 0));
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

        // Add a new circle
        _ = try self.physics_world.dynamic().circle(.{
            .position = ze.math.Vector2.init(200, 100),
            .radius = 20,
            .mass = 1.0,
            .restitution = 0.7,
            .friction = 0.2,
        });
    }

    // Display instructions
    if (rl.isKeyPressed(rl.KeyboardKey.h)) {
        std.debug.print("\n--- Controls ---\n", .{});
        std.debug.print("SPACE: Launch a new circle at mouse X position\n", .{});
        std.debug.print("LEFT/RIGHT: Apply horizontal force\n", .{});
        std.debug.print("UP: Apply upward force (jump)\n", .{});
        std.debug.print("R: Reset simulation\n", .{});
        std.debug.print("H: Show this help\n", .{});
    }
}

// Update callback
fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
    _ = ctx;
    _ = eng;
    _ = dt;

    // Draw instructions
    rl.drawText("SPACE: Launch circle | LEFT/RIGHT: Move | UP: Jump | R: Reset | H: Help", 10, 10, 20, rl.Color.white);
}
