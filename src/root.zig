//! ZigEngine - A 2D physics engine written in Zig
//!
//! This file is the main entry point for the library.

const std = @import("std");
const engine = @import("mod.zig");

// Re-export the core engine types and functionality in organized namespaces
pub const core = struct {
    pub const Engine = engine.Engine;
    pub const args = engine.core.args;
    pub const debug = engine.core.debug;
    pub const debug_helper = engine.core.debug_helper;
};

pub const math = struct {
    pub const Vector2 = engine.physics.Vector2;
};

pub const physics = struct {
    pub const World = engine.physics.PhysicsWorld;
    pub const BodyType = engine.physics.BodyType;
    pub const RigidBody = engine.physics.RigidBody;
    pub const Shape = engine.physics.Shape;
    pub const Circle = engine.physics.Circle;
    pub const Rectangle = engine.physics.Rectangle;
    pub const Forces = engine.physics.Forces;

    // Body creation options
    pub const StaticCircleOptions = engine.physics.world.StaticCircleOptions;
    pub const StaticRectangleOptions = engine.physics.world.StaticRectangleOptions;
    pub const DynamicCircleOptions = engine.physics.world.DynamicCircleOptions;
    pub const DynamicRectangleOptions = engine.physics.world.DynamicRectangleOptions;
    pub const KinematicCircleOptions = engine.physics.world.KinematicCircleOptions;
    pub const KinematicRectangleOptions = engine.physics.world.KinematicRectangleOptions;
};

pub const rendering = struct {
    pub const PhysicsRenderer = engine.renderer.PhysicsRenderer;
};

pub const input = struct {
    pub const InputManager = engine.input.InputManager;
};

// Test exports (can be removed or hidden in production)
pub export fn add(a: i32, b: i32) i32 {
    return a + b;
}

test "basic add functionality" {
    try testing.expect(add(3, 7) == 10);
}

const testing = std.testing;
