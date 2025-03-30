// ZigEngine - Main module exports
pub const core = @import("core/mod.zig");
pub const physics = @import("physics/mod.zig");
pub const input = @import("input/mod.zig");
pub const renderer = @import("renderer/mod.zig");

// Convenience re-exports of common types
pub const Engine = core.Engine;
pub const Vector2 = physics.Vector2;
pub const InputManager = input.InputManager;
pub const PhysicsRenderer = renderer.PhysicsRenderer;
