const std = @import("std");
const rl = @import("raylib");
const phys = @import("../physics/mod.zig");
const geometry = @import("../physics/geometry/mod.zig");
const ShapeRenderer = @import("shape_renderer.zig").ShapeRenderer;
const DebugRenderer = @import("debug_renderer.zig").DebugRenderer;

/// Main physics rendering system that handles both shapes and debug visualization
pub const PhysicsRenderer = struct {
    // Debug drawing flags
    draw_velocities: bool = true,
    draw_forces: bool = false,
    draw_normals: bool = false,
    draw_aabbs: bool = false,
    debug_mode: bool = false,

    // Child renderers
    shape_renderer: ShapeRenderer = ShapeRenderer{},
    debug_renderer: DebugRenderer = DebugRenderer{},

    /// Initialize a new physics renderer
    pub fn init() PhysicsRenderer {
        return PhysicsRenderer{};
    }

    /// Draw a single physics body
    pub fn drawBody(self: PhysicsRenderer, body: *phys.RigidBody) void {
        // Draw the shape itself
        self.shape_renderer.drawBody(body);

        // ALWAYS draw normals and AABBs if enabled, regardless of debug mode
        if (self.draw_normals) {
            // Debug output when drawing normals to trace execution
            std.debug.print("Drawing normals for body at ({d:.1},{d:.1})\n", .{ body.position.x, body.position.y });
            self.debug_renderer.drawNormalVectors(body);
        }

        if (self.draw_aabbs) {
            // Debug output when drawing AABBs to trace execution
            std.debug.print("Drawing AABB for body at ({d:.1},{d:.1})\n", .{ body.position.x, body.position.y });
            self.debug_renderer.drawAABB(body);
        }

        // Only show forces and velocities in debug mode (since they're more performance-intensive)
        if (self.debug_mode) {
            // Draw velocity vector if enabled
            if (self.draw_velocities and body.body_type != .static) {
                self.debug_renderer.drawVelocityVector(body);
            }

            // Draw force vectors if enabled
            if (self.draw_forces and body.body_type != .static) {
                self.debug_renderer.drawForceVector(body);
            }
        }
    }

    /// Draw all bodies in a physics world
    pub fn drawWorld(self: PhysicsRenderer, world: phys.PhysicsWorld) void {
        for (world.bodies.items) |body| {
            self.drawBody(body);
        }
    }

    /// Draw a coordinate grid
    pub fn drawGrid(self: PhysicsRenderer, width: i32, height: i32, cell_size: i32, color: rl.Color) void {
        _ = self;
        DebugRenderer.drawGrid(width, height, cell_size, color);
    }

    /// Set debug mode on/off
    pub fn setDebugMode(self: *PhysicsRenderer, enabled: bool) void {
        self.debug_mode = enabled;
    }

    /// Draw debug information text
    pub fn drawDebugInfo(world: phys.PhysicsWorld, paused: bool, x: i32, y: i32, color: rl.Color) void {
        DebugRenderer.drawDebugInfo(world, paused, x, y, color);
    }
};

// Helper function to rotate a point
fn rotatePoint(point: phys.Vector2, sin_val: f32, cos_val: f32) phys.Vector2 {
    return phys.Vector2.init(
        point.x * cos_val - point.y * sin_val,
        point.x * sin_val + point.y * cos_val,
    );
}
