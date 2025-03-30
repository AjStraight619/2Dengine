const RigidBody = @import("../body/body.zig").RigidBody;
const Vector2 = @import("../geometry/vector2.zig").Vector2;

pub const Forces = struct {
    pub fn applyGravity(body: *RigidBody, gravity: Vector2) void {
        if (body.body_type != .dynamic) return;
        const force = gravity.scale(body.mass);
        body.applyForce(force);
    }

    pub fn applyDrag(body: *RigidBody, coefficient: f32) void {
        if (body.body_type != .dynamic) return;
        const force = body.velocity.scale(-coefficient);
        body.applyForce(force);
    }

    // Other force functions...
};
