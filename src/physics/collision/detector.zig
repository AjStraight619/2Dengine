const std = @import("std");
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const Shape = @import("../shapes/shape.zig").Shape;
const Circle = @import("../shapes/shape.zig").Circle;
const Rectangle = @import("../shapes/shape.zig").Rectangle;
const RigidBody = @import("../body/body.zig").RigidBody;
const testCollision = @import("sat.zig").testCollision;
const SATResult = @import("sat.zig").SATResult;

/// Collision information
pub const Collision = struct {
    /// Direction of collision (points from body A to body B)
    normal: Vector2,

    /// Penetration depth
    depth: f32,

    /// Contact points in world space (up to 2 for 2D shapes)
    contact_points: [2]Vector2,

    /// Number of valid contact points
    contact_count: u8,

    /// Restitution coefficient for this collision
    restitution: f32,

    /// Friction coefficient for this collision
    friction: f32,
};

/// Collision detection system
pub const CollisionDetector = struct {
    /// Detect collision between two rigid bodies
    pub fn detectCollision(a: *RigidBody, b: *RigidBody) ?Collision {
        // First check AABBs for early out
        if (!aabbOverlap(a.aabb, b.aabb)) {
            return null;
        }

        // Perform detailed collision detection using SAT
        const sat_result = testCollision(a, b);

        // If no collision detected, return null
        if (!sat_result.collision) {
            return null;
        }

        // Calculate material properties for the collision
        const materials = calculateMaterialProperties(a, b);

        // Convert SAT result to Collision format
        return Collision{
            .normal = sat_result.mtv,
            .depth = sat_result.depth,
            .contact_points = sat_result.contact_points,
            .contact_count = sat_result.contact_count,
            .restitution = materials.restitution,
            .friction = materials.friction,
        };
    }

    /// Check if two AABBs overlap (for broadphase)
    fn aabbOverlap(a: anytype, b: anytype) bool {
        return !(a.max_x < b.min_x or
            a.min_x > b.max_x or
            a.max_y < b.min_y or
            a.min_y > b.max_y);
    }

    /// Calculate material properties for collision response
    fn calculateMaterialProperties(a: *const RigidBody, b: *const RigidBody) struct { restitution: f32, friction: f32 } {
        return .{
            // Use the lower of the two restitution values for a more stable simulation
            .restitution = @min(a.restitution, b.restitution),

            // Use the geometric mean for friction
            .friction = @sqrt(a.friction * b.friction),
        };
    }
};

// For backward compatibility
pub const Detector = CollisionDetector;
