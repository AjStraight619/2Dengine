const std = @import("std");
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const RigidBody = @import("../body/body.zig").RigidBody;
const AABB = @import("../geometry/aabb.zig").AABB;

/// Rectangular bounds structure for collision calculations
pub const Bounds = struct {
    left: f32,
    right: f32,
    top: f32,
    bottom: f32,

    /// Convert to AABB
    pub fn toAABB(self: Bounds) AABB {
        return AABB{
            .min_x = self.left,
            .min_y = self.top,
            .max_x = self.right,
            .max_y = self.bottom,
        };
    }

    /// Create from AABB
    pub fn fromAABB(aabb: AABB) Bounds {
        return Bounds{
            .left = aabb.min_x,
            .right = aabb.max_x,
            .top = aabb.min_y,
            .bottom = aabb.max_y,
        };
    }
};

/// Overlap information for collision resolution
pub const Overlaps = struct {
    left: f32, // Penetration from left side
    right: f32, // Penetration from right side
    top: f32, // Penetration from top side
    bottom: f32, // Penetration from bottom side
};

/// Penetration information
pub const Penetration = struct {
    normal: Vector2,
    depth: f32,
};

/// Collision utilities for shared functionality across different collision types
pub const CollisionUtils = struct {
    /// Calculate material properties for a collision between two rigid bodies
    pub fn calculateMaterialProperties(a: *const RigidBody, b: *const RigidBody) struct { restitution: f32, friction: f32 } {
        return .{
            .restitution = @min(a.restitution, b.restitution),
            .friction = @sqrt(a.friction * b.friction),
        };
    }

    /// Calculate rectangle bounds from a position and dimensions
    pub fn calculateRectangleBounds(position: Vector2, width: f32, height: f32) Bounds {
        const half_width = width * 0.5;
        const half_height = height * 0.5;

        return Bounds{
            .left = position.x - half_width,
            .right = position.x + half_width,
            .top = position.y - half_height,
            .bottom = position.y + half_height,
        };
    }

    /// Calculate circle bounds from a position and radius
    pub fn calculateCircleBounds(position: Vector2, radius: f32) Bounds {
        return Bounds{
            .left = position.x - radius,
            .right = position.x + radius,
            .top = position.y - radius,
            .bottom = position.y + radius,
        };
    }

    /// Check if two axis-aligned bounding boxes (AABBs) intersect
    pub fn checkAABBIntersection(bounds1: Bounds, bounds2: Bounds) bool {
        return !(bounds1.right < bounds2.left or
            bounds1.left > bounds2.right or
            bounds1.bottom < bounds2.top or
            bounds1.top > bounds2.bottom);
    }

    /// Calculate overlap amounts between two AABBs in all four directions
    pub fn calculateOverlaps(bounds1: Bounds, bounds2: Bounds) Overlaps {
        return Overlaps{
            .left = bounds1.right - bounds2.left, // How far object 1 penetrates from left of object 2
            .right = bounds2.right - bounds1.left, // How far object 1 penetrates from right of object 2
            .top = bounds1.bottom - bounds2.top, // How far object 1 penetrates from top of object 2
            .bottom = bounds2.bottom - bounds1.top, // How far object 1 penetrates from bottom of object 2
        };
    }

    /// Find the axis of minimum penetration from overlap values
    pub fn findMinimumPenetrationAxis(
        overlaps: Overlaps,
        circle_position: Vector2,
        rect_position: Vector2,
        is_falling: bool,
    ) Penetration {
        var smallest_overlap = overlaps.left;
        var normal = Vector2.init(-1, 0); // Left normal (points left to push circle away)

        // Check right overlap
        if (overlaps.right < smallest_overlap) {
            smallest_overlap = overlaps.right;
            normal = Vector2.init(1, 0); // Right normal
        }

        // Special case: If circle is above and falling toward rectangle, prioritize top collision
        if (circle_position.y < rect_position.y and is_falling) {
            normal = Vector2.init(0, -1); // Top normal (pushes circle up)
            smallest_overlap = overlaps.top;
        }
        // Otherwise continue with standard minimum penetration
        else if (overlaps.top < smallest_overlap) {
            smallest_overlap = overlaps.top;
            normal = Vector2.init(0, -1); // Top normal
        }

        // Check bottom overlap
        if (overlaps.bottom < smallest_overlap) {
            smallest_overlap = overlaps.bottom;
            normal = Vector2.init(0, 1); // Bottom normal
        }

        return Penetration{
            .normal = normal,
            .depth = smallest_overlap,
        };
    }

    /// Calculate contact point for circle-rectangle collision
    pub fn calculateContactPoint(
        circle_position: Vector2,
        normal: Vector2,
        circle_radius: f32,
    ) Vector2 {
        // Project from circle center along normal to find contact on perimeter
        return Vector2.init(circle_position.x - normal.x * circle_radius, circle_position.y - normal.y * circle_radius);
    }
};
