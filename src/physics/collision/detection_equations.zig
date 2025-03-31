const Vector2 = @import("../geometry/vector2.zig").Vector2;
const Rectangle = @import("../shapes/shape.zig").Rectangle;

/// Core physics equations for collision detection
pub const DetectionEquations = struct {
    //------------------------------------------------------------------------------
    // CIRCLE COLLISION EQUATIONS
    //------------------------------------------------------------------------------

    /// |center_B - center_A| < radius_A + radius_B
    /// Check if two circles are colliding
    pub fn circlesCollide(
        center_a: Vector2,
        radius_a: f32,
        center_b: Vector2,
        radius_b: f32,
    ) bool {
        const distance_sq = center_b.sub(center_a).lengthSquared();
        const radius_sum = radius_a + radius_b;
        return distance_sq < radius_sum * radius_sum;
    }

    /// Calculate the penetration depth between two colliding circles
    /// depth = radius_A + radius_B - |center_B - center_A|
    pub fn circlesPenetrationDepth(
        center_a: Vector2,
        radius_a: f32,
        center_b: Vector2,
        radius_b: f32,
    ) f32 {
        const distance = center_b.sub(center_a).length();
        const radius_sum = radius_a + radius_b;
        return radius_sum - distance;
    }

    /// Calculate the collision normal between two circles
    /// normal = normalize(center_B - center_A)
    pub fn circlesCollisionNormal(
        center_a: Vector2,
        center_b: Vector2,
    ) Vector2 {
        const delta = center_b.sub(center_a);
        const distance = delta.length();

        // If centers are too close, use a default up direction
        if (distance < 0.0001) {
            return Vector2.init(0, -1);
        }

        return delta.scale(1.0 / distance);
    }

    //------------------------------------------------------------------------------
    // RECTANGLE COLLISION EQUATIONS (AABB)
    //------------------------------------------------------------------------------

    /// AABB overlap test
    /// Check if two axis-aligned rectangles overlap
    pub fn aabbOverlap(
        min_a: Vector2,
        max_a: Vector2,
        min_b: Vector2,
        max_b: Vector2,
    ) bool {
        return !(max_a.x < min_b.x or
            min_a.x > max_b.x or
            max_a.y < min_b.y or
            min_a.y > max_b.y);
    }

    /// Calculate penetration in x and y for two AABBs
    pub fn aabbPenetration(
        pos_a: Vector2,
        half_width_a: f32,
        half_height_a: f32,
        pos_b: Vector2,
        half_width_b: f32,
        half_height_b: f32,
    ) struct { overlap_x: f32, overlap_y: f32 } {
        // Calculate the distance between centers
        const delta = pos_b.sub(pos_a);

        // Calculate overlap in x and y
        const overlap_x = half_width_a + half_width_b - @abs(delta.x);
        const overlap_y = half_height_a + half_height_b - @abs(delta.y);

        return .{ .overlap_x = overlap_x, .overlap_y = overlap_y };
    }

    /// Calculate the minimum penetration normal for AABBs
    pub fn aabbCollisionNormal(
        delta: Vector2,
        overlap_x: f32,
        overlap_y: f32,
    ) Vector2 {
        var normal = Vector2.zero();

        if (overlap_x < overlap_y) {
            // X axis has less penetration
            normal.x = if (delta.x < 0) -1.0 else 1.0;
        } else {
            // Y axis has less penetration
            normal.y = if (delta.y < 0) -1.0 else 1.0;
        }

        return normal;
    }

    /// Calculate penetration depth for AABBs (minimum of x and y)
    pub fn aabbPenetrationDepth(overlap_x: f32, overlap_y: f32) f32 {
        return @min(overlap_x, overlap_y);
    }
};
