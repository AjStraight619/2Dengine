const std = @import("std");
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const Shape = @import("../shapes/shape.zig").Shape;
const Circle = @import("../shapes/shape.zig").Circle;
const Rectangle = @import("../shapes/shape.zig").Rectangle;
const RigidBody = @import("../body/body.zig").RigidBody;
const transform = @import("../geometry/transform.zig");
const BoundsCalculator = @import("../shapes/bounds.zig").BoundsCalculator;
const utils = @import("utils.zig");
const CollisionUtils = utils.CollisionUtils;
const Bounds = utils.Bounds;
const Overlaps = utils.Overlaps;
const Penetration = utils.Penetration;

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
        if (!BoundsCalculator.aabbOverlap(a.aabb, b.aabb)) {
            return null;
        }

        // Then do narrow-phase detection based on shape types
        const result = switch (a.shape) {
            .circle => |circle_a| switch (b.shape) {
                .circle => |circle_b| circleVsCircle(a, circle_a, b, circle_b),
                .rectangle => |rect_b| circleVsRectangle(a, circle_a, b, rect_b),
            },
            .rectangle => |rect_a| switch (b.shape) {
                .circle => |circle_b| rectangleVsCircle(b, circle_b, a, rect_a),
                .rectangle => |rect_b| rectangleVsRectangle(a, rect_a, b, rect_b),
            },
        };

        return result;
    }

    /// Detect collision between two circles
    fn circleVsCircle(a: *RigidBody, circle_a: Circle, b: *RigidBody, circle_b: Circle) ?Collision {
        // Calculate distance between centers
        const delta = b.position.sub(a.position);
        const distance_squared = delta.lengthSquared();

        // Sum of radii
        const radius_sum = circle_a.radius + circle_b.radius;

        // Check if circles overlap
        if (distance_squared >= radius_sum * radius_sum) {
            return null; // No collision
        }

        // Calculate actual distance
        const distance = @sqrt(distance_squared);

        // Create collision info
        const normal = if (distance > 0.0001) delta.scale(1.0 / distance) else Vector2.init(0, -1);
        const depth = radius_sum - distance;

        // Calculate contact point (halfway between surfaces)
        const contact_point = a.position.add(normal.scale(circle_a.radius - depth * 0.5));

        // Calculate material properties
        const materials = CollisionUtils.calculateMaterialProperties(a, b);

        return Collision{
            .normal = normal,
            .depth = depth,
            .contact_points = [2]Vector2{ contact_point, Vector2.zero() },
            .contact_count = 1,
            .restitution = materials.restitution,
            .friction = materials.friction,
        };
    }

    /// Detect collision between a circle and a rectangle
    fn circleVsRectangle(a: *RigidBody, circle: Circle, b: *RigidBody, rect: Rectangle) ?Collision {
        // Get bounds for both shapes
        const rect_bounds = CollisionUtils.calculateRectangleBounds(b.position, rect.width, rect.height);
        const circle_bounds = CollisionUtils.calculateCircleBounds(a.position, circle.radius);

        // Check for intersection
        if (!CollisionUtils.checkAABBIntersection(circle_bounds, rect_bounds)) {
            return null; // No collision
        }

        // Calculate overlaps
        const overlaps = CollisionUtils.calculateOverlaps(circle_bounds, rect_bounds);

        // Find minimum penetration axis
        const is_falling = a.velocity.y > 0;
        const min_penetration = CollisionUtils.findMinimumPenetrationAxis(overlaps, a.position, b.position, is_falling);

        // Calculate contact point
        const contact_point = CollisionUtils.calculateContactPoint(a.position, min_penetration.normal, circle.radius);

        // Calculate material properties
        const materials = CollisionUtils.calculateMaterialProperties(a, b);

        return Collision{
            .normal = min_penetration.normal,
            .depth = min_penetration.depth,
            .contact_points = [2]Vector2{ contact_point, Vector2.zero() },
            .contact_count = 1,
            .restitution = materials.restitution,
            .friction = materials.friction,
        };
    }

    /// Detect collision between a rectangle and a circle (flipped args)
    fn rectangleVsCircle(a: *RigidBody, circle: Circle, b: *RigidBody, rect: Rectangle) ?Collision {
        // Get bounds for both shapes
        const rect_bounds = CollisionUtils.calculateRectangleBounds(b.position, rect.width, rect.height);
        const circle_bounds = CollisionUtils.calculateCircleBounds(a.position, circle.radius);

        // Check for intersection
        if (!CollisionUtils.checkAABBIntersection(circle_bounds, rect_bounds)) {
            return null; // No collision
        }

        // Calculate overlaps
        const overlaps = CollisionUtils.calculateOverlaps(circle_bounds, rect_bounds);

        // Find minimum penetration axis
        const is_falling = a.velocity.y > 0;
        const min_penetration = CollisionUtils.findMinimumPenetrationAxis(overlaps, a.position, b.position, is_falling);

        // Calculate contact point
        const contact_point = CollisionUtils.calculateContactPoint(a.position, min_penetration.normal, circle.radius);

        // Calculate material properties
        const materials = CollisionUtils.calculateMaterialProperties(a, b);

        return Collision{
            .normal = min_penetration.normal,
            .depth = min_penetration.depth,
            .contact_points = [2]Vector2{ contact_point, Vector2.zero() },
            .contact_count = 1,
            .restitution = materials.restitution,
            .friction = materials.friction,
        };
    }

    /// Detect collision between two rectangles
    fn rectangleVsRectangle(a: *RigidBody, rect_a: Rectangle, b: *RigidBody, rect_b: Rectangle) ?Collision {
        // For simplicity, let's first handle only the case of axis-aligned rectangles
        // We'll handle rotated rectangles later
        if (rect_a.angle == 0.0 and rect_b.angle == 0.0) {
            // Use separating axis theorem (SAT) for AABBs
            const half_width_a = rect_a.width * 0.5;
            const half_height_a = rect_a.height * 0.5;
            const half_width_b = rect_b.width * 0.5;
            const half_height_b = rect_b.height * 0.5;

            // Calculate the distance between centers
            const delta = b.position.sub(a.position);

            // Calculate overlap in x and y
            const overlap_x = half_width_a + half_width_b - @abs(delta.x);
            const overlap_y = half_height_a + half_height_b - @abs(delta.y);

            // If either overlap is negative, there's no collision
            if (overlap_x < 0 or overlap_y < 0) {
                return null;
            }

            // Determine normal and depth (using smallest penetration axis)
            var normal = Vector2.zero();
            var depth: f32 = 0;

            if (overlap_x < overlap_y) {
                // X axis has less penetration
                normal.x = if (delta.x < 0) -1.0 else 1.0;
                depth = overlap_x;
            } else {
                // Y axis has less penetration
                normal.y = if (delta.y < 0) -1.0 else 1.0;
                depth = overlap_y;
            }

            // Calculate contact point
            // In most cases, there would be two contact points, but we'll simplify
            // by using a single contact at the midpoint of the overlap
            const contact_point = a.position.add(normal.scale(half_width_a - depth * 0.5));

            // Calculate material properties
            const materials = CollisionUtils.calculateMaterialProperties(a, b);

            return Collision{
                .normal = normal,
                .depth = depth,
                .contact_points = [2]Vector2{ contact_point, Vector2.zero() },
                .contact_count = 1,
                .restitution = materials.restitution,
                .friction = materials.friction,
            };
        } else {
            // For rotated rectangles, we would need to implement the full SAT
            // with projected axes. This is more complex and would be a good addition
            // in a future update.
            return null;
        }
    }
};
