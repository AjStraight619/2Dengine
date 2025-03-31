const std = @import("std");
const Vector2 = @import("../geometry/vector2.zig").Vector2;
const Rectangle = @import("../shapes/shape.zig").Rectangle;
const Circle = @import("../shapes/shape.zig").Circle;
const Shape = @import("../shapes/shape.zig").Shape;
const RigidBody = @import("../body/body.zig").RigidBody;

/// Result of a Separating Axis test
pub const SATResult = struct {
    /// Is there a collision?
    collision: bool,
    /// Minimum translation vector (points from shape A to shape B)
    mtv: Vector2,
    /// Penetration depth (positive if collision)
    depth: f32,
    /// Closest points on the shapes' perimeters
    contact_points: [2]Vector2,
    /// Number of contact points
    contact_count: u8 = 0,
};

/// Implementation of Separating Axis Theorem for convex shape collision detection
pub const SAT = struct {};

/// Test collision between two shapes using SAT
pub fn testCollision(a: *const RigidBody, b: *const RigidBody) SATResult {
    const result = switch (a.shape) {
        .circle => |circle_a| switch (b.shape) {
            .circle => |circle_b| circleVsCircle(a.position, circle_a, b.position, circle_b),
            .rectangle => |rect_b| circleVsRectangle(a.position, circle_a, b.position, rect_b),
        },
        .rectangle => |rect_a| switch (b.shape) {
            .circle => |circle_b| rectangleVsCircle(a.position, rect_a, b.position, circle_b),
            .rectangle => |rect_b| rectangleVsRectangle(a.position, rect_a, b.position, rect_b),
        },
    };

    // Log collision result
    if (result.collision) {
        std.debug.print("\n=== COLLISION DETECTED ===\n", .{});
        std.debug.print("Type: {s} vs {s}\n", .{ @tagName(a.shape), @tagName(b.shape) });
        std.debug.print("A pos: ({d:.4}, {d:.4}), B pos: ({d:.4}, {d:.4})\n", .{ a.position.x, a.position.y, b.position.x, b.position.y });
        std.debug.print("MTV: ({d:.4}, {d:.4}), Depth: {d:.4}\n", .{ result.mtv.x, result.mtv.y, result.depth });
        std.debug.print("Contact points: {d} [({d:.4}, {d:.4}), ({d:.4}, {d:.4})]\n", .{ result.contact_count, result.contact_points[0].x, result.contact_points[0].y, result.contact_points[1].x, result.contact_points[1].y });
    }

    return result;
}

/// Test collision between two circles
fn circleVsCircle(pos_a: Vector2, circle_a: Circle, pos_b: Vector2, circle_b: Circle) SATResult {
    // Debug info
    std.debug.print("\n--- Circle vs Circle Test ---\n", .{});
    std.debug.print("Circle A: pos=({d:.4}, {d:.4}), radius={d:.4}\n", .{ pos_a.x, pos_a.y, circle_a.radius });
    std.debug.print("Circle B: pos=({d:.4}, {d:.4}), radius={d:.4}\n", .{ pos_b.x, pos_b.y, circle_b.radius });

    // Calculate distance between centers
    const delta = pos_b.sub(pos_a);
    const distance_squared = delta.lengthSquared();

    // Sum of radii
    const radius_sum = circle_a.radius + circle_b.radius;

    std.debug.print("Distance^2={d:.4}, Radius sum^2={d:.4}\n", .{ distance_squared, radius_sum * radius_sum });

    // Check if circles overlap
    if (distance_squared >= radius_sum * radius_sum) {
        std.debug.print("No collision detected\n", .{});
        return SATResult{
            .collision = false,
            .mtv = Vector2.zero(),
            .depth = 0,
            .contact_points = [2]Vector2{ Vector2.zero(), Vector2.zero() },
            .contact_count = 0,
        };
    }

    // Calculate actual distance
    const distance = @sqrt(distance_squared);

    // Calculate penetration depth
    const depth = radius_sum - distance;

    // Calculate minimum translation vector
    const mtv = if (distance > 0.0001)
        delta.scale(1.0 / distance)
    else
        Vector2.init(0, -1); // Default direction if centers are exactly the same

    // Calculate contact point
    const contact_point = pos_a.add(mtv.scale(circle_a.radius));

    return SATResult{
        .collision = true,
        .mtv = mtv,
        .depth = depth,
        .contact_points = [2]Vector2{ contact_point, Vector2.zero() },
        .contact_count = 1,
    };
}

/// Test collision between a circle and a rectangle
fn circleVsRectangle(circle_pos: Vector2, circle: Circle, rect_pos: Vector2, rect: Rectangle) SATResult {
    // Debug info
    std.debug.print("\n--- Circle vs Rectangle Test ---\n", .{});
    std.debug.print("Circle: pos=({d:.4}, {d:.4}), radius={d:.4}\n", .{ circle_pos.x, circle_pos.y, circle.radius });
    std.debug.print("Rectangle: pos=({d:.4}, {d:.4}), size=({d:.4}x{d:.4}), angle={d:.4}\n", .{ rect_pos.x, rect_pos.y, rect.width, rect.height, rect.angle });

    // Get the rectangle's half extents
    const half_width = rect.width * 0.5;
    const half_height = rect.height * 0.5;

    // Transform circle center to rectangle's local space
    var local_circle_pos = circle_pos.sub(rect_pos);

    // If rectangle is rotated, rotate the circle's position
    if (!rect.isAxisAligned()) {
        std.debug.print("Rectangle is rotated, transforming circle to local space\n", .{});
        // Rotate circle position by negative of rectangle's angle
        const cos_angle = @cos(-rect.angle);
        const sin_angle = @sin(-rect.angle);
        local_circle_pos = Vector2.init(local_circle_pos.x * cos_angle - local_circle_pos.y * sin_angle, local_circle_pos.x * sin_angle + local_circle_pos.y * cos_angle);
    }

    std.debug.print("Circle in rect local space: ({d:.4}, {d:.4})\n", .{ local_circle_pos.x, local_circle_pos.y });

    // Find closest point on rectangle to circle center
    const closest_x = std.math.clamp(local_circle_pos.x, -half_width, half_width);
    const closest_y = std.math.clamp(local_circle_pos.y, -half_height, half_height);

    // If closest point is inside circle, we have a collision
    const closest_point = Vector2.init(closest_x, closest_y);
    const distance_vec = local_circle_pos.sub(closest_point);
    const distance_squared = distance_vec.lengthSquared();

    if (distance_squared > circle.radius * circle.radius) {
        return SATResult{
            .collision = false,
            .mtv = Vector2.zero(),
            .depth = 0,
            .contact_points = [2]Vector2{ Vector2.zero(), Vector2.zero() },
            .contact_count = 0,
        };
    }

    // Calculate penetration depth
    const distance = @sqrt(distance_squared);
    const depth = circle.radius - distance;

    // Calculate minimum translation vector in local space
    const mtv_local = if (distance > 0.0001)
        distance_vec.scale(-1.0 / distance)
    else if (@abs(local_circle_pos.x) > @abs(local_circle_pos.y))
        Vector2.init(if (local_circle_pos.x > 0) 1.0 else -1.0, 0)
    else
        Vector2.init(0, if (local_circle_pos.y > 0) 1.0 else -1.0);

    // Transform MTV back to world space
    var mtv = mtv_local;
    if (!rect.isAxisAligned()) {
        // Rotate MTV by rectangle's angle
        const cos_angle = @cos(rect.angle);
        const sin_angle = @sin(rect.angle);
        mtv = Vector2.init(mtv_local.x * cos_angle - mtv_local.y * sin_angle, mtv_local.x * sin_angle + mtv_local.y * cos_angle);
    }

    // Calculate contact point (on circle perimeter)
    const contact_point = circle_pos.sub(mtv.scale(circle.radius));

    return SATResult{
        .collision = true,
        .mtv = mtv,
        .depth = depth,
        .contact_points = [2]Vector2{ contact_point, Vector2.zero() },
        .contact_count = 1,
    };
}

/// Test collision between a rectangle and a circle (flipped arguments)
fn rectangleVsCircle(rect_pos: Vector2, rect: Rectangle, circle_pos: Vector2, circle: Circle) SATResult {
    // Reuse the existing function but flip the mtv direction
    var result = circleVsRectangle(circle_pos, circle, rect_pos, rect);
    // Flip MTV direction since we flipped the shapes
    if (result.collision) {
        result.mtv = result.mtv.scale(-1.0);
    }
    return result;
}

/// Test collision between two rectangles using SAT
fn rectangleVsRectangle(pos_a: Vector2, rect_a: Rectangle, pos_b: Vector2, rect_b: Rectangle) SATResult {
    // Debug info
    std.debug.print("\n--- Rectangle vs Rectangle Test ---\n", .{});
    std.debug.print("Rect A: pos=({d:.4}, {d:.4}), size=({d:.4}x{d:.4}), angle={d:.4}\n", .{ pos_a.x, pos_a.y, rect_a.width, rect_a.height, rect_a.angle });
    std.debug.print("Rect B: pos=({d:.4}, {d:.4}), size=({d:.4}x{d:.4}), angle={d:.4}\n", .{ pos_b.x, pos_b.y, rect_b.width, rect_b.height, rect_b.angle });

    // Fast case: Both rectangles are axis-aligned
    if (rect_a.isAxisAligned() and rect_b.isAxisAligned()) {
        std.debug.print("Both rectangles are axis-aligned, using AABB test\n", .{});
        return aabbVsAabb(pos_a, rect_a.width, rect_a.height, pos_b, rect_b.width, rect_b.height);
    }

    std.debug.print("Using full SAT for oriented rectangles\n", .{});

    // Full SAT for oriented rectangles
    // Get half-extents
    const half_width_a = rect_a.width * 0.5;
    const half_height_a = rect_a.height * 0.5;
    const half_width_b = rect_b.width * 0.5;
    const half_height_b = rect_b.height * 0.5;

    // Get the four corners of both rectangles
    const corners_a = getCorners(pos_a, half_width_a, half_height_a, rect_a.angle);
    const corners_b = getCorners(pos_b, half_width_b, half_height_b, rect_b.angle);

    // Get the edge normals for both rectangles
    const normals_a = getEdgeNormals(corners_a);
    const normals_b = getEdgeNormals(corners_b);

    // Variables to track minimum overlap and axis
    var min_overlap: f32 = std.math.floatMax(f32);
    var min_axis = Vector2.zero();

    // Project both shapes onto each normal and check for overlap
    for (normals_a) |normal| {
        const projection_result = projectShapes(corners_a, corners_b, normal);
        if (!projection_result.overlap) {
            // Found a separating axis, no collision
            return SATResult{
                .collision = false,
                .mtv = Vector2.zero(),
                .depth = 0,
                .contact_points = [2]Vector2{ Vector2.zero(), Vector2.zero() },
                .contact_count = 0,
            };
        }

        // Keep track of minimum overlap and its axis
        if (projection_result.overlap_amount < min_overlap) {
            min_overlap = projection_result.overlap_amount;
            min_axis = normal;
        }
    }

    // Test normals from the second rectangle
    for (normals_b) |normal| {
        const projection_result = projectShapes(corners_a, corners_b, normal);
        if (!projection_result.overlap) {
            // Found a separating axis, no collision
            return SATResult{
                .collision = false,
                .mtv = Vector2.zero(),
                .depth = 0,
                .contact_points = [2]Vector2{ Vector2.zero(), Vector2.zero() },
                .contact_count = 0,
            };
        }

        // Keep track of minimum overlap and its axis
        if (projection_result.overlap_amount < min_overlap) {
            min_overlap = projection_result.overlap_amount;
            min_axis = normal;
        }
    }

    // Ensure the MTV points from A to B
    const center_to_center = pos_b.sub(pos_a);
    if (center_to_center.dot(min_axis) < 0) {
        min_axis = min_axis.scale(-1.0);
    }

    // Find contact points (clipping algorithm)
    const contacts = findContactPoints(corners_a, corners_b, min_axis);

    return SATResult{
        .collision = true,
        .mtv = min_axis,
        .depth = min_overlap,
        .contact_points = contacts.points,
        .contact_count = contacts.count,
    };
}

/// Test collision between two axis-aligned rectangles (AABB)
fn aabbVsAabb(pos_a: Vector2, width_a: f32, height_a: f32, pos_b: Vector2, width_b: f32, height_b: f32) SATResult {
    // Half dimensions
    const half_width_a = width_a * 0.5;
    const half_height_a = height_a * 0.5;
    const half_width_b = width_b * 0.5;
    const half_height_b = height_b * 0.5;

    // Calculate min/max for both AABBs
    const min_a = Vector2.init(pos_a.x - half_width_a, pos_a.y - half_height_a);
    const max_a = Vector2.init(pos_a.x + half_width_a, pos_a.y + half_height_a);
    const min_b = Vector2.init(pos_b.x - half_width_b, pos_b.y - half_height_b);
    const max_b = Vector2.init(pos_b.x + half_width_b, pos_b.y + half_height_b);

    std.debug.print("AABB A: min=({d:.4}, {d:.4}), max=({d:.4}, {d:.4})\n", .{ min_a.x, min_a.y, max_a.x, max_a.y });
    std.debug.print("AABB B: min=({d:.4}, {d:.4}), max=({d:.4}, {d:.4})\n", .{ min_b.x, min_b.y, max_b.x, max_b.y });

    // Check for overlap
    if (min_a.x > max_b.x or max_a.x < min_b.x or
        min_a.y > max_b.y or max_a.y < min_b.y)
    {
        std.debug.print("No AABB overlap detected\n", .{});
        return SATResult{
            .collision = false,
            .mtv = Vector2.zero(),
            .depth = 0,
            .contact_points = [2]Vector2{ Vector2.zero(), Vector2.zero() },
            .contact_count = 0,
        };
    }

    // Calculate overlap in x and y
    const x_overlap = @min(max_a.x, max_b.x) - @max(min_a.x, min_b.x);
    const y_overlap = @min(max_a.y, max_b.y) - @max(min_a.y, min_b.y);

    std.debug.print("Overlap: x={d:.4}, y={d:.4}\n", .{ x_overlap, y_overlap });

    // Calculate vector from center of A to center of B
    const delta = pos_b.sub(pos_a);
    std.debug.print("Delta vector: ({d:.4}, {d:.4})\n", .{ delta.x, delta.y });

    var mtv = Vector2.zero();
    var depth: f32 = 0;

    // Add a bias for vertical collisions to reduce sliding
    // This helps with objects resting on top of each other
    const vertical_bias = 1.1; // 10% bias for vertical collisions

    // Apply vertical bias only when the difference is within a threshold and one object is above the other
    const is_vertical_stack = @abs(delta.x) < half_width_a + half_width_b * 0.8 and
        @abs(delta.y) > 0;

    if (is_vertical_stack and y_overlap < x_overlap * vertical_bias) {
        // Use Y axis (vertical collision)
        mtv.y = if (delta.y < 0) -1.0 else 1.0;
        depth = y_overlap;
        std.debug.print("Using vertical collision with bias\n", .{});
    } else if (x_overlap < y_overlap) {
        // X axis has less penetration
        mtv.x = if (delta.x < 0) -1.0 else 1.0;
        depth = x_overlap;
        std.debug.print("Using horizontal collision\n", .{});
    } else {
        // Y axis has less penetration
        mtv.y = if (delta.y < 0) -1.0 else 1.0;
        depth = y_overlap;
        std.debug.print("Using vertical collision\n", .{});
    }

    // Calculate contact points - add more detailed contact information
    var contacts: [2]Vector2 = undefined;
    const contact_count: u8 = 1;

    if (@abs(mtv.x) > 0.5) {
        // Contact along x-axis
        const contact_y = if (min_a.y > min_b.y)
            @min(max_a.y, max_b.y) - (min_a.y - min_b.y) / 2.0
        else
            @min(max_a.y, max_b.y) - (min_b.y - min_a.y) / 2.0;

        if (mtv.x > 0) {
            // Contact on A's right edge
            contacts[0] = Vector2.init(max_a.x, contact_y);
        } else {
            // Contact on A's left edge
            contacts[0] = Vector2.init(min_a.x, contact_y);
        }
    } else {
        // Contact along y-axis
        const contact_x = if (min_a.x > min_b.x)
            @min(max_a.x, max_b.x) - (min_a.x - min_b.x) / 2.0
        else
            @min(max_a.x, max_b.x) - (min_b.x - min_a.x) / 2.0;

        if (mtv.y > 0) {
            // Contact on A's bottom edge
            contacts[0] = Vector2.init(contact_x, max_a.y);
        } else {
            // Contact on A's top edge
            contacts[0] = Vector2.init(contact_x, min_a.y);
        }
    }

    contacts[1] = Vector2.zero();

    std.debug.print("MTV: ({d:.4}, {d:.4}), Depth: {d:.4}\n", .{ mtv.x, mtv.y, depth });

    return SATResult{
        .collision = true,
        .mtv = mtv,
        .depth = depth,
        .contact_points = contacts,
        .contact_count = contact_count,
    };
}

/// Get the four corners of a rectangle
fn getCorners(pos: Vector2, half_width: f32, half_height: f32, angle: f32) [4]Vector2 {
    const cos_angle = @cos(angle);
    const sin_angle = @sin(angle);

    var corners: [4]Vector2 = undefined;

    // Local space corners (clockwise from top-left)
    const local_corners = [4]Vector2{
        Vector2.init(-half_width, -half_height),
        Vector2.init(half_width, -half_height),
        Vector2.init(half_width, half_height),
        Vector2.init(-half_width, half_height),
    };

    // Rotate and translate corners to world space
    for (0..4) |i| {
        // Apply rotation
        corners[i] = Vector2.init(local_corners[i].x * cos_angle - local_corners[i].y * sin_angle, local_corners[i].x * sin_angle + local_corners[i].y * cos_angle);

        // Apply translation
        corners[i] = corners[i].add(pos);
    }

    return corners;
}

/// Calculate edge normals from corners
fn getEdgeNormals(corners: [4]Vector2) [2]Vector2 {
    // We only need 2 normals for a rectangle (perpendicular edges)
    var normals: [2]Vector2 = undefined;

    // Calculate edge vectors
    const edge1 = corners[1].sub(corners[0]);
    const edge2 = corners[3].sub(corners[0]);

    // Calculate normalized perpendicular vectors (normals)
    normals[0] = Vector2.init(-edge1.y, edge1.x).normalize();
    normals[1] = Vector2.init(-edge2.y, edge2.x).normalize();

    return normals;
}

/// Project shapes onto an axis and check for overlap
fn projectShapes(corners_a: [4]Vector2, corners_b: [4]Vector2, axis: Vector2) struct { overlap: bool, overlap_amount: f32 } {
    // Initialize min/max for both shapes
    var min_a: f32 = std.math.floatMax(f32);
    var max_a: f32 = -std.math.floatMax(f32);
    var min_b: f32 = std.math.floatMax(f32);
    var max_b: f32 = -std.math.floatMax(f32);

    // Project shape A onto axis
    for (corners_a) |corner| {
        const projection = corner.dot(axis);
        min_a = @min(min_a, projection);
        max_a = @max(max_a, projection);
    }

    // Project shape B onto axis
    for (corners_b) |corner| {
        const projection = corner.dot(axis);
        min_b = @min(min_b, projection);
        max_b = @max(max_b, projection);
    }

    // Check for overlap
    const overlap = (min_a <= max_b and max_a >= min_b);

    // Calculate overlap amount
    const overlap_amount = if (overlap) @min(max_a, max_b) - @max(min_a, min_b) else 0;

    return .{ .overlap = overlap, .overlap_amount = overlap_amount };
}

/// Find contact points between two rectangles
fn findContactPoints(corners_a: [4]Vector2, corners_b: [4]Vector2, separating_axis: Vector2) struct { points: [2]Vector2, count: u8 } {
    // This is a simplified approach - we'll return at most 2 contact points

    // Initialize result
    var result = std.mem.zeroes([2]Vector2);
    var count: u8 = 0;

    // Find the edge of shape A most aligned with the separating axis
    var reference_face_index: usize = 0;
    var best_alignment: f32 = -std.math.floatMax(f32);

    for (0..4) |i| {
        const next_i = (i + 1) % 4;
        const edge = corners_a[next_i].sub(corners_a[i]);
        const edge_normal = Vector2.init(-edge.y, edge.x).normalize();
        const alignment = @abs(edge_normal.dot(separating_axis));

        if (alignment > best_alignment) {
            best_alignment = alignment;
            reference_face_index = i;
        }
    }

    // Use the identified reference face
    const reference_index = reference_face_index;
    const next_index = (reference_index + 1) % 4;
    const reference_face = [2]Vector2{ corners_a[reference_index], corners_a[next_index] };

    // Find the closest points from shape B to the reference face
    for (corners_b) |point| {
        if (isPointOnSegment(point, reference_face[0], reference_face[1])) {
            if (count < 2) {
                result[count] = point;
                count += 1;
            }
        }
    }

    // If we didn't find any contact points, use the closest vertex
    if (count == 0) {
        var closest_distance: f32 = std.math.floatMax(f32);
        var closest_index: usize = 0;

        for (corners_b, 0..) |point, i| {
            const dist = pointToLineDistance(point, reference_face[0], reference_face[1]);
            if (dist < closest_distance) {
                closest_distance = dist;
                closest_index = i;
            }
        }

        result[0] = corners_b[closest_index];
        count = 1;
    }

    return .{ .points = result, .count = count };
}

/// Check if a point lies approximately on a line segment
fn isPointOnSegment(point: Vector2, a: Vector2, b: Vector2) bool {
    const line_segment = b.sub(a);
    const point_rel = point.sub(a);

    // Project point onto line segment
    const line_length_squared = line_segment.lengthSquared();
    if (line_length_squared < 0.0001) return false;

    const t = @max(0, @min(1, point_rel.dot(line_segment) / line_length_squared));
    const closest_point = a.add(line_segment.scale(t));

    // Check if point is close enough to segment
    const distance_squared = point.sub(closest_point).lengthSquared();
    return distance_squared < 0.01; // Small epsilon for proximity check
}

/// Calculate the distance from a point to a line segment
fn pointToLineDistance(point: Vector2, a: Vector2, b: Vector2) f32 {
    const line_segment = b.sub(a);
    const point_rel = point.sub(a);

    // Project point onto line segment
    const line_length_squared = line_segment.lengthSquared();
    if (line_length_squared < 0.0001) return point_rel.length();

    const t = @max(0, @min(1, point_rel.dot(line_segment) / line_length_squared));
    const closest_point = a.add(line_segment.scale(t));

    // Return distance to closest point
    return point.sub(closest_point).length();
}
