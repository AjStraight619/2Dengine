const std = @import("std");
const RigidBody = @import("../body/body.zig").RigidBody;
const AABB = @import("../geometry/aabb.zig").AABB;

/// Pair of bodies that potentially collide
pub const BodyPair = struct {
    a: *RigidBody,
    b: *RigidBody,
};

/// Broadphase collision detection system
pub const BroadPhase = struct {
    allocator: std.mem.Allocator,

    /// Initialize a new broadphase system
    pub fn init(allocator: std.mem.Allocator) BroadPhase {
        return BroadPhase{
            .allocator = allocator,
        };
    }

    /// Simple sweep and prune method on X-axis
    /// Returns an array of potentially colliding body pairs
    pub fn findPotentialCollisions(self: *BroadPhase, bodies: []const *RigidBody) !std.ArrayList(BodyPair) {
        var pairs = std.ArrayList(BodyPair).init(self.allocator);
        errdefer pairs.deinit();

        // Special case: 0 or 1 bodies
        if (bodies.len <= 1) {
            return pairs;
        }

        // Sweep and Prune algorithm:
        // 1. For each pair of bodies
        // 2. Check if their AABBs overlap
        // 3. If so, add them to potential collisions
        for (bodies, 0..) |body_a, i| {
            for (bodies[i + 1 ..]) |body_b| {
                // Skip if both bodies are static (they can't move, so won't collide)
                if (body_a.body_type == .static and body_b.body_type == .static) {
                    continue;
                }

                // Check if AABBs overlap
                if (aabbOverlap(body_a.aabb, body_b.aabb)) {
                    try pairs.append(BodyPair{ .a = body_a, .b = body_b });
                }
            }
        }

        return pairs;
    }

    /// Check if two AABBs overlap
    fn aabbOverlap(a: AABB, b: AABB) bool {
        // Check for overlap in both axes
        const x_overlap = (a.min_x <= b.max_x) and (a.max_x >= b.min_x);
        const y_overlap = (a.min_y <= b.max_y) and (a.max_y >= b.min_y);
        return x_overlap and y_overlap;
    }
};

// More advanced spatial partitioning could be added here in future:
// - Spatial Hashing
// - Quadtree
// - Dynamic AABB Tree
// etc.
