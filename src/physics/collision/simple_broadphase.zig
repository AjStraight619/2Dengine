const std = @import("std");
const RigidBody = @import("../body/body.zig").RigidBody;
const AABB = @import("../geometry/aabb.zig").AABB;
const Vector2 = @import("../geometry/vector2.zig").Vector2;

/// Pair of bodies that potentially collide
pub const BodyPair = struct {
    a: *RigidBody,
    b: *RigidBody,
};

/// A simple brute-force broadphase implementation
/// Uses direct AABB comparison without spatial partitioning
/// Less efficient but more stable for debugging
pub const SimpleBroadphase = struct {
    allocator: std.mem.Allocator,
    debug_enabled: bool = false,

    pub fn init(allocator: std.mem.Allocator) SimpleBroadphase {
        return SimpleBroadphase{
            .allocator = allocator,
        };
    }

    pub fn setDebug(self: *SimpleBroadphase, enabled: bool) void {
        self.debug_enabled = enabled;
    }

    /// Find all potential collisions using brute force AABB comparison
    pub fn findPotentialCollisions(self: *SimpleBroadphase, bodies: []const *RigidBody) !std.ArrayList(BodyPair) {
        var pairs = std.ArrayList(BodyPair).init(self.allocator);
        errdefer pairs.deinit();

        if (self.debug_enabled) {
            std.debug.print("\n=== SimpleBroadphase: Testing {d} bodies ===\n", .{bodies.len});
        }

        // Special case: 0 or 1 bodies
        if (bodies.len <= 1) {
            return pairs;
        }

        // Simple O(n²) comparison of all bodies against each other
        for (bodies, 0..) |body_a, i| {
            for (bodies[i + 1 ..]) |body_b| {
                // Skip if both bodies are static (they can't move, so won't collide)
                if (body_a.body_type == .static and body_b.body_type == .static) {
                    continue;
                }

                // Check if AABBs overlap
                if (self.aabbOverlap(body_a.aabb, body_b.aabb)) {
                    if (self.debug_enabled) {
                        std.debug.print("  Potential collision: body#{d} with body#{d}\n", .{
                            @intFromPtr(body_a) & 0xFFFF,
                            @intFromPtr(body_b) & 0xFFFF,
                        });
                    }

                    try pairs.append(BodyPair{
                        .a = @constCast(body_a),
                        .b = @constCast(body_b),
                    });
                }
            }
        }

        if (self.debug_enabled) {
            std.debug.print("  Found {d} potential collision pairs\n", .{pairs.items.len});
        }

        return pairs;
    }

    /// Check if two AABBs overlap
    fn aabbOverlap(self: SimpleBroadphase, a: AABB, b: AABB) bool {
        _ = self;
        // Check for overlap in both axes
        const x_overlap = (a.min_x <= b.max_x) and (a.max_x >= b.min_x);
        const y_overlap = (a.min_y <= b.max_y) and (a.max_y >= b.min_y);

        return x_overlap and y_overlap;
    }
};
