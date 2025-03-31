const std = @import("std");
const RigidBody = @import("../body/body.zig").RigidBody;
const AABB = @import("../geometry/aabb.zig").AABB;
const Vector2 = @import("../geometry/vector2.zig").Vector2;

/// Pair of bodies that potentially collide
pub const BodyPair = struct {
    a: *RigidBody,
    b: *RigidBody,
};

/// Cell coordinate in the spatial hash grid
const Cell = struct {
    x: i32,
    y: i32,

    pub fn init(x: i32, y: i32) Cell {
        return Cell{ .x = x, .y = y };
    }

    pub fn hash(self: Cell) u64 {
        // Simple spatial hash function
        return @as(u64, @intCast(@as(u32, @intCast(self.x)) *% 73856093)) ^
            @as(u64, @intCast(@as(u32, @intCast(self.y)) *% 19349663));
    }
};

/// Broadphase collision detection system using spatial hashing
pub const BroadPhase = struct {
    allocator: std.mem.Allocator,
    cell_size: f32 = 200.0, // Size of each grid cell - increased from 50.0 to 200.0

    /// Initialize a new broadphase system
    pub fn init(allocator: std.mem.Allocator) BroadPhase {
        return BroadPhase{
            .allocator = allocator,
        };
    }

    /// Set the cell size for the spatial hash grid
    pub fn setCellSize(self: *BroadPhase, size: f32) void {
        self.cell_size = size;
    }

    /// Get the cell coordinate for a position
    fn getCell(self: BroadPhase, position: Vector2) Cell {
        const x = @as(i32, @intFromFloat(@floor(position.x / self.cell_size)));
        const y = @as(i32, @intFromFloat(@floor(position.y / self.cell_size)));
        return Cell.init(x, y);
    }

    /// Get all cells that an AABB overlaps with
    fn getCellsForAABB(self: BroadPhase, aabb: AABB) []Cell {
        var cells = std.ArrayList(Cell).init(self.allocator);
        defer cells.deinit();

        // Calculate min and max grid cells
        const min_cell_x = @as(i32, @intFromFloat(@floor(aabb.min_x / self.cell_size)));
        const min_cell_y = @as(i32, @intFromFloat(@floor(aabb.min_y / self.cell_size)));
        const max_cell_x = @as(i32, @intFromFloat(@floor(aabb.max_x / self.cell_size)));
        const max_cell_y = @as(i32, @intFromFloat(@floor(aabb.max_y / self.cell_size)));

        // Add all cells in the range
        var y = min_cell_y;
        while (y <= max_cell_y) : (y += 1) {
            var x = min_cell_x;
            while (x <= max_cell_x) : (x += 1) {
                cells.append(Cell.init(x, y)) catch continue;
            }
        }

        return cells.toOwnedSlice() catch &[_]Cell{};
    }

    /// Spatial hashing method to find potential collision pairs
    /// Returns an array of potentially colliding body pairs
    pub fn findPotentialCollisions(self: *BroadPhase, bodies: []const *RigidBody) !std.ArrayList(BodyPair) {
        var pairs = std.ArrayList(BodyPair).init(self.allocator);
        errdefer pairs.deinit();

        // Special case: 0 or 1 bodies
        if (bodies.len <= 1) {
            return pairs;
        }

        // Create spatial hash map: cell hash -> bodies in that cell
        var grid = std.AutoHashMap(u64, std.ArrayList(*const RigidBody)).init(self.allocator);
        defer {
            var it = grid.valueIterator();
            while (it.next()) |list| {
                list.deinit();
            }
            grid.deinit();
        }

        // Insert bodies into cells they overlap
        for (bodies) |body| {
            // Get all cells this body's AABB overlaps
            const min_cell_x = @as(i32, @intFromFloat(@floor(body.aabb.min_x / self.cell_size)));
            const min_cell_y = @as(i32, @intFromFloat(@floor(body.aabb.min_y / self.cell_size)));
            const max_cell_x = @as(i32, @intFromFloat(@floor(body.aabb.max_x / self.cell_size)));
            const max_cell_y = @as(i32, @intFromFloat(@floor(body.aabb.max_y / self.cell_size)));

            var y = min_cell_y;
            while (y <= max_cell_y) : (y += 1) {
                var x = min_cell_x;
                while (x <= max_cell_x) : (x += 1) {
                    const cell = Cell.init(x, y);
                    const cell_hash = cell.hash();

                    var entry = try grid.getOrPut(cell_hash);
                    if (!entry.found_existing) {
                        entry.value_ptr.* = std.ArrayList(*const RigidBody).init(self.allocator);
                    }

                    try entry.value_ptr.append(body);
                }
            }
        }

        // Track pairs we've already checked to avoid duplicates
        var checked_pairs = std.AutoHashMap(u64, void).init(self.allocator);
        defer checked_pairs.deinit();

        // For each cell, check bodies in that cell for potential collisions
        var it = grid.valueIterator();
        while (it.next()) |bodies_in_cell| {
            const cell_bodies = bodies_in_cell.items;

            // Check all pairs in this cell
            for (cell_bodies, 0..) |body_a, i| {
                for (cell_bodies[i + 1 ..]) |body_b| {
                    // Skip if both bodies are static (they can't move, so won't collide)
                    if (body_a.body_type == .static and body_b.body_type == .static) {
                        continue;
                    }

                    // Create a unique pair ID to avoid duplicate checks
                    const ptr_a = @intFromPtr(body_a);
                    const ptr_b = @intFromPtr(body_b);
                    const pair_id = if (ptr_a < ptr_b)
                        (ptr_a << 32) | ptr_b
                    else
                        (ptr_b << 32) | ptr_a;

                    // Skip if we've already checked this pair
                    if (checked_pairs.contains(pair_id)) continue;
                    try checked_pairs.put(pair_id, {});

                    // Check if AABBs overlap
                    if (aabbOverlap(body_a.aabb, body_b.aabb)) {
                        // Log extreme velocities (might indicate instability)
                        const vel_a_len = body_a.velocity.length();
                        const vel_b_len = body_b.velocity.length();
                        const high_velocity_threshold: f32 = 1000.0;

                        if (vel_a_len > high_velocity_threshold or vel_b_len > high_velocity_threshold) {
                            std.debug.print("⚠️ HIGH VELOCITY DETECTED! Body A vel={d:.2}, Body B vel={d:.2}\n", .{ vel_a_len, vel_b_len });
                            std.debug.print("  Body A pos=({d:.2},{d:.2}), Body B pos=({d:.2},{d:.2})\n", .{ body_a.position.x, body_a.position.y, body_b.position.x, body_b.position.y });
                            std.debug.print("  Body A type={s}, Body B type={s}\n", .{ @tagName(body_a.body_type), @tagName(body_b.body_type) });
                        }

                        try pairs.append(BodyPair{
                            .a = @constCast(body_a),
                            .b = @constCast(body_b),
                        });
                    }
                }
            }
        }

        // Log collision count if it's very high
        if (pairs.items.len > 100) {
            std.debug.print("⚠️ HIGH COLLISION COUNT: {d} potential collisions\n", .{pairs.items.len});
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

/// Additional broadphase implementations for different use cases
/// Spatial Grid - a simpler spatial partitioning system for high performance
pub const SpatialGrid = struct {
    allocator: std.mem.Allocator,
    cell_size: f32,
    grid: std.ArrayList(std.ArrayList(*RigidBody)),
    width: usize,
    height: usize,

    pub fn init(allocator: std.mem.Allocator, world_width: f32, world_height: f32, cell_size: f32) !SpatialGrid {
        const width = @as(usize, @intFromFloat(@ceil(world_width / cell_size)));
        const height = @as(usize, @intFromFloat(@ceil(world_height / cell_size)));

        var grid = std.ArrayList(std.ArrayList(*RigidBody)).init(allocator);
        try grid.resize(width * height);

        // Initialize each cell
        for (0..width * height) |i| {
            grid.items[i] = std.ArrayList(*RigidBody).init(allocator);
        }

        return SpatialGrid{
            .allocator = allocator,
            .cell_size = cell_size,
            .grid = grid,
            .width = width,
            .height = height,
        };
    }

    pub fn deinit(self: *SpatialGrid) void {
        for (self.grid.items) |*cell| {
            cell.deinit();
        }
        self.grid.deinit();
    }

    // Additional methods would be implemented for a complete spatial grid system
};

// More advanced spatial partitioning could be added here in future:
// - Spatial Hashing
// - Quadtree
// - Dynamic AABB Tree
// etc.
