/// Collision detection and resolution module
const std = @import("std");

pub const Broadphase = @import("broadphase.zig").BroadPhase;
pub const Detector = @import("detector.zig").CollisionDetector;
pub const Collision = @import("detector.zig").Collision;
pub const Resolver = @import("resolver.zig").CollisionResolver;

// New imports
pub const SAT = @import("sat.zig").SAT;
pub const testCollision = @import("sat.zig").testCollision;
pub const SATResult = @import("sat.zig").SATResult;

// Utility re-exports
pub const utils = @import("utils.zig");
pub const detection_equations = @import("detection_equations.zig");
pub const physics_equations = @import("physics_equations.zig");

// Collision utility exports
pub const CollisionUtils = @import("utils.zig").CollisionUtils;
pub const Bounds = @import("utils.zig").Bounds;
pub const Overlaps = @import("utils.zig").Overlaps;
pub const Penetration = @import("utils.zig").Penetration;

// Re-export utility functions
pub const detectCollision = Detector.detectCollision;
