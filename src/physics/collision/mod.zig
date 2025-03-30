/// Collision detection and resolution module
pub const Detector = @import("detector.zig").CollisionDetector;
pub const Collision = @import("detector.zig").Collision;
pub const Resolver = @import("resolver.zig").CollisionResolver;
pub const Broadphase = @import("broadphase.zig").BroadPhase;
pub const BodyPair = @import("broadphase.zig").BodyPair;

// Collision utility exports
pub const CollisionUtils = @import("utils.zig").CollisionUtils;
pub const Bounds = @import("utils.zig").Bounds;
pub const Overlaps = @import("utils.zig").Overlaps;
pub const Penetration = @import("utils.zig").Penetration;

// Re-export utility functions
pub const detectCollision = Detector.detectCollision;
