// Physics module re-exports
pub const Vector2 = @import("geometry/vector2.zig").Vector2;
pub const AABB = @import("geometry/aabb.zig").AABB;
pub const PhysicsWorld = @import("world.zig").PhysicsWorld;
pub const RigidBody = @import("body/body.zig").RigidBody;
pub const BodyType = @import("body/body.zig").BodyType;
pub const Shape = @import("shapes/shape.zig").Shape;
pub const Circle = @import("shapes/shape.zig").Circle;
pub const Rectangle = @import("shapes/shape.zig").Rectangle;
pub const Forces = @import("dynamics/forces.zig").Forces;
pub const Integrator = @import("dynamics/integration.zig").Integrator;

// Logger for physics debug messages
pub const logger = @import("logger.zig");

// Submodules
pub const body = struct {
    pub const options = @import("body/options.zig");
    pub const factory = @import("body/factory.zig");

    // Re-export important types
    pub const RigidBody = @import("body/body.zig").RigidBody;
    pub const BodyType = @import("body/body.zig").BodyType;

    // Helper functions
    pub const applyStaticBodyOptions = factory.applyStaticBodyOptions;
    pub const applyDynamicBodyOptions = factory.applyDynamicBodyOptions;
    pub const applyKinematicBodyOptions = factory.applyKinematicBodyOptions;
};

pub const dynamics = struct {
    pub const Forces = @import("dynamics/forces.zig").Forces;
    pub const Integrator = @import("dynamics/integration.zig").Integrator;
};

// New stability module for simulation safeguards and sleep management
pub const stability = @import("stability/mod.zig");

// Factory option structures - re-exported for convenience
pub const StaticCircleOptions = @import("body/options.zig").StaticCircleOptions;
pub const StaticRectangleOptions = @import("body/options.zig").StaticRectangleOptions;
pub const DynamicCircleOptions = @import("body/options.zig").DynamicCircleOptions;
pub const DynamicRectangleOptions = @import("body/options.zig").DynamicRectangleOptions;
pub const KinematicCircleOptions = @import("body/options.zig").KinematicCircleOptions;
pub const KinematicRectangleOptions = @import("body/options.zig").KinematicRectangleOptions;

// Factory interfaces - re-exported for convenience
pub const StaticBodyFactory = @import("body/factory.zig").StaticBodyFactory;
pub const DynamicBodyFactory = @import("body/factory.zig").DynamicBodyFactory;
pub const KinematicBodyFactory = @import("body/factory.zig").KinematicBodyFactory;

// Helper functions - re-exported for convenience
pub const applyStaticBodyOptions = @import("body/factory.zig").applyStaticBodyOptions;
pub const applyDynamicBodyOptions = @import("body/factory.zig").applyDynamicBodyOptions;
pub const applyKinematicBodyOptions = @import("body/factory.zig").applyKinematicBodyOptions;

// Shape-related utilities
pub const ShapeProperties = @import("shapes/properties.zig").ShapeProperties;
pub const BoundsCalculator = @import("shapes/bounds.zig").BoundsCalculator;

// Expose submodules
pub const geometry = @import("geometry/mod.zig");
pub const collision = @import("collision/mod.zig");
pub const world = @import("world.zig");
