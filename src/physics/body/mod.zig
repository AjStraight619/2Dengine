// Body module exports
pub const RigidBody = @import("body.zig").RigidBody;
pub const BodyType = @import("body.zig").BodyType;

// Factory for creating physics bodies
pub const factory = @import("factory.zig");
pub const StaticBodyFactory = factory.StaticBodyFactory;
pub const DynamicBodyFactory = factory.DynamicBodyFactory;
pub const KinematicBodyFactory = factory.KinematicBodyFactory;

// Options for body creation
pub const options = @import("options.zig");
pub const StaticCircleOptions = options.StaticCircleOptions;
pub const StaticRectangleOptions = options.StaticRectangleOptions;
pub const DynamicCircleOptions = options.DynamicCircleOptions;
pub const DynamicRectangleOptions = options.DynamicRectangleOptions;
pub const KinematicCircleOptions = options.KinematicCircleOptions;
pub const KinematicRectangleOptions = options.KinematicRectangleOptions;

// Helper functions
pub const applyStaticBodyOptions = factory.applyStaticBodyOptions;
pub const applyDynamicBodyOptions = factory.applyDynamicBodyOptions;
pub const applyKinematicBodyOptions = factory.applyKinematicBodyOptions;
