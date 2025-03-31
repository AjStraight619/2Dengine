// Re-export shape types
pub const Shape = @import("shape.zig").Shape;
pub const Circle = @import("shape.zig").Circle;
pub const Rectangle = @import("shape.zig").Rectangle;

// Shape-related utilities
pub const ShapeProperties = @import("properties.zig").ShapeProperties;
pub const BoundsCalculator = @import("bounds.zig").BoundsCalculator;
