const std = @import("std");
const Shape = @import("shape.zig").Shape;
const Circle = @import("shape.zig").Circle;
const Rectangle = @import("shape.zig").Rectangle;

/// Physical properties calculator for different shape types
pub const ShapeProperties = struct {
    /// Calculate moment of inertia for any shape
    pub fn getMomentOfInertia(shape: Shape, mass: f32) f32 {
        return switch (shape) {
            .circle => |c| getCircleMomentOfInertia(c, mass),
            .rectangle => |r| getRectangleMomentOfInertia(r, mass),
        };
    }

    /// Calculate moment of inertia for a circle
    pub fn getCircleMomentOfInertia(circle: Circle, mass: f32) f32 {
        // I = (1/2) * m * r²
        return 0.5 * mass * circle.radius * circle.radius;
    }

    /// Calculate moment of inertia for a rectangle
    pub fn getRectangleMomentOfInertia(rect: Rectangle, mass: f32) f32 {
        // I = (1/12) * m * (w² + h²)
        return (mass / 12.0) * (rect.width * rect.width + rect.height * rect.height);
    }

    /// Calculate area of any shape
    pub fn getArea(shape: Shape) f32 {
        return switch (shape) {
            .circle => |c| getCircleArea(c),
            .rectangle => |r| getRectangleArea(r),
        };
    }

    /// Calculate area of a circle
    pub fn getCircleArea(circle: Circle) f32 {
        return std.math.pi * circle.radius * circle.radius;
    }

    /// Calculate area of a rectangle
    pub fn getRectangleArea(rect: Rectangle) f32 {
        return rect.width * rect.height;
    }

    /// Calculate density based on mass and shape
    pub fn getDensity(shape: Shape, mass: f32) f32 {
        const area = getArea(shape);
        return if (area > 0) mass / area else 0;
    }
};
