const Vector2 = @import("vector2.zig").Vector2;

pub const AABB = struct {
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,

    pub fn fromSize(center: Vector2, width: f32, height: f32) AABB {
        return AABB{
            .min_x = center.x - width / 2,
            .min_y = center.y - height / 2,
            .max_x = center.x + width / 2,
            .max_y = center.y + height / 2,
        };
    }
};
