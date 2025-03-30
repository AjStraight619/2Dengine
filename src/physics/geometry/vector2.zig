pub const Vector2 = struct {
    x: f32,
    y: f32,

    pub fn init(x: f32, y: f32) Vector2 {
        return Vector2{ .x = x, .y = y };
    }

    pub fn add(a: Vector2, b: Vector2) Vector2 {
        return Vector2{
            .x = a.x + b.x,
            .y = a.y + b.y,
        };
    }

    pub fn sub(a: Vector2, b: Vector2) Vector2 {
        return Vector2{
            .x = a.x - b.x,
            .y = a.y - b.y,
        };
    }

    pub fn mul(a: Vector2, b: Vector2) Vector2 {
        return Vector2{
            .x = a.x * b.x,
            .y = a.y * b.y,
        };
    }

    pub fn div(a: Vector2, b: Vector2) Vector2 {
        if (b.x == 0 or b.y == 0) {
            return Vector2{ .x = 0, .y = 0 };
        }
        return Vector2{
            .x = a.x / b.x,
            .y = a.y / b.y,
        };
    }

    pub fn zero() Vector2 {
        return Vector2{ .x = 0, .y = 0 };
    }

    pub fn one() Vector2 {
        return Vector2{ .x = 1, .y = 1 };
    }

    pub fn dot(a: Vector2, b: Vector2) f32 {
        return a.x * b.x + a.y * b.y;
    }

    pub fn length(a: Vector2) f32 {
        return @sqrt(a.x * a.x + a.y * a.y);
    }

    // Fixed normalize method
    pub fn normalize(a: Vector2) Vector2 {
        const len = a.length();
        if (len < 0.0001) return Vector2.zero();
        return Vector2{
            .x = a.x / len,
            .y = a.y / len,
        };
    }

    // Squared length (more efficient when just comparing distances)
    pub fn lengthSquared(a: Vector2) f32 {
        return a.x * a.x + a.y * a.y;
    }

    // Distance between vectors
    pub fn distance(a: Vector2, b: Vector2) f32 {
        return a.sub(b).length();
    }

    // Scale by a scalar (not another vector)
    pub fn scale(a: Vector2, scalar: f32) Vector2 {
        return Vector2{
            .x = a.x * scalar,
            .y = a.y * scalar,
        };
    }

    // Perpendicular vector (rotate 90 degrees)
    pub fn perpendicular(a: Vector2) Vector2 {
        return Vector2{
            .x = -a.y,
            .y = a.x,
        };
    }

    // 2D cross product (returns scalar)
    pub fn cross(a: Vector2, b: Vector2) f32 {
        return a.x * b.y - a.y * b.x;
    }

    // Reflect vector around normal
    pub fn reflect(a: Vector2, normal: Vector2) Vector2 {
        const normalized = normal.normalize();
        const dot2 = 2.0 * a.dot(normalized);
        return a.sub(normalized.scale(dot2));
    }

    // Rotate vector by angle (in radians)
    pub fn rotate(a: Vector2, angle: f32) Vector2 {
        const sin_val = @sin(angle);
        const cos_val = @cos(angle);
        return Vector2{
            .x = a.x * cos_val - a.y * sin_val,
            .y = a.x * sin_val + a.y * cos_val,
        };
    }

    // Linear interpolation between vectors
    pub fn lerp(a: Vector2, b: Vector2, t: f32) Vector2 {
        return Vector2{
            .x = a.x + (b.x - a.x) * t,
            .y = a.y + (b.y - a.y) * t,
        };
    }

    // Check if vectors are approximately equal
    pub fn equals(a: Vector2, b: Vector2, epsilon: f32) bool {
        return @abs(a.x - b.x) < epsilon and
            @abs(a.y - b.y) < epsilon;
    }
};
