const Vector2 = @import("../geometry/vector2.zig").Vector2;
const RigidBody = @import("../body/body.zig").RigidBody;
const Collision = @import("detector.zig").Collision;
const std = @import("std");
const detector = @import("detector.zig");
const CollisionPhysics = @import("physics_equations.zig").CollisionPhysics;

/// Collision resolution system
pub const CollisionResolver = struct {
    /// Resolve a collision between two bodies
    pub fn resolveCollision(a: *RigidBody, b: *RigidBody, collision: Collision) void {
        // Skip resolution if both bodies are static (not just either one)
        if (a.body_type == .static and b.body_type == .static) {
            return;
        }

        std.debug.print("\n>>> RESOLVING COLLISION <<<\n", .{});
        std.debug.print("A: type={}, static={}, pos=({d},{d}), vel=({d},{d})\n", .{ a.shape, a.body_type == .static, a.position.x, a.position.y, a.velocity.x, a.velocity.y });
        std.debug.print("B: type={}, static={}, pos=({d},{d}), vel=({d},{d})\n", .{ b.shape, b.body_type == .static, b.position.x, b.position.y, b.velocity.x, b.velocity.y });
        std.debug.print("Collision normal=({d},{d}) - THIS POINTS FROM A TO B\n", .{ collision.normal.x, collision.normal.y });

        // Position correction (prevents sinking)
        resolvePositions(a, b, collision);

        // Impulse resolution (bounce and friction)
        resolveVelocities(a, b, collision);

        std.debug.print("After resolution - A: pos=({d},{d}), vel=({d},{d})\n", .{ a.position.x, a.position.y, a.velocity.x, a.velocity.y });
        std.debug.print("After resolution - B: pos=({d},{d}), vel=({d},{d})\n", .{ b.position.x, b.position.y, b.velocity.x, b.velocity.y });
    }

    /// Resolve positions to prevent objects from sinking into each other
    fn resolvePositions(a: *RigidBody, b: *RigidBody, collision: Collision) void {
        // Skip if both bodies are static
        if (a.body_type == .static and b.body_type == .static) {
            return;
        }

        std.debug.print(">>> POSITION CORRECTION <<<\n", .{});
        std.debug.print("A body type: {s}, B body type: {s}\n", .{ @tagName(a.body_type), @tagName(b.body_type) });
        std.debug.print("Current A pos: ({d:.4},{d:.4}), B pos: ({d:.4},{d:.4})\n", .{ a.position.x, a.position.y, b.position.x, b.position.y });
        std.debug.print("Collision normal: ({d:.4},{d:.4}), depth: {d:.4}\n", .{ collision.normal.x, collision.normal.y, collision.depth });

        // Check if this is a horizontal collision between rectangles
        const is_horizontal = @abs(collision.normal.x) > 0.9; // Increased threshold for more accurate detection
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;

        // Check for vertical collision with ground
        const is_vertical = @abs(collision.normal.y) > 0.9;
        const is_ground_collision = is_vertical and collision.normal.y < 0;
        const vertical_rect_ground_collision = is_ground_collision and is_rect_vs_rect;

        // Use different correction factors based on collision orientation and types
        var correction_factor: f32 = 1.01; // Default small bias

        // For horizontal rect-rect collisions, use stable correction
        if (horizontal_rect_collision) {
            correction_factor = 1.05; // Reduced from 1.2 for more stable behavior
            std.debug.print("HORIZONTAL rect-rect collision - using moderate position correction: {d}\n", .{correction_factor});
        }
        // For ground collisions (normal pointing up), use stronger correction
        else if (vertical_rect_ground_collision) {
            correction_factor = 1.1; // 10% stronger correction for ground collisions
            std.debug.print("GROUND collision - using stronger position correction: {d}\n", .{correction_factor});
        }

        const correction = CollisionPhysics.calculatePositionCorrection(collision.normal, collision.depth, correction_factor);
        std.debug.print("Correction vector: ({d:.4},{d:.4})\n", .{ correction.x, correction.y });

        // Apply the correction based on mass properties
        if (a.body_type == .static) {
            // Only push body B (static A can't move)
            b.position = b.position.add(correction);
            std.debug.print("Static A: moved B to ({d:.4},{d:.4})\n", .{ b.position.x, b.position.y });
        } else if (b.body_type == .static) {
            // Only push body A (static B can't move)
            a.position = a.position.sub(correction);
            std.debug.print("Static B: moved A to ({d:.4},{d:.4})\n", .{ a.position.x, a.position.y });
        } else {
            // Push both bodies based on their relative masses
            const total_inv_mass = a.inverse_mass + b.inverse_mass;

            if (total_inv_mass <= 0.0001) return;

            const a_ratio = CollisionPhysics.calculateCorrectionRatio(a.inverse_mass, b.inverse_mass);
            const b_ratio = CollisionPhysics.calculateCorrectionRatio(b.inverse_mass, a.inverse_mass);

            std.debug.print("Dynamic-Dynamic: A ratio: {d:.4}, B ratio: {d:.4}\n", .{ a_ratio, b_ratio });

            a.position = a.position.sub(correction.scale(a_ratio));
            b.position = b.position.add(correction.scale(b_ratio));
            std.debug.print("Moved both: A to ({d:.4},{d:.4}), B to ({d:.4},{d:.4})\n", .{ a.position.x, a.position.y, b.position.x, b.position.y });
        }

        // Update AABBs after position changes
        a.updateAABB();
        b.updateAABB();
    }

    /// Resolve velocities using impulses
    fn resolveVelocities(a: *RigidBody, b: *RigidBody, collision: Collision) void {
        // Skip resolution if both bodies are static
        if (a.body_type == .static and b.body_type == .static) {
            return;
        }

        // Check for extremely high velocities and log them
        const vel_a_len = a.velocity.length();
        const vel_b_len = b.velocity.length();
        const velocity_threshold: f32 = 500.0;

        if (vel_a_len > velocity_threshold or vel_b_len > velocity_threshold) {
            std.debug.print("\n🚨 EXTREME VELOCITY DURING RESOLUTION:\n", .{});
            std.debug.print("  A: vel={d:.2} at ({d:.2},{d:.2})\n", .{ vel_a_len, a.position.x, a.position.y });
            std.debug.print("  B: vel={d:.2} at ({d:.2},{d:.2})\n", .{ vel_b_len, b.position.x, b.position.y });
            std.debug.print("  Normal: ({d:.2},{d:.2}), depth: {d:.2}\n", .{ collision.normal.x, collision.normal.y, collision.depth });
        }

        std.debug.print(">>> VELOCITY RESOLUTION <<<\n", .{});
        std.debug.print("Before: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Check if this is a vertical collision between rectangles with very small velocity
        // This is likely a "resting" scenario that causes jittering
        const is_vertical = @abs(collision.normal.y) > 0.9;
        const is_rect_vs_rect = a.shape == .rectangle and b.shape == .rectangle;
        const vertical_rect_collision = is_vertical and is_rect_vs_rect;

        // DEBUG: Special logging for near-rest vertical rect collisions
        if (vertical_rect_collision) {
            const velocity_near_zero = @abs(b.velocity.y) < 0.1;
            if (velocity_near_zero) {
                std.debug.print("JITTER-DEBUG: Near-rest vertical rect collision detected!\n", .{});
                std.debug.print("JITTER-DEBUG: Velocity before resolution: {d:.8}\n", .{b.velocity.y});
                std.debug.print("JITTER-DEBUG: Gravity applied this frame: {d:.8}\n", .{b.force.y});
            }
        }

        // Get the contact point
        const contact = collision.contact_points[0];
        std.debug.print("Contact point: ({d:.4},{d:.4})\n", .{ contact.x, contact.y });

        // Calculate relative velocity at contact point
        const ra = contact.sub(a.position);
        const rb = contact.sub(b.position);
        const relative_velocity = CollisionPhysics.calculateRelativeVelocity(a.velocity, b.velocity, a.angular_velocity, b.angular_velocity, ra, rb);
        std.debug.print("Relative velocity: ({d:.4},{d:.4})\n", .{ relative_velocity.x, relative_velocity.y });

        // Project relative velocity onto the collision normal
        const normal_velocity = relative_velocity.dot(collision.normal);
        std.debug.print("Relative velocity along normal: {d:.4}\n", .{normal_velocity});

        // Better separation handling - only skip if objects are DEFINITELY separating
        // Increase separation tolerance for horizontal collisions to prevent sticking
        var separation_tolerance: f32 = 0.001; // Default very small tolerance

        // Check if this is a horizontal collision between rectangles
        const is_horizontal = @abs(collision.normal.x) > 0.9;
        const horizontal_rect_collision = is_horizontal and is_rect_vs_rect;

        // For horizontal wall collisions, we need to be more aggressive about zeroing out velocity
        // to prevent the object from bouncing back and forth between the wall and ground
        if (horizontal_rect_collision) {
            // For ANY wall collision with low velocity, completely stop horizontal motion
            // (increased threshold from 1.0 to 5.0 to catch more cases)
            if (@abs(normal_velocity) < 5.0) {
                std.debug.print("WALL COLLISION: Complete horizontal stop applied\n", .{});

                // For RIGHT wall collision (normal pointing left), zero X velocity completely
                if (collision.normal.x < 0 and b.body_type == .dynamic) {
                    b.velocity.x = 0;
                    // Force sleeping sooner to prevent further collisions
                    b.low_velocity_frames += 8;
                    if (b.low_velocity_frames > 10) {
                        b.is_sleeping = true;
                        std.debug.print("🛌 FORCED SLEEP: Object at wall is now sleeping\n", .{});
                    }
                }

                // For LEFT wall collision (normal pointing right), zero X velocity completely
                if (collision.normal.x > 0 and a.body_type == .dynamic) {
                    a.velocity.x = 0;
                    // Force sleeping sooner
                    a.low_velocity_frames += 8;
                    if (a.low_velocity_frames > 10) {
                        a.is_sleeping = true;
                        std.debug.print("🛌 FORCED SLEEP: Object at wall is now sleeping\n", .{});
                    }
                }

                // Don't even apply any impulse for wall collisions - just stop
                if (normal_velocity < 0) {
                    std.debug.print("⚠️ WALL IMPACT: Skipping normal impulse resolution\n", .{});
                    return;
                }
            }
        }

        if (horizontal_rect_collision) {
            // Much higher tolerance for horizontal rectangle collisions
            separation_tolerance = 0.05; // 50x higher tolerance for horizontal rect collisions
        }

        // Increased resting contact detection for vertical collisions
        if (is_vertical) {
            // For vertical collisions, increase tolerance for resting contacts
            if (@abs(normal_velocity) < 0.1) {
                separation_tolerance = 0.1; // Higher tolerance for near-zero velocity
            }
        }

        if (normal_velocity > separation_tolerance) {
            std.debug.print("Objects DEFINITELY separating, skipping impulse\n", .{});
            return;
        }

        // Calculate impulse factors
        const ra_cross_n = ra.cross(collision.normal);
        const rb_cross_n = rb.cross(collision.normal);

        const inv_mass_sum = a.inverse_mass + b.inverse_mass +
            (ra_cross_n * ra_cross_n) * a.inverse_inertia +
            (rb_cross_n * rb_cross_n) * b.inverse_inertia;

        if (inv_mass_sum <= 0.0001) return;

        // Calculate restitution based on collision type
        var restitution = collision.restitution;

        // For horizontal rectangles, drastically reduce restitution to prevent bouncing
        if (horizontal_rect_collision) {
            // ZERO restitution for horizontal collisions - no bounce at all
            restitution = 0.0;
            std.debug.print("Horizontal rect-rect collision: ZERO restitution applied\n", .{});
        }

        // Reduce restitution for vertical collisions as well
        if (vertical_rect_collision) {
            // Objects resting on top of each other should have less bounce
            if (collision.normal.y < 0 and @abs(normal_velocity) < 1.0) {
                restitution *= 0.06; // Reduced to 6% of original restitution
                std.debug.print("Vertical rect-rect collision (top): reducing restitution to {d:.4}\n", .{restitution});
            }
        }

        // Adjust restitution using physics equations
        restitution = CollisionPhysics.adjustRestitution(collision.normal, normal_velocity, restitution);

        // Reduce bounciness for low-speed collisions (linear damping)
        const velocity_threshold_for_damping = 10.0;
        if (@abs(normal_velocity) < velocity_threshold_for_damping) {
            restitution *= @abs(normal_velocity) / velocity_threshold_for_damping;
        }

        // Calculate impulse magnitude
        var j = CollisionPhysics.calculateImpulseMagnitude(normal_velocity, restitution, inv_mass_sum);

        // Cap impulse to avoid numerical explosions - REDUCED MAX IMPULSE FOR STABILITY
        const max_impulse = 500.0; // Significantly reduced from 1000.0
        if (@abs(j) > max_impulse) {
            std.debug.print("⚠️ EXCESSIVE IMPULSE CAPPED: {d:.4} → {d:.4}\n", .{ j, std.math.sign(j) * max_impulse });
            j = std.math.sign(j) * max_impulse;
        }

        std.debug.print("Impulse magnitude: {d:.4}\n", .{j});

        // Apply the impulse along the normal only
        const impulse = CollisionPhysics.calculateImpulseVector(collision.normal, j);
        std.debug.print("Impulse vector: ({d:.4},{d:.4})\n", .{ impulse.x, impulse.y });

        // Log before applying impulse
        std.debug.print("Before impulse: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Apply impulse
        applyImpulse(a, b, impulse, ra, rb);

        // Log after applying impulse
        std.debug.print("After impulse: A vel=({d:.6},{d:.6}), B vel=({d:.6},{d:.6})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });

        // Check for sudden large changes in velocity - could indicate instability
        const new_vel_a_len = a.velocity.length();
        const new_vel_b_len = b.velocity.length();
        const velocity_change_a = @abs(new_vel_a_len - vel_a_len);
        const velocity_change_b = @abs(new_vel_b_len - vel_b_len);

        if (velocity_change_a > 100.0 or velocity_change_b > 100.0) {
            std.debug.print("⚠️ LARGE VELOCITY CHANGE DETECTED!\n", .{});
            std.debug.print("  A: {d:.2} → {d:.2} (change: {d:.2})\n", .{ vel_a_len, new_vel_a_len, velocity_change_a });
            std.debug.print("  B: {d:.2} → {d:.2} (change: {d:.2})\n", .{ vel_b_len, new_vel_b_len, velocity_change_b });
        }

        // Apply friction
        applyFriction(a, b, collision, relative_velocity, normal_velocity, j, ra, rb, horizontal_rect_collision);

        // Enhanced sleep detection system for interacting objects
        if (a.body_type == .dynamic and b.body_type == .dynamic) {
            // For objects in contact with each other:
            // 1. They should have similar sleep states to avoid one being awake while pushing a sleeping one
            // 2. They should have similar velocity thresholds to sleep together

            const sleep_velocity_threshold: f32 = 0.05;
            const a_low_vel = a.velocity.lengthSquared() < sleep_velocity_threshold * sleep_velocity_threshold;
            const b_low_vel = b.velocity.lengthSquared() < sleep_velocity_threshold * sleep_velocity_threshold;

            // Both objects have low velocity - increase their sleep counters
            if (a_low_vel and b_low_vel) {
                // Increment both counters
                a.low_velocity_frames += 1;
                b.low_velocity_frames += 1;

                // If either object has enough frames to sleep, make BOTH sleep
                if (a.low_velocity_frames > 8 or b.low_velocity_frames > 8) { // Lower threshold for contacting objects
                    a.is_sleeping = true;
                    b.is_sleeping = true;
                    a.velocity = Vector2.zero();
                    b.velocity = Vector2.zero();
                    std.debug.print("CONTACT SLEEP: Both objects put to sleep\n", .{});
                }
            } else {
                // If either has significant velocity, wake up both
                if (!a_low_vel or !b_low_vel) {
                    a.low_velocity_frames = 0;
                    b.low_velocity_frames = 0;
                    a.is_sleeping = false;
                    b.is_sleeping = false;
                }
            }
        } else {
            // Special case for wall collisions - put objects to sleep faster
            const is_wall_collision = @abs(collision.normal.x) > 0.95;

            // Original sleep logic for single dynamic body against static
            if (a.body_type == .dynamic) {
                const sleep_velocity_threshold: f32 = 0.05;
                if (a.velocity.lengthSquared() < sleep_velocity_threshold * sleep_velocity_threshold) {
                    // Wall collisions make objects sleep faster
                    if (is_wall_collision) {
                        a.low_velocity_frames += 2; // Double increment for wall contacts
                        std.debug.print("Using horizontal wall collision with higher bias\n", .{});
                        std.debug.print("MTV: ({d:.4}, {d:.4}), Depth: {d:.4}\n", .{ collision.normal.x, collision.normal.y, collision.depth });

                        // Even faster sleep for objects right against a wall
                        if (a.velocity.lengthSquared() < 0.01 * 0.01) {
                            a.low_velocity_frames += 2;
                            // Immediately zero horizontal velocity for wall contact
                            if (@abs(collision.normal.x) > 0.9) {
                                a.velocity.x = 0;
                            }
                        }
                    } else {
                        a.low_velocity_frames += 1;
                    }

                    if (a.low_velocity_frames > 6 and is_wall_collision) {
                        a.is_sleeping = true;
                        a.velocity = Vector2.zero();
                        std.debug.print("WALL RESTING: Object put to sleep\n", .{});
                    } else if (a.low_velocity_frames > 10) {
                        a.is_sleeping = true;
                        a.velocity = Vector2.zero();
                    }
                } else {
                    a.low_velocity_frames = 0;
                }
            }

            if (b.body_type == .dynamic) {
                const sleep_velocity_threshold: f32 = 0.05;
                if (b.velocity.lengthSquared() < sleep_velocity_threshold * sleep_velocity_threshold) {
                    // Wall collisions make objects sleep faster
                    if (is_wall_collision) {
                        b.low_velocity_frames += 2; // Double increment for wall contacts

                        // Even faster sleep for objects right against a wall
                        if (b.velocity.lengthSquared() < 0.01 * 0.01) {
                            b.low_velocity_frames += 2;
                            // Immediately zero horizontal velocity for wall contact
                            if (@abs(collision.normal.x) > 0.9) {
                                b.velocity.x = 0;
                            }
                        }
                    } else {
                        b.low_velocity_frames += 1;
                    }

                    if (b.low_velocity_frames > 6 and is_wall_collision) {
                        b.is_sleeping = true;
                        b.velocity = Vector2.zero();
                        std.debug.print("WALL RESTING: Object put to sleep\n", .{});
                    } else if (b.low_velocity_frames > 10) {
                        b.is_sleeping = true;
                        b.velocity = Vector2.zero();
                    }
                } else {
                    b.low_velocity_frames = 0;
                }
            }
        }

        // Final velocities after all impulses
        std.debug.print("Final velocities: A=({d:.6},{d:.6}), B=({d:.6},{d:.6})\n", .{ a.velocity.x, a.velocity.y, b.velocity.x, b.velocity.y });
    }

    /// Apply friction impulse
    fn applyFriction(a: *RigidBody, b: *RigidBody, collision: Collision, relative_velocity: Vector2, normal_velocity: f32, normal_impulse: f32, ra: Vector2, rb: Vector2, is_horizontal_rect_collision: bool) void {
        _ = is_horizontal_rect_collision; // Parameter still needed for API compatibility

        // Check for vertical collision condition (for resting objects)
        const is_vertical = @abs(collision.normal.y) > 0.9;
        const is_ground_collision = is_vertical and collision.normal.y < 0;

        // Calculate tangent vector (perpendicular to normal)
        var tangent = CollisionPhysics.calculateFrictionTangent(relative_velocity, collision.normal, normal_velocity);
        const tangent_length = tangent.length();

        // Only normalize if tangent has meaningful length
        if (tangent_length > 0.0001) {
            tangent = CollisionPhysics.normalizeTangentVector(tangent);
        } else {
            // For objects resting (almost zero tangent vel), assume horizontal friction direction
            if (is_ground_collision) {
                // Create a horizontal friction direction for rolling objects
                tangent = Vector2.init(1.0, 0.0);
                std.debug.print("GROUND CONTACT: Using horizontal friction for rolling objects\n", .{});
            } else {
                // For other cases, try to get a perpendicular vector to the normal
                if (@abs(collision.normal.x) > @abs(collision.normal.y)) {
                    tangent = Vector2.init(0.0, 1.0);
                } else {
                    tangent = Vector2.init(1.0, 0.0);
                }
            }
        }

        // Calculate friction impulse factor
        // Coulomb's Law of Friction (max_friction = mu * normal_force)
        var friction_coef = CollisionPhysics.calculateFrictionCoefficient(a.friction, b.friction);

        // Special case: Increase friction for ground contacts to properly handle rolling
        if (is_ground_collision) {
            // Both static and kinetic friction apply to rolling objects on ground
            if (tangent_length < 0.1) {
                // Static friction (rolling resistance) - slightly stronger
                friction_coef *= 1.2;
                std.debug.print("ROLLING RESISTANCE: Applying static friction to rolling object\n", .{});
            }
        }

        // Calculate friction impulse magnitude (limited by normal impulse)
        var j_t = -(relative_velocity.dot(tangent)) / (a.inverse_mass + b.inverse_mass +
            (ra.cross(tangent) * ra.cross(tangent)) * a.inverse_inertia +
            (rb.cross(tangent) * rb.cross(tangent)) * b.inverse_inertia);

        // Ensure minimum friction for very slow-moving objects on ground
        if (is_ground_collision and @abs(j_t) < 0.01 and
            ((@abs(a.velocity.x) > 0.01 and a.body_type == .dynamic) or
                (@abs(b.velocity.x) > 0.01 and b.body_type == .dynamic)))
        {
            // For extremely slow-moving objects, just stop them completely
            if (a.body_type == .dynamic and @abs(a.velocity.x) < 0.1) {
                a.velocity.x = 0;
                a.low_velocity_frames += 1;
                std.debug.print("RESTING CONTACT: Zeroing extremely slow A velocity\n", .{});
                j_t = 0; // No impulse needed
            } else if (b.body_type == .dynamic and @abs(b.velocity.x) < 0.1) {
                b.velocity.x = 0;
                b.low_velocity_frames += 1;
                std.debug.print("RESTING CONTACT: Zeroing extremely slow B velocity\n", .{});
                j_t = 0; // No impulse needed
            } else {
                // FIX: Calculate the exact impulse needed to stop the object
                if (a.body_type == .dynamic and @abs(a.velocity.x) > 0.001) {
                    // Calculate the impulse needed to stop the object exactly (not overshoot)
                    const impulse_to_stop_a = -a.velocity.x * a.mass;
                    j_t = std.math.clamp(j_t, -@abs(impulse_to_stop_a), @abs(impulse_to_stop_a));
                    std.debug.print("EXACT FRICTION: Applying just enough to stop object A: {d:.6}\n", .{j_t});
                } else if (b.body_type == .dynamic and @abs(b.velocity.x) > 0.001) {
                    // Calculate the impulse needed to stop the object exactly (not overshoot)
                    const impulse_to_stop_b = -b.velocity.x * b.mass;
                    j_t = std.math.clamp(j_t, -@abs(impulse_to_stop_b), @abs(impulse_to_stop_b));
                    std.debug.print("EXACT FRICTION: Applying just enough to stop object B: {d:.6}\n", .{j_t});
                } else {
                    // Extremely small velocity - just zero it out completely
                    if (a.body_type == .dynamic) a.velocity.x = 0;
                    if (b.body_type == .dynamic) b.velocity.x = 0;
                    j_t = 0; // No friction impulse needed
                    std.debug.print("ZEROING VELOCITY: Objects essentially at rest\n", .{});
                }
            }
        }

        // Clamp friction impulse magnitude
        j_t = CollisionPhysics.clampFrictionImpulse(j_t, normal_impulse, friction_coef);

        // Calculate friction impulse vector
        const friction_impulse = tangent.scale(j_t);

        // Store initial velocities for debugging
        const a_vel_before = a.velocity;
        const b_vel_before = b.velocity;

        // Debug print the friction impulse
        std.debug.print("Friction impulse: ({d:.4},{d:.4})\n", .{ friction_impulse.x, friction_impulse.y });

        // CRITICAL FIX: Don't apply friction in the y-direction if it would cause the object to move upward
        var safe_friction_impulse = friction_impulse;

        // For objects on the ground (vertical collision with normal pointing up)
        if (is_vertical and collision.normal.y < 0) {
            // If friction would cause object to move upward, zero out the y component
            if ((a.body_type == .dynamic and (a.velocity.y < 0 and friction_impulse.y * a.inverse_mass > @abs(a.velocity.y))) or
                (b.body_type == .dynamic and (b.velocity.y < 0 and friction_impulse.y * b.inverse_mass < -@abs(b.velocity.y))))
            {
                std.debug.print("⚠️ PREVENTING INVALID UPWARD FRICTION!\n", .{});
                safe_friction_impulse.y = 0;
            }
        }

        // Apply friction impulse
        if (a.body_type == .dynamic) {
            a.velocity = a.velocity.add(safe_friction_impulse.scale(-a.inverse_mass));
            a.angular_velocity -= ra.cross(safe_friction_impulse) * a.inverse_inertia;
        }

        if (b.body_type == .dynamic) {
            b.velocity = b.velocity.add(safe_friction_impulse.scale(b.inverse_mass));
            b.angular_velocity += rb.cross(safe_friction_impulse) * b.inverse_inertia;
        }

        // Check if friction caused significant vertical velocity changes (could indicate instability)
        if (a.body_type == .dynamic and b.body_type == .dynamic) {
            // Check if signs flipped (negative to positive or vice versa) - indicates unstable friction
            const a_y_sign_changed = (a_vel_before.y < 0 and a.velocity.y > 0) or (a_vel_before.y > 0 and a.velocity.y < 0);
            const b_y_sign_changed = (b_vel_before.y < 0 and b.velocity.y > 0) or (b_vel_before.y > 0 and b.velocity.y < 0);

            if (a_y_sign_changed or b_y_sign_changed) {
                std.debug.print("🚨 UNSTABLE FRICTION DETECTED: Vertical velocity direction changed!\n", .{});
                std.debug.print("  A: y-vel {d:.6} → {d:.6}\n", .{ a_vel_before.y, a.velocity.y });
                std.debug.print("  B: y-vel {d:.6} → {d:.6}\n", .{ b_vel_before.y, b.velocity.y });
            }
        }
    }

    /// Apply an impulse to both bodies
    fn applyImpulse(a: *RigidBody, b: *RigidBody, impulse: Vector2, ra: Vector2, rb: Vector2) void {
        // Apply linear impulse
        if (a.body_type != .static) {
            const vel_change = CollisionPhysics.calculateVelocityChange(impulse, -a.inverse_mass);
            a.velocity = a.velocity.add(vel_change);
        }

        if (b.body_type != .static) {
            const vel_change = CollisionPhysics.calculateVelocityChange(impulse, b.inverse_mass);
            b.velocity = b.velocity.add(vel_change);
        }

        // Apply angular impulse
        if (a.body_type != .static) {
            a.angular_velocity += CollisionPhysics.calculateAngularVelocityChange(impulse.scale(-1), ra, a.inverse_inertia);
        }

        if (b.body_type != .static) {
            b.angular_velocity += CollisionPhysics.calculateAngularVelocityChange(impulse, rb, b.inverse_inertia);
        }
    }
};
