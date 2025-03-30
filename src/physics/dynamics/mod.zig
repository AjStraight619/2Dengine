// Dynamics module exports
pub const Forces = @import("forces.zig").Forces;
pub const Integrator = @import("integration.zig").Integrator;

// Re-export specific integration methods for convenience
pub const euler = Integrator.euler;
pub const semiImplicitEuler = Integrator.semiImplicitEuler;
pub const verlet = Integrator.verlet;

// Re-export specific force methods for convenience
pub const applyGravity = Forces.applyGravity;
pub const applyDrag = Forces.applyDrag;
