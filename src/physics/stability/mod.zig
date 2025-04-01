// Export stability components
pub const SleepManager = @import("sleep.zig").SleepManager;
pub const StabilitySafeguards = @import("safeguards.zig").StabilitySafeguards;

// Convenience re-exports of commonly used functions
pub const forceCollectiveSleep = SleepManager.forceCollectiveSleep;
pub const handleContactSleeping = SleepManager.handleContactSleeping;
pub const handleWallCollisionSleep = SleepManager.handleWallCollisionSleep;
pub const applyVelocityCaps = StabilitySafeguards.applyVelocityCaps;
pub const checkExtremeCollision = StabilitySafeguards.checkExtremeCollision;
pub const sanitizeVelocity = StabilitySafeguards.sanitizeVelocity;
