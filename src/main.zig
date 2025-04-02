//! ZigEngine - Main executable
//! This demonstrates how to use the engine

const std = @import("std");
const rl = @import("raylib");
const ze = @import("zigengine_lib");

pub fn main() !void {
    // Initialize allocator
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    // Parse command-line arguments
    const options = try ze.core.args.parseArgs(allocator);

    // Initialize the engine with the parsed options
    var game_engine = try ze.core.Engine.init(allocator, 800, 600, "ZigEngine Demo", options);
    defer game_engine.deinit();

    // Define empty handlers for the engine run
    const dummyHandleInput = struct {
        fn handleInput(ctx: *anyopaque, eng: *ze.core.Engine) !void {
            _ = ctx;
            _ = eng;
        }
    }.handleInput;

    const dummyUpdate = struct {
        fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
            _ = ctx;
            _ = eng;
            _ = dt;
        }
    }.update;

    try game_engine.run(@ptrCast(&game_engine), dummyHandleInput, dummyUpdate);
}
