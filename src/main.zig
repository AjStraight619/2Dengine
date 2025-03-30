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

test "simple test" {
    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // Try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}

test "use other module" {
    try std.testing.expectEqual(@as(i32, 150), lib.add(100, 50));
}

test "fuzz example" {
    const Context = struct {
        fn testOne(context: @This(), input: []const u8) anyerror!void {
            _ = context;
            // Try passing `--fuzz` to `zig build test` and see if it manages to fail this test case!
            try std.testing.expect(!std.mem.eql(u8, "canyoufindme", input));
        }
    };
    try std.testing.fuzz(Context{}, Context.testOne, .{});
}

/// This imports the separate module containing `root.zig`. Take a look in `build.zig` for details.
const lib = @import("zigengine_lib");
