const ze = @import("zigengine_lib");

pub fn main() !void {
    var game_engine = ze.core.Engine.init();

    try game_engine.run();
}
