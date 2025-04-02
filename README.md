# ZigEngine

A lightweight game engine written in Zig using Raylib. ZigEngine provides a simple framework for building 2D games with a clean API and minimal boilerplate.

## Features

- Simple initialization API
- Command-line argument parsing
- Clean architecture with engine core and systems
- Raylib integration for rendering and input handling
- Memory-safe design using Zig's allocator pattern

## Requirements

- Zig compiler (latest version recommended) 0.14.0
- Raylib

## Project Structure

```
zigengine/
├── src/
│   ├── main.zig          # Demo application
│   └── zigengine_lib/    # Engine library code
│       ├── core/         # Core engine systems
│       ├── utils/        # Utility functions
│       └── renderer/     # Rendering system
├── build.zig            # Build configuration
├── examples/            # Example applications
└── README.md
```

## Architecture

ZigEngine follows a modular design:

- **Core Engine**: Manages the game loop, initialization, and cleanup
- **Input System**: Handles keyboard, mouse, and other input devices
- **Rendering**: Abstracts Raylib for drawing sprites, text, and shapes
- **Resource Management**: Efficiently manages game assets using Zig allocators

## Getting Started

Clone the repository:

```
git clone https://github.com/yourusername/zigengine.git
cd zigengine
```

Build the project:

```
zig build
```

Run the demo:

```
zig build run
```

## Usage

Basic game setup:

```zig
// Initialize allocator
var gpa = std.heap.GeneralPurposeAllocator(.{}){};
defer _ = gpa.deinit();
const allocator = gpa.allocator();

// Parse command-line arguments
const options = try ze.core.args.parseArgs(allocator);

// Initialize the engine
var game_engine = try ze.core.Engine.init(allocator, width, height, "My Game", options);
defer game_engine.deinit();

// Define input handler
const handleInput = struct {
    fn handle(ctx: *anyopaque, eng: *ze.core.Engine) !void {
        // Handle keyboard/mouse input here
    }
}.handle;

// Define update logic
const update = struct {
    fn update(ctx: *anyopaque, eng: *ze.core.Engine, dt: f32) !void {
        // Update game state here using delta time (dt)
    }
}.update;

// Run the game loop
try game_engine.run(context, handleInput, update);
```

## Examples

The `examples/` directory contains various sample applications demonstrating different features of ZigEngine:

- **basic**: Simple physics with a bouncing circle
- **collision**: Different collision scenarios
- **circlevsrect**: Circle vs. rectangle collision testing
- **circlevscircle**: Circle vs. circle collision testing
- **forces**: Demo of various physics forces
- **friction_test**: Testing friction coefficients
- **performance_test**: Test engine performance with many objects
- **rotation**: Tests with rotational physics

Run any example with:

```
zig build run-example -- example_name
```

### Using the Debug System

ZigEngine includes a unified debug system that provides visualization of forces, velocities, collisions, and performance metrics. To use it in your examples:

```zig
// Enable debug mode for an example
game_engine.debug_mode = true;
game_engine.show_debug_overlay = true;

// The preferred way to run with debug features enabled
try game_engine.runWithDebug(@ptrCast(&game_engine), handleInput, update);

// In your input handler, process debug keys:
fn handleInput(ctx: *anyopaque, eng: *ze.core.Engine) !void {
    // This processes all debug-related keyboard shortcuts
    eng.debug_system.processDebugKeys(eng);
    
    // Rest of your input handling
}
```

**Available Debug Controls:**
- **D**: Toggle debug mode
- **O**: Toggle overlay
- **F**: Toggle force vectors
- **V**: Toggle velocity vectors
- **N**: Toggle normal vectors
- **B**: Toggle AABBs
- **+/-**: Adjust force scale visualization

**Note:** Avoid drawing your own debug information in the top-left corner to prevent overlap with the engine's debug overlay.

## Command-line Options

ZigEngine supports various command-line options for customization:

```
--width=X       Set window width
--height=Y      Set window height
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

MIT 