const std = @import("std");
const rl = @import("raylib");

// Define the different input device types supported
pub const InputDevice = enum {
    keyboard,
    mouse,
    gamepad,
};

// Define input identifiers
pub const InputCode = union(InputDevice) {
    keyboard: rl.KeyboardKey,
    mouse: rl.MouseButton,
    gamepad: struct {
        button: rl.GamepadButton,
        gamepad_id: i32 = 0,
    },
};

// Input states
pub const InputState = enum {
    pressed, // Just pressed this frame
    released, // Just released this frame
    down, // Currently held down
    up, // Currently up
};

// The action identifier - can be any string
pub const ActionId = []const u8;

// Binding between an action and an input
pub const InputBinding = struct {
    action: ActionId,
    input: InputCode,
};

// Callback function signature
pub const ActionCallback = *const fn (context: *anyopaque) void;

// Action handler
pub const ActionHandler = struct {
    callback: ActionCallback,
    context: *anyopaque,
    state: InputState,
};

// Helper function to convert a generic callback pointer to a specific type
pub fn getCallbackContext(comptime T: type, context: *anyopaque) *T {
    const aligned: *align(@alignOf(T)) anyopaque = @alignCast(context);
    return @ptrCast(aligned);
}

// Input Manager
pub const InputManager = struct {
    allocator: std.mem.Allocator,
    bindings: std.StringArrayHashMap(std.ArrayList(InputCode)),
    handlers: std.StringArrayHashMap(std.ArrayList(ActionHandler)),

    // Track previous frame key states
    prev_key_states: std.AutoHashMap(rl.KeyboardKey, bool),
    prev_mouse_states: std.AutoHashMap(rl.MouseButton, bool),

    pub fn init(allocator: std.mem.Allocator) !*InputManager {
        const manager = try allocator.create(InputManager);
        manager.* = InputManager{
            .allocator = allocator,
            .bindings = std.StringArrayHashMap(std.ArrayList(InputCode)).init(allocator),
            .handlers = std.StringArrayHashMap(std.ArrayList(ActionHandler)).init(allocator),
            .prev_key_states = std.AutoHashMap(rl.KeyboardKey, bool).init(allocator),
            .prev_mouse_states = std.AutoHashMap(rl.MouseButton, bool).init(allocator),
        };
        return manager;
    }

    pub fn deinit(self: *InputManager) void {
        // Free all action handler arrays
        var handler_it = self.handlers.iterator();
        while (handler_it.next()) |entry| {
            entry.value_ptr.deinit();
        }
        self.handlers.deinit();

        // Free all input binding arrays
        var binding_it = self.bindings.iterator();
        while (binding_it.next()) |entry| {
            entry.value_ptr.deinit();
        }
        self.bindings.deinit();

        // Free state maps
        self.prev_key_states.deinit();
        self.prev_mouse_states.deinit();

        // Free the manager itself
        self.allocator.destroy(self);
    }

    // Bind an input to an action
    pub fn bindInput(self: *InputManager, action: ActionId, input: InputCode) !void {
        // Get or create the list of inputs for this action
        if (!self.bindings.contains(action)) {
            try self.bindings.put(action, std.ArrayList(InputCode).init(self.allocator));
        }

        var inputs = self.bindings.getPtr(action).?;
        try inputs.append(input);
    }

    // Bind multiple inputs to an action
    pub fn bindInputs(self: *InputManager, action: ActionId, inputs: []const InputCode) !void {
        for (inputs) |input| {
            try self.bindInput(action, input);
        }
    }

    // Unbind an input from an action
    pub fn unbindInput(self: *InputManager, action: ActionId, input: InputCode) !void {
        if (!self.bindings.contains(action)) {
            return;
        }

        var inputs = self.bindings.getPtr(action).?;

        // Find and remove the input
        var i: usize = 0;
        while (i < inputs.items.len) {
            // This is a simple comparison - you might need a more complex one based on your needs
            const current = inputs.items[i];
            const matches = switch (input) {
                .keyboard => |key| current == .keyboard and current.keyboard == key,
                .mouse => |button| current == .mouse and current.mouse == button,
                .gamepad => |gamepad| current == .gamepad and
                    current.gamepad.button == gamepad.button and
                    current.gamepad.gamepad_id == gamepad.gamepad_id,
            };

            if (matches) {
                _ = inputs.swapRemove(i);
            } else {
                i += 1;
            }
        }
    }

    // Register a callback for an action
    pub fn registerAction(self: *InputManager, action: ActionId, callback: ActionCallback, context: *anyopaque, state: InputState) !void {
        // Get or create the list of handlers for this action
        if (!self.handlers.contains(action)) {
            try self.handlers.put(action, std.ArrayList(ActionHandler).init(self.allocator));
        }

        var handlers = self.handlers.getPtr(action).?;
        try handlers.append(.{
            .callback = callback,
            .context = context,
            .state = state,
        });
    }

    // Unregister an action callback
    pub fn unregisterAction(self: *InputManager, action: ActionId, callback: ActionCallback, context: *anyopaque) void {
        if (!self.handlers.contains(action)) {
            return;
        }

        var handlers = self.handlers.getPtr(action).?;

        // Find and remove the handler
        var i: usize = 0;
        while (i < handlers.items.len) {
            if (handlers.items[i].callback == callback and handlers.items[i].context == context) {
                _ = handlers.swapRemove(i);
            } else {
                i += 1;
            }
        }
    }

    // Check if an action is in a specific state
    pub fn isActionState(self: *InputManager, action: ActionId, state: InputState) bool {
        if (!self.bindings.contains(action)) {
            return false;
        }

        const bindings = self.bindings.get(action).?;
        for (bindings.items) |input| {
            const is_active = switch (input) {
                .keyboard => |key| self.isKeyState(key, state),
                .mouse => |button| self.isMouseState(button, state),
                .gamepad => |gamepad| self.isGamepadState(gamepad.button, gamepad.gamepad_id, state),
            };

            if (is_active) {
                return true;
            }
        }

        return false;
    }

    // Convenience functions for each state
    pub fn isActionPressed(self: *InputManager, action: ActionId) bool {
        return self.isActionState(action, .pressed);
    }

    pub fn isActionReleased(self: *InputManager, action: ActionId) bool {
        return self.isActionState(action, .released);
    }

    pub fn isActionDown(self: *InputManager, action: ActionId) bool {
        return self.isActionState(action, .down);
    }

    pub fn isActionUp(self: *InputManager, action: ActionId) bool {
        return self.isActionState(action, .up);
    }

    // Check if a key is in a specific state
    fn isKeyState(self: *InputManager, key: rl.KeyboardKey, state: InputState) bool {
        const is_down = rl.isKeyDown(key);
        const was_down = self.prev_key_states.get(key) orelse false;

        return switch (state) {
            .pressed => is_down and !was_down,
            .released => !is_down and was_down,
            .down => is_down,
            .up => !is_down,
        };
    }

    // Check if a mouse button is in a specific state
    fn isMouseState(self: *InputManager, button: rl.MouseButton, state: InputState) bool {
        const is_down = rl.isMouseButtonDown(button);
        const was_down = self.prev_mouse_states.get(button) orelse false;

        return switch (state) {
            .pressed => is_down and !was_down,
            .released => !is_down and was_down,
            .down => is_down,
            .up => !is_down,
        };
    }

    // Check if a gamepad button is in a specific state
    fn isGamepadState(self: *InputManager, button: rl.GamepadButton, gamepad_id: i32, state: InputState) bool {
        _ = self; // unused

        const is_down = rl.isGamepadButtonDown(gamepad_id, button);
        // Note: We don't track previous gamepad states yet - would need additional maps

        return switch (state) {
            .pressed => rl.isGamepadButtonPressed(gamepad_id, button),
            .released => rl.isGamepadButtonReleased(gamepad_id, button),
            .down => is_down,
            .up => !is_down,
        };
    }

    // Update the input manager (call once per frame)
    pub fn update(self: *InputManager) !void {
        // First, trigger all registered callbacks
        self.triggerCallbacks();

        // Then update the previous state maps for next frame
        try self.updatePreviousStates();
    }

    // Trigger all registered callbacks based on current input states
    fn triggerCallbacks(self: *InputManager) void {
        var handler_it = self.handlers.iterator();
        while (handler_it.next()) |entry| {
            const action = entry.key_ptr.*;
            const handlers = entry.value_ptr;

            for (handlers.items) |handler| {
                if (self.isActionState(action, handler.state)) {
                    handler.callback(handler.context);
                }
            }
        }
    }

    // Store the current input states for comparison next frame
    fn updatePreviousStates(self: *InputManager) !void {
        // Clear previous maps
        self.prev_key_states.clearRetainingCapacity();
        self.prev_mouse_states.clearRetainingCapacity();

        // Store all keys we care about
        var binding_it = self.bindings.iterator();
        while (binding_it.next()) |entry| {
            for (entry.value_ptr.items) |input| {
                switch (input) {
                    .keyboard => |key| {
                        try self.prev_key_states.put(key, rl.isKeyDown(key));
                    },
                    .mouse => |button| {
                        try self.prev_mouse_states.put(button, rl.isMouseButtonDown(button));
                    },
                    .gamepad => {
                        // We don't store gamepad states yet - could add if needed
                    },
                }
            }
        }
    }
};
