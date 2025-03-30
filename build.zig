const std = @import("std");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    // Get raylib dependency
    const raylib_dep = b.dependency("raylib", .{
        .target = target,
        .optimize = optimize,
    });

    // Get raylib module
    const raylib_module = raylib_dep.module("raylib");

    // Get the raylib artifact for linking
    const raylib_artifact = raylib_dep.artifact("raylib");

    // Create all our engine modules
    const physics_mod = b.createModule(.{
        .root_source_file = b.path("src/physics/mod.zig"),
        .target = target,
        .optimize = optimize,
    });

    const input_mod = b.createModule(.{
        .root_source_file = b.path("src/input/mod.zig"),
        .target = target,
        .optimize = optimize,
    });

    const renderer_mod = b.createModule(.{
        .root_source_file = b.path("src/renderer/mod.zig"),
        .target = target,
        .optimize = optimize,
    });

    const core_mod = b.createModule(.{
        .root_source_file = b.path("src/core/mod.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Set up module dependencies
    physics_mod.addImport("raylib", raylib_module);
    input_mod.addImport("raylib", raylib_module);
    renderer_mod.addImport("raylib", raylib_module);
    renderer_mod.addImport("physics", physics_mod);

    core_mod.addImport("physics", physics_mod);
    core_mod.addImport("input", input_mod);
    core_mod.addImport("renderer", renderer_mod);
    core_mod.addImport("raylib", raylib_module);

    // Create the main engine module
    const engine_mod = b.createModule(.{
        .root_source_file = b.path("src/mod.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Engine module depends on all other modules
    engine_mod.addImport("core", core_mod);
    engine_mod.addImport("physics", physics_mod);
    engine_mod.addImport("input", input_mod);
    engine_mod.addImport("renderer", renderer_mod);
    engine_mod.addImport("raylib", raylib_module);

    // This creates a "module", which represents a collection of source files alongside
    // some compilation options, such as optimization mode and linked system libraries.
    // Every executable or library we compile will be based on one or more modules.
    const lib_mod = b.createModule(.{
        // `root_source_file` is the Zig "entry point" of the module. If a module
        // only contains e.g. external object files, you can make this `null`.
        // In this case the main source file is merely a path, however, in more
        // complicated build scripts, this could be a generated file.
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Add raylib import to the library module
    lib_mod.addImport("raylib", raylib_module);
    lib_mod.addImport("engine", engine_mod);

    // We will also create a module for our other entry point, 'main.zig'.
    const exe_mod = b.createModule(.{
        // `root_source_file` is the Zig "entry point" of the module. If a module
        // only contains e.g. external object files, you can make this `null`.
        // In this case the main source file is merely a path, however, in more
        // complicated build scripts, this could be a generated file.
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });

    // Modules can depend on one another using the `std.Build.Module.addImport` function.
    // This is what allows Zig source code to use `@import("foo")` where 'foo' is not a
    // file path. In this case, we set up `exe_mod` to import `lib_mod`.
    exe_mod.addImport("zigengine_lib", lib_mod);

    // Add raylib import to the executable module
    exe_mod.addImport("raylib", raylib_module);

    // Now, we will create a static library based on the module we created above.
    // This creates a `std.Build.Step.Compile`, which is the build step responsible
    // for actually invoking the compiler.
    const lib = b.addLibrary(.{
        .linkage = .static,
        .name = "zigengine",
        .root_module = lib_mod,
    });

    // Link against the raylib artifact
    lib.linkLibrary(raylib_artifact);

    // This declares intent for the library to be installed into the standard
    // location when the user invokes the "install" step (the default step when
    // running `zig build`).
    b.installArtifact(lib);

    // This creates another `std.Build.Step.Compile`, but this one builds an executable
    // rather than a static library.
    const exe = b.addExecutable(.{
        .name = "zigengine",
        .root_module = exe_mod,
    });

    // Link against the raylib artifact
    exe.linkLibrary(raylib_artifact);

    // This declares intent for the executable to be installed into the
    // standard location when the user invokes the "install" step (the default
    // step when running `zig build`).
    b.installArtifact(exe);

    // This *creates* a Run step in the build graph, to be executed when another
    // step is evaluated that depends on it. The next line below will establish
    // such a dependency.
    const run_cmd = b.addRunArtifact(exe);

    // By making the run step depend on the install step, it will be run from the
    // installation directory rather than directly from within the cache directory.
    // This is not necessary, however, if the application depends on other installed
    // files, this ensures they will be present and in the expected location.
    run_cmd.step.dependOn(b.getInstallStep());

    // This allows the user to pass arguments to the application in the build
    // command itself, like this: `zig build run -- arg1 arg2 etc`
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    // This creates a build step. It will be visible in the `zig build --help` menu,
    // and can be selected like this: `zig build run`
    // This will evaluate the `run` step rather than the default, which is "install".
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    // Creates a step for unit testing. This only builds the test executable
    // but does not run it.
    const lib_unit_tests = b.addTest(.{
        .root_module = lib_mod,
    });

    const run_lib_unit_tests = b.addRunArtifact(lib_unit_tests);

    const exe_unit_tests = b.addTest(.{
        .root_module = exe_mod,
    });

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);

    // Similar to creating the run step earlier, this exposes a `test` step to
    // the `zig build --help` menu, providing a way for the user to request
    // running the unit tests.
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_lib_unit_tests.step);
    test_step.dependOn(&run_exe_unit_tests.step);

    // Build examples
    const examples_step = b.step("examples", "Build all examples");

    // Define the examples to build
    const examples = [_]struct { name: []const u8, path: []const u8 }{
        .{ .name = "basic", .path = "examples/basic/main.zig" },
        .{ .name = "collision", .path = "examples/collision/main.zig" },
    };

    for (examples) |example| {
        // Create a module for this example
        const example_mod = b.createModule(.{
            .root_source_file = b.path(example.path),
            .target = target,
            .optimize = optimize,
        });

        // Add dependencies
        example_mod.addImport("raylib", raylib_module);
        example_mod.addImport("zigengine_lib", lib_mod);

        // Create the executable
        const example_exe = b.addExecutable(.{
            .name = example.name,
            .root_module = example_mod,
        });

        // Link against raylib
        example_exe.linkLibrary(raylib_artifact);

        // Install the example to the "examples" output directory
        b.installArtifact(example_exe);

        // Create a run step for this example
        const run_example_cmd = b.addRunArtifact(example_exe);

        // Allow passing args to the example
        if (b.args) |args| {
            run_example_cmd.addArgs(args);
        }

        // Create a step specifically for this example
        const run_example_step = b.step(b.fmt("run-{s}", .{example.name}), b.fmt("Run the {s} example", .{example.name}));
        run_example_step.dependOn(&run_example_cmd.step);

        // Add this example to the general examples step
        examples_step.dependOn(&example_exe.step);
    }
}
