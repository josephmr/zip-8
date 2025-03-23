const std = @import("std");
const builtin = @import("builtin");
const rl = @import("raylib");
const rg = @import("raygui");
const Cpu = @import("./cpu.zig");
const Instruction = @import("./instruction.zig").Instruction;

const debug = false;
const instructions_per_frame = if (debug) 1 else 11;

pub var text_font: rl.Font = undefined;
pub const font_size = 20;
pub const resolution_multiplier = 14;
pub const game_width = 64 * resolution_multiplier;
pub const game_height = 32 * resolution_multiplier;
pub const gui_width = 32 * resolution_multiplier;
pub const gui_height = 32 * resolution_multiplier;

fn hexDump(buf: []const u8) void {
    for (0..buf.len / 16) |offset| {
        const addr = offset * 16;
        std.debug.print("{X:0>4}:", .{addr});
        for (0..8) |i| {
            std.debug.print(" {X:0>2}{X:0>2}", .{ buf[addr + i * 2], buf[addr + i * 2 + 1] });
        }
        std.debug.print("\n", .{});
    }
}

const native_os = builtin.os.tag;
var debug_allocator: std.heap.DebugAllocator(.{}) = .init;

pub fn main() !void {
    const gpa, const is_debug = gpa: {
        if (native_os == .wasi) break :gpa .{ std.heap.wasm_allocator, false };
        break :gpa switch (builtin.mode) {
            .Debug, .ReleaseSafe => .{ debug_allocator.allocator(), true },
            .ReleaseFast, .ReleaseSmall => .{ std.heap.smp_allocator, false },
        };
    };
    defer if (is_debug) {
        _ = debug_allocator.deinit();
    };

    rl.initWindow(game_width + gui_width, game_height + gui_height, "zip-8");
    defer rl.closeWindow();

    text_font = try rl.loadFontEx("resources/fira_mono.otf", font_size, null);
    rg.guiLoadStyle("resources/style_amber.rgs");

    var state = try Cpu.init();
    // try state.loadRom("roms/1-chip8-logo.ch8");
    // try state.loadRom("roms/2-ibm-logo.ch8");
    // try state.loadRom("roms/3-corax+.ch8");
    // try state.loadRom("roms/4-flags.ch8");
    // try state.loadRom("roms/5-quirks.ch8");
    // try state.loadRom("roms/maze.ch8");
    // try state.loadRom("roms/airplane.ch8");
    // try state.loadRom("roms/rps.ch8");
    // try state.loadRom("roms/delay_timer_test.ch8");
    // try state.loadRom("roms/draw_offset.ch8");

    {
        const read = try std.fs.cwd().readFileAlloc(gpa, "roms/br8kout.ch8", 4096 - 0x200);
        defer gpa.free(read);
        state.loadRom(read);
    }

    var has_drawn = false;
    rl.setTargetFPS(60);
    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();
        rl.clearBackground(rl.Color.black);

        // draw once before starting to step
        if (!has_drawn) {
            state.draw();
            has_drawn = true;
            continue;
        }

        if (debug and !rl.isKeyPressed(rl.KeyboardKey.s)) {
            state.draw();
            continue;
        }

        state.delay -|= 1;
        state.sound -|= 1;

        for (0..instructions_per_frame) |_| {
            state.step();
        }
        state.draw();
    }
}
