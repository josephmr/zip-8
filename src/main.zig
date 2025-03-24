const std = @import("std");
const builtin = @import("builtin");
const rl = @import("raylib");
const rg = @import("raygui");
const Cpu = @import("./cpu.zig");
const Instruction = @import("./instruction.zig").Instruction;
const ArrayList = std.ArrayListUnmanaged;

const debug = false;
const instructions_per_frame = if (debug) 1 else 11;

pub var text_font: rl.Font = undefined;
pub const font_size = 20;
pub const resolution_multiplier = 14;
pub const game_width = 64 * resolution_multiplier;
pub const game_height = 32 * resolution_multiplier;
pub const gui_width = 32 * resolution_multiplier;
pub const gui_height = 32 * resolution_multiplier;
pub const window_width = game_width + gui_width;
pub const window_height = game_height + gui_height;

const native_os = builtin.os.tag;
var debug_allocator: std.heap.DebugAllocator(.{}) = .init;

const State = enum {
    picker,
    game,
};

var current_state: State = .picker;

var picker_scroll_index: i32 = 0;
var picker_active: i32 = -1;
var picker_focus: i32 = -1;

fn draw_picker(roms: ArrayList([*:0]const u8)) ?[]const u8 {
    _ = rg.guiListViewEx(
        rl.Rectangle.init(window_width / 3, window_height / 4, window_width / 3, window_height / 2),
        roms.items,
        &picker_scroll_index,
        &picker_active,
        &picker_focus,
    );
    if (picker_active >= 0 and picker_active < roms.items.len) {
        return std.mem.span(roms.items[@intCast(picker_active)]);
    }
    return null;
}

pub fn main() !void {
    const gpa, const is_debug = gpa: {
        // TODO: unsure why std.heap.wasm_allocator isn't working here just freezes on first alloc in browser
        if (native_os == .wasi or native_os == .emscripten) {
            break :gpa .{ std.heap.c_allocator, false };
        }
        break :gpa switch (builtin.mode) {
            .Debug, .ReleaseSafe => .{ debug_allocator.allocator(), true },
            .ReleaseFast, .ReleaseSmall => .{ std.heap.smp_allocator, false },
        };
    };
    defer if (is_debug) {
        _ = debug_allocator.deinit();
    };

    rl.initWindow(window_width, window_height, "zip-8");
    defer rl.closeWindow();

    text_font = try rl.loadFontEx("resources/fira_mono.otf", font_size, null);
    rg.guiLoadStyle("resources/style_amber.rgs");

    var cpu = try Cpu.init();

    var roms_list = ArrayList([*:0]const u8){};
    defer {
        for (roms_list.items) |rom| {
            gpa.free(std.mem.span(rom));
        }
        roms_list.deinit(gpa);
    }
    // HACK: cannot list dir in wasm, just hardcoding list of roms for now
    //      maybe the list of roms can be retrieved from the embed files somehow
    switch (builtin.os.tag) {
        .wasi, .emscripten => {
            try roms_list.append(gpa, "br8kout.ch8");
            try roms_list.append(gpa, "airplane.ch8");
            try roms_list.append(gpa, "rps.ch8");
            try roms_list.append(gpa, "1-chip8-logo.ch8");
            try roms_list.append(gpa, "2-ibm-logo.ch8");
            try roms_list.append(gpa, "3-corax+.ch8");
            try roms_list.append(gpa, "4-flags.ch8");
            try roms_list.append(gpa, "5-quirks.ch8");
        },
        else => {
            var roms = try std.fs.cwd().openDir("roms", .{ .iterate = true });
            defer roms.close();

            var roms_iter = roms.iterate();
            while (try roms_iter.next()) |rom| {
                try roms_list.append(gpa, try gpa.dupeZ(u8, rom.name));
            }
        },
    }

    var has_drawn = false;
    rl.setTargetFPS(60);
    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();
        rl.clearBackground(rl.Color.black);

        switch (current_state) {
            .picker => {
                const rom_name = draw_picker(roms_list);
                if (rom_name) |name| {
                    const path = try std.fs.path.join(gpa, &[_][]const u8{ "roms", name });
                    defer gpa.free(path);
                    const read = try std.fs.cwd().readFileAlloc(gpa, path, 4096 - 0x200);
                    defer gpa.free(read);
                    cpu.loadRom(read);

                    picker_active = -1;
                    picker_scroll_index = 0;
                    picker_focus = -1;
                    current_state = .game;
                    has_drawn = false;
                }
            },
            .game => {
                if (rl.isKeyPressed(rl.KeyboardKey.m)) {
                    current_state = .picker;
                    cpu = try Cpu.init();
                    continue;
                }

                // draw once before starting to step
                if (!has_drawn) {
                    cpu.draw();
                    has_drawn = true;
                    continue;
                }

                if (debug and !rl.isKeyPressed(rl.KeyboardKey.s)) {
                    cpu.draw();
                    continue;
                }

                cpu.delay -|= 1;
                cpu.sound -|= 1;

                for (0..instructions_per_frame) |_| {
                    cpu.step();
                }
                cpu.draw();
            },
        }
    }
}
