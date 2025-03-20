const std = @import("std");
const rl = @import("raylib");
const rg = @import("raygui");
const Instruction = @import("./instruction.zig").Instruction;

var text_font: rl.Font = undefined;
const debug = false;
const instructions_per_frame = if (debug) 1 else 11;
const font_size = 20;
const resolution_multiplier = 14;
const game_width = 64 * resolution_multiplier;
const game_height = 32 * resolution_multiplier;
const gui_width = 32 * resolution_multiplier;
const gui_height = 32 * resolution_multiplier;

const keys = [_]rl.KeyboardKey{
    rl.KeyboardKey.x,
    rl.KeyboardKey.one,
    rl.KeyboardKey.two,
    rl.KeyboardKey.three,
    rl.KeyboardKey.q,
    rl.KeyboardKey.w,
    rl.KeyboardKey.e,
    rl.KeyboardKey.a,
    rl.KeyboardKey.s,
    rl.KeyboardKey.d,
    rl.KeyboardKey.z,
    rl.KeyboardKey.c,
    rl.KeyboardKey.four,
    rl.KeyboardKey.r,
    rl.KeyboardKey.f,
    rl.KeyboardKey.v,
};

const font = [_]u8{
    0xF0, 0x90, 0x90, 0x90, 0xF0, // 0
    0x20, 0x60, 0x20, 0x20, 0x70, // 1
    0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
    0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
    0x90, 0x90, 0xF0, 0x10, 0x10, // 4
    0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
    0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
    0xF0, 0x10, 0x20, 0x40, 0x40, // 7
    0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
    0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
    0xF0, 0x90, 0xF0, 0x90, 0x90, // A
    0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
    0xF0, 0x80, 0x80, 0x80, 0xF0, // C
    0xE0, 0x90, 0x90, 0x90, 0xE0, // D
    0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
    0xF0, 0x80, 0xF0, 0x80, 0x80, // F
};

const State = struct {
    ram: [4096]u8,

    // registers
    registers: [16]u8,
    i: u16,

    // special sound/delay registers
    sound: u8,
    delay: u8,

    // program counter
    pc: u16,

    // stack pointer
    sp: u16,
    stack: [16]u16,

    // display buffer
    display: [32][8]u8,

    // rng generator
    rng: std.Random.DefaultPrng,

    pub const empty: State = .{
        .ram = font ++ [_]u8{0} ** (4096 - font.len),
        .registers = @splat(0),
        .sound = 0,
        .delay = 0,
        .i = 0,
        .pc = 0x200,
        .sp = 0,
        .stack = @splat(0),
        .display = @splat(@splat(0)),
        .rng = undefined,
    };

    fn init() !State {
        var state = State.empty;
        var seed: u64 = undefined;
        try std.posix.getrandom(std.mem.asBytes(&seed));
        state.rng = std.Random.DefaultPrng.init(seed);
        return state;
    }

    fn loadRom(state: *State, path: []const u8) !void {
        const read = try std.fs.cwd().readFile(path, state.ram[0x200..]);
        std.log.debug("loadRom read {} bytes", .{read.len});
    }

    fn pcValue(state: State) u16 {
        return std.mem.readInt(u16, state.ram[state.pc..][0..2], .big);
    }

    fn step(state: *State) void {
        const instruction = Instruction.decode(state.pcValue());

        const vx = instruction.xyn.x;
        const vy = instruction.xyn.y;
        const n = instruction.xyn.n;
        const nn = instruction.xnn.nn;
        const nnn = instruction.nnn.nnn;

        switch (instruction.op_code().?) {
            .clear => {
                state.display = @splat(@splat(0));
            },
            .ret => {
                std.debug.assert(state.sp > 0);
                state.sp -= 1;
                const addr = state.stack[state.sp];
                state.pc = addr;
            },
            .jump => {
                state.pc = nnn;
                return;
            },
            .call => {
                state.stack[state.sp] = state.pc;
                state.sp += 1;
                state.pc = nnn;
                return;
            },
            .skip_eq_xnn => {
                if (state.registers[vx] == nn) {
                    state.pc += 2;
                }
            },
            .skip_neq_xnn => {
                if (state.registers[vx] != nn) {
                    state.pc += 2;
                }
            },
            .skip_eq_xy => {
                if (state.registers[vx] == state.registers[vy]) {
                    state.pc += 2;
                }
            },
            .load_byte => {
                state.registers[vx] = nn;
            },
            .add_byte => {
                state.registers[vx] +%= nn;
            },
            .set_xy => {
                state.registers[vx] = state.registers[vy];
            },
            .set_or => {
                state.registers[vx] |= state.registers[vy];
            },
            .set_and => {
                state.registers[vx] &= state.registers[vy];
            },
            .set_xor => {
                state.registers[vx] ^= state.registers[vy];
            },
            .add_xy => {
                const result = @addWithOverflow(state.registers[vx], state.registers[vy]);
                state.registers[vx] = result[0];
                state.registers[0xF] = result[1];
            },
            .sub_xy => {
                const result = @subWithOverflow(state.registers[vx], state.registers[vy]);
                state.registers[vx] = result[0];
                state.registers[0xF] = if (result[1] == 0b1) 0b0 else 0b1;
            },
            .set_sh_right => {
                state.registers[vx] = state.registers[vy];
                const flag = state.registers[vx] & 0x01;
                state.registers[vx] = state.registers[vy] >> 1;
                state.registers[0xF] = flag;
            },
            .sub_yx => {
                const result = @subWithOverflow(state.registers[vy], state.registers[vx]);
                state.registers[vx] = result[0];
                state.registers[0xF] = if (result[1] == 0b1) 0b0 else 0b1;
            },
            .set_sh_left => {
                const result = @shlWithOverflow(state.registers[vy], 1);
                state.registers[vx] = result[0];
                state.registers[0xF] = result[1];
            },
            .skip_neq_xy => {
                if (state.registers[vx] != state.registers[vy]) {
                    state.pc += 2;
                }
            },
            .load_addr => {
                state.i = nnn;
            },
            .jump_v0 => {
                state.pc = state.registers[0x0] + nnn;
                return;
            },
            .rand => {
                state.registers[vx] = @as(u8, @truncate(state.rng.next())) & nn;
            },
            .draw => {
                const x = state.registers[vx] % 64;
                const y = state.registers[vy] % 32;

                const sprite_len = n;
                const sprite = state.ram[state.i..][0..sprite_len];

                // reset flag register
                state.registers[0xF] = 0;

                for (sprite, y..) |byte, row| {
                    const byte_index = @divTrunc(x, 8);

                    // Do not wrap sprites that clip
                    if (byte_index > 7) continue;
                    if (row > 31) continue;

                    const offset: u3 = @intCast(@rem(x, 8));
                    if (offset == 0) { // aligned
                        // test and set flag register before XOR
                        const prev = state.display[row][byte_index];
                        state.display[row][byte_index] ^= byte;
                        if (prev & byte > 0) {
                            state.registers[0xF] = 1;
                        }
                    } else { // unaligned
                        if (state.display[row][byte_index] & (byte >> offset) > 0) {
                            state.registers[0xF] = 1;
                        }
                        // set bottom offset bits of first byte XOR with top bits of byte
                        state.display[row][byte_index] ^= byte >> offset;
                        // set top 8 - offset bits of next byte XOR with bottom bits of byte
                        if (byte_index == 7) continue;
                        if (state.display[row][byte_index + 1] & (byte << @intCast(@as(u4, 8) - offset)) > 0) {
                            state.registers[0xF] = 1;
                        }
                        state.display[row][byte_index + 1] ^= byte << @intCast(@as(u4, 8) - offset);
                    }
                }
            },
            .skip_input => {
                if (rl.isKeyDown(keys[state.registers[vx]])) {
                    state.pc += 2;
                }
            },
            .skip_not_input => {
                if (rl.isKeyUp(keys[state.registers[vx]])) {
                    state.pc += 2;
                }
            },
            .input => {
                for (keys, 0..) |key, value| {
                    if (rl.isKeyReleased(key)) {
                        state.registers[vx] = @intCast(value);
                        state.pc += 2;
                    }
                }
                return;
            },
            .read_delay => {
                state.registers[vx] = state.delay;
            },
            .set_delay => {
                state.delay = state.registers[vx];
            },
            .set_sound => {
                state.sound = state.registers[vx];
            },
            .add_i => {
                state.i += state.registers[vx];
            },
            .font => {
                const sprite_num: u16 = state.registers[vx] & 0x0F;
                state.i = 0x0000 + sprite_num * 5;
            },
            .bcd => {
                const hundreds = (state.registers[vx] / 100) % 10;
                const tens = (state.registers[vx] / 10) % 10;
                const ones = (state.registers[vx]) % 10;
                state.ram[state.i] = hundreds;
                state.ram[state.i + 1] = tens;
                state.ram[state.i + 2] = ones;
            },
            .write => {
                for (0..vx + 1) |index| {
                    state.ram[state.i] = state.registers[index];
                    state.i += 1;
                }
            },
            .read => {
                for (0..vx + 1) |index| {
                    state.registers[index] = state.ram[state.i];
                    state.i += 1;
                }
            },
        }

        state.pc += 2;
    }

    fn draw_game(state: State) void {
        for (state.display, 0..) |row, y| {
            for (row, 0..) |byte, x| {
                for (0..8) |index| {
                    const bit_index = 7 - index; // draw most significant bit first
                    const bit = (byte >> @intCast(bit_index)) & 0b1;
                    rl.drawRectangle(
                        @intCast((x * 8 + index) * resolution_multiplier),
                        @intCast(y * resolution_multiplier),
                        resolution_multiplier,
                        resolution_multiplier,
                        if (bit == 0b1) rl.Color.white else rl.Color.black,
                    );
                }
            }
        }
    }

    fn draw_gui(state: State) void {
        _ = rg.guiPanel(rl.Rectangle.init(
            game_width,
            0,
            gui_width,
            game_height,
        ), "instructions");

        const spacing = 2;

        const start_pc = if (state.pc % 2 == 0) @max(state.pc - 20, 0x0200) else @max(state.pc - 20, 0x0201);
        const num_instructions = 20;
        for (0..num_instructions) |i| {
            const pc = start_pc + i * 2;
            const offset: i32 = @intCast(i);
            const instBytes = std.mem.readInt(u16, state.ram[pc..][0..2], .big);
            const instruction = Instruction.decode(instBytes);
            draw_text(
                instruction.op_code_debug_string(@intCast(pc)),
                game_width + 4,
                30 + offset * font_size + offset * spacing,
                if (pc == state.pc) rl.Color.red else null,
            );
        }

        _ = rg.guiPanel(rl.Rectangle.init(
            game_width,
            game_height,
            gui_width,
            gui_height,
        ), "registers");

        for (0..16) |i| {
            const offset: i32 = @intCast(i);
            draw_text(
                rl.textFormat("V%X:\t0x%02X", .{ i, state.registers[i] }),
                game_width + 4,
                game_height + 30 + offset * font_size + offset * spacing,
                null,
            );
        }

        draw_text(
            rl.textFormat("PC:\t0x%04X", .{state.pc}),
            game_width + 4 + gui_width / 3,
            game_height + 30,
            null,
        );
        draw_text(
            rl.textFormat("I:\t\t0x%04X", .{state.i}),
            game_width + 4 + gui_width / 3,
            game_height + 30 + font_size + spacing,
            null,
        );
        draw_text(
            rl.textFormat("DL:\t0x%04X", .{state.delay}),
            game_width + 4 + gui_width / 3,
            game_height + 30 + 3 * (font_size + spacing),
            null,
        );
        draw_text(
            rl.textFormat("SD:\t0x%04X", .{state.sound}),
            game_width + 4 + gui_width / 3,
            game_height + 30 + 4 * (font_size + spacing),
            null,
        );
        draw_text(
            rl.textFormat("ST:\t0x%04X", .{state.sp}),
            game_width + 4 + gui_width / 3,
            game_height + 30 + 6 * (font_size + spacing),
            null,
        );

        for (0..16) |i| {
            const offset: i32 = @intCast(i);
            draw_text(
                rl.textFormat("S%X:\t0x%02X", .{ i, state.stack[i] }),
                game_width + 4 + gui_width / 3 * 2,
                game_height + 30 + offset * font_size + offset * spacing,
                null,
            );
        }

        _ = rg.guiPanel(rl.Rectangle.init(
            0,
            game_height,
            game_width,
            gui_height,
        ), "info");
    }

    fn draw(state: State) void {
        state.draw_game();
        state.draw_gui();
    }

    fn dumpDisplayASCII(state: State) void {
        for (state.display) |row| {
            for (row) |byte| {
                std.debug.print("{b:0>8}", .{byte});
            }
            std.debug.print("\n", .{});
        }
    }
};

fn draw_text(text: [:0]const u8, x: i32, y: i32, color: ?rl.Color) void {
    rl.drawTextEx(
        text_font,
        text,
        rl.Vector2.init(@floatFromInt(x), @floatFromInt(y)),
        font_size,
        0,
        color orelse rl.Color.ray_white,
    );
}

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

pub fn main() !void {
    rl.initWindow(game_width + gui_width, game_height + gui_height, "zip-8");
    defer rl.closeWindow();

    text_font = try rl.loadFontEx("resources/fira_mono.otf", font_size, null);
    rg.guiLoadStyle("resources/style_amber.rgs");

    var state = try State.init();
    // try state.loadRom("roms/1-chip8-logo.ch8");
    // try state.loadRom("roms/2-ibm-logo.ch8");
    // try state.loadRom("roms/3-corax+.ch8");
    // try state.loadRom("roms/4-flags.ch8");
    // try state.loadRom("roms/5-quirks.ch8");
    // try state.loadRom("roms/maze.ch8");
    // try state.loadRom("roms/airplane.ch8");
    // try state.loadRom("roms/rps.ch8");
    try state.loadRom("roms/br8kout.ch8");
    // try state.loadRom("roms/delay_timer_test.ch8");
    // try state.loadRom("roms/draw_offset.ch8");

    var has_drawn = false;
    rl.setTargetFPS(60);
    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();
        rl.clearBackground(rl.Color.black);

        if (state.delay > 0) {
            state.delay -= 1;
        }
        if (state.sound > 0) {
            state.sound -= 1;
        }

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

        for (0..instructions_per_frame) |_| {
            state.step();
        }
        state.draw();
    }
}
