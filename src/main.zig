const std = @import("std");
const rl = @import("raylib");

const resolution_multiplier = 8;

test "instruction is" {
    try std.testing.expect(Instruction.clear.is(0x00E0));
    try std.testing.expect(!Instruction.clear.is(0x10E0));

    try std.testing.expect(Instruction.load_byte.is(0x6000));
    try std.testing.expect(Instruction.load_byte.is(0x6FFF));
    try std.testing.expect(Instruction.load_byte.is(0x6888));
    try std.testing.expect(!Instruction.load_byte.is(0x7000));
    try std.testing.expect(!Instruction.load_byte.is(0xF000));
    try std.testing.expect(!Instruction.load_byte.is(0x0000));
    try std.testing.expect(!Instruction.load_byte.is(0x00E0));
}

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
    0xF0, 0xB0, 0xF0, 0x10, 0xF0, // 5
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

const Instruction = enum(u32) {
    // Each instruction is a combination of a bitmask and an opcode.
    // This is used as CHIP-8 does not have a regularly sized opcode.
    clear = 0xFFFF_00E0,
    ret = 0xFFFF_00EE,
    jump = 0xF000_1000,
    call = 0xF000_2000,
    skip_eq_xnn = 0xF000_3000,
    skip_neq_xnn = 0xF000_4000,
    skip_eq_xy = 0xF00F_5000,
    load_byte = 0xF000_6000,
    add_byte = 0xF000_7000,
    set_xy = 0xF00F_8000,
    set_or = 0xF00F_8001,
    set_and = 0xF00F_8002,
    set_xor = 0xF00F_8003,
    add_xy = 0xF00F_8004,
    sub_xy = 0xF00F_8005,
    set_sh_right = 0xF00F_8006,
    sub_yx = 0xF00F_8007,
    set_sh_left = 0xF00F_800E,
    skip_neq_xy = 0xF00F_9000,
    load_addr = 0xF000_A000,
    jump_v0 = 0xF000_B000,
    rand = 0xF000_C000,
    draw = 0xF000_D000,
    skip_input = 0xF0FF_E09E,
    skip_not_input = 0xF0FF_E0A1,
    input = 0xF0FF_F00A,
    read_delay = 0xF0FF_F007,
    set_delay = 0xF0FF_F015,
    set_sound = 0xF0FF_F018,
    add_i = 0xF0FF_F01E,
    font = 0xF0FF_F029,
    bcd = 0xF0FF_F033,
    write = 0xF0FF_F055,
    read = 0xF0FF_F065,

    fn is(inst: Instruction, val: u16) bool {
        const int = @intFromEnum(inst);
        const mask = int >> 16;
        const opcode = 0x0000FFFF & int;
        return val & mask == opcode;
    }

    fn from(value: u16) Instruction {
        inline for (std.meta.fields(Instruction)) |inst| {
            const en: Instruction = @enumFromInt(inst.value);
            if (en.is(value)) return en;
        }
        std.log.debug("Failed to find matching instruction: 0x{X:0>4}", .{value});
        unreachable;
    }
};

const State = struct {
    ram: [4096]u8,

    // registers
    registers: [16]u8,
    i: u16,

    // special sound/delay registers and their mutexes
    sound: u8,
    sound_mutex: std.Thread.Mutex,
    sound_condition: std.Thread.Condition,
    delay: u8,
    delay_mutex: std.Thread.Mutex,
    delay_condition: std.Thread.Condition,

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
        .sound_mutex = std.Thread.Mutex{},
        .sound_condition = std.Thread.Condition{},
        .delay = 0,
        .delay_mutex = std.Thread.Mutex{},
        .delay_condition = std.Thread.Condition{},
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
        const instBytes = state.pcValue();
        const instruction = Instruction.from(instBytes);
        std.log.debug("instruction 0x{X:0>4}: 0x{X:0>2}{X:0>2} -- {}", .{ state.pc, state.ram[state.pc], state.ram[state.pc + 1], instruction });

        const vx = state.ram[state.pc] & 0x0F;
        const vy = (state.ram[state.pc + 1] & 0xF0) >> 4;

        switch (instruction) {
            .clear => {
                std.log.debug("\tclear", .{});
                for (0..state.display.len) |row| {
                    for (0..8) |col| {
                        state.display[row][col] = 0x00;
                    }
                }
            },
            .ret => {
                std.debug.assert(state.sp >= 0);
                const addr = state.stack[state.sp - 1];
                state.sp -= 1;
                state.pc = addr;
                std.log.debug("\tret -- PC = 0x{X:0>4}", .{addr});
            },
            .jump => {
                const addr = 0x0FFF & instBytes;
                state.pc = addr;
                std.log.debug("\tjump -- PC = 0x{X:0>4}", .{addr});
                return;
            },
            .call => {
                const addr = state.pcValue() & 0x0FFF;
                state.stack[state.sp] = state.pc;
                state.sp += 1;
                state.pc = addr;
                std.log.debug("\tcall -- PC = 0x{X:0>4}", .{addr});
                return;
            },
            .skip_eq_xnn => {
                const nn = state.ram[state.pc + 1];
                if (state.registers[vx] == nn) {
                    state.pc += 2;
                }
                std.log.debug("\tskip_eq_xnn -- {}", .{state.registers[vx] == nn});
            },
            .skip_neq_xnn => {
                const nn = state.ram[state.pc + 1];
                if (state.registers[vx] != nn) {
                    state.pc += 2;
                }
                std.log.debug("\tskip_neq_xnn -- {}", .{state.registers[vx] != nn});
            },
            .skip_eq_xy => {
                if (state.registers[vx] == state.registers[vy]) {
                    state.pc += 2;
                }
                std.log.debug("\tskip_eq_xy -- {}", .{state.registers[vx] == state.registers[vy]});
            },
            .load_byte => {
                std.debug.assert(vx >= 0 and vx <= 0xF);

                const byte = state.ram[state.pc + 1];

                state.registers[vx] = byte;
                std.log.debug("\tload byte -- V[0x{X}] = 0x{X:0>2}", .{ vx, byte });
            },
            .add_byte => {
                std.debug.assert(vx >= 0 and vx <= 0xF);

                const byte = state.ram[state.pc + 1];

                state.registers[vx] +%= byte;
                std.log.debug("\tadd byte -- V[0x{X}] = 0x{X:0>2}", .{ vx, byte });
            },
            .set_xy => {
                state.registers[vx] = state.registers[vy];
                std.log.debug("\tset V[0x{X}] = V[0x{X}] ({X:0>2}", .{ vx, vy, state.registers[vy] });
            },
            .set_or => {
                state.registers[vx] |= state.registers[vy];
                std.log.debug("\tset V[0x{X}] |= V[0x{X}]", .{ vx, vy });
            },
            .set_and => {
                state.registers[vx] &= state.registers[vy];
                std.log.debug("\tset V[0x{X}] &= V[0x{X}]", .{ vx, vy });
            },
            .set_xor => {
                state.registers[vx] ^= state.registers[vy];
                std.log.debug("\tset V[0x{X}] ^= V[0x{X}]", .{ vx, vy });
            },
            .add_xy => {
                const result = @addWithOverflow(state.registers[vx], state.registers[vy]);
                state.registers[vx] = result[0];
                state.registers[0xF] = result[1];
                std.log.debug("\tadd_xy V[0x{X}] += V[0x{X}]", .{ vx, vy });
            },
            .sub_xy => {
                const result = @subWithOverflow(state.registers[vx], state.registers[vy]);
                state.registers[vx] = result[0];
                state.registers[0xF] = if (result[1] == 0b1) 0b0 else 0b1;
                std.log.debug("\tsub_xy V[0x{X}] -= V[0x{X}]", .{ vx, vy });
            },
            .set_sh_right => {
                state.registers[vx] = state.registers[vy];
                const flag = state.registers[vx] & 0x01;
                state.registers[vx] = state.registers[vy] >> 1;
                state.registers[0xF] = flag;
                std.log.debug("\tshift_right_1 V[0x{X}] = V[0x{X}] >> 1", .{ vx, vy });
            },
            .sub_yx => {
                const result = @subWithOverflow(state.registers[vy], state.registers[vx]);
                state.registers[vx] = result[0];
                state.registers[0xF] = if (result[1] == 0b1) 0b0 else 0b1;
                std.log.debug("\tsub_yx V[0x{X}] = V[0x{X}] - V[0x{X}]", .{ vx, vy, vx });
            },
            .set_sh_left => {
                const result = @shlWithOverflow(state.registers[vy], 1);
                state.registers[vx] = result[0];
                state.registers[0xF] = result[1];
                std.log.debug("\tshift_left_1 V[0x{X}] = V[0x{X}] << 1", .{ vx, vy });
            },
            .skip_neq_xy => {
                if (state.registers[vx] != state.registers[vy]) {
                    state.pc += 2;
                }
                std.log.debug("\tskip_neq_xy -- {}", .{state.registers[vx] != state.registers[vy]});
            },
            .load_addr => {
                const addr = 0x0FFF & instBytes;
                state.i = addr;
                std.log.debug("\tload addr -- I = 0x{X:0>4}", .{addr});
            },
            .jump_v0 => {
                const addr = 0x0FFF & instBytes;
                state.pc = state.registers[0x0] + addr;
                return;
            },
            .rand => {
                const kk = state.ram[state.pc + 1];
                state.registers[vx] = @as(u8, @truncate(state.rng.next())) & kk;
            },
            .draw => {
                const x = state.registers[vx] % 64;
                const y = state.registers[vy] % 32;

                const sprite_len = state.ram[state.pc + 1] & 0x0F;
                const sprite = state.ram[state.i..][0..sprite_len];
                std.log.debug("\tdraw {any} at {},{} from registers 0x{X} 0x{X}", .{ sprite, x, y, vx, vy });

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
                        if (state.display[row][byte_index] & ~byte >= 0) {
                            state.registers[0xF] = 1;
                        }
                        state.display[row][byte_index] ^= byte;
                    } else { // unaligned
                        if (state.display[row][byte_index] & ~byte >> offset >= 0) {
                            state.registers[0xF] = 1;
                        }
                        // set bottom offset bits of first byte XOR with top bits of byte
                        state.display[row][byte_index] ^= byte >> offset;
                        // set top 8 - offset bits of next byte XOR with bottom bits of byte
                        if (byte_index == 7) continue;
                        if (state.display[row][byte_index + 1] & ~byte >> offset >= 0) {
                            state.registers[0xF] = 1;
                        }
                        state.display[row][byte_index + 1] ^= byte << @intCast(@as(u4, 8) - offset);
                    }
                }
            },
            .skip_input => {
                std.log.debug("\tchecking for IS key pressed: {} {}", .{ state.registers[vx], keys[state.registers[vx]] });
                if (rl.isKeyDown(keys[state.registers[vx]])) {
                    std.log.debug("\t\tskipping instruction!", .{});
                    state.pc += 2;
                }
            },
            .skip_not_input => {
                std.log.debug("\tchecking for NOT key pressed: {} {}", .{ state.registers[vx], keys[state.registers[vx]] });
                if (rl.isKeyUp(keys[state.registers[vx]])) {
                    std.log.debug("\t\tskipping instruction!", .{});
                    state.pc += 2;
                }
            },
            .input => {
                inf: while (true) {
                    for (keys, 0..) |key, value| {
                        if (rl.isKeyReleased(key)) {
                            state.registers[vx] = @intCast(value);
                            break :inf;
                        }
                    }
                }
            },
            .read_delay => {
                state.registers[vx] = state.delay;
            },
            .set_delay => {
                state.delay_mutex.lock();
                defer state.delay_mutex.unlock();
                state.delay = state.registers[vx];
                state.delay_condition.signal();
            },
            .set_sound => {
                state.sound = state.registers[vx];
            },
            .add_i => {
                state.i += state.registers[vx];
                std.log.debug("\tadd_i I += V[0x{X}]", .{vx});
            },
            .font => {
                const sprite_num: u16 = state.registers[vx] & 0x0F;
                state.i = 0x0000 + sprite_num * 5;
                std.log.debug("\tfont I = 0x{X:0>4} -- sprite {}", .{ state.i, sprite_num });
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

    fn draw(state: State) void {
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

    fn dumpDisplayASCII(state: State) void {
        for (state.display) |row| {
            for (row) |byte| {
                std.debug.print("{b:0>8}", .{byte});
            }
            std.debug.print("\n", .{});
        }
    }
};

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

fn process(state: *State) void {
    while (true) {
        state.step();
        std.Thread.sleep(std.time.ns_per_s / 500);
    }
}

fn sound_timer(state: *State) void {
    // TODO: implement sound timer like delay -- maybe comptime?
    _ = state;
}

fn delay_timer(state: *State) void {
    const timer_wait_ns = std.time.ns_per_s / 60;

    while (true) {
        state.delay_mutex.lock();
        while (state.delay == 0) {
            state.delay_condition.wait(&state.delay_mutex);
        }
        state.delay_mutex.unlock();
        var wait_time: i128 = timer_wait_ns;
        var now: i128 = undefined;
        while (state.delay > 0) {
            std.Thread.sleep(@intCast(wait_time));
            now = std.time.nanoTimestamp();
            state.delay_mutex.lock();
            if (state.delay > 0) {
                state.delay -= 1;
            }
            state.delay_mutex.unlock();

            const time_elapsed = std.time.nanoTimestamp() - now;
            if (time_elapsed < timer_wait_ns) {
                wait_time = timer_wait_ns - time_elapsed;
            }
        }
    }
}

pub fn main() !void {
    const screenWidth = 64 * resolution_multiplier;
    const screenHeight = 32 * resolution_multiplier;

    rl.initWindow(screenWidth, screenHeight, "zip-8");
    defer rl.closeWindow();

    var state = try State.init();
    // try state.loadRom("roms/1-chip8-logo.ch8");
    // try state.loadRom("roms/2-ibm-logo.ch8");
    // try state.loadRom("roms/3-corax+.ch8");
    // try state.loadRom("roms/4-flags.ch8");
    // try state.loadRom("roms/5-quirks.ch8");
    // try state.loadRom("roms/maze.ch8");
    try state.loadRom("roms/airplane.ch8"); // not working
    // try state.loadRom("roms/rps.ch8");
    // try state.loadRom("roms/br8kout.ch8"); // not working
    // try state.loadRom("roms/delay_timer_test.ch8");

    const thread = try std.Thread.spawn(.{}, process, .{&state});
    std.Thread.detach(thread);

    const sound_timer_thread = try std.Thread.spawn(.{}, sound_timer, .{&state});
    std.Thread.detach(sound_timer_thread);

    const delay_timer_thread = try std.Thread.spawn(.{}, delay_timer, .{&state});
    std.Thread.detach(delay_timer_thread);

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(rl.Color.black);
        state.draw();
    }
}
