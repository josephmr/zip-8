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

const font = [_]u8{
    0xF0, 0x90, 0x90, 0x90, 0xF0,
    0x20, 0x60, 0x20, 0x20, 0x70,
    0xF0, 0x10, 0xF0, 0x80, 0xF0,
    0xF0, 0x10, 0xF0, 0x10, 0xF0,
    0x90, 0x90, 0xF0, 0x10, 0x10,
    0xF0, 0xB0, 0xF0, 0x10, 0xF0,
    0xF0, 0x80, 0xF0, 0x90, 0xF0,
    0xF0, 0x10, 0x20, 0x40, 0x40,
    0xF0, 0x90, 0xF0, 0x90, 0xF0,
    0xF0, 0x90, 0xF0, 0x10, 0xF0,
    0xF0, 0x90, 0xF0, 0x90, 0x90,
    0xE0, 0x90, 0xE0, 0x90, 0xE0,
    0xF0, 0x80, 0x80, 0x80, 0xF0,
    0xE0, 0x90, 0x90, 0x90, 0xE0,
    0xF0, 0x80, 0xF0, 0x80, 0xF0,
    0xF0, 0x80, 0xF0, 0x80, 0x80,
};

const Instruction = enum(u32) {
    // Each instruction is a combination of a bitmask and an opcode.
    // This is used as CHIP-8 does not have a regularly sized opcode.
    clear = 0xFFFF_00E0,
    load_byte = 0xF000_6000,
    load_addr = 0xF000_A000,
    add_byte = 0xF000_7000,
    draw = 0xF000_D000,
    jump = 0xF000_1000,

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
        unreachable;
    }
};

const State = struct {
    ram: [4096]u8,

    // registers
    registers: [16]u8,
    sound: u8,
    delay: u8,
    i: u16,

    // program counter
    pc: u16,

    // stack pointer
    sp: u16,
    stack: [16]u16,

    // display buffer
    display: [32][8]u8,

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
    };

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

        switch (instruction) {
            .clear => {
                std.log.debug("\tclear", .{});
            },
            .load_byte => {
                const vx = state.ram[state.pc] & 0x0F;
                std.debug.assert(vx >= 0 and vx <= 0xF);

                const byte = state.ram[state.pc + 1];

                state.registers[vx] = byte;
                std.log.debug("\tload byte -- V[0x{X}] = 0x{X:0>2}", .{ vx, byte });
            },
            .load_addr => {
                const addr = 0x0FFF & instBytes;
                state.i = addr;
                std.log.debug("\tload addr -- I = 0x{X:0>4}", .{addr});
            },
            .add_byte => {
                const vx = state.ram[state.pc] & 0x0F;
                std.debug.assert(vx >= 0 and vx <= 0xF);

                const byte = state.ram[state.pc + 1];

                state.registers[vx] += byte;
                std.log.debug("\tadd byte -- V[0x{X}] = 0x{X:0>2}", .{ vx, byte });
            },
            .jump => {
                const addr = 0x0FFF & instBytes;
                std.debug.assert(addr % 2 == 0);
                state.pc = addr;
                std.log.debug("\tjump -- PC = 0x{X:0>4}", .{addr});
                return;
            },
            .draw => {
                const vx = state.ram[state.pc] & 0x0F;
                std.debug.assert(vx >= 0 and vx <= 0xF);
                const vy = (state.ram[state.pc + 1] & 0xF0) >> 4;
                std.debug.assert(vy >= 0 and vx <= 0xF);
                const x = state.registers[vx];
                std.debug.assert(x >= 0 and x <= 64);
                const y = state.registers[vy];
                std.debug.assert(y >= 0 and y <= 32);
                const sprite_len = state.ram[state.pc + 1] & 0x0F;
                const sprite = state.ram[state.i..][0..sprite_len];
                std.log.debug("\tdraw {any} at {},{} from registers 0x{X} 0x{X}", .{ sprite, x, y, vx, vy });
                for (sprite, y..) |byte, row| {
                    state.display[row][x / 8] ^= byte;
                    if (state.display[row][x / 8] & byte >= 0) {
                        state.registers[0xF] = 1;
                    }
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
        std.Thread.sleep(std.time.ns_per_s / 2);
    }
}

pub fn main() !void {
    const screenWidth = 64 * resolution_multiplier;
    const screenHeight = 32 * resolution_multiplier;

    rl.initWindow(screenWidth, screenHeight, "zip-8");
    defer rl.closeWindow();

    var state = State.empty;
    try state.loadRom("roms/1-chip8-logo.ch8");

    hexDump(state.ram[0x200..0x310]);

    const thread = try std.Thread.spawn(.{}, process, .{&state});
    std.Thread.detach(thread);

    while (!rl.windowShouldClose()) {
        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(rl.Color.black);
        state.draw();
    }
}
