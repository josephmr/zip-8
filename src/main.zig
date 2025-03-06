const std = @import("std");

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

const Instruction = enum(u32) {
    // Each instruction is a combination of a bitmask and an opcode.
    // This is used as CHIP-8 does not have a regularly sized opcode.
    clear = 0xFFFF_00E0,
    load_byte = 0xF000_6000,
    load_addr = 0xF000_A000,
    draw = 0xF000_D000,
    jump = 0xF000_1000,

    fn is(inst: Instruction, val: u16) bool {
        const int = @intFromEnum(inst);
        const mask = int >> 16;
        const opcode = 0x0000FFFF & int;
        return val & mask == opcode;
    }
};

// TODO zig 0.14 update to .empty pattern
const State = struct {
    ram: [4096]u8 = [_]u8{0} ** 4096,

    // registers
    registers: [16]u8 = [_]u8{0} ** 16,
    sound: u8 = 0,
    delay: u8 = 0,
    i: u16 = 0,

    // program counter
    pc: u16 = 0x200,

    // stack pointer
    sp: u16 = 0,
    stack: [16]u16 = [_]u16{0} ** 16,

    fn instruction(state: State) u16 {
        return std.mem.readInt(u16, state.ram[state.pc..][0..2], .big);
    }

    fn step(state: *State) void {
        const inst = state.instruction();
        std.log.debug("instruction 0x{X:0>4}: 0x{X:0>4}", .{ state.pc, inst });

        if (Instruction.clear.is(inst)) state.instClear();
        if (Instruction.load_byte.is(inst)) state.instLoadByte();
        if (Instruction.load_addr.is(inst)) state.instLoadAddr();
        if (Instruction.jump.is(inst)) {
            state.instJump();
            return;
        }
        if (Instruction.draw.is(inst)) state.instDraw();

        state.pc += 2;
    }

    fn instClear(_: State) void {
        std.log.debug("\tclear", .{});
    }

    fn instLoadByte(state: *State) void {
        const vx = state.ram[state.pc] & 0x0F;
        const byte = state.ram[state.pc + 1];
        state.registers[vx] = byte;
        std.log.debug("\tload byte -- V[0x{X}] = 0x{X:0>2}", .{ vx, byte });
    }

    fn instLoadAddr(state: *State) void {
        const inst = state.instruction();
        const addr = 0x0FFF & inst;
        state.i = addr;
        std.log.debug("\tload addr -- I = 0x{X:0>4}", .{addr});
    }

    fn instJump(state: *State) void {
        const inst = state.instruction();
        const addr = 0x0FFF & inst;
        state.pc = addr;
        std.log.debug("\tjump -- PC = 0x{X:0>4}", .{addr});
        std.debug.assert(addr % 2 == 0);
    }

    fn instDraw(_: State) void {
        std.log.debug("\tdraw", .{});
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

pub fn main() !void {
    var state = State{};
    const read = try std.fs.cwd().readFile("roms/1-chip8-logo.ch8", state.ram[0x200..]);
    std.log.debug("Read {} bytes", .{read.len});

    hexDump(state.ram[0x200..0x310]);

    while (true) {
        state.step();
    }
}
