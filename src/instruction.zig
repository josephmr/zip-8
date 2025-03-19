const std = @import("std");
const rl = @import("raylib");

pub const OpCode = enum {
    clear,
    ret,
    jump,
    call,
    skip_eq_xnn,
    skip_neq_xnn,
    skip_eq_xy,
    load_byte,
    add_byte,
    set_xy,
    set_or,
    set_and,
    set_xor,
    add_xy,
    sub_xy,
    set_sh_right,
    sub_yx,
    set_sh_left,
    skip_neq_xy,
    load_addr,
    jump_v0,
    rand,
    draw,
    skip_input,
    skip_not_input,
    read_delay,
    input,
    set_delay,
    set_sound,
    add_i,
    font,
    bcd,
    write,
    read,
};

const Op = packed struct(u16) {
    group: u4,
    _: u8,
    op: u4,
};

const XYN = packed struct(u16) {
    n: u4,
    y: u4,
    x: u4,
    _: u4,
};

const NNN = packed struct(u16) {
    nnn: u12,
    _: u4,
};

const XNN = packed struct(u16) {
    nn: u8,
    x: u4,
    _: u4,
};

pub const Instruction = packed union {
    op: Op,
    xyn: XYN,
    nnn: NNN,
    xnn: XNN,

    pub fn decode(value: u16) Instruction {
        return @bitCast(value);
    }

    pub fn op_code(self: Instruction) ?OpCode {
        return switch (self.op.op) {
            0x0 => switch (self.nnn.nnn) {
                0x0E0 => .clear,
                0x0EE => .ret,
                else => null,
            },
            0x1 => .jump,
            0x2 => .call,
            0x3 => .skip_eq_xnn,
            0x4 => .skip_neq_xnn,
            0x5 => .skip_eq_xy,
            0x6 => .load_byte,
            0x7 => .add_byte,
            0x8 => switch (self.op.group) {
                0x0 => .set_xy,
                0x1 => .set_or,
                0x2 => .set_and,
                0x3 => .set_xor,
                0x4 => .add_xy,
                0x5 => .sub_xy,
                0x6 => .set_sh_right,
                0x7 => .sub_yx,
                0xE => .set_sh_left,
                else => null,
            },
            0x9 => .skip_neq_xy,
            0xA => .load_addr,
            0xB => .jump_v0,
            0xC => .rand,
            0xD => .draw,
            0xE => switch (self.xnn.nn) {
                0x9E => .skip_input,
                0xA1 => .skip_not_input,
                else => null,
            },
            0xF => switch (self.xnn.nn) {
                0x07 => .read_delay,
                0x0A => .input,
                0x15 => .set_delay,
                0x18 => .set_sound,
                0x1E => .add_i,
                0x29 => .font,
                0x33 => .bcd,
                0x55 => .write,
                0x65 => .read,
                else => null,
            },
        };
    }

    // TODO: remove dependency on raylib and pc, just stringify op_code and args
    pub fn op_code_debug_string(self: Instruction, pc: u16) [:0]const u8 {
        const op = self.op_code();

        if (op == null) {
            return rl.textFormat("%04X -- ---", .{pc});
        }

        const nnn: u16 = self.nnn.nnn;
        const vx: u16 = self.xyn.x;
        const vy: u16 = self.xyn.y;
        const nn: u8 = self.xnn.nn;
        const n: u8 = self.xyn.n;

        switch (op.?) {
            .clear => {
                return rl.textFormat("%04X -- CLR", .{pc});
            },
            .ret => {
                return rl.textFormat("%04X -- RET", .{pc});
            },
            .jump => {
                return rl.textFormat("%04X -- JMP\t#%03X", .{ pc, nnn });
            },
            .call => {
                return rl.textFormat("%04X -- CALL\t#%04X", .{ pc, nnn });
            },
            .skip_eq_xnn => {
                return rl.textFormat("%04X -- SKEQ\tV%X, #%02X", .{ pc, vx, nn });
            },
            .skip_neq_xnn => {
                return rl.textFormat("%04X -- SKNEQ\tV%X, #%02X", .{ pc, vx, nn });
            },
            .skip_eq_xy => {
                return rl.textFormat("%04X -- SKEQ\tV%X, V%X", .{ pc, vx, vy });
            },
            .load_byte => {
                return rl.textFormat("%04X -- LD\tV%X, #%02X", .{ pc, vx, nn });
            },
            .add_byte => {
                return rl.textFormat("%04X -- ADD\tV%X, #%02X", .{ pc, vx, nn });
            },
            .set_xy => {
                return rl.textFormat("%04X -- LD\tV%X, V%X", .{ pc, vx, vy });
            },
            .set_or => {
                return rl.textFormat("%04X -- OR\tV%X, V%X", .{ pc, vx, vy });
            },
            .set_and => {
                return rl.textFormat("%04X -- AND\tV%X, V%X", .{ pc, vx, vy });
            },
            .set_xor => {
                return rl.textFormat("%04X -- XOR\tV%X, V%X", .{ pc, vx, vy });
            },
            .add_xy => {
                return rl.textFormat("%04X -- ADD\tV%X, V%X", .{ pc, vx, vy });
            },
            .sub_xy => {
                return rl.textFormat("%04X -- SUB\tV%X, V%X", .{ pc, vx, vy });
            },
            .set_sh_right => {
                return rl.textFormat("%04X -- SHR\tV%X, V%X", .{ pc, vx, vy });
            },
            .sub_yx => {
                return rl.textFormat("%04X -- SUB\tV%X, V%X", .{ pc, vx, vy });
            },
            .set_sh_left => {
                return rl.textFormat("%04X -- SHL\tV%X, V%X", .{ pc, vx, vy });
            },
            .skip_neq_xy => {
                return rl.textFormat("%04X -- SKNEQ\tV%X, V%X", .{ pc, vx, vy });
            },
            .load_addr => {
                return rl.textFormat("%04X -- LD\tI, #%04X", .{ pc, nnn });
            },
            .jump_v0 => {
                return rl.textFormat("%04X -- JMP\tV0 + #%04X", .{ pc, nnn });
            },
            .rand => {
                return rl.textFormat("%04X -- RAND\tV%X, #%02X", .{ pc, vx, nn });
            },
            .draw => {
                return rl.textFormat("%04X -- DRAW\tV%X, V%X, #%02X", .{ pc, vx, vy, n });
            },
            .skip_input => {
                return rl.textFormat("%04X -- SKINP\tV%X", .{
                    pc,
                    vx,
                });
            },
            .skip_not_input => {
                return rl.textFormat("%04X -- SKNINP\tV%X", .{
                    pc,
                    vx,
                });
            },
            .read_delay => {
                return rl.textFormat("%04X -- RDL\tV%X", .{
                    pc,
                    vx,
                });
            },
            .input => {
                return rl.textFormat("%04X -- INP\tV%X", .{
                    pc,
                    vx,
                });
            },
            .set_delay => {
                return rl.textFormat("%04X -- SDEL\tV%X", .{
                    pc,
                    vx,
                });
            },
            .set_sound => {
                return rl.textFormat("%04X -- SSD\tV%X", .{
                    pc,
                    vx,
                });
            },
            .add_i => {
                return rl.textFormat("%04X -- ADD\tI, V%X", .{
                    pc,
                    vx,
                });
            },
            .font => {
                return rl.textFormat("%04X -- FNT\tI, V%X", .{
                    pc,
                    vx,
                });
            },
            .bcd => {
                return rl.textFormat("%04X -- BCD\tI, V%X", .{
                    pc,
                    vx,
                });
            },
            .write => {
                return rl.textFormat("%04X -- LD\tI, V0 -> V%X", .{
                    pc,
                    vx,
                });
            },
            .read => {
                return rl.textFormat("%04X -- FILL\tV0 -> V%X, I", .{
                    pc,
                    vx,
                });
            },
        }
    }
};

test "instruction accessors" {
    const op: Instruction = Instruction.decode(@as(u16, 0xABCD));

    try std.testing.expectEqual(op.op.op, 0xA);
    try std.testing.expectEqual(op.op.group, 0xD);

    try std.testing.expectEqual(op.xyn.x, 0xB);
    try std.testing.expectEqual(op.xyn.y, 0xC);
    try std.testing.expectEqual(op.xyn.n, 0xD);

    try std.testing.expectEqual(op.nnn.nnn, 0xBCD);

    try std.testing.expectEqual(op.xnn.x, 0xB);
    try std.testing.expectEqual(op.xnn.nn, 0xCD);
}
