# amaranth hdl

import shutil
import subprocess
from pathlib import Path

from amaranth import *
from amaranth.back import verilog
from amaranth.hdl import IOBufferInstance
from amaranth.lib import wiring
from amaranth.lib.wiring import Elaboratable, In, Out, Signal, Signature
from amaranth.sim import Simulator


# Create a new module
class BitGridCell(wiring.Component):
    in_up: In(1)
    in_down: In(1)
    in_left: In(1)
    in_right: In(1)

    out_up: Out(1)
    out_down: Out(1)
    out_left: Out(1)
    out_right: Out(1)

    run_rst: In(1)
    run_enable: In(1)

    program_up: In(16)
    program_down: In(16)
    program_left: In(16)
    program_right: In(16)

    program_rst: In(1)
    program_enable: In(1)

    def elaborate(self, platform):
        m = Module()

        out_reg = Signal(4, init=0)
        in_concat = Signal(4)

        f_up = Signal(1)
        f_down = Signal(1)
        f_left = Signal(1)
        f_right = Signal(1)

        program_up_reg = Signal(16, init=0)
        program_down_reg = Signal(16, init=0)
        program_left_reg = Signal(16, init=0)
        program_right_reg = Signal(16, init=0)

        with m.If(self.program_rst):
            m.d.sync += program_up_reg.eq(0)
            m.d.sync += program_down_reg.eq(0)
            m.d.sync += program_left_reg.eq(0)
            m.d.sync += program_right_reg.eq(0)
        with m.Elif(self.program_enable):
            m.d.sync += program_up_reg.eq(self.program_up)
            m.d.sync += program_down_reg.eq(self.program_down)
            m.d.sync += program_left_reg.eq(self.program_left)
            m.d.sync += program_right_reg.eq(self.program_right)

        m.d.comb += in_concat.eq(
            Cat(self.in_up, self.in_down, self.in_left, self.in_right)
        )

        m.d.comb += f_up.eq(program_up_reg.bit_select(in_concat, 1))
        m.d.comb += f_down.eq(program_down_reg.bit_select(in_concat, 1))
        m.d.comb += f_left.eq(program_left_reg.bit_select(in_concat, 1))
        m.d.comb += f_right.eq(program_right_reg.bit_select(in_concat, 1))

        with m.If(self.run_rst):
            m.d.sync += out_reg.eq(0)
        with m.Elif(self.run_enable):
            m.d.sync += out_reg.eq(Cat(f_up, f_down, f_left, f_right))

        m.d.comb += self.out_up.eq(out_reg[0])
        m.d.comb += self.out_down.eq(out_reg[1])
        m.d.comb += self.out_left.eq(out_reg[2])
        m.d.comb += self.out_right.eq(out_reg[3])

        return m


class ShiftRegisterParallelOut(wiring.Component):
    def __init__(self, width):
        self.width = width
        super().__init__(
            {
                "rst": Out(1),
                "enable": In(1),
                "data_in": In(1),
                "data_out": Out(1),
                "data_parallel": Out(width),
            }
        )

    def elaborate(self, platform):
        m = Module()

        self.data = Signal(self.width, init=0)

        with m.If(self.rst):
            m.d.sync += self.data.eq(0)
        with m.Elif(self.enable):
            m.d.sync += self.data.eq(Cat(self.data[1:], self.data_in))

        m.d.comb += self.data_out.eq(self.data[0])
        m.d.comb += self.data_parallel.eq(self.data)

        return m


class ShiftRegisterParallelIn(wiring.Component):
    def __init__(self, width):
        self.width = width
        super().__init__(
            {
                "rst": In(1),
                "enable": In(1),
                "data_in": In(1),
                "data_out": Out(1),
                "data_parallel": In(width),
                "data_copy": In(1),
            }
        )

    def elaborate(self, platform):
        m = Module()

        self.data = Signal(self.width, init=0)

        with m.If(self.rst):
            m.d.sync += self.data.eq(0)
        with m.Elif(self.data_copy):
            m.d.sync += self.data.eq(self.data_parallel)
        with m.Elif(self.enable):
            m.d.sync += self.data.eq(Cat(self.data[1:], self.data_in))

        m.d.comb += self.data_out.eq(self.data[0])

        return m


class BitGridArray(wiring.Component):
    global_program_rst: In(1)
    global_program_enable: In(1)

    global_run_rst: In(1)
    global_run_enable: In(1)

    global_program_mem_rst: In(1)
    global_program_mem_enable: In(1)
    global_program_mem_in: In(1)

    global_state_mem_copy: In(1)
    global_state_mem_rst: In(1)
    global_state_mem_enable: In(1)
    global_state_mem_out: Out(1)

    def __init__(self, row, col):
        super().__init__()

        # using a row col corrdinate system
        self.row = row
        self.col = col

        self.cells = {}
        for i in range(row):
            for j in range(col):
                self.cells[(i, j)] = BitGridCell()

        self.program_shift_registers = {}
        for i in range(row):
            for j in range(col):
                self.program_shift_registers[(i, j)] = ShiftRegisterParallelOut(16 * 4)

        self.state_shift_registers = {}
        for i in range(row):
            for j in range(col):
                self.state_shift_registers[(i, j)] = ShiftRegisterParallelIn(4)

    def elaborate(self, platform):
        m = Module()

        for i in range(self.row):
            for j in range(self.col):
                m.submodules[f"cell_{i}_{j}"] = self.cells[(i, j)]
                m.submodules[f"prog_{i}_{j}"] = self.program_shift_registers[(i, j)]
                m.submodules[f"state_{i}_{j}"] = self.state_shift_registers[(i, j)]

        for i in range(self.row):
            for j in range(self.col):
                up_idx: int | None = None
                down_idx: int | None = None
                left_idx: int | None = None
                right_idx: int | None = None

                up_idx = i - 1 if i - 1 >= 0 else None
                down_idx = i + 1 if i + 1 < self.row else None
                left_idx = j - 1 if j - 1 >= 0 else None
                right_idx = j + 1 if j + 1 < self.col else None

                if up_idx is not None:
                    m.d.comb += self.cells[(i, j)].in_up.eq(
                        self.cells[(up_idx, j)].out_down
                    )
                if down_idx is not None:
                    m.d.comb += self.cells[(i, j)].in_down.eq(
                        self.cells[(down_idx, j)].out_up
                    )
                if left_idx is not None:
                    m.d.comb += self.cells[(i, j)].in_left.eq(
                        self.cells[(i, left_idx)].out_right
                    )
                if right_idx is not None:
                    m.d.comb += self.cells[(i, j)].in_right.eq(
                        self.cells[(i, right_idx)].out_left
                    )

        for i in range(self.row):
            for j in range(self.col):
                m.d.comb += self.cells[(i, j)].run_enable.eq(self.global_run_enable)
                m.d.comb += self.cells[(i, j)].run_rst.eq(self.global_run_rst)

                m.d.comb += self.cells[(i, j)].program_rst.eq(self.global_program_rst)
                m.d.comb += self.cells[(i, j)].program_enable.eq(
                    self.global_program_enable
                )

        for i in range(self.row):
            for j in range(self.col):
                m.d.comb += self.cells[(i, j)].program_up.eq(
                    self.program_shift_registers[(i, j)].data_parallel[0:16]
                )
                m.d.comb += self.cells[(i, j)].program_down.eq(
                    self.program_shift_registers[(i, j)].data_parallel[16:32]
                )
                m.d.comb += self.cells[(i, j)].program_left.eq(
                    self.program_shift_registers[(i, j)].data_parallel[32:48]
                )
                m.d.comb += self.cells[(i, j)].program_right.eq(
                    self.program_shift_registers[(i, j)].data_parallel[48:64]
                )

                m.d.comb += self.program_shift_registers[(i, j)].rst.eq(
                    self.global_program_mem_rst
                )
                m.d.comb += self.program_shift_registers[(i, j)].enable.eq(
                    self.global_program_mem_enable
                )

        for i in range(self.row):
            for j in range(self.col):
                m.d.comb += self.state_shift_registers[(i, j)].data_parallel.eq(
                    Cat(
                        self.cells[(i, j)].out_up,
                        self.cells[(i, j)].out_down,
                        self.cells[(i, j)].out_left,
                        self.cells[(i, j)].out_right,
                    )
                )

                m.d.comb += self.state_shift_registers[(i, j)].rst.eq(
                    self.global_state_mem_rst
                )
                m.d.comb += self.state_shift_registers[(i, j)].enable.eq(
                    self.global_state_mem_enable
                )
                m.d.comb += self.state_shift_registers[(i, j)].data_copy.eq(
                    self.global_state_mem_copy
                )

        for i in range(self.row):
            for j in range(self.col):
                # if row is even data flows from left to right
                if i % 2 == 0:
                    if j - 1 >= 0:
                        m.d.comb += self.program_shift_registers[(i, j)].data_in.eq(
                            self.program_shift_registers[(i, j - 1)].data_out
                        )
                        m.d.comb += self.state_shift_registers[(i, j)].data_in.eq(
                            self.state_shift_registers[(i, j - 1)].data_out
                        )
                # if row is odd data flows from right to left
                else:
                    if j + 1 < self.col:
                        m.d.comb += self.program_shift_registers[(i, j)].data_in.eq(
                            self.program_shift_registers[(i, j + 1)].data_out
                        )
                        m.d.comb += self.state_shift_registers[(i, j)].data_in.eq(
                            self.state_shift_registers[(i, j + 1)].data_out
                        )

        for row in range(self.row):
            # if not the last row
            if row + 1 < self.row:
                # if even, data flows from cell[row][-1] to cell[row+1][-1]
                if row % 2 == 0:
                    m.d.comb += self.program_shift_registers[
                        (row + 1, self.col - 1)
                    ].data_in.eq(
                        self.program_shift_registers[(row, self.col - 1)].data_out
                    )
                    m.d.comb += self.state_shift_registers[
                        (row + 1, self.col - 1)
                    ].data_in.eq(
                        self.state_shift_registers[(row, self.col - 1)].data_out
                    )
                # if row is odd, data flows from cell[row][0] to cell[row+1][0]
                else:
                    m.d.comb += self.program_shift_registers[(row + 1, 0)].data_in.eq(
                        self.program_shift_registers[(row, 0)].data_out
                    )
                    m.d.comb += self.state_shift_registers[(row + 1, 0)].data_in.eq(
                        self.state_shift_registers[(row, 0)].data_out
                    )

        # first cell at [0][0]
        m.d.comb += self.program_shift_registers[(0, 0)].data_in.eq(
            self.global_program_mem_in
        )

        # last cell at [row-1][col-1]
        m.d.comb += self.global_state_mem_out.eq(
            self.state_shift_registers[(self.row - 1, self.col - 1)].data_out
        )

        return m


def run_testbenches(dut):
    sim = Simulator(dut)

    async def process(ctx):
        # swap left and right
        ctx.set(dut.program_up, 0x0000)
        ctx.set(dut.program_down, 0x0000)
        ctx.set(dut.program_left, 0x0000)
        ctx.set(dut.program_right, 0x0000)
        ctx.set(dut.program_enable, 1)
        ctx.set(dut.program_rst, 0)
        await ctx.tick()
        await ctx.tick()
        ctx.set(dut.program_enable, 0)
        await ctx.tick()

        print(
            ctx.get(dut.program_up),
            ctx.get(dut.program_down),
            ctx.get(dut.program_left),
            ctx.get(dut.program_right),
        )

        # now write the inputs
        ctx.set(dut.in_up, 0)
        ctx.set(dut.in_down, 0)
        ctx.set(dut.in_left, 1)
        ctx.set(dut.in_right, 0)
        ctx.set(dut.run_enable, 1)
        ctx.set(dut.run_rst, 0)
        await ctx.tick()
        await ctx.tick()
        await ctx.tick()
        print(
            ctx.get(dut.in_up),
            ctx.get(dut.in_down),
            ctx.get(dut.in_left),
            ctx.get(dut.in_right),
        )

        # check the outputs
        out_up = ctx.get(dut.out_up)
        out_down = ctx.get(dut.out_down)
        out_left = ctx.get(dut.out_left)
        out_right = ctx.get(dut.out_right)
        print(out_up, out_down, out_left, out_right)

    sim.add_clock(1e-6)
    sim.add_testbench(process)
    with sim.write_vcd("tb_sim.vcd"):
        sim.run()


def build_design(design, build_dir: Path):
    hw_dir = build_dir
    if hw_dir.exists():
        shutil.rmtree(hw_dir)
    hw_dir.mkdir()
    v_fp = hw_dir / "hw.v"
    print("Writing to", v_fp)
    v_fp.write_text(verilog.convert(design))

    # run yosys
    script = ""
    script += "read_verilog {}\n".format(v_fp.name)
    script += "hierarchy -check\n"
    script += "proc\n"
    script += "opt\n"
    script += "write_verilog hw_simple.v\n"
    script += "show -prefix hw_simple_vis -format dot\n"
    # script += "synth_xilinx -abc9 -flatten\n"
    script += "synth -flatten\n"
    script += "tee -o hw_stats.txt stat\n"
    script += "write_verilog hw_synth.v\n"
    script += "show -prefix hw_vis -format dot"
    script_fp = hw_dir / "script.ys"
    script_fp.write_text(script)

    log_fp = hw_dir / "yosys.log"

    print("Running yosys")
    p = subprocess.run(
        ["yosys", "-q", "-s", str(script_fp.name), "-l", str(log_fp.name)],
        cwd=hw_dir,
    )
    if p.returncode != 0:
        print(p.stdout)
        print(p.stderr)
        raise Exception("yosys failed")


if __name__ == "__main__":
    # bit_grid_cell = BitGridCell()
    # run_testbenches(bit_grid_cell)
    # build_design(bit_grid_cell, Path("./hw_cell"))

    bit_grid_array = BitGridArray(8, 8)
    build_design(bit_grid_array, Path("./hw_array"))
