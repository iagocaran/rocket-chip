package freechips.rocketchip.tile

import chisel3._
import chisel3.util._
import chisel3.dontTouch
import freechips.rocketchip.util.CompileOptions.NotStrictInferReset
import chisel3.{DontCare, WireInit, withClock, withReset}
import chisel3.internal.sourceinfo.SourceInfo
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.util._

case class VPUParams(
  // elen: Int = 32,
  vlen: Int = 128
)

object VALUFN {
  val FN_X    = BitPat("b???")
  val FN_ADD  = 0.U
  val FN_SUB  = 1.U
  val FN_MUL  = 2.U
}

class VPUDecoder(implicit p: Parameters) extends VPUModule()(p) {
  val io = IO(new Bundle {
    val inst = Input(Bits(32.W))
    val sigs = Output(new VPUCtrlSigs())
  })

  val default = List(N, N, N, N, N, N, N, VALUFN.FN_X)
  val h: Array[(BitPat, List[BitPat])] = Array(
    // VSETVL   -> List(),
    VSETVLI    -> List(Y, Y, N, N, N, N, N, VALUFN.FN_X),
    // VSETIVLI -> List(),
    VLE64_V    -> List(N, N, Y, N, N, N, N, VALUFN.FN_X),
    VSE64_V    -> List(N, N, N, Y, N, N, N, VALUFN.FN_X),
    VADD_VV    -> List(N, N, N, N, Y, N, N, VALUFN.FN_ADD),
    VMUL_VV    -> List(N, N, N, N, N, N, Y, VALUFN.FN_MUL),
    VREDSUM_VS -> List(N, N, N, N, N, Y, N, VALUFN.FN_ADD),
    // VDI      -> List(),
  )
  val decoder = DecodeLogic(io.inst, default, h)
  val s = io.sigs
  val sigs = Seq(s.wb, s.vset, s.load, s.store, s.op, s.aluFn)
  sigs zip decoder map { case(s, d) => s := d }
}

trait HasVPUCtrlSigs {
  val wb    = Bool() // Writeback
  val vset  = Bool() // 
  val load  = Bool() // 
  val store = Bool() // 
  val op    = Bool() // 
  // Memory Op
  val red   = Bool() // Reduction operation
  val mul   = Bool()
  val aluFn = Bits(VALUFN.FN_X.getWidth.W)
}

class VPUCtrlSigs extends Bundle with HasVPUCtrlSigs

abstract class VPUModule(implicit val p: Parameters) extends Module with HasCoreParameters with HasVPUParameters

trait HasVPUParameters {
  // require()
}

class VPUCoreIO(implicit p: Parameters) extends CoreBundle()(p) {
  val clock = Input(Clock())
  
  val mem = new HellaCacheIO
  
  val killd = Input(Bool())
  val killx = Input(Bool())
  val killm = Input(Bool())

  val inst = Input(Bits(32.W))
  val op1_data = Input(Bits(xLen.W))
  val op2_data = Input(Bits(xLen.W))
  
  val wb_data = Output(Valid(Bits(xLen.W)))

  val stall_pipeline = Output(Bool())

  val csr = new Bundle {
    val vconfig = Input(new VConfig())
    val vstart = Input(UInt(maxVLMax.log2.W))
    val vxrm = Input(UInt(2.W))
    val set_vs_dirty = Output(Bool())
    val set_vconfig = Flipped(Valid(new VConfig))
    val set_vstart = Flipped(Valid(vstart))
    val set_vxsat = Output(Bool())
  }

  val status = Input(new MStatus)
}

class VPUIO(implicit p: Parameters) extends VPUCoreIO()(p) {}

// TODO: Create classes for each instruction type
class VPUInstruction extends Bundle {}

class VPU(cfg: VPUParams)(implicit p: Parameters) extends VPUModule()(p) {
  val io = dontTouch(IO(new VPUIO()))

  val useClockGating = coreParams match {
    case r: RocketCoreParams => r.clockGate
    case _ => false
  }

  val clock_en_reg = Reg(Bool())

  val regfile = Mem(32, UInt(cfg.vlen.W))
  // vstart, vxsat, vxrm, vcsr, vl, vlenb
  val vtype   = Reg(new VType)

  val vdecoder = Module(new VPUDecoder)
  vdecoder.io.inst := io.inst

  val valid = Reg(Bool())
  val counter = RegInit(0.U(2.W))

  val sew = RegInit(0.U(3.W))
  val elem_data = RegInit(0.U(xLen.W))

  val stall_pipeline = RegInit(false.B)
  io.stall_pipeline := stall_pipeline

  val req_sent = RegInit(false.B)
  val wait_req = RegInit(false.B)
  val op_done  = RegInit(true.B)
  val req_valid = RegInit(false.B)

  val req_addr = RegInit(0.U(64.W))
  val steps    = RegInit(0.U(4.W))

  val d_insn = io.inst
  val e_insn = dontTouch(RegInit(0.U(32.W)))
  val m_insn = dontTouch(RegInit(0.U(32.W)))
  val w_insn = dontTouch(RegInit(0.U(32.W)))

  val base_address = RegInit(0.U(xLen.W))
  val wdata = RegInit(0.U(xLen.W))

  val s_idle :: s_req :: s_resp :: s_done :: Nil = Enum(4)
  val cur_stag = RegInit(s_idle)

  val de_ctrl = vdecoder.io.sigs
  val ex_ctrl = Reg(new VPUCtrlSigs())
  val me_ctrl = Reg(new VPUCtrlSigs())
  val wb_ctrl = Reg(new VPUCtrlSigs())

  class VPUImpl {
    // d_insn := Mux(stall_pipeline, d_insn, io.inst)
    e_insn := Mux(io.killd, e_insn, d_insn)
    m_insn := Mux(stall_pipeline, m_insn, e_insn)
    w_insn := Mux(stall_pipeline, w_insn, m_insn)

    // de_ctrl := id_ctrl
    ex_ctrl := Mux(stall_pipeline, ex_ctrl, de_ctrl)
    me_ctrl := Mux(stall_pipeline, me_ctrl, ex_ctrl)
    wb_ctrl := Mux(stall_pipeline, wb_ctrl, me_ctrl)

    // Register writeback
    io.wb_data.valid       := me_ctrl.wb

    // Memory interface
    io.mem.req.valid         := req_valid
    io.mem.req.bits.addr     := base_address
    io.mem.req.bits.tag      := 19.U// base_address(7, 0)
    io.mem.req.bits.cmd      := Mux(me_ctrl.store, M_XWR, M_XRD)
    io.mem.req.bits.size     := log2Ceil(8).U
    io.mem.req.bits.signed   := false.B

    io.mem.req.bits.data     := Mux(me_ctrl.store, wdata, 0.U)
    io.mem.s1_data.data      := Mux(me_ctrl.store, wdata, 0.U)

    io.mem.req.bits.phys     := false.B

    io.mem.req.bits.dprv     := io.status.dprv
    io.mem.req.bits.dv       := io.status.dv

    // VSETVLI
    {
      val ne_1 = cfg.vlen / 8
      val ne_2 = cfg.vlen / 16
      val ne_3 = cfg.vlen / 32
      val ne_4 = cfg.vlen / 64

      when (de_ctrl.vset) {
        val new_vtype = d_insn(30, 20)
        sew := new_vtype(5, 3)
      }
      
      when (ex_ctrl.vset) {
        elem_data := io.op1_data
        vtype.vsew := sew
        val n_elem = MuxLookup(sew, 0.U, Seq(
          "b000".U -> ne_1.U,
          "b001".U -> ne_2.U,
          "b010".U -> ne_3.U,
          "b011".U -> ne_4.U
        ))
        io.wb_data.bits := n_elem.min(elem_data)
      }

      when (wb_ctrl.vset) {
      }
    }
    
    // VLE
    {
      when (ex_ctrl.load) {
        base_address := io.op1_data
        when (cur_stag === s_done) {
        }.otherwise{
          stall_pipeline := true.B
        }
      }
      when (me_ctrl.load) {
        val vm    = m_insn(25)
        val rd    = m_insn(11, 7)
        val width = m_insn(14, 12)

        switch (cur_stag) {
          is (s_idle) {
            req_valid := true.B
            cur_stag := s_req
          }
          is (s_req) {
            when (io.mem.req.ready && io.mem.resp.bits.tag === 19.U) {
              cur_stag := s_resp
              req_valid := false.B
            }
          }
          is (s_resp) {
            when (io.mem.resp.valid) {
               when (steps === 0.U) {
                base_address := base_address + 8.U
                regfile(rd) := Cat(io.mem.resp.bits.data, 0.U(64.W))
                steps := steps + 1.U
                cur_stag := s_req
                req_valid := true.B
              }
              when (steps === 1.U) {
                // regfile(rd) := Cat(regfile(rd)(127, 64), io.mem.resp.bits.data)
                stall_pipeline := false.B
                cur_stag := s_done
                steps := 0.U
                req_sent := false.B
              }
            }

          }
          is (s_done) {
            regfile(rd) := Cat(regfile(rd)(127, 64), io.mem.resp.bits.data)
            cur_stag := s_idle
          }
        }
      }
    }

    // VADD
    {
      when (ex_ctrl.op) {
        val vd  = e_insn(11, 7)
        val vs1 = e_insn(19, 15)
        val vs2 = e_insn(24, 20)
        val vm  = e_insn(25)

        switch (ex_ctrl.aluFn) {
          is (VALUFN.FN_ADD) {
            regfile(vd) := Cat(regfile(vs1)(127, 64) + regfile(vs2)(127, 64), regfile(vs1)(63, 0) + regfile(vs2)(63, 0))
          }
        }
      }
    }

    // VMUL_VV
    {
      when (ex_ctrl.mul) {
        val vd  = e_insn(11, 7)
        val vs1 = e_insn(19, 15)
        val vs2 = e_insn(24, 20)
        val vm  = e_insn(25)

        regfile(vd) := Cat(regfile(vs1)(127, 64) * regfile(vs2)(127, 64), regfile(vs1)(63, 0) * regfile(vs2)(63, 0))
      }
    }

    // VREDSUM
    {
      when (ex_ctrl.red) {
        val vd  = e_insn(11, 7)
        val vs1 = e_insn(19, 15)
        val vs2 = e_insn(24, 20)
        val vm  = e_insn(25)

        switch (ex_ctrl.aluFn) {
          is (VALUFN.FN_ADD) {
            regfile(vd) := regfile(vs1) + regfile(vs2)(127, 64) + regfile(vs2)(63, 0)
          }
        }
      }
    }

    // VSE
    {
      when (ex_ctrl.store) {
        val vs3   = e_insn(11, 7) // Store data
        base_address := io.op1_data
        wdata := regfile(vs3)(127, 64)
        req_valid := true.B
        when (cur_stag === s_done) {
        }.otherwise{
          stall_pipeline := true.B
        }
      }
      when (me_ctrl.store) {
        val vm    = m_insn(25)
        val vs3   = m_insn(11, 7) // Store data
        val width = m_insn(14, 12)

        switch (cur_stag) {
          is (s_idle) {
            req_valid := true.B
            when (io.mem.req.ready && io.mem.resp.bits.tag === 19.U) {
              cur_stag := s_req
            }
          }
          is (s_req) {
            cur_stag := s_resp
          }
          is (s_resp) {
            when (io.mem.resp.valid) {
              req_valid := false.B
               when (steps === 0.U) {
                base_address := base_address + 8.U
                wdata := regfile(vs3)(63, 0)
                steps := steps + 1.U
                cur_stag := s_idle
                // cur_stag := s_req
                // req_valid := true.B
              }
              when (steps === 1.U) {
                stall_pipeline := false.B
                cur_stag := s_done
                steps := 0.U
                req_sent := false.B
              }
            }
          }
          is (s_done) {
            cur_stag := s_idle
          }
        }
      }
    }
  }
  val vpuImpl = withClock (io.clock) { new VPUImpl }
}
