package freechips.rocketchip.tile

import chisel3._
import chisel3.util._
import freechips.rocketchip.util.CompileOptions.NotStrictInferReset
import chisel3.{DontCare, WireInit, withClock, withReset}
import chisel3.internal.sourceinfo.SourceInfo
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket._
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property

case class VPUParams(
  // elen: Int = 32,
  vlen: Int = 128
)


class VPUDecoder(implicit p: Parameters) extends VPUModule()(p) {
  val io = IO(new Bundle {
    val inst = Input(Bits(32.W))
    val sigs = Output(new VPUCtrlSigs())
  })

  val default = List(N, N)
  val h: Array[(BitPat, List[BitPat])] = Array(
    // VSETVL   -> List(),
    VSETVLI  -> List(Y, Y),
    // VSETIVLI -> List(),
    VLE64_V  -> List(Y, N),
    // VSE64_V  -> List(),
    // VADD_VV  -> List(),
    // VMUL_VV  -> List(),
    // VDI      -> List(),
  )
  val decoder = DecodeLogic(io.inst, default, h)
  val s = io.sigs
  val sigs = Seq(s.wb, s.vset)
  sigs zip decoder map { case(s, d) => s := d }
}

trait HasVPUCtrlSigs {
  val wb = Bool()   // Writeback
  val vset = Bool() // 
}

class VPUCtrlSigs extends Bundle with HasVPUCtrlSigs

abstract class VPUModule(implicit val p: Parameters) extends Module with HasCoreParameters with HasVPUParameters

trait HasVPUParameters {
  // require()
}

class VPUCoreIO(implicit p: Parameters) extends CoreBundle()(p) {
  val mem = new HellaCacheIO
  
  val clock = Input(Clock())
  
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
}

class VPUIO(implicit p: Parameters) extends VPUCoreIO()(p) {
  
}

class VPU(cfg: VPUParams)(implicit p: Parameters) extends VPUModule()(p) {
  val io = IO(new VPUIO())

  val useClockGating = coreParams match {
    case r: RocketCoreParams => r.clockGate
    case _ => false
  }

  val clock_en_reg = Reg(Bool())
  // val clock_en = clock_en_reg || io.cp_req.valid

  val regfile = Mem(32, UInt(cfg.vlen.W))

  val vdecoder = Module(new VPUDecoder)
  vdecoder.io.inst := io.inst
  val wb = vdecoder.io.sigs.wb

  val valid = Reg(Bool())
  val counter = RegInit(0.U(2.W))
  val wb_data = io.wb_data.bits

  val s_ex :: s_mem :: s_wb :: Nil = Enum(3)
  val cur_stag = RegInit(s_ex)
  
  class VPUImpl {
    when (vdecoder.io.sigs.vset) {
      val vtype = io.op2_data(30, 20)
      val sew = vtype(5, 3)

      val ne_1 = cfg.vlen / 8
      val ne_2 = cfg.vlen / 16
      val ne_3 = cfg.vlen / 32
      val ne_4 = cfg.vlen / 64

      val n_elem = MuxLookup(sew, 0.U, Seq(
        0.U -> ne_1.U,
        1.U -> ne_2.U,
        2.U -> ne_3.U,
        3.U -> ne_4.U
      ))

      switch (cur_stag) {
        is (s_ex) {
          io.wb_data.bits := io.op1_data - n_elem
          cur_stag := s_mem
        }
        is (s_mem) {
          
          cur_stag := s_wb
        }
        is (s_wb) {
          io.wb_data.valid := true.B
          cur_stag := s_ex
        }
      }
    }.otherwise{
      io.wb_data.valid := false.B
    }

    /*
    when (wb) {
      valid := true.B
    }
    when (valid) {
      when (counter === 1.U) {
        counter := 0.U
        valid := false.B
        io.wb_data.valid := true.B
      }.otherwise{
        counter := counter + 1.U
        io.wb_data.valid := false.B
      }
    }
    */
  }
  val vpuImpl = withClock (io.clock) { new VPUImpl }
}


  // RegEnable?
  // RegNext => igual um Reg, porém sem precisar declarar.
  // io.out := RegNext(io.in + 1.U) // Cria um registrador na saída que recebe in + 1

  // RegInit => Cria um registrador com valor inicial/ de reset
  // val myReg = RegInit(UInt(12.W), 0.U)
  // val myReg = RegInit(0.U(12.W))

  // withClock(clock) { ... } // Define o clock de tudo que estiver dentro do bloco
  // withReset(clock) { ... } // Define o reset de tudo que estiver dentro do bloco

