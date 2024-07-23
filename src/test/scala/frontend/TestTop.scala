package frontend

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.{ChiselStage, FirtoolOption}
import device.{AXI4MemorySlave, AXI4RAM}
import freechips.rocketchip.amba.axi4.{AXI4Buffer, AXI4SlaveNode, AXI4SlaveParameters, AXI4SlavePortParameters}
import freechips.rocketchip.diplomacy.{AddressSet, DisableMonitors, LazyModule, LazyModuleImp, RegionType, TransferSizes}
import freechips.rocketchip.tilelink.{TLBuffer, TLRAM, TLSourceShrinker, TLToAXI4, TLXbar}
import org.chipsalliance.cde.config.Parameters
import system.HasSoCParameter
import top.DefaultConfig
import xiangshan.{XSCoreParameters, XSCoreParamsKey}
import xiangshan.frontend.Frontend

class TestTop (implicit p: Parameters) extends LazyModule
  with HasSoCParameter
  {

  private val l_frontend = LazyModule(new Frontend())
  private val memAddrMask = (1L << 37) - 1L
  private val memRange = AddressSet(0x00000000L, memAddrMask).subtract(AddressSet(0x00000000L, 0x7FFFFFFFL))

  private val l_ram = LazyModule(new AXI4RAM(
    memRange, 16L * 1024 * 1024 * 1024, true,
    true, 32, 2
  ))

  private val xbar = TLXbar()

  xbar := TLBuffer() := l_frontend.icache.clientNode
  xbar := TLBuffer() := l_frontend.instrUncache.clientNode

  l_ram.node := AXI4Buffer() := TLToAXI4() := xbar

  lazy val module = new LazyModuleImp(this) {
    val frontend = l_frontend.module
    frontend.io := DontCare
    frontend.io.hartId := 0.U
    frontend.io.reset_vector := 80000000L.U
  }

}

object TestTop extends App {
  val config = new DefaultConfig(1).alterPartial({
    case XSCoreParamsKey => XSCoreParameters()
  })
  val top = DisableMonitors(p => LazyModule(new TestTop()(p)))(config)

  (new ChiselStage).execute(Array("--target", "verilog") ++ args, Seq(
    FirtoolOption("-O=release"),
    FirtoolOption("--disable-all-randomization"),
    FirtoolOption("--disable-annotation-unknown"),
    FirtoolOption("--strip-debug-info"),
    FirtoolOption("--lower-memories"),
    FirtoolOption("--lowering-options=noAlwaysComb," +
      " disallowPortDeclSharing, disallowLocalVariables," +
      " emittedLineLength=120, explicitBitcast, locationInfoStyle=plain," +
      " disallowExpressionInliningInPorts, disallowMuxInlining"),
    ChiselGeneratorAnnotation(() => top.module),
  ))
}