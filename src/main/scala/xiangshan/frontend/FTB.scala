/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.frontend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xs.utils._
import xs.utils.mbist.MBISTPipeline
import xs.utils.perf.HasPerfLogging
import xs.utils.sram.SRAMTemplate

import scala.{Tuple2 => &}


trait FTBParams extends HasXSParameter with HasBPUConst {
  val numEntries = FtbSize
  val numWays    = FtbWays
  val numSets    = numEntries/numWays // 512
  val tagSize    = 20

  val TAR_STAT_SZ = 2
  def TAR_FIT = 0.U(TAR_STAT_SZ.W)
  def TAR_OVF = 1.U(TAR_STAT_SZ.W)
  def TAR_UDF = 2.U(TAR_STAT_SZ.W)

  def BR_OFFSET_LEN = 12
  def JMP_OFFSET_LEN = 20
}

class FtbSlot()(implicit p: Parameters) extends XSBundle with FTBParams {
  val offset  = UInt(log2Ceil(PredictWidth).W)
  val lower   = UInt(JMP_OFFSET_LEN.W)
  val tarStat = UInt(TAR_STAT_SZ.W)
  val sharing = Bool()
  val valid   = Bool()




}

class FTBEntry(implicit p: Parameters) extends XSBundle with FTBParams with BPUUtils {
  
  /** Slot information */
  val valid   = Bool()
  val offset  = UInt(log2Ceil(PredictWidth).W)
  val tarStat = UInt(TAR_STAT_SZ.W)
  val lower   = UInt(JMP_OFFSET_LEN.W)
  val sharing = Bool()  // means is branch

  /** Partial Fall-Through Address */
  val carry       = Bool()
  val pftAddr     = UInt(log2Up(PredictWidth).W)

  /** Jump type */ //TODO: compat with sharing
  val isCall      = Bool()
  val isRet       = Bool()
  val isJalr      = Bool()

  val last_may_be_rvi_call = Bool()

  val always_taken = Bool()

  def setLowerStatByTarget(pc: UInt, target: UInt, isShare: Boolean = false): Unit = {
    def getTargetStatByHigher(pc_higher: UInt, target_higher: UInt) =
      Mux(target_higher > pc_higher, TAR_OVF,
        Mux(target_higher < pc_higher, TAR_UDF, TAR_FIT))

    val offLen = if (isShare) BR_OFFSET_LEN else JMP_OFFSET_LEN
    val pc_higher = pc(VAddrBits - 1, offLen + 1)
    val target_higher = target(VAddrBits - 1, offLen + 1)
    val stat = getTargetStatByHigher(pc_higher, target_higher)
    val lower = ZeroExt(target(offLen, 1), JMP_OFFSET_LEN)

    this.lower := lower
    this.tarStat := stat
    this.sharing := isShare.B
  }

  def getTarget(pc: UInt): UInt = {

    def getTarget(offLen: Int)(pc: UInt, lower: UInt, stat: UInt): UInt = {
      val h = pc(VAddrBits - 1, offLen + 1)
      val higher = Wire(UInt((VAddrBits - offLen - 1).W))
      higher := h
      val target =
        Cat(
          Mux1H(Seq(
            (stat === TAR_OVF, higher + 1.U),
            (stat === TAR_UDF, higher - 1.U),
            (stat === TAR_FIT, higher),
          )),
          lower(offLen - 1, 0), 0.U(1.W)
        )
      require(target.getWidth == VAddrBits)
      require(offLen != 0)
      target
    }

    Mux(sharing,
      getTarget(BR_OFFSET_LEN)(pc, lower, tarStat),
      getTarget(JMP_OFFSET_LEN)(pc, lower, tarStat)
    )
  }

  def isJal = !isJalr
  def getFallThrough(pc: UInt) = getFallThroughAddr(pc, carry, pftAddr)

  def getBrMaskByOffset(offset: UInt): Bool = this.valid && this.offset <= offset && this.sharing

  def getBrRecordedVec(offset: UInt): Bool = this.valid && this.offset === offset && this.sharing

  def brIsSaved(offset: UInt): Bool = getBrRecordedVec(offset)

  def brValids: Bool = this.valid && this.sharing

  def noEmptySlotForNewBr: Bool = this.valid

  def newBrCanNotInsert(offset: UInt): Bool = this.valid && this.offset < offset

  def jmpValid: Bool = this.valid && !this.sharing

  def brOffset: UInt = this.offset
}

class FTBEntryWithTag(implicit p: Parameters) extends XSBundle with FTBParams with BPUUtils {
  val entry = new FTBEntry
  val tag = UInt(tagSize.W)
}

class FTBMeta(implicit p: Parameters) extends XSBundle with FTBParams {
  val writeWay = UInt(log2Ceil(numWays).W)
  val hit = Bool()
  val fauFtbHit = if (EnableFauFTB) Some(Bool()) else None
  val pred_cycle = if (!env.FPGAPlatform) Some(UInt(64.W)) else None
}

object FTBMeta {
  def apply(writeWay: UInt, hit: Bool, fauhit: Bool, pred_cycle: UInt)(implicit p: Parameters): FTBMeta = {
    val e = Wire(new FTBMeta)
    e.writeWay := writeWay
    e.hit := hit
    e.fauFtbHit.foreach(_ := fauhit)
    e.pred_cycle.foreach(_ := pred_cycle)
    e
  }
}

class FTB(parentName:String = "Unknown")(implicit p: Parameters) extends BasePredictor with FTBParams with BPUUtils
  with HasCircularQueuePtrHelper with HasPerfEvents {
  override val meta_size = WireInit(0.U.asTypeOf(new FTBMeta)).getWidth

  val ftbAddr = new TableAddr(log2Up(numSets), 1)

  class FTBBank(val numSets: Int, val nWays: Int) extends XSModule with BPUUtils with HasPerfLogging {
    val io = IO(new Bundle {
      val s1_fire = Input(Bool())

      /** When Ftb hit, read_hits.valid is True and read_hits.bits is OneHot of hit way.
       *  When Ftb miss, read_hits.valid is False and read_hits.bits is OneHot of allocation way.
       */
      /** Predict read */
      val req_pc = Flipped(DecoupledIO(UInt(VAddrBits.W)))
      val read_resp = Output(new FTBEntry)
      val read_hits = Valid(UInt(log2Ceil(numWays).W))
      /** Update read */
      val u_req_pc = Flipped(DecoupledIO(UInt(VAddrBits.W)))
      val update_hits = Valid(UInt(log2Ceil(numWays).W))
      val update_access = Input(Bool())
      /** Update write */
      val update_pc = Input(UInt(VAddrBits.W))
      val update_write_data = Flipped(Valid(new FTBEntryWithTag))
      val update_write_way = Input(UInt(log2Ceil(numWays).W))
      val update_write_alloc = Input(Bool())
    })

    // Extract holdRead logic to fix bug that update read override predict read result
    val ftb = Module(new SRAMTemplate(new FTBEntryWithTag, set = numSets, way = numWays, shouldReset = true, holdRead = false, singlePort = true,
      hasMbist = coreParams.hasMbist,
      hasShareBus = coreParams.hasShareBus,
      parentName = parentName
    ))
    val ftb_r_entries = ftb.io.r.resp.data.map(_.entry)

    val pred_rdata = HoldUnless(ftb.io.r.resp.data, RegNext(io.req_pc.valid && !io.update_access))
    ftb.io.r.req.valid := io.req_pc.valid || io.u_req_pc.valid // io.s0_fire
    ftb.io.r.req.bits.setIdx := Mux(io.u_req_pc.valid, ftbAddr.getIdx(io.u_req_pc.bits), ftbAddr.getIdx(io.req_pc.bits)) // s0_idx

    assert(!(io.req_pc.valid && io.u_req_pc.valid))

    io.req_pc.ready := ftb.io.r.req.ready
    io.u_req_pc.ready := ftb.io.r.req.ready

    val req_tag = RegEnable(ftbAddr.getTag(io.req_pc.bits)(tagSize-1, 0), io.req_pc.valid)
    val req_idx = RegEnable(ftbAddr.getIdx(io.req_pc.bits), io.req_pc.valid)

    val u_req_tag = RegEnable(ftbAddr.getTag(io.u_req_pc.bits)(tagSize-1, 0), io.u_req_pc.valid)

    val read_entries = pred_rdata.map(_.entry)
    val read_tags    = pred_rdata.map(_.tag)

    val total_hits: Vec[Bool] = VecInit((0 until numWays).map(b => read_tags(b) === req_tag && read_entries(b).valid && io.s1_fire))
    val hit: Bool = total_hits.reduce(_||_)

    val hit_way: UInt = OHToUInt(total_hits)

    val u_total_hits = VecInit((0 until numWays).map(b =>
        ftb.io.r.resp.data(b).tag === u_req_tag && ftb.io.r.resp.data(b).entry.valid && RegNext(io.update_access)))
    val u_hit = u_total_hits.reduce(_||_)
    // val hit_way_1h = VecInit(PriorityEncoderOH(total_hits))
    val u_hit_way = OHToUInt(u_total_hits)

    for (n <- 1 to numWays) {
      XSPerfAccumulate(f"ftb_pred_${n}_way_hit", PopCount(total_hits) === n.U)
      XSPerfAccumulate(f"ftb_update_${n}_way_hit", PopCount(u_total_hits) === n.U)
    }

    val replacer = ReplacementPolicy.fromString(Some("setplru"), numWays, numSets)
    // val allocWriteWay = replacer.way(req_idx)

    val touch_set = Seq.fill(1)(Wire(UInt(log2Ceil(numSets).W)))
    val touch_way = Seq.fill(1)(Wire(Valid(UInt(log2Ceil(numWays).W))))

    val write_set = Wire(UInt(log2Ceil(numSets).W))
    val write_way = Wire(Valid(UInt(log2Ceil(numWays).W)))

    val read_set = Wire(UInt(log2Ceil(numSets).W))
    val read_way = Wire(Valid(UInt(log2Ceil(numWays).W)))

    read_set := req_idx
    read_way.valid := hit
    read_way.bits  := hit_way

    touch_set.head := Mux(write_way.valid, write_set, read_set)

    touch_way.head.valid := write_way.valid || read_way.valid
    touch_way.head.bits := Mux(write_way.valid, write_way.bits, read_way.bits)

    replacer.access(touch_set, touch_way)

    def allocWay(valids: UInt, idx: UInt): UInt = {
      if (numWays > 1) {
        val w = Wire(UInt(log2Up(numWays).W))
        val valid = WireInit(valids.andR)
        w := Mux(valid, replacer.way(idx), PriorityEncoder(~valids))
        w
      } else {
        val w = WireInit(0.U(log2Up(numWays).W))
        w
      }
    }

    io.read_resp := Mux1H(total_hits, read_entries) // Mux1H
    io.read_hits.valid := hit
    io.read_hits.bits := hit_way

    io.update_hits.valid := u_hit
    io.update_hits.bits := u_hit_way

    // Update logic
    val u_valid = io.update_write_data.valid
    val u_data = io.update_write_data.bits
    val u_idx = ftbAddr.getIdx(io.update_pc)
    val allocWriteWay = allocWay(RegNext(VecInit(ftb_r_entries.map(_.valid))).asUInt, u_idx)
    val u_way = Mux(io.update_write_alloc, allocWriteWay, io.update_write_way)
    val u_mask = UIntToOH(u_way)

    for (i <- 0 until numWays) {
      XSPerfAccumulate(f"ftb_replace_way$i", u_valid && io.update_write_alloc && u_way === i.U)
      XSPerfAccumulate(f"ftb_replace_way${i}_has_empty", u_valid && io.update_write_alloc && !ftb_r_entries.map(_.valid).reduce(_&&_) && u_way === i.U)
      XSPerfAccumulate(f"ftb_hit_way$i", hit && !io.update_access && hit_way === i.U)
    }

    ftb.io.w.apply(u_valid, u_data, u_idx, u_mask)

    // for replacer
    write_set := u_idx
    write_way.valid := u_valid
    write_way.bits := Mux(io.update_write_alloc, allocWriteWay, io.update_write_way)

  } // FTBBank

  val ftbBank = Module(new FTBBank(numSets, numWays))
  val mbistPipeline = if (coreParams.hasMbist && coreParams.hasShareBus) {
    MBISTPipeline.PlaceMbistPipeline(1, s"${parentName}_mbistPipe")
  } else {
    None
  }

  ftbBank.io.req_pc.valid := io.s0_fire(dupForFtb)
  ftbBank.io.req_pc.bits := s0_pc_dup(dupForFtb)

  val btb_enable_dup = RegNext(dup(io.ctrl.btb_enable))
  val s2_ftb_entry_dup = io.s1_fire.map(f => RegEnable(ftbBank.io.read_resp, f))
  val s3_ftb_entry_dup = io.s2_fire.zip(s2_ftb_entry_dup).map {case (f, e) => RegEnable(e, f)}
  
  val s1_ftb_hit = ftbBank.io.read_hits.valid && io.ctrl.btb_enable
  val s1_uftb_hit_dup = io.in.bits.resp_in(0).s1.full_pred.map(_.hit)
  val s2_ftb_hit_dup = io.s1_fire.map(f => RegEnable(s1_ftb_hit, f))
  val s2_uftb_hit_dup =
    if (EnableFauFTB) {
      io.s1_fire.zip(s1_uftb_hit_dup).map {case (f, h) => RegEnable(h, f)}
    } else {
      s2_ftb_hit_dup
    }
  val s2_real_hit_dup = s2_ftb_hit_dup.zip(s2_uftb_hit_dup).map(tp => tp._1 || tp._2)
  val s3_hit_dup = io.s2_fire.zip(s2_real_hit_dup).map {case (f, h) => RegEnable(h, f)}
  val writeWay = ftbBank.io.read_hits.bits

  io.out := io.in.bits.resp_in(0)

  val s1_latch_call_is_rvc   = DontCare // TODO: modify when add RAS

  io.out.s2.full_pred.zip(s2_real_hit_dup).foreach {case (fp, h) => fp.hit := h}
  val s2_uftb_full_pred_dup = io.s1_fire.zip(io.in.bits.resp_in(0).s1.full_pred).map {case (f, fp) => RegEnable(fp, f)}
  for (full_pred & s2_ftb_entry & s2_pc & s1_pc & s1_fire & s2_uftb_full_pred & s2_hit & s2_uftb_hit <-
    io.out.s2.full_pred zip s2_ftb_entry_dup zip s2_pc_dup zip s1_pc_dup zip io.s1_fire zip s2_uftb_full_pred_dup zip
    s2_ftb_hit_dup zip s2_uftb_hit_dup) {
      if (EnableFauFTB) {
        // use uftb pred when ftb not hit but uftb hit
        when (!s2_hit && s2_uftb_hit) {
          full_pred := s2_uftb_full_pred
        }.otherwise {
          full_pred.fromFtbEntry(s2_ftb_entry, s2_pc, Some((s1_pc, s1_fire)))
        }
      } else {
        full_pred.fromFtbEntry(s2_ftb_entry, s2_pc, Some((s1_pc, s1_fire)))
      }
    }

  // s3 
  val s3_full_pred = io.s2_fire.zip(io.out.s2.full_pred).map {case (f, fp) => RegEnable(fp, f)}
  // br_taken_mask from SC in stage3 is covered here, will be recovered in always taken logic
  io.out.s3.full_pred := s3_full_pred

  val s3_fauftb_hit_ftb_miss = RegEnable(!s2_ftb_hit_dup(dupForFtb) && s2_uftb_hit_dup(dupForFtb), io.s2_fire(dupForFtb))
  io.out.last_stage_ftb_entry := Mux(s3_fauftb_hit_ftb_miss, io.in.bits.resp_in(0).last_stage_ftb_entry, s3_ftb_entry_dup(dupForFtb))
  io.out.last_stage_meta := RegEnable(RegEnable(FTBMeta(writeWay.asUInt, s1_ftb_hit, s1_uftb_hit_dup(dupForFtb), GTimer()).asUInt, io.s1_fire(dupForFtb)), io.s2_fire(dupForFtb))

  // always taken logic
  for (out_fp & in_fp & s2_hit & s2_ftb_entry <-
    io.out.s2.full_pred zip io.in.bits.resp_in(0).s2.full_pred zip s2_ftb_hit_dup zip s2_ftb_entry_dup)
    out_fp.br_taken_mask := in_fp.br_taken_mask || s2_hit && s2_ftb_entry.always_taken
  for (out_fp & in_fp & s3_hit & s3_ftb_entry <-
    io.out.s3.full_pred zip io.in.bits.resp_in(0).s3.full_pred zip s3_hit_dup zip s3_ftb_entry_dup)
    out_fp.br_taken_mask := in_fp.br_taken_mask || s3_hit && s3_ftb_entry.always_taken


  // Update logic
  val u = io.update(dupForFtb)
  val update = u.bits

  val u_meta = update.meta.asTypeOf(new FTBMeta)
  // we do not update ftb on fauFtb hit and ftb miss
  val update_uftb_hit_ftb_miss = u_meta.fauFtbHit.getOrElse(false.B) && !u_meta.hit
  val u_valid = u.valid && !u.bits.old_entry && !(update_uftb_hit_ftb_miss)

  val updateDelay2 = Pipe(u, 2)
  val delay2_pc = updateDelay2.bits.pc
  val delay2_entry = updateDelay2.bits.ftb_entry

  
  val update_now = u_valid && u_meta.hit
  val update_need_read = u_valid && !u_meta.hit
  // stall one more cycle because we use a whole cycle to do update read tag hit
  io.s1_ready := ftbBank.io.req_pc.ready && !(update_need_read) && !RegNext(update_need_read)

  ftbBank.io.u_req_pc.valid := update_need_read
  ftbBank.io.u_req_pc.bits := update.pc

  val ftb_write = Wire(new FTBEntryWithTag)
  ftb_write.entry := Mux(update_now, update.ftb_entry, delay2_entry)
  ftb_write.tag   := ftbAddr.getTag(Mux(update_now, update.pc, delay2_pc))(tagSize-1, 0)

  val write_valid = update_now || DelayN(u_valid && !u_meta.hit, 2)

  ftbBank.io.update_write_data.valid := write_valid
  ftbBank.io.update_write_data.bits := ftb_write
  ftbBank.io.update_pc          := Mux(update_now, update.pc,       delay2_pc)
  ftbBank.io.update_write_way   := Mux(update_now, u_meta.writeWay, RegNext(ftbBank.io.update_hits.bits)) // use it one cycle later
  ftbBank.io.update_write_alloc := Mux(update_now, false.B,         RegNext(!ftbBank.io.update_hits.valid)) // use it one cycle later
  ftbBank.io.update_access := u_valid && !u_meta.hit
  ftbBank.io.s1_fire := io.s1_fire(dupForFtb)

  XSDebug("req_v=%b, req_pc=%x, ready=%b (resp at next cycle)\n", io.s0_fire(dupForFtb), s0_pc_dup(dupForFtb), ftbBank.io.req_pc.ready)
  XSDebug("s2_hit=%b, hit_way=%b\n", s2_ftb_hit_dup(dupForFtb), writeWay.asUInt)
  XSDebug("s2_br_taken_mask=%b, s2_real_taken_mask=%b\n",
    io.in.bits.resp_in(dupForFtb).s2.full_pred(dupForFtb).br_taken_mask, io.out.s2.full_pred(dupForFtb).real_slot_taken_mask)
  XSDebug("s2_target=%x\n", io.out.s2.target(dupForFtb))

  XSPerfAccumulate("ftb_read_hits", RegNext(io.s0_fire(dupForFtb)) && s1_ftb_hit)
  XSPerfAccumulate("ftb_read_misses", RegNext(io.s0_fire(dupForFtb)) && !s1_ftb_hit)

  XSPerfAccumulate("ftb_commit_hits", u.valid && u_meta.hit)
  XSPerfAccumulate("ftb_commit_misses", u.valid && !u_meta.hit)

  XSPerfAccumulate("ftb_update_req", u.valid)

  XSPerfAccumulate("ftb_update_ignored_old_entry", u.valid && u.bits.old_entry)
  XSPerfAccumulate("ftb_update_ignored_fauftb_hit", u.valid && update_uftb_hit_ftb_miss)
  XSPerfAccumulate("ftb_updated", u_valid)

  override val perfEvents = Seq(
    ("ftb_commit_hits            ", u.valid  &&  u_meta.hit),
    ("ftb_commit_misses          ", u.valid  && !u_meta.hit),
  )
  generatePerfEvent()
}
