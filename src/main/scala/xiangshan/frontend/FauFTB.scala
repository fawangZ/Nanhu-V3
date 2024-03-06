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
import utils._
import xs.utils._
import xiangshan._
import scala.{Tuple2 => &}

trait FauFTBParams extends HasXSParameter with HasBPUConst {
  val numWays = 64
  val tagSize = 16

  def special_idx_for_dup: Int = dupForTageSC
  def getTag(pc: UInt): UInt = pc(tagSize + instOffsetBits - 1, instOffsetBits)
}

class FauFTBWay(implicit p: Parameters) extends XSModule with FauFTBParams {
  val io = IO(new Bundle{
    /** Predict read */
    val req_tag = Input(UInt(tagSize.W))
    val resp = Output(new FTBEntry)
    val resp_hit = Output(Bool())

    /** Update read */
    val update_req_tag = Input(UInt(tagSize.W))
    val update_hit = Output(Bool())
    val tag_read = Output(UInt(tagSize.W))

    /** Update write */
    val write_valid = Input(Bool())
    val write_entry = Input(new FTBEntry)
    val write_tag = Input(UInt(tagSize.W))
  })

  val data = Reg(new FTBEntry)
  val tag = Reg(UInt(tagSize.W))
  val valid = RegInit(false.B)

  io.resp := data
  io.resp_hit := tag === io.req_tag && valid
  // write bypass to avoid multiple hit
  io.update_hit := ((tag === io.update_req_tag) && valid) ||
                   ((io.write_tag === io.update_req_tag) && io.write_valid)
  io.tag_read := tag

  when (io.write_valid) {
    when (!valid) {
      valid := true.B
    }
    tag   := io.write_tag
    data  := io.write_entry
  }
}


class FauFTB(implicit p: Parameters) extends BasePredictor with FauFTBParams {
  
  class FauFTBMeta(implicit p: Parameters) extends XSBundle with FauFTBParams {
    val pred_way = UInt(log2Ceil(numWays).W)
    val hit = Bool()
  }
  val resp_meta = Wire(new FauFTBMeta)
  override val meta_size = resp_meta.getWidth
  override val is_fast_pred = true

  class FauFTBBank(implicit p: Parameters) extends XSModule with FauFTBParams {
    val io = IO(new Bundle {
      val req_tag = Input(UInt(tagSize.W))
      val resp_hit_oh = Output(Vec(numWays, Bool()))
      val resp_entries = Output(Vec(numWays, new FTBEntry))
      val resp_ctrs = Output(Vec(numWays, UInt(2.W)))

      val update_req_tag = Input(UInt(tagSize.W))
      val update_hit_oh = Output(Vec(numWays, Bool()))

      val write_valid_oh = Input(Vec(numWays, Bool()))
      val write_tag = Input(UInt(tagSize.W))
      val write_entry = Input(new FTBEntry)

      val write_ctrs_valid = Input(Vec(numWays, Bool()))
      val write_ctrs = Input(Vec(numWays, UInt(2.W)))
    })

    val ways = Seq.tabulate(numWays)(_ => Module(new FauFTBWay))
    // numWays * numBr
    val ctrs = Seq.tabulate(numWays)(_ => RegInit(2.U(2.W)))

    // pred req
    ways.foreach(_.io.req_tag := io.req_tag)

    // pred resp
    io.resp_hit_oh  := ways.map(_.io.resp_hit)
    io.resp_entries := ways.map(_.io.resp)
    io.resp_ctrs    := ctrs

    // update req
    ways.foreach(_.io.update_req_tag := io.update_req_tag)
    io.update_hit_oh := VecInit(ways.map(_.io.update_hit))

    // write req
    ways.zip(io.write_valid_oh).foreach{ case (w, v) => w.io.write_valid := v }
    ways.foreach(_.io.write_tag   := io.write_tag)
    ways.foreach(_.io.write_entry := io.write_entry)

    // write ctrs
    for (ctr & valid & w_ctr <- ctrs zip io.write_ctrs_valid zip io.write_ctrs) {
      when (valid) {
        ctr := w_ctr
      }
    }
  }

  // bank 1 for tage, bank 0 for others
  val banks = Module(new FauFTBBank)
  dontTouch(banks.io)
  
  val replacer = ReplacementPolicy.fromString("plru", numWays)
  val replacer_touch_ways = Wire(Vec(2, Valid(UInt(log2Ceil(numWays).W))))

  val s1_fire_dup = Wire(Bool())
  s1_fire_dup := io.s1_fire(dupForUbtb)

  // pred req
  banks.io.req_tag := getTag(s1_pc_dup(dupForUbtb))

  // pred resp
  val s1_hit_oh_dup: UInt = banks.io.resp_hit_oh.asUInt
  val s1_hit_dup: Bool = s1_hit_oh_dup.orR
  val s1_hit_way_dup: UInt = OHToUInt(s1_hit_oh_dup)
  val s1_possible_full_preds_dup = Wire(Vec(numWays, new FullBranchPrediction))
  
  for (w <- 0 until numWays) {
    val fp = s1_possible_full_preds_dup(w)
    val entry = banks.io.resp_entries(w)
    val s1_pc = s1_pc_dup(dupForUbtb)
    fp.fromFtbEntry(entry, s1_pc)
    fp.hit := DontCare
    val ctr = banks.io.resp_ctrs(w)
    fp.br_taken_mask := ctr(1) || entry.always_taken
  }

  // pred update replacer state
  replacer_touch_ways(0).valid := RegNext(s1_fire_dup && s1_hit_dup)
  replacer_touch_ways(0).bits  := RegEnable(s1_hit_way_dup, s1_fire_dup && s1_hit_dup)


  val s1_hit_full_pred_dup = Mux1H(s1_hit_oh_dup, s1_possible_full_preds_dup)
  XSError(PopCount(s1_hit_oh_dup) > 1.U, "fauftb has multiple hits!\n")

  val fauftb_enable_dup = RegNext(dup(io.ctrl.ubtb_enable))

  io.out.s1.full_pred.foreach(_ := s1_hit_full_pred_dup)
  io.out.s1.full_pred.zip(fauftb_enable_dup).foreach {case (fp, en) => fp.hit := s1_hit_dup && en}

  for (i <- 1 until numDup) {
    XSError(io.out.s1.full_pred(i).asUInt =/= io.out.s1.full_pred(0).asUInt,
      p"fauftb s1 pred $i differs from pred 0\n")
  }

  // assign metas
  io.out.last_stage_meta := resp_meta.asUInt
  resp_meta.hit := RegEnable(RegEnable(s1_hit_dup, io.s1_fire(dupForUbtb)), io.s2_fire(dupForUbtb))
  resp_meta.pred_way := RegEnable(RegEnable(s1_hit_way_dup, io.s1_fire(dupForUbtb)), io.s2_fire(dupForUbtb))


  val s1_ftb_entry = Mux1H(s1_hit_oh_dup, banks.io.resp_entries)
  io.out.last_stage_ftb_entry := RegEnable(RegEnable(s1_ftb_entry, io.s1_fire(dupForUbtb)), io.s2_fire(dupForUbtb))

  /********************** update ***********************/
  // s0: update_valid, read and tag comparison
  // s1: alloc_way and write

  // s0
  val us = io.update(dupForUbtb)
  val u_valids = io.update(dupForUbtb).valid

  val u_meta_dup = us.bits.meta.asTypeOf(new FauFTBMeta)
  val u_s0_tag_dup = getTag(us.bits.pc)

  banks.io.update_req_tag := u_s0_tag_dup

  val u_s0_hit_oh_dup = banks.io.update_hit_oh.asUInt
  val u_s0_hit_dup = u_s0_hit_oh_dup.orR
  val u_s0_br_update_valids_dup =
    us.bits.ftb_entry.brValids && us.valid && !us.bits.ftb_entry.always_taken

  // s1
  val u_s1_valid_dup = RegNext(dup(us.valid, numWays+1)) // reduce fanouts
  val u_s1_tag_dup: UInt = RegEnable(u_s0_tag_dup, u_valids)
  val u_s1_hit_oh_dup: UInt = RegEnable(u_s0_hit_oh_dup, u_valids)
  val u_s1_hit_dup: Bool = RegEnable(u_s0_hit_dup, u_valids)
  val u_s1_alloc_way_dup: UInt = replacer.way
  val u_s1_write_way_oh_dup: UInt = Mux(u_s1_hit_dup, u_s1_hit_oh_dup, UIntToOH(u_s1_alloc_way_dup))
  val u_s1_ftb_entry_dup = RegEnable(us.bits.ftb_entry, us.valid)
  val u_s1_ways_write_valid_dup = Wire(Vec(numWays, Bool()))

  u_s1_ways_write_valid_dup := VecInit((0 until numWays).map(w => u_s1_write_way_oh_dup(w).asBool && u_s1_valid_dup(w)))
  for (w <- 0 until numWays) {
    banks.io.write_valid_oh(w) := u_s1_ways_write_valid_dup(w)
    banks.io.write_tag         := u_s1_tag_dup
    banks.io.write_entry       := u_s1_ftb_entry_dup
  }


  // update saturating counters
  val u_s1_br_update_valids_dup = RegEnable(u_s0_br_update_valids_dup, us.valid)
  val u_s1_br_takens_dup        = RegEnable(us.bits.br_taken_mask,  us.valid)

  for (w <- 0 until numWays) {
      banks.io.write_ctrs(w) := satUpdate(banks.io.resp_ctrs(w), 2, u_s1_br_takens_dup)
      banks.io.write_ctrs_valid(w) := u_s1_br_update_valids_dup && u_s1_ways_write_valid_dup(w)

  }
  // commit update replacer state
  replacer_touch_ways(1).valid := u_s1_valid_dup.last
  replacer_touch_ways(1).bits  := OHToUInt(u_s1_write_way_oh_dup)
  /******** update replacer *********/
  replacer.access(replacer_touch_ways)

  /********************** perf counters **********************/
  val s0_fire_next_cycle = RegNext(io.s0_fire(dupForUbtb))
  val u_pred_hit_way_map   = (0 until numWays).map(w => s0_fire_next_cycle && s1_hit_dup && s1_hit_way_dup === w.U)
  val u_commit_hit_way_map = (0 until numWays).map(w => us.valid && u_meta_dup.hit && u_meta_dup.pred_way === w.U)
  XSPerfAccumulate("uftb_read_hits",   s0_fire_next_cycle &&  s1_hit_dup)
  XSPerfAccumulate("uftb_read_misses", s0_fire_next_cycle && !s1_hit_dup)
  XSPerfAccumulate("uftb_commit_hits",   us.valid &&  u_meta_dup.hit)
  XSPerfAccumulate("uftb_commit_misses", us.valid && !u_meta_dup.hit)
  XSPerfAccumulate("uftb_commit_read_hit_pred_miss", us.valid && !u_meta_dup.hit && u_s0_hit_oh_dup(0).orR)
  for (w <- 0 until numWays) {
    XSPerfAccumulate(f"uftb_pred_hit_way_${w}",   u_pred_hit_way_map(w))
    XSPerfAccumulate(f"uftb_commit_hit_way_${w}", u_commit_hit_way_map(w))
    XSPerfAccumulate(f"uftb_replace_way_${w}", !u_s1_hit_dup(0) && u_s1_alloc_way_dup(0) === w.U)
  }

  override val perfEvents = Seq(
    ("fauftb_commit_hit       ", us.valid &&  u_meta_dup.hit),
    ("fauftb_commit_miss      ", us.valid && !u_meta_dup.hit),
  )
  generatePerfEvent()
  
}
