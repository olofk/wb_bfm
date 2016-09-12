/*
* Wishbone BFM testbench
* Copyright (C) 2013-2015 Olof Kindgren <olof.kindgren@gmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

module wb_bfm_tb;   

   vlog_tb_utils vlog_tb_utils0();
   vlog_tap_generator #("wb_bfm.tap", 1) vtg();

   localparam aw = 32;
   localparam dw = 32;
   
   reg	   wb_clk = 1'b1;
   reg	   wb_rst = 1'b1;
   
   always #5 wb_clk <= ~wb_clk;
   initial  #100 wb_rst <= 0;

   wire    done;
   
   wire [aw-1:0] wb_m2s_adr;
   wire [dw-1:0] wb_m2s_dat;
   wire [3:0] 	 wb_m2s_sel;
   wire 	 wb_m2s_we ;
   wire 	 wb_m2s_cyc;
   wire 	 wb_m2s_stb;
   wire [2:0] 	 wb_m2s_cti;
   wire [1:0] 	 wb_m2s_bte;
   wire [dw-1:0] wb_s2m_dat;
   wire 	 wb_s2m_ack;
   wire 	 wb_s2m_err;
   wire 	 wb_s2m_rty;

   wb_bfm_transactor
     #(.MEM_HIGH (32'h00007fff),
       .AUTORUN (0),
       .VERBOSE (0))
   master
     (.wb_clk_i (wb_clk),
      .wb_rst_i (wb_rst),
      .wb_adr_o (wb_m2s_adr),
      .wb_dat_o (wb_m2s_dat),
      .wb_sel_o (wb_m2s_sel),
      .wb_we_o  (wb_m2s_we ),
      .wb_cyc_o (wb_m2s_cyc),
      .wb_stb_o (wb_m2s_stb),
      .wb_cti_o (wb_m2s_cti),
      .wb_bte_o (wb_m2s_bte),
      .wb_dat_i (wb_s2m_dat),
      .wb_ack_i (wb_s2m_ack),
      .wb_err_i (wb_s2m_err),
      .wb_rty_i (wb_s2m_rty),
      .done     (done));
   
   wb_bfm_memory #(.DEBUG (0))
   wb_mem_model0
     (.wb_clk_i (wb_clk),
      .wb_rst_i (wb_rst),
      .wb_adr_i (wb_m2s_adr),
      .wb_dat_i (wb_m2s_dat),
      .wb_sel_i (wb_m2s_sel),
      .wb_we_i  (wb_m2s_we ),
      .wb_cyc_i (wb_m2s_cyc),
      .wb_stb_i (wb_m2s_stb),
      .wb_cti_i (wb_m2s_cti),
      .wb_bte_i (wb_m2s_bte),
      .wb_dat_o (wb_s2m_dat),
      .wb_ack_o (wb_s2m_ack),
      .wb_err_o (wb_s2m_err),
      .wb_rty_o (wb_s2m_rty));

   integer 	 TRANSACTIONS;
   integer 	 SUBTRANSACTIONS;
   integer 	 SEED;

   initial begin
      //Grab CLI parameters
      if($value$plusargs("transactions=%d", TRANSACTIONS))
	master.set_transactions(TRANSACTIONS);
      if($value$plusargs("subtransactions=%d", SUBTRANSACTIONS))
	master.set_subtransactions(SUBTRANSACTIONS);
      if($value$plusargs("seed=%d", SEED))
	master.SEED = SEED;

      master.display_settings;
      master.run;
      master.display_stats;
   end

   always @(posedge done) begin
      vtg.ok("All tests complete");
      $display("All tests complete");
      $finish;
   end

endmodule
