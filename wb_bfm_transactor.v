/*
* Wishbone BFM transactor
* Copyright (C) 2013-2015 Olof Kindgren <olof.kindgren@gmail.com>
*               2014-2015 Matt Holdsworth <matt.holdsworth@latticesemi.com>
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

/*TODO:
 Test byte masks
 Add timeout
 Add FIFO mode
  */
module wb_bfm_transactor # (
    parameter                aw              = 32,
    parameter                dw              = 32,
    parameter                MEM_HIGH        = 32'hffffffff,
    parameter                MEM_LOW         = 0,
    parameter                TRANSACTIONS_PARAM    = 1000,
    parameter                SUBTRANSACTIONS_PARAM = 100,
    parameter                VERBOSE         = 0,
    parameter                MAX_BURST_LEN   = 32,
    parameter                MAX_WAIT_STATES = 8,
    parameter                CLASSIC_PROB          = 33,
    parameter                CONST_BURST_PROB      = 33,
    parameter                INCR_BURST_PROB       = 34,
    parameter                SEED_PARAM            = 0
  ) (
    input 	      wb_clk_i,
    input 	      wb_rst_i,
    output [aw-1:0]   wb_adr_o,
    output [dw-1:0]   wb_dat_o,
    output [dw/8-1:0] wb_sel_o,
    output 	      wb_we_o,
    output 	      wb_cyc_o,
    output 	      wb_stb_o,
    output [2:0]      wb_cti_o,
    output [1:0]      wb_bte_o,
    input [dw-1:0]    wb_dat_i,
    input 	      wb_ack_i,
    input 	      wb_err_i,
    input 	      wb_rty_i,
    output reg 	      done
  );

  `include "wb_common_params.v"

   localparam ADR_LSB = $clog2(dw/8);
   
  integer                    SEED;
  integer                    TRANSACTIONS;
  integer                    SUBTRANSACTIONS;

  // Grab CLI Values
  initial begin
    if(!$value$plusargs("transactions=%d", TRANSACTIONS))
      TRANSACTIONS    = TRANSACTIONS_PARAM;
    if(!$value$plusargs("subtransactions=%d", SUBTRANSACTIONS))
      SUBTRANSACTIONS = SUBTRANSACTIONS_PARAM;
    if(!$value$plusargs("seed=%d", SEED))
      SEED = SEED_PARAM;
  end

   integer cnt_cti_classic = 0;
   integer cnt_cti_const_burst = 0;
   integer cnt_cti_inc_burst = 0;
   integer cnt_cti_invalid = 0;

   integer cnt_bte_linear  = 0;
   integer cnt_bte_wrap_4  = 0;
   integer cnt_bte_wrap_8  = 0;
   integer cnt_bte_wrap_16 = 0;

  wb_bfm_master #(
		  .dw (dw),
    .MAX_BURST_LEN           (MAX_BURST_LEN),
    .MAX_WAIT_STATES         (MAX_WAIT_STATES),
    .VERBOSE                 (VERBOSE)
  ) bfm (
    .wb_clk_i                (wb_clk_i),
    .wb_rst_i                (wb_rst_i),
    .wb_adr_o                (wb_adr_o),
    .wb_dat_o                (wb_dat_o),
    .wb_sel_o                (wb_sel_o),
    .wb_we_o                 (wb_we_o), 
    .wb_cyc_o                (wb_cyc_o),
    .wb_stb_o                (wb_stb_o),
    .wb_cti_o                (wb_cti_o),
    .wb_bte_o                (wb_bte_o),
    .wb_dat_i                (wb_dat_i),
    .wb_ack_i                (wb_ack_i),
    .wb_err_i                (wb_err_i),
    .wb_rty_i                (wb_rty_i)
  );

   function [aw-1:0] gen_adr;
      input integer low;
      input integer high;
      begin
          gen_adr = (low + ({$random(SEED)} % (high-low))) &  {{aw-ADR_LSB{1'b1}},{ADR_LSB{1'b0}}};
      end
   endfunction

   function [2:0] gen_cycle_type;
      input integer cycle_type_prob;
      begin
         if (cycle_type_prob <= CLASSIC_PROB) begin
            gen_cycle_type                = 3'b000;
         end else if (cycle_type_prob <= (CLASSIC_PROB + CONST_BURST_PROB)) begin
            gen_cycle_type                = CTI_CONST_BURST;
         end else begin
            gen_cycle_type                = CTI_INC_BURST;
         end
      end
   endfunction

   function [aw+3+2+32-1:0] gen_cycle_params;
      input [aw-1:0] adr_min_i;
      input [aw-1:0] adr_max_i;

      reg [aw-1:0]   adr_low;
      reg [aw-1:0]   adr_high;

      reg [aw-1:0]   address;
      reg [2:0]      cycle_type;
      reg [1:0]      burst_type;
      reg [31:0]     burst_length;
      
      begin
         adr_low  = 0;
         adr_high = 0;
         // Repeat check for MEM_LOW/MEM_HIGH bounds until satisfied
         while((adr_high > adr_max_i) || (adr_low < adr_min_i) || (adr_high == adr_low)) begin
            address                = gen_adr(adr_min_i, adr_max_i);
            cycle_type             = gen_cycle_type({$random(SEED)} % 100);

	    burst_type = (cycle_type === CTI_INC_BURST) ? ({$random(SEED)} % 4) : 0;

	    burst_length = (cycle_type === CTI_CLASSIC) ? 1 :
			   ({$random(SEED)} % MAX_BURST_LEN) + 1;

              {adr_high, adr_low} = adr_range(address, burst_length, cycle_type, burst_type);
         end
	 gen_cycle_params = {address, cycle_type, burst_type, burst_length};
      end
   endfunction
      
   /*Return a 2*aw array with the highest and lowest accessed addresses
    based on starting address and burst type
    TODO: Account for short wrap bursts. Fix for 8-bit mode*/
   function [2*aw-1:0] adr_range;
     input [aw-1:0]          adr_i;
     input [$clog2(MAX_BURST_LEN+1):0] len_i;
     input [2:0]             cti_i;
     input [1:0]             bte_i;
     parameter               bpw = dw/8; //Bytes per word
     reg [aw-1:0]            adr;
     reg [aw-1:0]            adr_high;
     reg [aw-1:0]            adr_low;
      integer 		     shift;
      
   begin
     //if (bpw == 4) begin
      shift = $clog2(bpw);
       adr                   = adr_i>>shift;
      if (cti_i === CTI_INC_BURST)
       case (bte_i)
         BTE_LINEAR : begin
           adr_high          = adr+len_i;
           adr_low           = adr;
         end
         BTE_WRAP_4   : begin
           adr_high          = adr[aw-1:2]*4+4;
           adr_low           = adr[aw-1:2]*4;
         end
         BTE_WRAP_8   : begin
           adr_high          = adr[aw-1:3]*8+8;
           adr_low           = adr[aw-1:3]*8;
         end
         BTE_WRAP_16  : begin
           adr_high          = adr[aw-1:4]*16+16;
           adr_low           = adr[aw-1:4]*16;
         end
         default : begin
           $error("%d : Illegal burst type (%b)", $time, bte_i);
           adr_range         = {2*aw{1'bx}};
         end
       endcase // case (bte_i)
      else begin
         adr_high          = adr+1;
         adr_low           = adr;
      end

      adr_high = (adr_high << shift)-1;
      adr_low  = adr_low << shift;
       adr_range             = {adr_high, adr_low};
     //end
   end
   endfunction

   //Gather transaction statistics
   //TODO: Record shortest/longest bursts.
   task update_stats;
      input [2:0] cti;
      input [1:0] bte;
      input integer burst_length;
      begin
	 case (cti)
	   CTI_CLASSIC:     cnt_cti_classic = cnt_cti_classic + 1;
	   CTI_CONST_BURST: cnt_cti_const_burst = cnt_cti_const_burst + 1;
	   CTI_INC_BURST:   cnt_cti_inc_burst = cnt_cti_inc_burst + 1;
	   default:         cnt_cti_invalid = cnt_cti_invalid + 1;
	 endcase // case (cti)
	 if (cti === CTI_INC_BURST)
	   case (bte)
             BTE_LINEAR  : cnt_bte_linear  = cnt_bte_linear  + 1;
             BTE_WRAP_4  : cnt_bte_wrap_4  = cnt_bte_wrap_4  + 1;
             BTE_WRAP_8  : cnt_bte_wrap_8  = cnt_bte_wrap_8  + 1;
             BTE_WRAP_16 : cnt_bte_wrap_16 = cnt_bte_wrap_16 + 1;
	     default : $error("Invalid BTE %2b", bte);
	   endcase // case (bte)
      end
   endtask

   task display_stats;
      begin
	 $display("#################################");
	 $display("##### Cycle Type Statistics #####");
	 $display("#################################");
	 $display("Invalid cycle types   : %0d", cnt_cti_invalid);
	 $display("Classic cycles        : %0d", cnt_cti_classic);
	 $display("Constant burst cycles : %0d", cnt_cti_const_burst);
	 $display("Increment burst cycles: %0d", cnt_cti_inc_burst);
	 $display("   Linear bursts      : %0d", cnt_bte_linear);
	 $display("   4-beat bursts      : %0d", cnt_bte_wrap_4);
	 $display("   8-beat bursts      : %0d", cnt_bte_wrap_8);
	 $display("  16-beat bursts      : %0d", cnt_bte_wrap_16);
      end
   endtask

   task display_subtransaction;
      input [aw-1:0] address;
      input [2:0]    cycle_type;
      input [1:0]    burst_type;
      input integer  burst_length;
      input 	     wr;
      
      begin
	 if (VERBOSE > 0) begin
	    $write("  Subtransaction %0d.%0d ", transaction, subtransaction);
	    if (wr)
	      $write("(Write)");
	    else
	      $write("(Read) ");
	    $display(": Start Address: %h, Cycle Type: %b, Burst Type: %b, Burst Length: %0d", address, cycle_type, burst_type, burst_length);
	 end
      end
   endtask
   
   // Task to fill Write Data array.
   // random data will be used.
   task fill_wdata_array;
     input  [31:0]            burst_length; 

     integer 		      word;
      
     begin
       // Fill write data array
       for(word = 0; word <= burst_length-1; word = word + 1) begin
          bfm.write_data[word] = $random;
       end
     end
   endtask
   
   integer                   burst_length;
   reg [1:0]                 burst_type;
   
   reg [2:0]                 cycle_type;

   integer                   transaction;
   integer                   subtransaction;
  
   reg                       err;
  
   reg [aw-1:0]              t_address;
   reg [aw-1:0]              t_adr_high;
   reg [aw-1:0]              t_adr_low;
   reg [aw-1:0]              st_address;
   reg                       st_type;

   initial begin
      bfm.reset;
      done    = 0;
      st_type = 0;
      err     = 0;

      $display("##############################################################");
      $display("############# Wishbone Master Test Configuration #############");
      $display("##############################################################");
      $display("");
      $display("%m:");
      $display("  Memory High Address   : %h", MEM_HIGH);
      $display("  Memory Low Address    : %h", MEM_LOW);
      $display("  Transactions          : %0d", TRANSACTIONS);
      $display("  Subtransactions       : %0d", SUBTRANSACTIONS);
      $display("  Max Burst Length      : %0d", MAX_BURST_LEN);
      $display("  Max Wait States       : %0d", MAX_WAIT_STATES);
      $display("  Classic Cycle Prob    : %0d", CLASSIC_PROB);
      $display("  Const Addr Cycle Prob : %0d", CONST_BURST_PROB);
      $display("  Incr Addr Cycle Prob  : %0d", INCR_BURST_PROB);
      $display("  Write Data            : Random");
      $display("  Buffer Data           : Mirrors RAM");
      $display("  $random Seed          : %0d", SEED);
      $display("  Verbosity             : %0d", VERBOSE);
      $display("");
      $display("############# Starting Wishbone Master Tests...  #############");
      $display("");

      for(transaction = 1 ; transaction <= TRANSACTIONS; transaction = transaction + 1) begin
        if (VERBOSE>0)
          $display("%m : Transaction: %0d/%0d", transaction, TRANSACTIONS);
        else if(!(transaction%(SUBTRANSACTIONS/10)))
          $display("%m : Transaction: %0d/%0d", transaction, TRANSACTIONS);

        // Generate the random value for the number of wait states. This will
        // be used for all of this transaction
        bfm.wait_states                 = {$random(SEED)} % (MAX_WAIT_STATES+1);
        if (VERBOSE>2)
          $display("  Number of Wait States for Transaction %0d is %0d", transaction, bfm.wait_states);

        if (SUBTRANSACTIONS == 0) begin
          // No subtransactions, just run basic test: Write Burst Transaction
          // followed by identical Read Burst Transaction.
          $display("  Running Basic Tests Only. To run fully randomised tests set number of subtransactions >= 1");
         
	  {t_address, cycle_type, burst_type, burst_length} = gen_cycle_params(MEM_LOW, MEM_HIGH);

          // Write Transaction
          if (VERBOSE>0)
            $display("  Transaction %0d (Write): Start Address: %h, Cycle Type: %b, Burst Type: %b, Burst Length: %0d", transaction, t_address, cycle_type, burst_type, burst_length);
          
          // Fill Write Array then Send the Write Transaction
          fill_wdata_array(burst_length);
          bfm.write_burst(t_address, t_address, {(dw/8){1'b1}}, cycle_type, burst_type, burst_length, err);
	  update_stats(cycle_type, burst_type, burst_length);

          // Read data can be read back from wishbone memory.
          if (VERBOSE>0)
            $display("  Transaction %0d (Read): Start Address: %h, Cycle Type: %b, Burst Type: %b, Burst Length: %0d", transaction, t_address, cycle_type, burst_type, burst_length);
          bfm.read_burst_comp(t_adr_low, t_address, {dw/8{1'b1}}, cycle_type, burst_type, burst_length, err);
	  update_stats(cycle_type, burst_type, burst_length);

          if (VERBOSE>0)
              $display("Transaction %0d Completed Successfully (Start Address: %h, Cycle Type: %b, Burst Type=%b, Burst Length=%0d)", transaction, t_address, cycle_type, burst_type, burst_length);
        end else begin
          // Fully randomised test suite.

          // Check if initial base address and max burst length lie within
          // MEM_HIGH/MEM_LOW bounds. If not, regenerate random values until condition met.
          t_adr_high  = 0;
          t_adr_low   = 0;
          while((t_adr_high > MEM_HIGH) || (t_adr_low < MEM_LOW) || (t_adr_high == t_adr_low)) begin
            t_address                   = gen_adr(MEM_LOW, MEM_HIGH);
            {t_adr_high,t_adr_low}      = adr_range(t_address, MAX_BURST_LEN, CTI_INC_BURST, BTE_LINEAR);
          end

          // Write Transaction
          if (VERBOSE>0)
            $display("  Transaction %0d Initialisation (Write): Start Address: %h, Burst Length: %0d", transaction, t_address, MAX_BURST_LEN);
          
          // Fill Write Array then Send the Write Transaction
          fill_wdata_array(MAX_BURST_LEN);
          bfm.write_burst(t_address, t_address, {dw/8{1'b1}}, CTI_INC_BURST, BTE_LINEAR, MAX_BURST_LEN, err);
          update_stats(cycle_type, burst_type, burst_length);

          // Read data can be read back from wishbone memory.
          if (VERBOSE>0)
            $display("  Transaction %0d Initialisation (Read): Start Address: %h, Burst Length: %0d", transaction, t_address, MAX_BURST_LEN);
          bfm.read_burst_comp(t_address, t_address, {dw/8{1'b1}}, CTI_INC_BURST, BTE_LINEAR, MAX_BURST_LEN, err);
          update_stats(cycle_type, burst_type, burst_length);

          if (VERBOSE>0)
            $display("Transaction %0d initialisation ok (Start Address: %h, Cycle Type: %b, Burst Type: %b, Burst Length: %0d)", transaction, t_address, CTI_INC_BURST, BTE_LINEAR, MAX_BURST_LEN);

          // Start subtransaction loop.
          for (subtransaction = 1; subtransaction <= SUBTRANSACTIONS ; subtransaction = subtransaction + 1) begin

            // Transaction Type: 0=Read, 1=Write
            st_type                     = {$random(SEED)} % 2;

            {st_address, cycle_type, burst_type, burst_length} = gen_cycle_params(t_adr_low, t_adr_high);
                 
            display_subtransaction(st_address, cycle_type, burst_type, burst_length, st_type);

            if (~st_type) begin

              // Send Read Transaction
              bfm.read_burst_comp(t_address, st_address, {dw/8{1'b1}}, cycle_type, burst_type, burst_length, err);
             
            end else begin

              // Fill Write Array then Send the Write Transaction
              fill_wdata_array(burst_length);
              bfm.write_burst(t_address, st_address, {dw/8{1'b1}}, cycle_type, burst_type, burst_length, err);
              
            end // if (st_type)
            update_stats(cycle_type, burst_type, burst_length);
          end // for (subtransaction=0;...
          if (VERBOSE>0)
            $display("Transaction %0d Completed Successfully", transaction);

        end // if (SUBTRANSACTIONS == 0)

        // Clear Buffer Data before next transaction
        bfm.clear_buffer_data;
      end // for (transaction=0;...
      done = 1;
      display_stats;
   end
   
endmodule
