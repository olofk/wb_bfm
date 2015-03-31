/*TODO:
 Test byte masks
 Add timeout
 Add FIFO mode
  */
module wb_bfm_transactor # (
    parameter                aw              = 32,
    parameter                dw              = 32,
    parameter                VERBOSE         = 0,
    parameter                MAX_BURST_LEN   = 32,
    parameter                MAX_WAIT_STATES = 8,
    parameter                MEM_LOW         = 0,
    parameter                USER_WRITE_DATA = 0,
    parameter                MEM_HIGH        = 32'hffffffff
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
   
  reg [dw-1:0]               user_data[0:MAX_BURST_LEN-1];
  integer                    word;

  integer                    SEED;
  integer                    TRANSACTIONS;
  integer                    SUBTRANSACTIONS;

  initial begin
    if(!$value$plusargs("transactions=%d", TRANSACTIONS))
      TRANSACTIONS = 1000;
    if(!$value$plusargs("subtransactions=%d", SUBTRANSACTIONS))
      SUBTRANSACTIONS = 50;
    if(!$value$plusargs("seed=%d", SEED))
      SEED = 2;
  end

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

   /*Return a 2*aw array with the highest and lowest accessed addresses
    based on starting address and burst type
    TODO: Account for short wrap bursts. Fix for 8-bit mode*/
   function [2*aw-1:0] adr_range;
     input [aw-1:0]          adr_i;
     input [$clog2(MAX_BURST_LEN+1):0] len_i;
     input [2:0]             burst_type_i;
     parameter               bpw = dw/8; //Bytes per word
     reg [aw-1:0]            adr;
     reg [aw-1:0]            adr_high;
     reg [aw-1:0]            adr_low;
      integer 		     shift;
      
   begin
     //if (bpw == 4) begin
      shift = $clog2(bpw);
       adr                   = adr_i>>shift;
       case (burst_type_i)
         LINEAR_BURST   : begin
           adr_high          = adr+len_i;
           adr_low           = adr;
         end
         WRAP_4_BURST   : begin
           adr_high          = adr[aw-1:2]*4+4;
           adr_low           = adr[aw-1:2]*4;
         end
         WRAP_8_BURST   : begin
           adr_high          = adr[aw-1:3]*8+8;
           adr_low           = adr[aw-1:3]*8;
         end
         WRAP_16_BURST  : begin
           adr_high          = adr[aw-1:4]*16+16;
           adr_low           = adr[aw-1:4]*16;
         end
         CONSTANT_BURST : begin
           adr_high          = adr+1;
           adr_low           = adr;
         end
         default : begin
           $error("%d : Illegal burst type (%b)", $time, burst_type);
           adr_range         = {2*aw{1'bx}};
         end
       endcase // case (burst_type)
      adr_high = (adr_high << shift)-1;
      adr_low  = adr_low << shift;
       adr_range             = {adr_high, adr_low};
     //end
   end
   endfunction

   // Task to fill Write Data array. Set the USER_WRITE_DATA top level
   // parameter to any non-zero value to initialise from user_data[]. Otherwise
   // random data will be used.
   task fill_wdata_array;
     input  [31:0]            burst_length; 

     begin
       // Fill write data array
       for(word = 0; word <= burst_length-1; word = word + 1) begin
         if (USER_WRITE_DATA) begin
           bfm.write_data[word] = user_data[word];
         end else begin
           bfm.write_data[word] = {$random,$random}; //FIXME
         end
       end
     end
   endtask
   
   integer                   burst_length;
   reg [2:0]                 burst_type;

   integer                   transaction;
   integer                   subtransaction;
  
   reg                       err;
  
   reg [aw-1:0]              t_address;
   reg [aw-1:0]              t_adr_high;
   reg [aw-1:0]              t_adr_low;
   reg [aw-1:0]              st_address;
   reg [aw-1:0]              st_adr_high;
   reg [aw-1:0]              st_adr_low;
   reg                       st_type;

   initial begin
      bfm.reset;
      done    = 0;
      st_type = 0;
      err     = 0;

      $display("%m : Running %0d transactions, %0d subtransactions per transaction", TRANSACTIONS, SUBTRANSACTIONS);
      $display("Max burst length=%0d", MAX_BURST_LEN);

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
         
          // Generate initial transaction parameters
          t_address                     = gen_adr(MEM_LOW, MEM_HIGH);
          burst_length                  = ({$random(SEED)} % MAX_BURST_LEN) + 1;
          burst_type                    = ({$random(SEED)} % 3);
          {t_adr_high, t_adr_low}       = adr_range(t_address, burst_length, burst_type);

          // Check if initial base address and max burst length lie within
          // MEM_HIGH/MEM_LOW bounds. If not, regenerate random values until condition met.
          while((t_adr_high > MEM_HIGH) || (t_adr_low < MEM_LOW)) begin
            t_address                   = gen_adr(MEM_LOW, MEM_HIGH);
            burst_length                = ({$random(SEED)} % MAX_BURST_LEN) + 1;
            burst_type                  = ({$random(SEED)} % 3);
            {t_adr_high,t_adr_low}      = adr_range(t_address, burst_length, burst_type);
          end

          // Write Transaction
          if (VERBOSE>0)
            $display("  Transaction %0d (Write): Start Address: %h, Burst Length: %0d Burst Type: %0d", transaction, t_address, burst_length, burst_type);
          
          // Fill Write Array then Send the Write Transaction
          fill_wdata_array(burst_length);
          bfm.write_burst(t_adr_low, t_address, {(dw/8){1'b1}}, burst_length, burst_type, err);

          // Read data can be read back from wishbone memory.
          if (VERBOSE>0)
            $display("  Transaction %0d (Read): Start Address: %h, Burst Length: %0d, Burst Type: %0d", transaction, t_address, burst_length, burst_type);
          bfm.read_burst(t_adr_low, t_address, {dw/8{1'b1}}, burst_length, burst_type, err);

          if (VERBOSE>0)
              $display("Transaction %0d Completed Successfully (Start Address: %h, Burst Length=%0d, Burst Type=%0d)", transaction, t_address, burst_length, burst_type);
        end else begin
          // Fully randomised test suite.

          // Generate initial transaction parameters
          t_address                     = gen_adr(MEM_LOW, MEM_HIGH);
          burst_length                  = ({$random(SEED)} % MAX_BURST_LEN) + 1;
          burst_type                    = ({$random(SEED)} % 3);
          {t_adr_high, t_adr_low}       = adr_range(t_address, MAX_BURST_LEN, LINEAR_BURST);

          // Check if initial base address and max burst length lie within
          // MEM_HIGH/MEM_LOW bounds. If not, regenerate random values until condition met.
          while((t_adr_high > MEM_HIGH) || (t_adr_low < MEM_LOW)) begin
            t_address                   = gen_adr(MEM_LOW, MEM_HIGH);
            burst_length                = ({$random(SEED)} % MAX_BURST_LEN) + 1;
            burst_type                  = ({$random(SEED)} % 3);
            {t_adr_high,t_adr_low}      = adr_range(t_address, MAX_BURST_LEN, LINEAR_BURST);
          end

          // Write Transaction
          if (VERBOSE>0)
            $display("  Transaction %0d Initialisation (Write): Start Address: %h, Burst Length: %0d", transaction, t_address, MAX_BURST_LEN);
          
          // Fill Write Array then Send the Write Transaction
          fill_wdata_array(MAX_BURST_LEN);
          bfm.write_burst(t_address, t_address, {dw/8{1'b1}}, MAX_BURST_LEN, LINEAR_BURST, err);

          // Read data can be read back from wishbone memory.
          if (VERBOSE>0)
            $display("  Transaction %0d Initialisation (Read): Start Address: %h, Burst Length: %0d", transaction, t_address, MAX_BURST_LEN);
          bfm.read_burst(t_address, t_address, {dw/8{1'b1}}, MAX_BURST_LEN, LINEAR_BURST, err);

          if (VERBOSE>0)
            $display("Transaction %0d initialisation ok (Start Address: %h, Burst Length: %0d, Burst Type: %0d)", transaction, t_address, MAX_BURST_LEN, LINEAR_BURST);

          // Start subtransaction loop.
          for (subtransaction = 1; subtransaction <= SUBTRANSACTIONS ; subtransaction = subtransaction + 1) begin
            if (VERBOSE>0)
              $display("%m : Subtransaction: %0d/%0d", subtransaction, SUBTRANSACTIONS);
            //else if(!(subtransaction%(SUBTRANSACTIONS/10)))
            //  $display("%m : Subtransaction: %0d/%0d", subtransaction, SUBTRANSACTIONS);

            // Transaction Type: 0=Read, 1=Write
            st_type                     = {$random(SEED)} % 2;

            // Generate initial base address for subtransaction
            st_address                  = gen_adr(t_adr_low, t_adr_high);
            burst_length                = ({$random(SEED)} % MAX_BURST_LEN) + 1;
            burst_type                  = ({$random(SEED)} % 3);
            {st_adr_high, st_adr_low}   = adr_range(st_address, burst_length, burst_type);

            // Repeat check for MEM_LOW/MEM_HIGH bounds
            while((st_adr_high > t_adr_high) || (st_adr_low < t_adr_low)) begin
              st_address                = gen_adr(t_adr_low, t_adr_high);
              burst_length              = ({$random(SEED)} % MAX_BURST_LEN) + 1;
              burst_type                = ({$random(SEED)} % 3);
              {st_adr_high, st_adr_low} = adr_range(st_address, burst_length, burst_type);
            end

//	     $display("High adr=%x",st_adr_high);
//	     $display("Low adr =%x",st_adr_low);
	     
            if (~st_type) begin
              if (VERBOSE>0)
                $display("  Subtransaction %0d.%0d (Read): Start Address: %h, Burst Type: %0h, Burst Length: %0d", transaction, subtransaction, st_address, burst_type, burst_length);

              // Send Read Transaction
              bfm.read_burst(t_address, st_address, {dw/8{1'b1}}, burst_length, burst_type, err);
             
            end else begin
              if (VERBOSE>0)
                $display("  Subtransaction %0d.%0d (Write): Start Address: %h, Burst Type: %0h, Burst Length: %0d", transaction, subtransaction, st_address, burst_type, burst_length);

              // Fill Write Array then Send the Write Transaction
              fill_wdata_array(burst_length);
              bfm.write_burst(t_address, st_address, {dw/8{1'b1}}, burst_length, burst_type, err);
              
            end // if (st_type)

            if (VERBOSE>0)
              $display("  Subtransaction %0d.%0d Completed Successfully", transaction, subtransaction);
          end // for (subtransaction=0;...
          if (VERBOSE>0)
            $display("Transaction %0d Completed Successfully", transaction);

          bfm.clear_buffer_data;
        end // if (SUBTRANSACTIONS == 0)

      end // for (transaction=0;...
      done = 1;
   end
   
endmodule
