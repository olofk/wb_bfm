module wb_bfm_master #(
  parameter                  aw              = 32,
  parameter                  dw              = 32,
  parameter                  Tp              = 0,
  parameter                  MAX_BURST_LEN   = 32,
  parameter                  MAX_WAIT_STATES = 8, 
  parameter                  VERBOSE         = 0
) (
  input 		wb_clk_i,
  input 		wb_rst_i,
  output reg [aw-1:0] 	wb_adr_o,
  output reg [dw-1:0] 	wb_dat_o,
  output reg [dw/8-1:0] wb_sel_o,
  output reg 		wb_we_o,
  output reg 		wb_cyc_o,
  output reg 		wb_stb_o,
  output reg [2:0] 	wb_cti_o,
  output reg [1:0] 	wb_bte_o,
  input [dw-1:0] 	wb_dat_i,
  input 		wb_ack_i,
  input 		wb_err_i,
  input 		wb_rty_i
);

`include "wb_common.v"

  localparam                 BUFFER_WIDTH = $clog2(MAX_BURST_LEN);
   localparam ADR_LSB = $clog2(dw/8);
   
  reg [aw-1:0]               addr;
  reg [31:0]                 index = 0;
  reg [dw-1:0]               data = {dw{1'b0}};
   reg [dw/8-1:0] 	     mask;
  reg                        op;

  reg [2:0]                  cycle_type;
  reg [2:0]                  burst_type;
  reg [31:0]                 burst_length;
  reg [aw-1:0]               buffer_addr_tmp;
  reg [BUFFER_WIDTH-1:0]     buffer_addr;

  reg [dw-1:0]               write_data[0:MAX_BURST_LEN-1];
  reg [dw-1:0]               buffer_data[0:MAX_BURST_LEN-1];

  reg [$clog2(MAX_WAIT_STATES):0] wait_states;
  reg [$clog2(MAX_WAIT_STATES):0] wait_states_cnt;


  integer                    word;

  task reset;
    begin
      wb_adr_o               = {aw{1'b0}};
      wb_dat_o               = {dw{1'b0}};
      wb_sel_o               = {dw/8{1'b0}};
      wb_we_o                = 1'b0;
      wb_cyc_o               = 1'b0;
      wb_stb_o               = 1'b0;
      wb_cti_o               = 3'b000;
      wb_bte_o               = 2'b00;
    end
  endtask

  task write;
    input [aw-1:0]           addr_i;
    input [dw-1:0]           data_i;
    input [dw/8-1:0]              mask_i;

    output                   err_o;

    begin
      addr                   = addr_i;
      data                   = data_i;
      mask                   = mask_i;
      cycle_type             = CTI_CLASSIC;
      op                     = WRITE;

      init;
      @(posedge wb_clk_i);
      next;
      err_o                  = wb_err_i;
     end
  endtask //

  task write_burst;
    input  [aw-1:0]          base_addr;
    input  [aw-1:0]          addr_i;
     input [dw/8-1:0] 	     mask_i;
    input  [2:0]             cycle_type_i;
    input  [2:0]             burst_type_i;
    input  [31:0]            burst_length_i;
    output                   err_o;
  begin

    addr                     = addr_i;
    buffer_addr_tmp          = addr_i - base_addr;
    buffer_addr              = buffer_addr_tmp[ADR_LSB+BUFFER_WIDTH-1:ADR_LSB];
    mask                     = mask_i;
    op                       = WRITE;
    burst_length             = burst_length_i;
    cycle_type               = cycle_type_i;
    burst_type               = burst_type_i;
    index                    = 0;
    err_o                    = 0;

    init;

    while(index < burst_length) begin
      buffer_data[buffer_addr] = write_data[index];
      data                     = write_data[index];

      if (VERBOSE>2) begin
        $display("    %t: Write Data %h written to buffer at address %h at iteration %0d", $time, write_data[index], buffer_addr, index);
        $display("    %t: Write Data %h written to memory at address %h at iteration %0d", $time, write_data[index], addr, index);
      end else if (VERBOSE>1) begin
        $display("    Write Data %h written to memory at address %h at iteration %0d", write_data[index], addr, index);
      end

      next;
      addr                   = wb_next_adr(addr, cycle_type, burst_type, dw);
      buffer_addr_tmp        = addr - base_addr;
      buffer_addr            = buffer_addr_tmp[ADR_LSB+BUFFER_WIDTH-1:ADR_LSB];
      index                  = index + 1;
    end

    clear_write_data;

    insert_wait_states;
  end
  endtask

  task read_burst;
    input  [aw-1:0]          base_addr;
    input  [aw-1:0]          addr_i;
    input  [dw/8-1:0]             mask_i;
    input  [2:0]             cycle_type_i;
    input  [1:0]             burst_type_i;
    input  [31:0]            burst_length_i;
    output                   err_o;

  begin

    addr                     = addr_i;
    buffer_addr_tmp          = addr_i - base_addr;
    buffer_addr              = buffer_addr_tmp[ADR_LSB+BUFFER_WIDTH-1:ADR_LSB];
    mask                     = mask_i;
    op                       = READ;
    cycle_type               = cycle_type_i;
    burst_type               = burst_type_i;
    burst_length             = burst_length_i;
    index                    = 0;
    err_o                    = 0;

    init;

    while(index < burst_length) begin
      next;
      data_compare(addr, data, index);
      addr                   = wb_next_adr(addr, cycle_type, burst_type, dw);
      buffer_addr_tmp        = addr - base_addr;
      buffer_addr            = buffer_addr_tmp[ADR_LSB+BUFFER_WIDTH-1:ADR_LSB];
      index                  = index + 1;
    end

    insert_wait_states;
  end
  endtask

  task data_compare;
    input [aw-1:0]           addr;
    input [dw-1:0]           read_data;
    input [31:0]             iteration;

    begin
      
      if (VERBOSE>2) begin
        $display("    %t: Comparing Read Data for iteration %0d at address: %h", $time, iteration, addr);
        $display("    %t: Read Data: %h, buffer data: %h, buffer address: %h", $time, read_data, buffer_data[buffer_addr], buffer_addr);
      end else if (VERBOSE>1) begin
        $display("    Comparing Read Data for iteration %0d at address: %h", iteration, addr);
      end

      if(buffer_data[buffer_addr] !== read_data) begin
        $error("Read data mismatch during iteration %0d at address %h", iteration, addr);
        $error("Expected %h", buffer_data[buffer_addr]);
        $error("Got      %h", read_data);
        #3 $finish;
      end else begin
        if (VERBOSE>1) $display("    Data Matched");
      end
    end
  endtask

  task insert_wait_states;
    begin

      wb_cyc_o               = #Tp 1'b0;
      wb_stb_o               = #Tp 1'b0;
      wb_we_o                = #Tp 1'b0;
      wb_cti_o               = #Tp 3'b000;
      wb_bte_o               = #Tp 2'b00;
      wb_sel_o               = #Tp {dw/8{1'b0}};
      wb_adr_o               = #Tp {aw{1'b0}};
      wb_dat_o               = #Tp {dw{1'b0}};

      for (wait_states_cnt = 0 ; wait_states_cnt < wait_states; wait_states_cnt = wait_states_cnt + 1) begin
        @(posedge wb_clk_i);
      end
    end
  endtask

  task clear_write_data;
    begin
      for(word = 0; word < MAX_BURST_LEN-1; word = word + 1) begin
        write_data[word]     = {dw{1'bx}};
      end
    end
  endtask

  task clear_buffer_data;
    begin
      for(word = 0; word < MAX_BURST_LEN-1; word = word + 1) begin
        buffer_data[word]    = 32'hxxxxxxxx;
      end
    end
  endtask

  //Low level tasks
  task init;
    begin
      if(wb_rst_i !== 1'b0) begin
        @(negedge wb_rst_i);
        @(posedge wb_clk_i);
      end

      wb_sel_o               <= #Tp mask;
      wb_we_o                <= #Tp op;
      wb_cyc_o               <= #Tp 1'b1;

      if(cycle_type == CTI_CLASSIC) begin
        if (VERBOSE > 1) $display("INIT: Classic Cycle");
        wb_cti_o             <= #Tp 3'b000;
        wb_bte_o             <= #Tp 2'b00;
      end else if(index == burst_length-1) begin
        if (VERBOSE > 1) $display("INIT: Burst - last cycle");
        wb_cti_o             <= #Tp 3'b111;
        wb_bte_o             <= #Tp 2'b00;
      end else if(cycle_type == CTI_CONST_BURST) begin
        if (VERBOSE > 1) $display("INIT: Const Burst cycle");
        wb_cti_o             <= #Tp 3'b001;
        wb_bte_o             <= #Tp 2'b00;
      end else begin
        if (VERBOSE > 1) $display("INIT: Incr Burst cycle");
        wb_cti_o             <=# Tp 3'b010;
        wb_bte_o             <=# Tp burst_type[1:0];
      end
    end
  endtask

  task next;
    begin
      wb_adr_o               <= #Tp addr;
      wb_dat_o               <= #Tp (op === WRITE) ? data : {dw{1'b0}};
      wb_stb_o               <= #Tp 1'b1; //FIXME: Add wait states

      if(cycle_type == CTI_CLASSIC) begin
        while (wb_ack_i !== 1'b1)
           @(posedge wb_clk_i);
        data                 = wb_dat_i;
        wb_stb_o             <= #Tp 1'b0;
        @(negedge wb_ack_i);
      end else begin
        if(index == burst_length-1)
          wb_cti_o           <= #Tp 3'b111;

        @(posedge wb_clk_i);
        while(wb_ack_i !== 1'b1)
          @(posedge wb_clk_i);
        data                 = wb_dat_i;
      end
    end
  endtask // while
endmodule
