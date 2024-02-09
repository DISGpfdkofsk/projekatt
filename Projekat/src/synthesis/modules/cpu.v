module cpu #(parameter ADDR_WIDTH = 6, parameter DATA_WIDTH = 16)
(
    clk, rst_n, mem_in, in, mem_we, mem_addr, mem_data, out, pc, sp
);

input clk, rst_n;
input [DATA_WIDTH-1:0] mem_in, in;
output reg mem_we;
output reg [ADDR_WIDTH-1:0] mem_addr;
output reg [DATA_WIDTH-1:0] mem_data;
output [DATA_WIDTH-1:0] out;
output [ADDR_WIDTH-1:0] pc, sp;


//Register instantiation

reg cl_pc, ld_pc, inc_pc, dec_pc, sr_pc, ir_pc, sl_pc, il_pc;
reg [ADDR_WIDTH-1:0] in_pc;
register #(ADDR_WIDTH) programCounter(
    .clk(clk),
    .rst_n(rst_n),
    .cl(cl_pc),
    .ld(ld_pc),
    .in(in_pc),
    .inc(inc_pc),
    .dec(dec_pc),
    .sr(sr_pc),
    .ir(ir_pc),
    .sl(sl_pc),
    .il(il_pc),
    .out(pc)
);

reg cl_sp, ld_sp, inc_sp, dec_sp, sr_sp, ir_sp, sl_sp, il_sp;
reg [ADDR_WIDTH-1:0] in_sp;
register #(ADDR_WIDTH) stackPointer(
    .clk(clk), 
    .rst_n(rst_n), 
    .cl(cl_sp), 
    .ld(ld_sp), 
    .in(in_sp), 
    .inc(inc_sp), 
    .dec(dec_sp), 
    .sr(sr_sp), 
    .ir(ir_sp), 
    .sl(sl_sp), 
    .il(il_sp),
    .out(sp)
);

reg cl_ir_low, ld_ir_low, inc_ir_low, dec_ir_low, sr_ir_low, ir_ir_low, sl_ir_low, il_ir_low;
reg [DATA_WIDTH-1:0] in_ir_low;
wire [DATA_WIDTH-1:0] out_ir_low;
register #(DATA_WIDTH) instructionRegisterLow(
    .clk(clk), 
    .rst_n(rst_n), 
    .cl(cl_ir_low), 
    .ld(ld_ir_low), 
    .in(in_ir_low),
    .inc(inc_ir_low), 
    .dec(dec_ir_low), 
    .sr(sr_ir_low), 
    .ir(ir_ir_low), 
    .sl(sl_ir_low), 
    .il(il_ir_low), 
    .out(out_ir_low)
);

reg cl_ir_high, ld_ir_high, inc_ir_high, dec_ir_high, sr_ir_high, ir_ir_high, sl_ir_high, il_ir_high;
reg [DATA_WIDTH-1:0] in_ir_high;
wire [DATA_WIDTH-1:0] out_ir_high;
register #(DATA_WIDTH) instructionRegisterHigh(
    .clk(clk), 
    .rst_n(rst_n), 
    .cl(cl_ir_high), 
    .ld(ld_ir_high), 
    .in(in_ir_high),
    .inc(inc_ir_high), 
    .dec(dec_ir_high), 
    .sr(sr_ir_high), 
    .ir(ir_ir_high), 
    .sl(sl_ir_high), 
    .il(il_ir_high), 
    .out(out_ir_high)
);

reg cl_ac, ld_ac, inc_ac, dec_ac, sr_ac, ir_ac, sl_ac, il_ac;
reg [DATA_WIDTH-1:0] in_ac;
wire [DATA_WIDTH-1:0] out_ac;
register #(DATA_WIDTH) accumulator(
    .clk(clk), 
    .rst_n(rst_n), 
    .cl(cl_ac), 
    .ld(ld_ac), 
    .in(in_ac), 
    .inc(inc_ac), 
    .dec(dec_ac), 
    .sr(sr_ac), 
    .ir(ir_ac), 
    .sl(sl_ac), 
    .il(il_ac), 
    .out(out_ac)
);


//Additional declarations

localparam start = 4'b0000;
localparam readInstruction = 4'b0001;
localparam readInstructionExtra = 4'b0010;
localparam decodeInstruction = 4'b0011;
localparam executeDIV = 4'b0100;
localparam executeMOV = 4'b0101;
localparam executeIN = 4'b0110;
localparam executeOUT = 4'b0111;
localparam executeADD = 4'b1000;
localparam executeSUB = 4'b1001;
localparam executeMUL = 4'b1010;
localparam executeSTOP = 4'b1011;

reg [3:0] state_reg, state_next;

reg [1:0] cnt_reg, cnt_next;

reg [DATA_WIDTH-1:0] out_reg, out_next;
assign out = out_reg;


//Sequential logic

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        state_reg <= 4'b0000;
        cnt_reg <= 2'b00;
        out_reg <= {DATA_WIDTH{1'b0}};
    end else begin
        state_reg <= state_next;
        cnt_reg <= cnt_next;
        out_reg <= out_next;
    end
end


//Combinatorial logic

always @(*) begin
    state_next = state_reg;
    cnt_next = cnt_reg;
    out_next = out_reg;

    cl_pc = 1'b0;
    ld_pc = 1'b0;
    in_pc = {ADDR_WIDTH{1'b0}};
    inc_pc = 1'b0;
    dec_pc = 1'b0;
    sr_pc = 1'b0;
    ir_pc = 1'b0;
    sl_pc = 1'b0;
    il_pc = 1'b0;

    cl_sp = 1'b0;
    ld_sp = 1'b0;
    in_sp = {ADDR_WIDTH{1'b0}};
    inc_sp = 1'b0;
    dec_sp = 1'b0;
    sr_sp = 1'b0;
    ir_sp = 1'b0;
    sl_sp = 1'b0;
    il_sp = 1'b0;

    cl_ir_high = 1'b0;
    ld_ir_high = 1'b0;
    in_ir_high = {DATA_WIDTH{1'b0}};
    inc_ir_high = 1'b0;
    dec_ir_high = 1'b0;
    sr_ir_high = 1'b0;
    ir_ir_high = 1'b0;
    sl_ir_high = 1'b0;
    il_ir_high = 1'b0;

    cl_ir_low = 1'b0;
    ld_ir_low = 1'b0;
    in_ir_low = {DATA_WIDTH{1'b0}};
    inc_ir_low = 1'b0;
    dec_ir_low = 1'b0;
    sr_ir_low = 1'b0;
    ir_ir_low = 1'b0;
    sl_ir_low = 1'b0;
    il_ir_low = 1'b0;

    cl_ac = 1'b0;
    ld_ac = 1'b0;
    in_ac = {DATA_WIDTH{1'b0}};
    inc_ac = 1'b0;
    dec_ac = 1'b0;
    sr_ac = 1'b0;
    ir_ac = 1'b0;
    sl_ac = 1'b0;
    il_ac = 1'b0;

    mem_we = 1'b0;
    mem_addr = {ADDR_WIDTH{1'b0}};
    mem_data = {DATA_WIDTH{1'b0}};

    case (state_reg)

        start: begin
            if (pc < {{ADDR_WIDTH-4{1'b0}}, 4'd8}) begin
                ld_pc = 1'b1;
                in_pc = {{ADDR_WIDTH-4{1'b0}}, 4'd8};
                state_next = start;
            end else begin
                state_next = readInstruction;
            end
        end

        readInstruction: begin
            if (cnt_reg == 2'b00) begin
                mem_addr = pc;
                cnt_next = 2'b01;
                state_next = readInstruction;
            end else if (cnt_reg == 2'b01) begin
                ld_ir_high = 1'b1;
                in_ir_high = mem_in;
                cnt_next = 2'b10;
                state_next = readInstruction;
            end else begin
                cnt_next = 2'b00;
                ld_pc = 1'b1;
                inc_pc = 1'b1;
                //MOV 2B
                if ({out_ir_high[DATA_WIDTH-1], out_ir_high[DATA_WIDTH-2], 
                    out_ir_high[DATA_WIDTH-3], out_ir_high[DATA_WIDTH-4],
                    out_ir_high[DATA_WIDTH-13], out_ir_high[DATA_WIDTH-14],
                    out_ir_high[DATA_WIDTH-15], out_ir_high[DATA_WIDTH-16]} == 8'b00001000) begin 
                    state_next = readInstructionExtra;
                end
                else begin
                    state_next = decodeInstruction;
                end
            end
        end
        readInstructionExtra: begin
            if (cnt_reg == 2'b00) begin
                mem_addr = pc;
                cnt_next = 2'b01;
                state_next = readInstructionExtra;
            end else if (cnt_reg == 2'b01) begin
                ld_ir_low = 1'b1;
                in_ir_low = mem_in;
                cnt_next = 2'b10;
                state_next = readInstructionExtra;
            end else begin
                cnt_next = 2'b00;
                ld_pc = 1'b1;
                inc_pc = 1'b1;
                state_next = decodeInstruction;
            end
        end

        decodeInstruction: begin
            if (out_ir_high[15:12] == 4'b0000) begin
                state_next = executeMOV;
            end else if (out_ir_high[15:12] == 4'b0001) begin
                state_next = executeADD;
            end else if (out_ir_high[15:12] == 4'b0010) begin
                state_next = executeSUB;
            end else if (out_ir_high[15:12] == 4'b0011) begin
                state_next = executeMUL;
            end else if (out_ir_high[15:12] == 4'b0100) begin
                state_next = executeDIV;
            end else if (out_ir_high[15:12] == 4'b0111) begin
                state_next = executeIN;
            end else if (out_ir_high[15:12] == 4'b1000) begin
                state_next = executeOUT;
            end else if (out_ir_high[15:12] == 4'b1111) begin
                state_next = executeSTOP;
            end else state_next = executeDIV;
        end

        executeIN: begin
            mem_we = 1;
            mem_addr = {{ADDR_WIDTH - 3{1'b0}}, out_ir_high[10:8]};
            mem_data = in;
            state_next = start;
        end
        executeOUT: begin
            if (cnt_reg == 2'b00) begin
                mem_addr = {{ADDR_WIDTH - 3{1'b0}}, out_ir_high[10:8]};
                cnt_next = 2'b01;
                state_next = executeOUT;
            end else if (cnt_reg == 2'b01) begin
                out_next = mem_in;
                cnt_next = 2'b00;
                state_next = start;
            end
        end

    endcase
end

endmodule