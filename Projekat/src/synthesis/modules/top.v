module top #(parameter DIVISOR = 50_000_000, FILE_NAME = "mem_init.mif", ADDR_WIDTH = 6, DATA_WIDTH = 16) 
(
    clk, rst_n, sw, led, hex
);

input clk, rst_n;
input [8:0] sw;
output [9:0] led;
output [27:0] hex;

wire clk_divided;
wire [ADDR_WIDTH-1:0] sp, pc;
wire [3:0] onesSP, tensSP, onesPC, tensPC;
wire we;
wire [ADDR_WIDTH-1:0] addr;
wire [DATA_WIDTH-1:0] data, out;
wire [DATA_WIDTH-6:0] throwaway;

clk_div #(DIVISOR) clk_div(.clk(clk), .rst_n(rst_n), .out(clk_divided));

memory #("mem_init.mif", ADDR_WIDTH, DATA_WIDTH) memory(.clk(clk_divided), .we(we), .addr(addr), .data(data), .out(out));

cpu #(ADDR_WIDTH, DATA_WIDTH) cpu(.clk(clk_divided), .rst_n(rst_n), .mem_in(out),
    .in({{DATA_WIDTH-4{1'b0}},sw[3:0]}), .mem_we(we), .mem_addr(addr), .mem_data(data), .out({throwaway[10:0], led[4:0]}), .pc(pc), .sp(sp));

bcd bcd1(.in(sp), .ones(onesSP), .tens(tensSP));
bcd bcd2(.in(pc), .ones(onesPC), .tens(tensPC));

ssd ssd11(.in(tensSP), .out(hex[27:21]));
ssd ssd12(.in(onesSP), .out(hex[20:14]));
ssd ssd21(.in(tensPC), .out(hex[13:7]));
ssd ssd22(.in(onesPC), .out(hex[6:0]));

endmodule