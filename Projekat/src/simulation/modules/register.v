module register(clk, rst_n, cl, ld, in, inc, dec, sr, ir, sl, il, out);

input clk, rst_n, cl, ld, inc, dec, sr, ir, sl, il;
input [3:0] in;
output [3:0] out;

reg [3:0] out_reg, out_next;
assign out = out_reg;

always @(posedge clk, negedge rst_n)
    if (!rst_n)
        out_reg <= 4'b0000;
    else
        out_reg <= out_next;

always @(*) begin
    out_next = out_reg;
    if (cl)
        out_next = 4'b0000;
    else if (ld)
        out_next = in;
    else if (inc)
        out_next = out_reg + 4'b0001;
    else if (dec)
        out_next = out_reg - 4'b0001;
    else if (sr)
        out_next = {ir, out_reg[3:1]};
    else if (sl)
        out_next = {out_reg[2:0], il};
end

endmodule