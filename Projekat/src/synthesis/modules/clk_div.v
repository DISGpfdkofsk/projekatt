module clk_div #(parameter DIVISOR = 50_000_000) (clk, rst_n, out);

input clk, rst_n;
output reg out;
reg [27:0] timer = 28'd0;

always @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        timer <= 28'd0;
        out <= 1'b0;
    end else begin
        timer <= timer + 28'd1;
        if (timer >= (DIVISOR - 1))
            timer <= 28'd0;
        out <= (timer<DIVISOR/2)?1'b1:1'b0;
    end
end

endmodule