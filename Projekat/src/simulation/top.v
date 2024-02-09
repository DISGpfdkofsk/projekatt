module top;

//Za testiranje ALU
reg [2:0] oc;
reg [3:0] a, b;
wire [3:0] f;
integer index;
alu alunit(.oc(oc), .a(a), .b(b), .f(f));

initial begin
    $monitor("Vreme = %0d, Operacioni kod = %b, Operand a = %d, Operand b = %d, Rezultat = %d", $time, oc, a, b, f);
end

//Za testiranje registra
reg rst_n;
reg clk;
reg cl;
reg ld;
reg [3:0] in;
reg inc;
reg dec;
reg sr;
reg ir;
reg sl;
reg il;
wire [3:0] out;
register r(.rst_n(rst_n), .clk(clk), .cl(cl), .ld(ld), .in(in), .inc(inc), .dec(dec), .sr(sr), .ir(ir), .sl(sl), .il(il), .out(out));

initial begin
    rst_n = 1'b0;
    clk = 1'b0;
    cl = 1'b0;
    ld = 1'b0;
    in = 4'b0000;
    inc = 1'b0;
    dec = 1'b0;
    sr = 1'b0;
    ir = 1'b0;
    sl = 1'b0;
    il = 1'b0;
    forever begin
        #5 clk = ~clk;
    end
end

always @(out) begin
    $strobe("Vreme = %0d, cl = %b, ld = %b, in = %b, inc = %b, dec = %b, sr = %b, ir = %b, sl = %b, il = %b, out = %b", $time, cl, ld, in,
            inc, dec, sr, ir, sl, il, out);
end


//Testiranje
initial begin
    for (index = 0; index < 1024; index = index + 1) begin
        {oc, a, b} = index;
        #5;
    end
    $stop;
    for (index = 1024; index < 2048; index = index + 1) begin
        {oc, a, b} = index;
        #5;
    end
    $stop;
    #7 rst_n = 1'b1;
    repeat(1000) begin
        #10;
        cl = $urandom % 2;
        ld = $urandom % 2;
        in = $urandom % 16;
        inc = $urandom % 2;
        dec = $urandom % 2;
        sr = $urandom % 2;
        ir = $urandom % 2;
        sl = $urandom % 2;
        il = $urandom % 2;
    end
    $finish;
end

endmodule

