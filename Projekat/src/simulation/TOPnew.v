module TOPnew;

  // Define parameters
  parameter ADDR_WIDTH = 6;
  parameter DATA_WIDTH = 16;

  // Inputs
  reg clk, rst_n;
  reg [DATA_WIDTH-1:0] mem_in, in;

  // Outputs
  wire [DATA_WIDTH-1:0] mem_data, out;
  wire [ADDR_WIDTH-1:0] pc, sp;
  wire mem_we;
  wire [ADDR_WIDTH-1:0] mem_addr;

  // Instantiate the CPU module
  cpu #(ADDR_WIDTH, DATA_WIDTH) uut (
    .clk(clk),
    .rst_n(rst_n),
    .mem_in(mem_in),
    .in(in),
    .mem_we(mem_we),
    .mem_addr(mem_addr),
    .mem_data(mem_data),
    .out(out),
    .pc(pc),
    .sp(sp)
  );

  initial begin
    #3
    forever #8 clk = ~clk;
  end

  initial begin
    clk = 1'b0;
    rst_n = 1'b1;
    #1 rst_n = 1'b0;
    #2 rst_n = 1'b1;
    #11 $monitor("%2d, %d, %b", $time, clk, pc);
    #50;
    $finish; 
  end

endmodule