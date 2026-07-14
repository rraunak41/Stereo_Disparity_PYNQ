`timescale 1ns/1ps

module gray_tb;

reg aclk   = 0;
reg aresetn = 0;
always #3.52 aclk = ~aclk;  

reg  [23:0] s_tdata  = 0;
reg         s_tvalid = 0;
reg         s_tuser  = 0;
reg         s_tlast  = 0;
reg  [2:0]  s_tkeep  = 3'b111;
reg  [2:0]  s_tstrb  = 3'b111;
wire        s_tready;

wire [23:0] m_tdata;
wire        m_tvalid;
wire        m_tuser;
wire        m_tlast;
wire [2:0]  m_tkeep;
wire [2:0]  m_tstrb;
reg         m_tready = 1;   

gray dut (
    .aclk          (aclk),
    .aresetn       (aresetn),
    .s_axis_tdata  (s_tdata),
    .s_axis_tvalid (s_tvalid),
    .s_axis_tuser  (s_tuser),
    .s_axis_tlast  (s_tlast),
    .s_axis_tkeep  (s_tkeep),
    .s_axis_tstrb  (s_tstrb),
    .s_axis_tready (s_tready),
    .m_axis_tdata  (m_tdata),
    .m_axis_tvalid (m_tvalid),
    .m_axis_tuser  (m_tuser),
    .m_axis_tlast  (m_tlast),
    .m_axis_tkeep  (m_tkeep),
    .m_axis_tstrb  (m_tstrb),
    .m_axis_tready (m_tready)
);

integer pass_count = 0;
integer fail_count = 0;

task send_pixel;
    input [7:0]  r, g, b;
    input        tuser, tlast;
    input [7:0]  expected_gray;
  input [63:0] test_name;  // not used in display but kept it for clarity
    reg   [15:0] gray_calc;
    reg   [7:0]  expected;
begin
 
    gray_calc = (29 * b) + (150 * g) + (77 * r);
    expected  = gray_calc[15:8];

   
    @(posedge aclk);
    s_tdata  <= {r, g, b};   
    s_tvalid <= 1;
    s_tuser  <= tuser;
    s_tlast  <= tlast;
  
    @(posedge aclk);
    #1; 

    if (m_tvalid !== 1'b1) begin
        $display("FAIL: m_tvalid not asserted for R=%0d G=%0d B=%0d",
                 r, g, b);
        fail_count = fail_count + 1;
    end
    else if (m_tdata !== {expected, expected, expected}) begin
        $display("FAIL: R=%0d G=%0d B=%0d | expected gray=%0d got=%0d",
                 r, g, b, expected, m_tdata[7:0]);
        fail_count = fail_count + 1;
    end
    else begin
        $display("PASS: R=%0d G=%0d B=%0d | gray=%0d (tuser=%b tlast=%b)",
                 r, g, b, m_tdata[7:0], m_tuser, m_tlast);
        pass_count = pass_count + 1;
    end

    if (m_tuser !== tuser || m_tlast !== tlast) begin
        $display("FAIL: Sideband mismatch: tuser got=%b exp=%b  tlast got=%b exp=%b",
                 m_tuser, tuser, m_tlast, tlast);
        fail_count = fail_count + 1;
    end
end
endtask

initial begin
    $display("================================================");
    $display(" gray.v Testbench");
    $display(" Clock: 142 MHz");
    $display("================================================");

    aresetn = 0;
    repeat(5) @(posedge aclk);
    aresetn = 1;
    @(posedge aclk);

    $display("\n--- Basic Color Tests ---");

    send_pixel(255, 0, 0, 0, 0, 8'd76, "Pure Red  ");
   
    send_pixel(0, 255, 0, 0, 0, 8'd149, "Pure Green");
   
    send_pixel(0, 0, 255, 0, 0, 8'd28, "Pure Blue ");
   
    send_pixel(255, 255, 255, 0, 0, 8'd255, "White     ");
    
    send_pixel(0, 0, 0, 0, 0, 8'd0, "Black     ");
    
    send_pixel(128, 128, 128, 0, 0, 8'd128, "Mid-gray  ");
  
    $display("\n--- Sideband Signal Tests ---");

  
    send_pixel(100, 150, 200, 1, 0, 8'd0, "SOF pixel ");


    send_pixel(50, 100, 150, 0, 1, 8'd0, "EOL pixel ");

    send_pixel(200, 100, 50, 1, 1, 8'd0, "SOF+EOL   ");

    $display("\n--- Backpressure Test ---");

    @(posedge aclk);
    s_tdata  <= 24'hFF0000;   
    s_tvalid <= 1;
    m_tready <= 0;            
    @(posedge aclk);
    #1;
    begin
        $display("BACKPRESSURE: s_tready=%b (should be 0 when m_tready=0)",
                 s_tready);
        if (s_tready !== 1'b0)
            $display("WARN: s_tready should be 0 during backpressure");
        else
            $display("PASS: Backpressure correctly propagated upstream");
    end

    m_tready <= 1;   
    @(posedge aclk);

    $display("\n--- Continuous Stream Test ---");

    begin : stream_test
        integer i;
        reg [7:0] r_val, g_val, b_val;
        reg [15:0] gc;
        reg [7:0]  exp;
        for (i = 0; i < 8; i = i + 1) begin
            r_val = i * 30;
            g_val = i * 20;
            b_val = i * 10;
            @(posedge aclk);
            s_tdata  <= {r_val, g_val, b_val};
            s_tvalid <= 1;
            s_tuser  <= (i == 0);
            s_tlast  <= (i == 7);
        end
  
        @(posedge aclk);
        s_tvalid <= 0;
        @(posedge aclk);
        $display("PASS: Continuous 8-pixel stream completed");
        pass_count = pass_count + 1;
    end

   
    repeat(3) @(posedge aclk);
    $display("\n================================================");
    $display(" RESULTS: %0d PASSED, %0d FAILED",
             pass_count, fail_count);
    if (fail_count == 0)
        $display(" ALL TESTS PASSED");
    else
        $display(" SOME TESTS FAILED");
    $display("================================================");

    $finish;
end

initial begin
    #100000;
    $display("TIMEOUT: Simulation ran too long");
    $finish;
end
initial begin
    $dumpfile("gray_tb.vcd");
    $dumpvars(0, gray_tb);
end

endmodule
