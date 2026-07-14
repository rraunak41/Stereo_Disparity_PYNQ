module gray (
    
    input  wire        aclk,       
    input  wire        aresetn,    

    
    input  wire [23:0] s_axis_tdata,   // BGR pixel: [23:16]=R [15:8]=G [7:0]=B
    input  wire        s_axis_tvalid,  // upstream data valid
    input  wire        s_axis_tuser,   // start-of-frame (SOF)
    input  wire        s_axis_tlast,   // end-of-line (EOL)
    input  wire [2:0]  s_axis_tkeep,   // byte enable (3 bytes)
    input  wire [2:0]  s_axis_tstrb,   // byte strobe
    output wire        s_axis_tready,  // backpressure to upstream

    
    output reg  [23:0] m_axis_tdata,   // gray replicated: {gray,gray,gray}
    output reg         m_axis_tvalid,  // output valid
    output reg         m_axis_tuser,   // SOF passthrough
    output reg         m_axis_tlast,   // EOL passthrough
    output reg  [2:0]  m_axis_tkeep,   // TKEEP passthrough
    output reg  [2:0]  m_axis_tstrb,   // TSTRB passthrough
    input  wire        m_axis_tready   // downstream backpressure
);


assign s_axis_tready = m_axis_tready;

wire [7:0] b_in = s_axis_tdata[7:0];
wire [7:0] g_in = s_axis_tdata[15:8];
wire [7:0] r_in = s_axis_tdata[23:16];

wire [15:0] b_weighted = 8'd29  * b_in;   // 16-bit product
wire [15:0] g_weighted = 8'd150 * g_in;  
wire [15:0] r_weighted = 8'd77  * r_in;  

wire [15:0] gray_sum   = b_weighted + g_weighted + r_weighted;
wire [7:0]  gray_val   = gray_sum[15:8]; 

always @(posedge aclk) begin
    if (!aresetn) begin
        
        m_axis_tvalid <= 1'b0;
        m_axis_tdata  <= 24'h000000;
        m_axis_tuser  <= 1'b0;
        m_axis_tlast  <= 1'b0;
        m_axis_tkeep  <= 3'b000;
        m_axis_tstrb  <= 3'b000;
    end
    else begin
        m_axis_tvalid <= s_axis_tvalid;

        if (s_axis_tvalid) begin
            m_axis_tdata  <= {gray_val, gray_val, gray_val};
            m_axis_tuser  <= s_axis_tuser;
            m_axis_tlast  <= s_axis_tlast;
            m_axis_tkeep  <= s_axis_tkeep;
            m_axis_tstrb  <= s_axis_tstrb;
        end
    end
end

endmodule
