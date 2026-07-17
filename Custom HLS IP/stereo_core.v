module stereo_core #(
    parameter HALF_WIDTH = 640,
    parameter MAX_DISP   = 48,
    parameter AGG_W      = 5,
    parameter AGG_HALF_W = 2
)(
    input  wire        aclk,
    input  wire        aresetn,

    // AXI4-Stream Slave
    input  wire [23:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    input  wire        s_axis_tuser,
    input  wire        s_axis_tlast,
    input  wire [2:0]  s_axis_tkeep,
    input  wire [2:0]  s_axis_tstrb,
    output wire        s_axis_tready,

    // AXI4-Stream Master
    output reg  [23:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    output reg         m_axis_tuser,
    output reg         m_axis_tlast,
    output reg  [2:0]  m_axis_tkeep,
    output reg  [2:0]  m_axis_tstrb,
    input  wire        m_axis_tready
);

// ================================================================
// BACKPRESSURE
// ================================================================
assign s_axis_tready = m_axis_tready;

wire [23:0] desc  = s_axis_tdata;
wire        fire  = s_axis_tvalid && s_axis_tready;

// ================================================================
// COLUMN / ROW COUNTERS
// ================================================================
reg [10:0] x;   // 0..1279
reg [9:0]  y;   // 0..479

always @(posedge aclk) begin
    if (!aresetn) begin
        x <= 0; y <= 0;
    end else if (fire) begin
        if (s_axis_tuser) begin
            x <= 0; y <= 0;
        end else if (s_axis_tlast) begin
            x <= 0; y <= y + 1;
        end else begin
            x <= x + 1;
        end
    end
end

reg [23:0] left_desc [0:HALF_WIDTH-1];

// Write port: store left Census descriptor
always @(posedge aclk) begin
    if (fire && (x < HALF_WIDTH)) begin
        left_desc[x] <= desc;
    end
end


function automatic [4:0] popcount24;
    input [23:0] v;
    reg [1:0] s0[0:11];
    reg [2:0] s1[0:5];
    reg [3:0] s2[0:2];
    integer   fi;
    begin
        for (fi = 0; fi < 12; fi = fi + 1)
            s0[fi] = v[2*fi] + v[2*fi+1];
        for (fi = 0; fi < 6; fi = fi + 1)
            s1[fi] = s0[2*fi] + s0[2*fi+1];
        for (fi = 0; fi < 3; fi = fi + 1)
            s2[fi] = s1[2*fi] + s1[2*fi+1];
        popcount24 = s2[0] + s2[1] + s2[2];
    end
endfunction

reg [6:0] ch [0:MAX_DISP-1][0:AGG_W-1];  // cost history [disp][age]

wire [10:0] xr = x - HALF_WIDTH;   // right-half column index

// Per-disparity: pixel Hamming cost and aggregated cost
wire [6:0]  pixel_cost [0:MAX_DISP-1];
wire [6:0]  agg_cost   [0:MAX_DISP-1];

genvar d, c;
generate
    for (d = 0; d < MAX_DISP; d = d + 1) begin : disp_loop

        // Read left descriptor at xr-d (guard against underflow)
        wire [23:0] left_d = (xr >= d) ? left_desc[xr - d] : 24'd0;

        // Hamming distance: popcount(left XOR right)
        assign pixel_cost[d] = (xr >= d)
                                ? popcount24(left_d ^ desc)
                                : 7'd24;  // max cost for out-of-range


        assign agg_cost[d] = ch[d][1] + ch[d][2] + ch[d][3]
                           + ch[d][4] + pixel_cost[d];

    end
endgenerate


reg [5:0] best_disp_comb;
reg [6:0] best_cost_comb;
reg [6:0] second_best_comb;

integer di;
always @(*) begin
    best_disp_comb   = 6'd0;
    best_cost_comb   = 7'd127;  // above max possible (24*5=120)
    second_best_comb = 7'd127;

    for (di = 0; di < MAX_DISP; di = di + 1) begin
        if (agg_cost[di] < best_cost_comb) begin
            second_best_comb = best_cost_comb;
            best_cost_comb   = agg_cost[di];
            best_disp_comb   = di;
        end else if (agg_cost[di] < second_best_comb) begin
            second_best_comb = agg_cost[di];
        end
    end
end


wire in_valid_zone = (xr >= (MAX_DISP + AGG_W - 1)) &&
                     (xr <  (HALF_WIDTH - AGG_HALF_W)) &&
                     (y  >= 1);

wire unique = (second_best_comb > (best_cost_comb + (best_cost_comb >> 2)));


wire [7:0] disparity_scaled;


function automatic [7:0] scale_disp;
    input [5:0] d;
    begin
        case (d)
             6'd0:  scale_disp = 8'd0;
             6'd1:  scale_disp = 8'd5;
             6'd2:  scale_disp = 8'd11;
             6'd3:  scale_disp = 8'd16;
             6'd4:  scale_disp = 8'd22;
             6'd5:  scale_disp = 8'd27;
             6'd6:  scale_disp = 8'd33;
             6'd7:  scale_disp = 8'd38;
             6'd8:  scale_disp = 8'd43;
             6'd9:  scale_disp = 8'd49;
             6'd10: scale_disp = 8'd54;
             6'd11: scale_disp = 8'd60;
             6'd12: scale_disp = 8'd65;
             6'd13: scale_disp = 8'd70;
             6'd14: scale_disp = 8'd76;
             6'd15: scale_disp = 8'd81;
             6'd16: scale_disp = 8'd87;
             6'd17: scale_disp = 8'd92;
             6'd18: scale_disp = 8'd97;
             6'd19: scale_disp = 8'd103;
             6'd20: scale_disp = 8'd108;
             6'd21: scale_disp = 8'd114;
             6'd22: scale_disp = 8'd119;
             6'd23: scale_disp = 8'd124;
             6'd24: scale_disp = 8'd130;
             6'd25: scale_disp = 8'd135;
             6'd26: scale_disp = 8'd141;
             6'd27: scale_disp = 8'd146;
             6'd28: scale_disp = 8'd152;
             6'd29: scale_disp = 8'd157;
             6'd30: scale_disp = 8'd162;
             6'd31: scale_disp = 8'd168;
             6'd32: scale_disp = 8'd173;
             6'd33: scale_disp = 8'd179;
             6'd34: scale_disp = 8'd184;
             6'd35: scale_disp = 8'd189;
             6'd36: scale_disp = 8'd195;
             6'd37: scale_disp = 8'd200;
             6'd38: scale_disp = 8'd206;
             6'd39: scale_disp = 8'd211;
             6'd40: scale_disp = 8'd216;
             6'd41: scale_disp = 8'd222;
             6'd42: scale_disp = 8'd227;
             6'd43: scale_disp = 8'd233;
             6'd44: scale_disp = 8'd238;
             6'd45: scale_disp = 8'd243;
             6'd46: scale_disp = 8'd249;
             6'd47: scale_disp = 8'd255;
             default: scale_disp = 8'd0;
        endcase
    end
endfunction

wire [7:0] disp_out_val = (x >= HALF_WIDTH && in_valid_zone && unique)
                           ? scale_disp(best_disp_comb)
                           : 8'd0;

wire [23:0] right_output = {disp_out_val, disp_out_val, disp_out_val};

// ================================================================
// COST HISTORY UPDATE + OUTPUT REGISTER
// On every valid right-half pixel:
//   shift cost history left, insert new pixel cost at tail
//   register final output
// On TUSER (SOF): initialize all cost history to 24 (max single cost)
// ================================================================
always @(posedge aclk) begin
    if (!aresetn) begin
        m_axis_tvalid <= 0;
        // Reset cost history
        begin : reset_ch
            integer ri, ci;
            for (ri = 0; ri < MAX_DISP; ri = ri + 1)
                for (ci = 0; ci < AGG_W; ci = ci + 1)
                    ch[ri][ci] <= 7'd24;
        end
    end
    else if (fire) begin
        // ── SOF: reset cost history and counters ─────────────────
        if (s_axis_tuser) begin
            begin : sof_reset
                integer sri, sci;
                for (sri = 0; sri < MAX_DISP; sri = sri + 1)
                    for (sci = 0; sci < AGG_W; sci = sci + 1)
                        ch[sri][sci] <= 7'd24;
            end
        end

        // ── Right half: update cost history ──────────────────────
        if (x >= HALF_WIDTH) begin
            begin : shift_ch
                integer shi;
                for (shi = 0; shi < MAX_DISP; shi = shi + 1) begin
                    ch[shi][0] <= ch[shi][1];
                    ch[shi][1] <= ch[shi][2];
                    ch[shi][2] <= ch[shi][3];
                    ch[shi][3] <= ch[shi][4];
                    ch[shi][4] <= pixel_cost[shi];
                end
            end
        end

        // ── Register output ───────────────────────────────────────
        m_axis_tvalid <= 1;
        m_axis_tuser  <= s_axis_tuser;
        m_axis_tlast  <= s_axis_tlast;
        m_axis_tkeep  <= s_axis_tkeep;
        m_axis_tstrb  <= s_axis_tstrb;

        if (x < HALF_WIDTH) begin
            // Left half: pass Census texture through unchanged
            m_axis_tdata <= desc;
        end else begin
            // Right half: output scaled disparity
            m_axis_tdata <= right_output;
        end
    end
    else begin
        m_axis_tvalid <= 0;
    end
end

endmodule
