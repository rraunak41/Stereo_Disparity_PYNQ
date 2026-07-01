

#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;
typedef ap_uint<24>        descriptor_t;


#define HALF_WIDTH  640     // pixels per eye (total width = 1280)
#define MAX_DISP     48     // disparity search range

#define AGG_W         5     // horizontal aggregation window (must be odd)
#define AGG_HALF_W    2     // AGG_W / 2


typedef ap_uint<7> cost_t;


static cost_t popcount24(ap_uint<24> x)
{
#pragma HLS INLINE
    ap_uint<5> cnt = 0;
    for (int i = 0; i < 24; i++) {
#pragma HLS UNROLL
        cnt += x[i];
    }
    return cnt;
}


void stereo_core(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{

#pragma HLS INTERFACE axis       port=stream_in
#pragma HLS INTERFACE axis       port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return


    static descriptor_t left_desc[HALF_WIDTH];
#pragma HLS BIND_STORAGE variable=left_desc type=ram_2p impl=bram


    static cost_t col_cost_hist[MAX_DISP][AGG_W];
#pragma HLS ARRAY_PARTITION variable=col_cost_hist type=complete dim=0

    static ap_uint<11> x = 0;    // 0..1279 (needs 11 bits)
    static ap_uint<10> y = 0;    // 0..479  (needs 10 bits)

    while (1) {
#pragma HLS PIPELINE II=1


        video_pixel p;
        stream_in.read(p);

        // Census descriptor coming from census_5x5 IP
        descriptor_t desc = p.data;

        video_pixel out = p;    // preserve TUSER/TLAST/TKEEP/TSTRB

        if (p.user)
        {
            x = 0;
            y = 0;

            // Initialize aggregation history
            for(int d = 0; d < MAX_DISP; d++)
            {
        #pragma HLS UNROLL
                for(int c = 0; c < AGG_W; c++)
                {
        #pragma HLS UNROLL
                    col_cost_hist[d][c] = 24;
                }
            }
        }


        if (x < HALF_WIDTH) {
            left_desc[x] = desc;    // store for matching against Right
            out.data = p.data;      // pass Census texture through
        }


        else {
            ap_uint<11> xr = x - HALF_WIDTH;   // 0..639 within Right half

            ap_uint<5> best_disp   = 0;
            cost_t     best_cost   = (cost_t)(24 * AGG_W + 1); // above max possible
            cost_t     second_best = (cost_t)(24 * AGG_W + 1);


            // All MAX_DISP candidates evaluated simultaneously in one clock.
            for (int d = 0; d < MAX_DISP; d++) {
#pragma HLS UNROLL

                // Single-pixel Hamming cost at disparity d
                cost_t pixel_cost;
                if (xr >= (ap_uint<11>)d) {
                    pixel_cost = popcount24(left_desc[xr - d] ^ desc);
                } else {
                    pixel_cost = 24;
                }

                // Shift column-cost history, insert new cost at tail
                for (int c = 0; c < AGG_W - 1; c++) {
#pragma HLS UNROLL
                    col_cost_hist[d][c] = col_cost_hist[d][c + 1];
                }
                col_cost_hist[d][AGG_W - 1] = pixel_cost;

                // Horizontal box-sum over AGG_W columns = aggregated cost
                cost_t agg_cost = 0;
                for (int c = 0; c < AGG_W; c++) {
#pragma HLS UNROLL
                    agg_cost += col_cost_hist[d][c];
                }


                if (agg_cost < best_cost) {
                    second_best = best_cost;
                    best_cost   = agg_cost;
                    best_disp   = d;
                } else if (agg_cost < second_best) {
                    second_best = agg_cost;
                }
            }

            bool in_valid_zone =
                (xr >= (ap_uint<11>)(MAX_DISP + AGG_W - 1)) &&
                (xr <  (ap_uint<11>)(HALF_WIDTH - AGG_HALF_W)) &&
                (y  >= 1);


            // Uniqueness check (25% margin)
            bool unique =
                (second_best > (cost_t)(best_cost + (best_cost >> 2)));

            ap_uint<8> disparity_out = 0;

            if (in_valid_zone && unique)
            {
                // Scale disparity to full 0-255 grayscale
            	disparity_out = best_disp;
            }

            out.data =
                ((ap_uint<24>)disparity_out << 16) |
                ((ap_uint<24>)disparity_out << 8 ) |
                 disparity_out;
        }


        stream_out.write(out);


        if (p.last) { x = 0; y++; }
        else        { x = x + 1; }
    }
}
