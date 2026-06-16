#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define HALF_WIDTH 640
#define MAX_DISP   32   // reduced from 64 for LUT budget

static ap_uint<5> popcount24(ap_uint<24> x)
{
#pragma HLS INLINE
    ap_uint<5> cnt = 0;
    for(int i = 0; i < 24; i++) {
#pragma HLS UNROLL
        cnt += x[i];
    }
    return cnt;
}

void stereo_match(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    static ap_uint<24> left_desc[HALF_WIDTH];
#pragma HLS BIND_STORAGE variable=left_desc type=ram_2p impl=bram

    static ap_uint<24> match_window[MAX_DISP];
#pragma HLS ARRAY_PARTITION variable=match_window type=complete

    static ap_uint<11> x = 0;   // explicit width, avoids silent int truncation

    while(1) {
#pragma HLS PIPELINE II=1
        video_pixel p;
        stream_in.read(p);
        ap_uint<24> desc = p.data;
        video_pixel out = p;

        // Reset on start-of-frame only; row position resets on TLAST (end-of-line)
        if (p.user) {
            x = 0;
        }

        if (x < HALF_WIDTH) {
            // LEFT half: store descriptor, pass through unchanged
            left_desc[x] = desc;
            out.data = p.data;
        }
        else {
            ap_uint<11> xr = x - HALF_WIDTH;

            // Shift window: match_window[d] will hold left_desc[xr - d]
            for (int i = MAX_DISP - 1; i > 0; i--) {
#pragma HLS UNROLL
                match_window[i] = match_window[i-1];
            }
            match_window[0] = left_desc[xr];

            ap_uint<6> best_disp = 0;
            ap_uint<5> best_cost = 25; // > max possible Hamming dist (24)

            if (xr >= MAX_DISP) {
                for (int d = 0; d < MAX_DISP; d++) {
#pragma HLS UNROLL
                    ap_uint<24> left = match_window[d];
                    ap_uint<5> cost = popcount24(left ^ desc);
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_disp = d;
                    }
                }
            }

            // Scale 0-31 disparity to 0-248 for 8-bit visualization
            ap_uint<8> disparity = best_disp * 8;
            out.data = ((ap_uint<24>)disparity << 16) |
                       ((ap_uint<24>)disparity << 8)  |
                        disparity;
        }

        stream_out.write(out);

        // Row position: reset at end-of-line (TLAST), increment otherwise
        if (p.last)
            x = 0;
        else
            x = x + 1;
    }
}
