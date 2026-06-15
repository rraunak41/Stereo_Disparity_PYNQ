#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define HALF_WIDTH 640
#define MAX_DISP   64

static ap_uint<5> popcount24(ap_uint<24> x)
{
#pragma HLS INLINE
    ap_uint<5> cnt = 0;
    for(int i=0; i<24; i++)
    {
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

    // Store Left eye descriptions
    static ap_uint<24> left_desc[HALF_WIDTH];
    #pragma HLS BIND_STORAGE variable=left_desc type=ram_2p impl=bram

    // Local registers (Flip-Flops) to completely bypass BRAM port limits
    static ap_uint<24> match_window[MAX_DISP];
    #pragma HLS ARRAY_PARTITION variable=match_window type=complete

    static int x = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p;
        stream_in.read(p);

        ap_uint<24> desc = p.data;
        video_pixel out = p;

        if(p.user) {
            x = 0;
            // Clean hardware window states on frame reset
            for(int i=0; i<MAX_DISP; i++) {
                #pragma HLS UNROLL
                match_window[i] = 0;
            }
        }

        //---------------------------------
        // LEFT IMAGE (Columns 0 to 639)
        //---------------------------------
        if(x < HALF_WIDTH)
        {
            left_desc[x] = desc;
            out.data = p.data; // Pass-through for live canvas monitoring
        }
        //---------------------------------
        // RIGHT IMAGE (Columns 640 to 1279)
        //---------------------------------
        else
        {
            int xr = x - HALF_WIDTH;

            // STEP 1: Fetch EXACTLY 1 Left descriptor per clock cycle from memory
            ap_uint<24> left_pixel_fetched = left_desc[xr];

            // STEP 2: Shift local window array by 1 position
            for(int i = MAX_DISP - 1; i > 0; i--) {
                #pragma HLS UNROLL
                match_window[i] = match_window[i - 1];
            }
            match_window[0] = left_pixel_fetched; // Stream new data into slot 0

            ap_uint<6> best_disp = 0;
            ap_uint<5> best_cost = 24; // Max possible Hamming Distance for 24 bits is 24

            // STEP 3: Execute 32-way fully unrolled combinational cost matching
            if(xr >= MAX_DISP)
            {
                for(int d=0; d<MAX_DISP; d++)
                {
#pragma HLS UNROLL
                    // Read from fast discrete registers instead of BRAM ports
                    ap_uint<24> left = match_window[d];
                    ap_uint<5> cost = popcount24(left ^ desc);

                    if(cost < best_cost)
                    {
                        best_cost = cost;
                        best_disp = d;
                    }
                }
            }

            // Scale the 0-31 disparity map dynamically up to full 8-bit visible depth (0-248)
            ap_uint<8> disparity = best_disp * 4;

            out.data = ((ap_uint<24>)disparity << 16) |
                       ((ap_uint<24>)disparity << 8 ) |
                        disparity;
        }

        stream_out.write(out);

        if(p.last)
            x = 0;
        else
            x++;
    }
}
