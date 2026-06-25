#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define HALF_WIDTH 640
#define MAX_DISP   32

//----------------------------------------------------
// 24-bit Hamming Distance
//----------------------------------------------------
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

    //----------------------------------------------------
    // Store left descriptors in BRAM
    //----------------------------------------------------
    static ap_uint<24> left_desc[HALF_WIDTH];

#pragma HLS BIND_STORAGE variable=left_desc type=ram_2p impl=bram

    static ap_uint<11> x = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        ap_uint<24> desc = p_in.data;

        video_pixel p_out = p_in;

        //----------------------------------------------------
        // Start of frame
        //----------------------------------------------------
        if(p_in.user)
        {
            x = 0;
        }

        //----------------------------------------------------
        // LEFT IMAGE
        //----------------------------------------------------
        if(x < HALF_WIDTH)
        {
            left_desc[x] = desc;

            //------------------------------------------------
            // Fill left half with black
            //------------------------------------------------
            p_out.data = 0;
        }

        //----------------------------------------------------
        // RIGHT IMAGE
        //----------------------------------------------------
        else
        {
            int xr = x - HALF_WIDTH;

            ap_uint<6> best_disp = 0;
            ap_uint<5> best_cost = 24;

            //------------------------------------------------
            // Search disparities
            //------------------------------------------------
            if(xr >= MAX_DISP)
            {
                SEARCH:
                for(int d=0; d<MAX_DISP; d++)
                {
#pragma HLS LOOP_TRIPCOUNT min=32 max=32

                    ap_uint<24> left =
                        left_desc[xr - d];

                    ap_uint<5> cost =
                        popcount24(left ^ desc);

                    if(cost < best_cost)
                    {
                        best_cost = cost;
                        best_disp = d;
                    }
                }
            }

            //------------------------------------------------
            // Reject flat regions
            //------------------------------------------------
            bool valid =
                (desc != 0) &&
                (desc != 0xFFFFFF);

            //------------------------------------------------
            // Confidence gate
            //------------------------------------------------
            ap_uint<8> disparity = 0;

            if(valid && (best_cost < 8))
            {
                disparity = best_disp;   // Raw disparity (0–32)
            }

            //------------------------------------------------
            // Display disparity
            //------------------------------------------------
            p_out.data =
                ((ap_uint<24>)disparity << 16) |
                ((ap_uint<24>)disparity << 8 ) |
                 (ap_uint<24>)disparity;
        }

        //----------------------------------------------------
        // Preserve AXI sidebands
        //----------------------------------------------------
        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        stream_out.write(p_out);

        //----------------------------------------------------
        // Coordinate update
        //----------------------------------------------------
        if(p_in.last)
            x = 0;
        else
            x++;
    }
}
