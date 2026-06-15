#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define HALF_WIDTH 640
#define MAX_DISP   16

static ap_uint<4> popcount8(ap_uint<8> x)
{
#pragma HLS INLINE

    ap_uint<4> cnt = 0;

    for(int i=0;i<8;i++)
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

    static ap_uint<8> left_desc[HALF_WIDTH];

#pragma HLS BIND_STORAGE variable=left_desc type=ram_2p impl=bram

    static int x = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        ap_uint<8> desc = p_in.data.range(7,0);

        if(p_in.user)
        {
            x = 0;
        }

        video_pixel p_out = p_in;

        //--------------------------------------------------
        // LEFT IMAGE
        //--------------------------------------------------

        if(x < HALF_WIDTH)
        {
            left_desc[x] = desc;

            // show census image for debugging
            p_out.data =
                ((ap_uint<24>)desc << 16) |
                ((ap_uint<24>)desc << 8 ) |
                desc;
        }

        //--------------------------------------------------
        // RIGHT IMAGE
        //--------------------------------------------------

        else
        {
            int xr = x - HALF_WIDTH;

            ap_uint<4> best_disp = 0;
            ap_uint<4> best_cost = 8;

            if(xr >= MAX_DISP)
            {
                for(int d=0; d<MAX_DISP; d++)
                {
#pragma HLS UNROLL

                    ap_uint<8> left =
                        left_desc[xr - d];

                    ap_uint<4> cost =
                        popcount8(left ^ desc);

                    if(cost < best_cost)
                    {
                        best_cost = cost;
                        best_disp = d;
                    }
                }
            }

            ap_uint<8> disparity =
                best_disp * 17;

            p_out.data =
                ((ap_uint<24>)disparity << 16) |
                ((ap_uint<24>)disparity << 8 ) |
                disparity;
        }

        stream_out.write(p_out);

        if(p_in.last)
            x = 0;
        else
            x++;
    }
}
