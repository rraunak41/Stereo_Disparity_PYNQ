#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

static ap_uint<8> depth_lut[256];

void depth_compute(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    //-------------------------------------------------
    // Initialize ROM once
    //-------------------------------------------------
    static bool init = false;

    if(!init)
    {
        depth_lut[0] = 255;

        for(int d=1; d<256; d++)
        {
#pragma HLS PIPELINE II=1

            int depth_cm = 5200 / d;

            if(depth_cm > 255)
                depth_cm = 255;

            depth_lut[d] =
                (ap_uint<8>)depth_cm;
        }

        init = true;
    }

    //-------------------------------------------------
    // Streaming
    //-------------------------------------------------
    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        video_pixel p_out = p_in;

        ap_uint<8> disparity =
            p_in.data.range(7,0);

        ap_uint<8> depth =
            depth_lut[disparity];

        p_out.data =
            ((ap_uint<24>)depth << 16) |
            ((ap_uint<24>)depth << 8 ) |
             depth;

        stream_out.write(p_out);
    }
}
