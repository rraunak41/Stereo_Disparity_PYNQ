#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

void depth_colormap(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        //--------------------------------------------------
        // Recover disparity value
        //--------------------------------------------------
        ap_uint<8> d =
            p_in.data.range(7,0);

        ap_uint<8> r = 0;
        ap_uint<8> g = 0;
        ap_uint<8> b = 0;

        //--------------------------------------------------
        // Invalid disparity
        //--------------------------------------------------
        if(d == 0)
        {
            r = 0;
            g = 0;
            b = 0;
        }

        //--------------------------------------------------
        // Blue -> Cyan
        //--------------------------------------------------
        else if(d < 64)
        {
            r = 0;
            g = d << 2;
            b = 255;
        }

        //--------------------------------------------------
        // Cyan -> Green
        //--------------------------------------------------
        else if(d < 128)
        {
            r = 0;
            g = 255;
            b = 255 - ((d - 64) << 2);
        }

        //--------------------------------------------------
        // Green -> Yellow
        //--------------------------------------------------
        else if(d < 192)
        {
            r = (d - 128) << 2;
            g = 255;
            b = 0;
        }

        //--------------------------------------------------
        // Yellow -> Red
        //--------------------------------------------------
        else
        {
            r = 255;
            g = 255 - ((d - 192) << 2);
            b = 0;
        }

        //--------------------------------------------------
        // Pack RGB output
        //--------------------------------------------------
        video_pixel p_out = p_in;

        p_out.data =
            ((ap_uint<24>)r << 16) |
            ((ap_uint<24>)g << 8 ) |
             (ap_uint<24>)b;

        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        stream_out.write(p_out);
    }
}
