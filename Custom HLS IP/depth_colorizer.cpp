#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

void depth_colorizer(
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

        // Disparity from previous stage
        ap_uint<8> disp = p_in.data.range(7,0);

        //--------------------------------------------------
        // Blue -> Red depth map
        //
        // disp=0   => Blue
        // disp=255 => Red
        //--------------------------------------------------

        ap_uint<8> r = disp;
        ap_uint<8> g = 0;
        ap_uint<8> b = 255 - disp;

        video_pixel p_out = p_in;

        p_out.data =
            ((ap_uint<24>)r << 16) |
            ((ap_uint<24>)g << 8 ) |
             b;

        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        stream_out.write(p_out);
    }
}
