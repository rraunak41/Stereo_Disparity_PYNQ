#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

void crosshair(
    hls::stream<video_pixel>& stream_in_24,
    hls::stream<video_pixel>& stream_out_24)
{
#pragma HLS INTERFACE axis port=stream_in_24
#pragma HLS INTERFACE axis port=stream_out_24
#pragma HLS INTERFACE ap_ctrl_none port=return

    static ap_uint<24> line0[640];
    static ap_uint<24> line1[640];

#pragma HLS BIND_STORAGE variable=line0 type=RAM_S2P impl=BRAM
#pragma HLS BIND_STORAGE variable=line1 type=RAM_S2P impl=BRAM

    static int x = 0;
    static int y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in_24.read(p_in);

        video_pixel p_out;

        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        if(p_in.user)
        {
            x = 0;
            y = 0;
        }

        ap_uint<24> current = p_in.data;

        ap_uint<24> top    = line0[x];
        ap_uint<24> middle = line1[x];
        ap_uint<24> bottom = current;

        // Update buffers
        line0[x] = line1[x];
        line1[x] = current;

        // Window center test:
        // Output the middle pixel from the 3-row neighborhood
        p_out.data = middle;

        stream_out_24.write(p_out);

        if(p_in.last)
        {
            x = 0;
            y++;
        }
        else
        {
            x++;
        }
    }
}
