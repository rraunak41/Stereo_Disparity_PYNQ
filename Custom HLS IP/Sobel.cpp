#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

void Sobel(
    hls::stream<video_pixel>& stream_in_24,
    hls::stream<video_pixel>& stream_out_24)
{
#pragma HLS INTERFACE axis port=stream_in_24
#pragma HLS INTERFACE axis port=stream_out_24
#pragma HLS INTERFACE ap_ctrl_none port=return

    static ap_uint<8> line0[640];
    static ap_uint<8> line1[640];

#pragma HLS BIND_STORAGE variable=line0 type=RAM_S2P impl=BRAM
#pragma HLS BIND_STORAGE variable=line1 type=RAM_S2P impl=BRAM

    static ap_uint<8> w[3][3];

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

        ap_uint<8> gray = p_in.data.range(7,0);

        // Shift window
        w[0][0] = w[0][1];
        w[0][1] = w[0][2];

        w[1][0] = w[1][1];
        w[1][1] = w[1][2];

        w[2][0] = w[2][1];
        w[2][1] = w[2][2];

        // Insert new column
        w[0][2] = line0[x];
        w[1][2] = line1[x];
        w[2][2] = gray;

        // Update line buffers
        line0[x] = line1[x];
        line1[x] = gray;

        ap_uint<8> edge = 0;

        if(x > 1 && y > 1)
        {
            int gx =
                -w[0][0] + w[0][2]
                -2*w[1][0] + 2*w[1][2]
                -w[2][0] + w[2][2];

            int gy =
                -w[0][0] -2*w[0][1] -w[0][2]
                +w[2][0] +2*w[2][1] +w[2][2];

            int mag = (gx < 0 ? -gx : gx)
                    + (gy < 0 ? -gy : gy);

            if(mag > 255)
                mag = 255;

            edge = (ap_uint<8>)mag;
        }

        p_out.data =
            ((ap_uint<24>)edge << 16) |
            ((ap_uint<24>)edge << 8 ) |
            ((ap_uint<24>)edge);

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
