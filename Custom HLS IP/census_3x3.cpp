#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define WIDTH 1280
#define BORDER 2

void census_3x3(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    // Two line buffers for 3x3 neighborhood generation
    static ap_uint<8> line0[WIDTH];
    static ap_uint<8> line1[WIDTH];

#pragma HLS BIND_STORAGE variable=line0 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line1 type=ram_1p impl=bram

    // Sliding window registers
    static ap_uint<8> w0[3];
    static ap_uint<8> w1[3];
    static ap_uint<8> w2[3];

    static int x = 0;
    static int y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        ap_uint<8> pix = p_in.data.range(7,0);

        //--------------------------------------
        // Start of Frame Detection
        //--------------------------------------
        if(p_in.user)
        {
            x = 0;
            y = 0;

            // Clear window state
            for(int i=0;i<3;i++)
            {
#pragma HLS UNROLL
                w0[i] = 0;
                w1[i] = 0;
                w2[i] = 0;
            }
        }

        //--------------------------------------
        // Read BRAM first
        //--------------------------------------
        ap_uint<8> cached_line0 = line0[x];
        ap_uint<8> cached_line1 = line1[x];

        //--------------------------------------
        // Shift window
        //--------------------------------------
        w0[0] = w0[1];
        w0[1] = w0[2];
        w0[2] = cached_line0;

        w1[0] = w1[1];
        w1[1] = w1[2];
        w1[2] = cached_line1;

        w2[0] = w2[1];
        w2[1] = w2[2];
        w2[2] = pix;

        //--------------------------------------
        // Update line buffers
        //--------------------------------------
        line0[x] = cached_line1;
        line1[x] = pix;

        //--------------------------------------
        // Center Pixel
        //--------------------------------------
        ap_uint<8> center = w1[1];

        //--------------------------------------
        // Full 8-bit Census Descriptor
        //--------------------------------------
        ap_uint<8> census = 0;

        census[0] = (w0[0] >= center);
        census[1] = (w0[1] >= center);
        census[2] = (w0[2] >= center);

        census[3] = (w1[0] >= center);
        census[4] = (w1[2] >= center);

        census[5] = (w2[0] >= center);
        census[6] = (w2[1] >= center);
        census[7] = (w2[2] >= center);

        //--------------------------------------
        // Boundary Protection
        //--------------------------------------
        ap_uint<8> output_val = census;

        if(x < BORDER || y < BORDER)
        {
            output_val = 0;
        }

        //--------------------------------------
        // Visualize Census Descriptor
        //--------------------------------------
        video_pixel p_out = p_in;

        p_out.data =
            ((ap_uint<24>)output_val << 16) |
            ((ap_uint<24>)output_val << 8 ) |
            ((ap_uint<24>)output_val);

        //--------------------------------------
        // Preserve AXI Video Sidebands
        //--------------------------------------
        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        stream_out.write(p_out);

        //--------------------------------------
        // Coordinate Update
        //--------------------------------------
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
