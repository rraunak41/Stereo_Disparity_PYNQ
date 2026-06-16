#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define WIDTH       1280
#define HALF_WIDTH  640
#define BORDER      2

void census_3x3(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    static ap_uint<8> line0[WIDTH];
    static ap_uint<8> line1[WIDTH];
#pragma HLS BIND_STORAGE variable=line0 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line1 type=ram_1p impl=bram

    static ap_uint<8> w0[3];
    static ap_uint<8> w1[3];
    static ap_uint<8> w2[3];
#pragma HLS ARRAY_PARTITION variable=w0 complete
#pragma HLS ARRAY_PARTITION variable=w1 complete
#pragma HLS ARRAY_PARTITION variable=w2 complete

    static ap_uint<11> x = 0;
    static ap_uint<10> y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        ap_uint<8> pix = p_in.data.range(7,0);

        // Reset the 3x3 window at frame start AND at the Left/Right seam (x==640).
        // Without this, the window at columns near 640 blends Left-image and
        // Right-image pixels together, corrupting the Census descriptor right
        // at the seam on every single row.
        bool seam_reset = (x == HALF_WIDTH);

        if(p_in.user || seam_reset)
        {
            for(int i=0;i<3;i++)
            {
#pragma HLS UNROLL
                w0[i] = 0;
                w1[i] = 0;
                w2[i] = 0;
            }
        }

        if(p_in.user)
        {
            x = 0;
            y = 0;
        }

        ap_uint<8> l0 = line0[x];
        ap_uint<8> l1 = line1[x];

        w0[0] = w0[1]; w0[1] = w0[2]; w0[2] = l0;
        w1[0] = w1[1]; w1[1] = w1[2]; w1[2] = l1;
        w2[0] = w2[1]; w2[1] = w2[2]; w2[2] = pix;

        line0[x] = l1;
        line1[x] = pix;

        ap_uint<8> c = w1[1];

        ap_uint<8> census = 0;
        census[0] = (w0[0] >= c);
        census[1] = (w0[1] >= c);
        census[2] = (w0[2] >= c);
        census[3] = (w1[0] >= c);
        census[4] = (w1[2] >= c);
        census[5] = (w2[0] >= c);
        census[6] = (w2[1] >= c);
        census[7] = (w2[2] >= c);

        ap_uint<8> out_val = census;

        // Position within whichever half (Left: 0-639, Right: 0-639 after offset)
        ap_uint<11> local_x = (x < HALF_WIDTH) ? x : (ap_uint<11>)(x - HALF_WIDTH);

        // Mask borders relative to EACH half independently, plus top rows.
        // This covers the seam-reset transient columns (0,1 of the Right half)
        // the same way it covers the real frame edges.
        if(local_x < BORDER ||
           local_x >= (HALF_WIDTH - BORDER) ||
           y < BORDER)
        {
            out_val = 0;
        }

        video_pixel p_out = p_in;
        p_out.data = ((ap_uint<24>)out_val << 16) |
                     ((ap_uint<24>)out_val << 8 ) |
                      out_val;

        stream_out.write(p_out);

        if(p_in.last) { x = 0; y++; }
        else           { x++; }
    }
}
