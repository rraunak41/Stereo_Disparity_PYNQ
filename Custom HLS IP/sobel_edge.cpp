#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define WIDTH 1280
#define BORDER 1

void sobel_edge(
    hls::stream<video_pixel> &stream_in,
    hls::stream<video_pixel> &stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    //----------------------------------------------------
    // Two line buffers
    //----------------------------------------------------
    static ap_uint<8> line0[WIDTH];
    static ap_uint<8> line1[WIDTH];

#pragma HLS BIND_STORAGE variable=line0 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line1 type=ram_1p impl=bram

    //----------------------------------------------------
    // 3×3 window
    //----------------------------------------------------
    static ap_uint<8> w[3][3];

#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    static int x = 0;
    static int y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        //----------------------------------------------------
        // Use grayscale pixel
        //----------------------------------------------------
        ap_uint<8> pix =
            p_in.data.range(7,0);

        //----------------------------------------------------
        // Frame start
        //----------------------------------------------------
        if(p_in.user)
        {
            x = 0;
            y = 0;

            for(int r=0;r<3;r++)
            {
#pragma HLS UNROLL
                for(int c=0;c<3;c++)
                {
#pragma HLS UNROLL
                    w[r][c]=0;
                }
            }
        }

        //----------------------------------------------------
        // Read line buffers
        //----------------------------------------------------
        ap_uint<8> r0 = line0[x];
        ap_uint<8> r1 = line1[x];

        //----------------------------------------------------
        // Shift window
        //----------------------------------------------------
        for(int i=0;i<3;i++)
        {
#pragma HLS UNROLL
            w[i][0]=w[i][1];
            w[i][1]=w[i][2];
        }

        //----------------------------------------------------
        // Insert column
        //----------------------------------------------------
        w[0][2]=r0;
        w[1][2]=r1;
        w[2][2]=pix;

        //----------------------------------------------------
        // Update buffers
        //----------------------------------------------------
        line0[x]=r1;
        line1[x]=pix;

        //----------------------------------------------------
        // Sobel Gx
        //----------------------------------------------------
        int gx =
            -w[0][0] + w[0][2]
          -2*w[1][0] +2*w[1][2]
          -w[2][0] + w[2][2];

        //----------------------------------------------------
        // Sobel Gy
        //----------------------------------------------------
        int gy =
            -w[0][0] -2*w[0][1] -w[0][2]
            +w[2][0] +2*w[2][1] +w[2][2];

        //----------------------------------------------------
        // Absolute values
        //----------------------------------------------------
        if(gx<0) gx=-gx;
        if(gy<0) gy=-gy;

        //----------------------------------------------------
        // Gradient approximation
        //----------------------------------------------------
        int grad = gx + gy;

        //----------------------------------------------------
        // Clamp
        //----------------------------------------------------
        if(grad>255)
            grad=255;

        //----------------------------------------------------
        // Threshold
        //----------------------------------------------------
        ap_uint<8> edge;

        if(grad>80)
            edge=255;
        else
            edge=0;

        //----------------------------------------------------
        // Border protection
        //----------------------------------------------------
        if(x<BORDER ||
           x>=WIDTH-BORDER ||
           y<BORDER)
        {
            edge=0;
        }

        //----------------------------------------------------
        // Output
        //----------------------------------------------------
        video_pixel p_out=p_in;

        p_out.data =
            ((ap_uint<24>)edge<<16)|
            ((ap_uint<24>)edge<<8)|
             edge;

        p_out.user=p_in.user;
        p_out.last=p_in.last;
        p_out.keep=p_in.keep;
        p_out.strb=p_in.strb;

        stream_out.write(p_out);

        //----------------------------------------------------
        // Coordinate update
        //----------------------------------------------------
        if(p_in.last)
        {
            x=0;
            y++;
        }
        else
        {
            x++;
        }
    }
}
