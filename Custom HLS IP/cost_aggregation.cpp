#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define WIDTH   1280
#define BORDER  1

void cost_aggregation(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    //--------------------------------------------------
    // Two line buffers
    //--------------------------------------------------
    static ap_uint<8> line0[WIDTH];
    static ap_uint<8> line1[WIDTH];

#pragma HLS BIND_STORAGE variable=line0 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line1 type=ram_1p impl=bram

    //--------------------------------------------------
    // 3×3 window
    //--------------------------------------------------
    static ap_uint<8> w[3][3];

#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    static int x = 0;
    static int y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        //--------------------------------------------------
        // Since stereo_match outputs color heatmaps,
        // recover disparity from grayscale intensity.
        //--------------------------------------------------
        ap_uint<8> disp =
            p_in.data.range(7,0);

        //--------------------------------------------------
        // Frame reset
        //--------------------------------------------------
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
                    w[r][c] = 0;
                }
            }
        }

        //--------------------------------------------------
        // Read line buffers
        //--------------------------------------------------
        ap_uint<8> r0 = line0[x];
        ap_uint<8> r1 = line1[x];

        //--------------------------------------------------
        // Shift window left
        //--------------------------------------------------
        for(int i=0;i<3;i++)
        {
#pragma HLS UNROLL
            w[i][0] = w[i][1];
            w[i][1] = w[i][2];
        }

        //--------------------------------------------------
        // Insert new column
        //--------------------------------------------------
        w[0][2] = r0;
        w[1][2] = r1;
        w[2][2] = disp;

        //--------------------------------------------------
        // Update line buffers
        //--------------------------------------------------
        line0[x] = r1;
        line1[x] = disp;

        //--------------------------------------------------
        // Sum 3×3 disparities
        //--------------------------------------------------
        ap_uint<12> sum = 0;

        for(int yy=0; yy<3; yy++)
        {
#pragma HLS UNROLL
            for(int xx=0; xx<3; xx++)
            {
#pragma HLS UNROLL
                sum += w[yy][xx];
            }
        }

        //--------------------------------------------------
        // Average
        //--------------------------------------------------
        ap_uint<8> avg =
            sum / 9;

        //--------------------------------------------------
        // Border protection
        //--------------------------------------------------
        if(x < BORDER ||
           x >= (WIDTH-BORDER) ||
           y < BORDER)
        {
            avg = 0;
        }

        //--------------------------------------------------
        // Re-apply color map
        //--------------------------------------------------
        ap_uint<8> r = 0;
        ap_uint<8> g = 0;
        ap_uint<8> b = 0;

        if(avg == 0)
        {
            r = 0;
            g = 0;
            b = 0;
        }
        else if(avg < 64)
        {
            r = 0;
            g = avg << 2;
            b = 255;
        }
        else if(avg < 128)
        {
            r = 0;
            g = 255;
            b = 255 - ((avg-64) << 2);
        }
        else if(avg < 192)
        {
            r = (avg-128) << 2;
            g = 255;
            b = 0;
        }
        else
        {
            r = 255;
            g = 255 - ((avg-192) << 2);
            b = 0;
        }

        //--------------------------------------------------
        // Output
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

        //--------------------------------------------------
        // Coordinate update
        //--------------------------------------------------
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
