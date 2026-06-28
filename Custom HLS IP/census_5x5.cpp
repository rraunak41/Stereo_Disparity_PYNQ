#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define IMG_WIDTH 1280
#define BORDER 2

void census_5x5(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    //---------------------------------------------------
    // Four line buffers
    //---------------------------------------------------
    static ap_uint<8> line0[IMG_WIDTH];
    static ap_uint<8> line1[IMG_WIDTH];
    static ap_uint<8> line2[IMG_WIDTH];
    static ap_uint<8> line3[IMG_WIDTH];

#pragma HLS BIND_STORAGE variable=line0 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line1 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line2 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line3 type=ram_1p impl=bram

    //---------------------------------------------------
    // 5x5 sliding window
    //---------------------------------------------------
    static ap_uint<8> w[5][5];
#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    static int x = 0;
    static int y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        ap_uint<8> pix =
            p_in.data.range(7,0);

        //---------------------------------------------------
        // Start of frame
        //---------------------------------------------------
        if(p_in.user)
        {
            x = 0;
            y = 0;

            for(int r=0;r<5;r++)
            {
#pragma HLS UNROLL
                for(int c=0;c<5;c++)
                {
#pragma HLS UNROLL
                    w[r][c] = 0;
                }
            }
        }

        //---------------------------------------------------
        // Read line buffers
        //---------------------------------------------------
        ap_uint<8> r0 = line0[x];
        ap_uint<8> r1 = line1[x];
        ap_uint<8> r2 = line2[x];
        ap_uint<8> r3 = line3[x];

        //---------------------------------------------------
        // Shift window left
        //---------------------------------------------------
        for(int i=0;i<5;i++)
        {
#pragma HLS UNROLL
            w[i][0] = w[i][1];
            w[i][1] = w[i][2];
            w[i][2] = w[i][3];
            w[i][3] = w[i][4];
        }

        //---------------------------------------------------
        // Insert new column
        //---------------------------------------------------
        w[0][4] = r0;
        w[1][4] = r1;
        w[2][4] = r2;
        w[3][4] = r3;
        w[4][4] = pix;

        //---------------------------------------------------
        // Update line buffers
        //---------------------------------------------------
        line0[x] = r1;
        line1[x] = r2;
        line2[x] = r3;
        line3[x] = pix;

        //---------------------------------------------------
        // Center pixel
        //---------------------------------------------------
        ap_uint<8> center =
            w[2][2];

        //---------------------------------------------------
        // Build 24-bit descriptor
        //---------------------------------------------------
        ap_uint<24> census = 0;

        int bit = 0;

        for(int yy=0; yy<5; yy++)
        {
#pragma HLS UNROLL
            for(int xx=0; xx<5; xx++)
            {
#pragma HLS UNROLL

                if(!(yy==2 && xx==2))
                {
                    census[bit] =
                        (w[yy][xx] >= center);

                    bit++;
                }
            }
        }

        //---------------------------------------------------
        // Border protection
        //---------------------------------------------------
        ap_uint<24> output =
            census;

        if(x < BORDER ||
           x >= (IMG_WIDTH-BORDER) ||
           y < BORDER)
        {
            output = 0;
        }

        //---------------------------------------------------
        // Output
        //---------------------------------------------------
        video_pixel p_out = p_in;

        p_out.data = output;

        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        stream_out.write(p_out);

        //---------------------------------------------------
        // Coordinate update
        //---------------------------------------------------
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
