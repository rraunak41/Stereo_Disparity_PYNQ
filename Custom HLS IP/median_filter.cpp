#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define WIDTH 640

static void sort9(ap_uint<8> a[9])
{
#pragma HLS INLINE

    for(int i=1;i<9;i++)
    {
#pragma HLS UNROLL

        ap_uint<8> key = a[i];
        int j = i - 1;

        while(j >= 0 && a[j] > key)
        {
#pragma HLS UNROLL
            a[j+1] = a[j];
            j--;
        }

        a[j+1] = key;
    }
}

void median_filter(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    //------------------------------------------------
    // Two line buffers
    //------------------------------------------------
    static ap_uint<8> line0[WIDTH];
    static ap_uint<8> line1[WIDTH];

#pragma HLS BIND_STORAGE variable=line0 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line1 type=ram_1p impl=bram

    //------------------------------------------------
    // 3x3 window
    //------------------------------------------------
    static ap_uint<8> w[3][3];
#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    static int x = 0;
    static int y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        ap_uint<8> disp =
            p_in.data.range(7,0);

        //------------------------------------------------
        // Start of frame
        //------------------------------------------------
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

        //------------------------------------------------
        // Read buffers
        //------------------------------------------------
        ap_uint<8> r0 = line0[x];
        ap_uint<8> r1 = line1[x];

        //------------------------------------------------
        // Shift window
        //------------------------------------------------
        for(int i=0;i<3;i++)
        {
#pragma HLS UNROLL
            w[i][0] = w[i][1];
            w[i][1] = w[i][2];
        }

        //------------------------------------------------
        // Insert column
        //------------------------------------------------
        w[0][2] = r0;
        w[1][2] = r1;
        w[2][2] = disp;

        //------------------------------------------------
        // Update buffers
        //------------------------------------------------
        line0[x] = r1;
        line1[x] = disp;

        //------------------------------------------------
        // Copy 3x3 window
        //------------------------------------------------
        ap_uint<8> vals[9];
#pragma HLS ARRAY_PARTITION variable=vals complete

        vals[0] = w[0][0];
        vals[1] = w[0][1];
        vals[2] = w[0][2];

        vals[3] = w[1][0];
        vals[4] = w[1][1];
        vals[5] = w[1][2];

        vals[6] = w[2][0];
        vals[7] = w[2][1];
        vals[8] = w[2][2];

        //------------------------------------------------
        // Median
        //------------------------------------------------
        sort9(vals);

        ap_uint<8> med = vals[4];

        //------------------------------------------------
        // Border protection
        //------------------------------------------------
        if(x < 1 || y < 1)
        {
            med = 0;
        }

        //------------------------------------------------
        // Output
        //------------------------------------------------
        video_pixel p_out = p_in;

        p_out.data =
            ((ap_uint<24>)med << 16) |
            ((ap_uint<24>)med << 8 ) |
             med;

        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        stream_out.write(p_out);

        //------------------------------------------------
        // Coordinates
        //------------------------------------------------
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
