#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define IMG_WIDTH 1280
#define BORDER 1

//-----------------------------------------------------
// Swap
//-----------------------------------------------------
static void swap(ap_uint<8> &a, ap_uint<8> &b)
{
#pragma HLS INLINE

    if(a > b)
    {
        ap_uint<8> t = a;
        a = b;
        b = t;
    }
}

//-----------------------------------------------------
// Median of 9
//-----------------------------------------------------
static ap_uint<8> median9(ap_uint<8> w[3][3])
{
#pragma HLS INLINE

    ap_uint<8> a[9];

#pragma HLS ARRAY_PARTITION variable=a complete

    a[0]=w[0][0];
    a[1]=w[0][1];
    a[2]=w[0][2];
    a[3]=w[1][0];
    a[4]=w[1][1];
    a[5]=w[1][2];
    a[6]=w[2][0];
    a[7]=w[2][1];
    a[8]=w[2][2];

    //-------------------------------------------------
    // Simple bubble sort
    //-------------------------------------------------

    for(int i=0;i<8;i++)
    {
#pragma HLS UNROLL

        for(int j=i+1;j<9;j++)
        {
#pragma HLS UNROLL

            swap(a[i],a[j]);
        }
    }

    return a[4];
}

//-----------------------------------------------------
// Median Filter
//-----------------------------------------------------
void median_filter_3x3(
        hls::stream<video_pixel> &stream_in,
        hls::stream<video_pixel> &stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    //-------------------------------------------------
    // Two line buffers
    //-------------------------------------------------

    static ap_uint<8> line0[IMG_WIDTH];
    static ap_uint<8> line1[IMG_WIDTH];

#pragma HLS BIND_STORAGE variable=line0 type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=line1 type=ram_1p impl=bram

    //-------------------------------------------------
    // Window
    //-------------------------------------------------

    static ap_uint<8> w[3][3];

#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    static int x=0;
    static int y=0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        //---------------------------------------------
        // Read pixel
        //---------------------------------------------

        video_pixel in;
        stream_in.read(in);

        ap_uint<8> pix =
            in.data.range(7,0);

        //---------------------------------------------
        // Start of frame
        //---------------------------------------------

        if(in.user)
        {
            x=0;
            y=0;

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

        //---------------------------------------------
        // Read line buffers
        //---------------------------------------------

        ap_uint<8> r0=line0[x];
        ap_uint<8> r1=line1[x];

        //---------------------------------------------
        // Shift window
        //---------------------------------------------

        for(int i=0;i<3;i++)
        {
#pragma HLS UNROLL

            w[i][0]=w[i][1];
            w[i][1]=w[i][2];
        }

        //---------------------------------------------
        // Insert new column
        //---------------------------------------------

        w[0][2]=r0;
        w[1][2]=r1;
        w[2][2]=pix;

        //---------------------------------------------
        // Update line buffers
        //---------------------------------------------

        line0[x]=r1;
        line1[x]=pix;

        //---------------------------------------------
        // Median
        //---------------------------------------------

        ap_uint<8> out_pix =
            median9(w);

        //---------------------------------------------
        // Border
        //---------------------------------------------

        if(x<BORDER ||
           x>=IMG_WIDTH-BORDER ||
           y<BORDER)
        {
            out_pix=0;
        }

        //---------------------------------------------
        // Output
        //---------------------------------------------

        video_pixel out=in;

        out.data =
            ((ap_uint<24>)out_pix<<16)|
            ((ap_uint<24>)out_pix<<8)|
             out_pix;

        stream_out.write(out);

        //---------------------------------------------
        // Coordinate update
        //---------------------------------------------

        if(in.last)
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
