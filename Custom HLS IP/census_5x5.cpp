#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define WIDTH  1280
#define BORDER 2

void census_5x5(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return

    //----------------------------------------------------------
    // Packed line buffer:
    // [31:24] row-4
    // [23:16] row-3
    // [15:8 ] row-2
    // [7 :0 ] row-1
    //----------------------------------------------------------

    static ap_uint<32> packed_line_buffer[WIDTH];

#pragma HLS BIND_STORAGE variable=packed_line_buffer type=ram_2p impl=bram

    //----------------------------------------------------------
    // 5x5 sliding window
    //----------------------------------------------------------

    static ap_uint<8> w[5][5];

#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    static int x = 0;
    static int y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in.read(p_in);

        ap_uint<8> pix = p_in.data.range(7,0);

        //------------------------------------------------------
        // Start of frame
        //------------------------------------------------------

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

        //------------------------------------------------------
        // Read packed history
        //------------------------------------------------------

        ap_uint<32> cache_word = packed_line_buffer[x];

        ap_uint<8> r0 = cache_word.range(31,24);
        ap_uint<8> r1 = cache_word.range(23,16);
        ap_uint<8> r2 = cache_word.range(15,8);
        ap_uint<8> r3 = cache_word.range(7,0);

        //------------------------------------------------------
        // Shift window left
        //------------------------------------------------------

        for(int row=0; row<5; row++)
        {
#pragma HLS UNROLL
            w[row][0] = w[row][1];
            w[row][1] = w[row][2];
            w[row][2] = w[row][3];
            w[row][3] = w[row][4];
        }

        //------------------------------------------------------
        // Insert newest column
        //------------------------------------------------------

        w[0][4] = r0;
        w[1][4] = r1;
        w[2][4] = r2;
        w[3][4] = r3;
        w[4][4] = pix;

        //------------------------------------------------------
        // Update packed BRAM
        //------------------------------------------------------

        ap_uint<32> update_word;

        update_word.range(31,24) = r1;
        update_word.range(23,16) = r2;
        update_word.range(15,8)  = r3;
        update_word.range(7,0)   = pix;

        packed_line_buffer[x] = update_word;

        //------------------------------------------------------
        // 5x5 Census Descriptor
        //------------------------------------------------------

        ap_uint<24> census = 0;

        ap_uint<8> center = w[2][2];

        int bit_idx = 0;

        for(int yy=0; yy<5; yy++)
        {
#pragma HLS UNROLL

            for(int xx=0; xx<5; xx++)
            {
#pragma HLS UNROLL

                if(!(yy==2 && xx==2))
                {
                    census[bit_idx] =
                        (w[yy][xx] >= center);

                    bit_idx++;
                }
            }
        }

        //------------------------------------------------------
        // Boundary protection
        //------------------------------------------------------

        bool valid =
            (x >= BORDER) &&
            (x < (WIDTH - BORDER)) &&
            (y >= BORDER);

        if(!valid)
        {
            census = 0;
        }

        //------------------------------------------------------
        // Debug Visualization
        //------------------------------------------------------
        // Keep full 24-bit census internally,
        // display lower 8 bits as grayscale.
        //------------------------------------------------------

        ap_uint<8> display =
            census.range(7,0);

        video_pixel p_out = p_in;

        p_out.data =
            ((ap_uint<24>)display << 16) |
            ((ap_uint<24>)display << 8 ) |
             (ap_uint<24>)display;

        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        stream_out.write(p_out);

        //------------------------------------------------------
        // Coordinate update
        //------------------------------------------------------

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
