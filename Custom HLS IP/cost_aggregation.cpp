#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

typedef ap_axiu<24,1,1,1> video_pixel;

#define HALF_WIDTH 640
#define FULL_WIDTH 1280
#define HEIGHT     480
#define BORDER     4

void cost_aggregation(
    hls::stream<video_pixel>& stream_in,
    hls::stream<video_pixel>& stream_out)
{
#pragma HLS INTERFACE axis port=stream_in
#pragma HLS INTERFACE axis port=stream_out
#pragma HLS INTERFACE ap_ctrl_none port=return


    static ap_uint<32> packed_line_buffer[HALF_WIDTH];
#pragma HLS BIND_STORAGE variable=packed_line_buffer type=ram_2p impl=bram


    static ap_uint<8> w[5][5];
#pragma HLS ARRAY_PARTITION variable=w complete dim=0

    static int x = 0;
    static int y = 0;

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p;
        stream_in.read(p);

        ap_uint<8> disp = p.data.range(7,0);

        if(p.user)
        {
            x = 0;
            y = 0;
            // Clear window states on frame reset
            for(int r=0; r<5; r++) {
                #pragma HLS UNROLL
                for(int c=0; c<5; c++) {
                    #pragma HLS UNROLL
                    w[r][c] = 0;
                }
            }
        }

        video_pixel out = p;


        if(x < HALF_WIDTH)
        {
            out.data = p.data; // Keep raw Left camera visualization untouched
        }

        else
        {
            int xr = x - HALF_WIDTH;


            ap_uint<32> cache_word = packed_line_buffer[xr];
            ap_uint<8> r0 = cache_word.range(31,24);
            ap_uint<8> r1 = cache_word.range(23,16);
            ap_uint<8> r2 = cache_word.range(15,8);
            ap_uint<8> r3 = cache_word.range(7,0);


            for(int i=0; i<5; i++)
            {
#pragma HLS UNROLL
                w[i][0] = w[i][1];
                w[i][1] = w[i][2];
                w[i][2] = w[i][3];
                w[i][3] = w[i][4];
            }


            w[0][4] = r0;
            w[1][4] = r1;
            w[2][4] = r2;
            w[3][4] = r3;
            w[4][4] = disp;


            ap_uint<32> update_word;
            update_word.range(31,24) = r1;
            update_word.range(23,16) = r2;
            update_word.range(15,8)  = r3;
            update_word.range(7,0)   = disp;
            packed_line_buffer[xr]   = update_word;


            ap_uint<16> sum = 0;
            for(int yy=0; yy<5; yy++)
            {
#pragma HLS UNROLL
                for(int xx=0; xx<5; xx++)
                {
#pragma HLS UNROLL
                    sum += w[yy][xx];
                }
            }


            ap_uint<24> scale_mult = sum * 41;
            ap_uint<8> avg = (ap_uint<8>)(scale_mult >> 10);


            if(xr < BORDER || y < BORDER)
            {
                avg = 0;
            }

            out.data = ((ap_uint<24>)avg << 16) |
                       ((ap_uint<24>)avg << 8 ) |
                        avg;
        }

        stream_out.write(out);

        if(p.last)
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
