#include "ap_axi_sdata.h"
#include "hls_stream.h"
#include "ap_int.h"

// Define a true 24-bit AXI4-Stream interface (3 bytes)
// This guarantees TDATA_NUM_BYTES = 3, matching Vivado perfectly.
typedef ap_axiu<24, 1, 1, 1> video_pixel;

void gray(
    hls::stream<video_pixel>& stream_in_24,
    hls::stream<video_pixel>& stream_out_24)
{
// Setup interfaces for the Vitis HLS 2022.1 Vivado flow target
#pragma HLS INTERFACE axis port=stream_in_24
#pragma HLS INTERFACE axis port=stream_out_24
#pragma HLS INTERFACE ap_ctrl_none port=return

    while(1)
    {
#pragma HLS PIPELINE II=1

        video_pixel p_in;
        stream_in_24.read(p_in);

        video_pixel p_out;
        // Pass independent TUSER and TLAST sideband wires directly through
        p_out.user = p_in.user;
        p_out.last = p_in.last;
        p_out.keep = p_in.keep;
        p_out.strb = p_in.strb;

        // Isolate the 24-bit data bus payload
        ap_uint<24> raw_data = p_in.data;

        // Extract the color channels from their exact 24-bit byte locations
        // Byte 0 (7:0) = Blue | Byte 1 (15:8) = Green | Byte 2 (23:16) = Red
        ap_uint<8> b = (ap_uint<8>)(raw_data & 0xFF);
        ap_uint<8> g = (ap_uint<8>)((raw_data >> 8) & 0xFF);
        ap_uint<8> r = (ap_uint<8>)((raw_data >> 16) & 0xFF);

        // Standard integer fixed-point grayscale matrix calculation
        ap_uint<16> gray_calc = (29 * b) + (150 * g) + (77 * r);
        ap_uint<8> gray_val = (ap_uint<8>)(gray_calc >> 8);

        // Pack the identical gray value back into all 3 bytes of the 24-bit payload
        p_out.data = ((ap_uint<24>)gray_val << 16) |
                     ((ap_uint<24>)gray_val << 8)  |
                     ((ap_uint<24>)gray_val);

        stream_out_24.write(p_out);
    }
}
