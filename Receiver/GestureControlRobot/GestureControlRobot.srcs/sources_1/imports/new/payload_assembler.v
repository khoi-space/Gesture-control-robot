`timescale 1ns / 1ps



module payload_assembler (
    input clk,
    input rst_n,

    // Interface from nrf24l01_controller module
    input [47:0] rx_payload_in,    // Input 6-byte payload (48 bits) all at once
    input data_valid,        // Flag indicating a new valid payload is present

    // Assembled output data (16 bits for each axis, matching Arduino 'int')
    output reg [15:0] x_axis_out,
    output reg [15:0] y_axis_out,
    output reg [15:0] z_axis_out,

    // Flag indicating a complete packet has been assembled
    output reg packet_ready
); 

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // --- Reset Logic ---
        x_axis_out <= 16'h0000;
        y_axis_out <= 16'h0000;
        z_axis_out <= 16'h0000;
        packet_ready <= 1'b0;
        $display("[Assembler] Reset");

    end else begin
        packet_ready <= 1'b0; // Default: pulse ready for only one clock cycle

        // --- Process Incoming 6-byte Payload ---
        if (data_valid) begin
            // Extract and assemble the 16-bit values from 48-bit payload (Little-Endian format):
            // Payload bit mapping: [47:40][39:32][31:24][23:16][15:8][7:0]
            //                      Byte5   Byte4  Byte3  Byte2  Byte1 Byte0
            
            // X Axis: Byte1 (MSB) + Byte0 (LSB)
            x_axis_out <= {rx_payload_in[15:8], rx_payload_in[7:0]};

            // Y Axis: Byte3 (MSB) + Byte2 (LSB)  
            y_axis_out <= {rx_payload_in[31:24], rx_payload_in[23:16]};

            // Z Axis: Byte5 (MSB) + Byte4 (LSB)
            z_axis_out <= {rx_payload_in[47:40], rx_payload_in[39:32]};

            packet_ready <= 1'b1; // Signal that assembled data is ready

            $display("[Assembler] Payload received: 0x%h", rx_payload_in);
            $display("[Assembler] Output: X=0x%h(%d) Y=0x%h(%d) Z=0x%h(%d)", 
                     {rx_payload_in[15:8], rx_payload_in[7:0]},
                     {rx_payload_in[15:8], rx_payload_in[7:0]},
                     {rx_payload_in[31:24], rx_payload_in[23:16]}, 
                     {rx_payload_in[31:24], rx_payload_in[23:16]}, 
                     {rx_payload_in[47:40], rx_payload_in[39:32]},
                     {rx_payload_in[47:40], rx_payload_in[39:32]});
        end
    end
end
endmodule