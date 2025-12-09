`timescale 1ns / 1ps

module data_parser (
    input wire clk,
    input wire reset,
    input wire [7:0] payload_0,
    input wire [7:0] payload_1,
    input wire [7:0] payload_2,
    input wire [7:0] payload_3,
    input wire [7:0] payload_4,
    input wire [7:0] payload_5,
    input wire data_ready,
    output reg [15:0] accel_x,
    output reg [15:0] accel_y,
    output reg [15:0] accel_z,
    output reg valid
);
    always @(posedge clk) begin
        if (reset) begin
            accel_x <= 0;
            accel_y <= 0;
            accel_z <= 0;
            valid <= 0;
        end else if (data_ready) begin
            // Arduino sends int (16-bit) as little-endian: low byte first, high byte second
            // Match Arduino struct data: xAxis, yAxis, zAxis (each 2 bytes)
            accel_x <= {payload_0, payload_1}; // X: payload_0 (low) + payload_1 (high)
            accel_y <= {payload_2, payload_3}; // Y: payload_2 (low) + payload_3 (high)  
            accel_z <= {payload_4, payload_5}; // Z: payload_4 (low) + payload_5 (high)
            valid <= 1;
        end else begin
            valid <= 0;
        end
    end
endmodule