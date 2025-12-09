`timescale 1ns / 1ps

// PWM Generator - matches Arduino analogWrite() functionality 
// Generates PWM signal with adjustable duty cycle (0-255)
// Compatible with motor driver speed control
module pwm_generator (
    input wire clk,           // System clock (100 MHz)
    input wire reset,         // Reset signal
    input wire [7:0] duty_cycle, // PWM duty cycle: 0=0%, 255=100%
    output reg pwm_out        // PWM output signal
);
    // Clock divider for PWM frequency control
    // 100MHz / 1024 / 256 ≈ 381 Hz PWM frequency (similar to Arduino)
    reg [9:0] clk_divider;    // Divides 100MHz clock by 1024
    reg [7:0] pwm_counter;    // 8-bit counter for PWM period (0-255)
    
    always @(posedge clk) begin
        if (reset) begin
            clk_divider <= 0;
            pwm_counter <= 0;
            pwm_out <= 0;
        end else begin
            // Clock division
            if (clk_divider == 1023) begin
                clk_divider <= 0;
                pwm_counter <= pwm_counter + 1;
            end else begin
                clk_divider <= clk_divider + 1;
            end
            
            // PWM generation: high when counter < duty_cycle
            // duty_cycle=0 → 0% duty, duty_cycle=255 → 100% duty  
            pwm_out <= (pwm_counter < duty_cycle) ? 1'b1 : 1'b0;
        end
    end
endmodule