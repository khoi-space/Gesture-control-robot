`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Motor Controller Module for Arty Z7
// Replaces Arduino L298N motor control logic
// Controls 2 DC motors based on gesture sensor data (X, Y, Z axis)
//////////////////////////////////////////////////////////////////////////////////

module motor_controller #(
    parameter CLK_FREQ = 100_000_000,     // 100MHz system clock
    parameter PWM_FREQ = 1000,            // 1kHz PWM frequency
    parameter TIMEOUT_MS = 2000           // 2 second timeout for connection loss
)(
    input clk,
    input rst_n,
    
    // Gesture data inputs from payload assembler
    input [15:0] x_axis,
    input [15:0] y_axis,
    input [15:0] z_axis,
    input data_valid,                      // Pulse when new data arrives
    
    // Motor control outputs
    output reg motor_a1,                   // IN1 - Motor A direction 1
    output reg motor_a2,                   // IN2 - Motor A direction 2
    output reg motor_b1,                   // IN3 - Motor B direction 1
    output reg motor_b2,                   // IN4 - Motor B direction 2
    output wire pwm_ena,                   // ENA - Motor A PWM speed control
    output wire pwm_enb                    // ENB - Motor B PWM speed control
);

    //=========================================================================
    // PWM Generator Parameters
    //=========================================================================
    localparam PWM_PERIOD = CLK_FREQ / PWM_FREQ;  // Number of clock cycles per PWM period
    localparam TIMEOUT_CYCLES = (CLK_FREQ / 1000) * TIMEOUT_MS;  // Timeout in clock cycles
    
    //=========================================================================
    // Internal Registers
    //=========================================================================
    reg [31:0] pwm_counter;               // PWM period counter
    reg [7:0] duty_cycle_a;               // Motor A duty cycle (0-255)
    reg [7:0] duty_cycle_b;               // Motor B duty cycle (0-255)
    reg [31:0] last_data_timer;           // Timer for connection timeout
    reg connection_active;                 // Flag indicating active connection
    
    // Latched gesture values - keep last valid data
    reg [15:0] x_axis_latched;
    reg [15:0] y_axis_latched;
    reg [15:0] z_axis_latched;
    reg data_valid_delayed;  // Delayed flag to trigger motor update
    reg data_valid_delayed2; // Extra delay for display to show correct values
    
    //=========================================================================
    // PWM Generation Logic
    //=========================================================================
    // Convert duty cycle (0-255) to PWM threshold
    wire [31:0] threshold_a = (PWM_PERIOD * duty_cycle_a) / 255;
    wire [31:0] threshold_b = (PWM_PERIOD * duty_cycle_b) / 255;
    
    // PWM counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_counter <= 32'd0;
        end else begin
            if (pwm_counter >= PWM_PERIOD - 1)
                pwm_counter <= 32'd0;
            else
                pwm_counter <= pwm_counter + 1'b1;
        end
    end
    
    // Generate PWM signals
    assign pwm_ena = (pwm_counter < threshold_a) ? 1'b1 : 1'b0;
    assign pwm_enb = (pwm_counter < threshold_b) ? 1'b1 : 1'b0;
    
    //=========================================================================
    // Latch Gesture Data & Connection Timeout Logic
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_data_timer <= 32'd0;
            connection_active <= 1'b0;
            x_axis_latched <= 16'd350;  // Neutral position
            y_axis_latched <= 16'd350;
            z_axis_latched <= 16'd350;
            data_valid_delayed <= 1'b0;
            data_valid_delayed2 <= 1'b0;
        end else begin
            // Delay data_valid by 1 clock to ensure latched values are ready
            data_valid_delayed <= data_valid;
            // Extra delay for display to show updated motor outputs
            data_valid_delayed2 <= data_valid_delayed;
            
            if (data_valid) begin
                // Latch new gesture data
                x_axis_latched <= x_axis;
                y_axis_latched <= y_axis;
                z_axis_latched <= z_axis;
                
                // Reset timer when new data arrives
                last_data_timer <= 32'd0;
                connection_active <= 1'b1;
            end else if (last_data_timer < TIMEOUT_CYCLES) begin
                last_data_timer <= last_data_timer + 1'b1;
            end else begin
                // Timeout - connection lost
                connection_active <= 1'b0;
            end
        end
    end
    
    //=========================================================================
    // Motor Control State Machine
    // Based on Arduino GestureReceiver logic
    //=========================================================================
    
    // Gesture thresholds (matching Arduino code - CALIBRATED FROM REAL MEASUREMENT)
    localparam FORWARD_THRESHOLD  = 16'd395;   // Y > 395
    localparam FORWARD_MAX        = 16'd420;   // Y >= 420 (max speed)
    localparam BACKWARD_THRESHOLD = 16'd320;   // Y < 320
    localparam BACKWARD_MAX       = 16'd310;   // Y <= 310 (max speed)
    localparam LEFT_THRESHOLD     = 16'd320;   // X < 320
    localparam RIGHT_THRESHOLD    = 16'd395;   // X > 395
    
    // STOP zone (neutral position from real measurement)
    localparam STOP_X_MIN = 16'd320;
    localparam STOP_X_MAX = 16'd395;
    localparam STOP_Y_MIN = 16'd320;
    localparam STOP_Y_MAX = 16'd395;
    
    // Z-axis spin control
    localparam Z_SPIN_THRESHOLD = 16'd395;     // Z > 395 to spin
    localparam SPIN_SPEED = 8'd150;            // Speed for spinning in place
    // LEFT/RIGHT differential speed parameters
    localparam TURN_SPEED_LOW  = 8'd80;        // Slow wheel for turning
    localparam TURN_SPEED_HIGH = 8'd220;       // Fast wheel for turning
    
    // Motor control logic - continuously evaluate latched gesture data
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            motor_a1 <= 1'b0;
            motor_a2 <= 1'b0;
            motor_b1 <= 1'b0;
            motor_b2 <= 1'b0;
            duty_cycle_a <= 8'd0;
            duty_cycle_b <= 8'd0;
        end else begin
            if (!connection_active) begin
                // STOP - Connection lost or no data
                motor_a1 <= 1'b0;
                motor_a2 <= 1'b0;
                motor_b1 <= 1'b0;
                motor_b2 <= 1'b0;
                duty_cycle_a <= 8'd0;
                duty_cycle_b <= 8'd0;
                
            end else begin
                // Continuously evaluate latched gesture data - CALIBRATED LOGIC
                // Priority order: Z-SPIN > FORWARD > BACKWARD > LEFT > RIGHT > STOP
                
                if (z_axis_latched > Z_SPIN_THRESHOLD) begin
                    //---------------------------------------------
                    // Z-AXIS SPIN - Z > 395 (spin counter-clockwise, HIGHEST PRIORITY)
                    //---------------------------------------------
                    motor_a1 <= 1'b1;  // Motor A backward
                    motor_a2 <= 1'b0;
                    motor_b1 <= 1'b1;  // Motor B forward
                    motor_b2 <= 1'b0;
                    duty_cycle_a <= SPIN_SPEED;  // Both wheels 150
                    duty_cycle_b <= SPIN_SPEED;
                    
                end else if (y_axis_latched > FORWARD_THRESHOLD) begin
                    //---------------------------------------------
                    // FORWARD - Y > 395 (straight forward only)
                    //---------------------------------------------
                    motor_a1 <= 1'b0;
                    motor_a2 <= 1'b1;
                    motor_b1 <= 1'b1;
                    motor_b2 <= 1'b0;
                    
                    // Variable speed based on Y-axis
                    if (y_axis_latched >= FORWARD_MAX) begin
                        duty_cycle_a <= 8'd255;
                        duty_cycle_b <= 8'd255;
                    end else begin
                        duty_cycle_a <= 8'd100 + ((y_axis_latched - FORWARD_THRESHOLD) * 155 / 25);
                        duty_cycle_b <= 8'd100 + ((y_axis_latched - FORWARD_THRESHOLD) * 155 / 25);
                    end
                    
                end else if (y_axis_latched < BACKWARD_THRESHOLD) begin
                    //---------------------------------------------
                    // BACKWARD - Y < 320 (tốc độ biến đổi)
                    //---------------------------------------------
                    motor_a1 <= 1'b1;
                    motor_a2 <= 1'b0;
                    motor_b1 <= 1'b0;
                    motor_b2 <= 1'b1;
                    
                    // Variable speed: 100 to 255 (inverse mapping)
                    if (y_axis_latched <= BACKWARD_MAX) begin
                        duty_cycle_a <= 8'd255;  // Max speed
                        duty_cycle_b <= 8'd255;
                    end else begin
                        // Map Y(310-320) to speed(255-100) - inverse
                        duty_cycle_a <= 8'd255 - ((y_axis_latched - BACKWARD_MAX) * 155 / 10);
                        duty_cycle_b <= 8'd255 - ((y_axis_latched - BACKWARD_MAX) * 155 / 10);
                    end
                    
                end else if (x_axis_latched < LEFT_THRESHOLD) begin
                    //---------------------------------------------
                    // LEFT - X < 320 (differential speed turn)
                    //---------------------------------------------
                    motor_a1 <= 1'b0;
                    motor_a2 <= 1'b1;
                    motor_b1 <= 1'b1;
                    motor_b2 <= 1'b0;
                    duty_cycle_a <= TURN_SPEED_LOW;   // Motor A slow (80)
                    duty_cycle_b <= TURN_SPEED_HIGH;  // Motor B fast (220)
                    
                end else if (x_axis_latched > RIGHT_THRESHOLD) begin
                    //---------------------------------------------
                    // RIGHT - X > 395 (differential speed turn)
                    //---------------------------------------------
                    motor_a1 <= 1'b0;
                    motor_a2 <= 1'b1;
                    motor_b1 <= 1'b1;
                    motor_b2 <= 1'b0;
                    duty_cycle_a <= TURN_SPEED_HIGH;  // Motor A fast (220)
                    duty_cycle_b <= TURN_SPEED_LOW;   // Motor B slow (80)
                    
                end else begin
                    //---------------------------------------------
                    // STOP - Neutral zone or outside all zones (LOWEST PRIORITY)
                    //---------------------------------------------
                    motor_a1 <= 1'b0;
                    motor_a2 <= 1'b0;
                    motor_b1 <= 1'b0;
                    motor_b2 <= 1'b0;
                    duty_cycle_a <= 8'd0;
                    duty_cycle_b <= 8'd0;
                end
            end
        end
    end
    
    //=========================================================================
    // Debug Display (for simulation)
    //=========================================================================
    always @(posedge clk) begin
        if (data_valid_delayed2 && connection_active) begin
            $display("[Motor] X=%d Y=%d Z=%d", x_axis_latched, y_axis_latched, z_axis_latched);
            
            if (y_axis_latched > FORWARD_THRESHOLD)
                $display("[Motor] Action: FORWARD, Speed: ENA=%d ENB=%d", duty_cycle_a, duty_cycle_b);
            else if (y_axis_latched < BACKWARD_THRESHOLD)
                $display("[Motor] Action: BACKWARD, Speed: ENA=%d ENB=%d", duty_cycle_a, duty_cycle_b);
            else if (x_axis_latched < LEFT_THRESHOLD)
                $display("[Motor] Action: LEFT");
            else if (x_axis_latched > RIGHT_THRESHOLD)
                $display("[Motor] Action: RIGHT");
            else
                $display("[Motor] Action: STOP");
        end
    end

endmodule
