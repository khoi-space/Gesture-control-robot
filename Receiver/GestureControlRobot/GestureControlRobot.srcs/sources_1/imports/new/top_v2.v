module top_v2 (
    input clk,
    input reset_btn,
    output nrf_ce,
    output nrf_csn,
    input nrf_irq,
    output nrf_sck,
    output nrf_mosi,
    input nrf_miso,
    output payload_ready,
    output [3:0] leds,
    
    // Motor control outputs
    output motor_a1,        // IN1 - Motor A direction 1
    output motor_a2,        // IN2 - Motor A direction 2
    output motor_b1,        // IN3 - Motor B direction 1
    output motor_b2,        // IN4 - Motor B direction 2
    output pwm_ena,         // ENA - Motor A PWM speed control
    output pwm_enb          // ENB - Motor B PWM speed control
);

    localparam integer RESET_FILTER_MAX = 20'hFFFFF;
    localparam integer STARTUP_DELAY_CYCLES = 9'd255;
    localparam [26:0] LED_HOLD_TICKS = 27'd1_000_000;
    // Expected payload in little-endian byte order (LSB-first from nRF24L01)
    localparam [47:0] EXPECTED_PAYLOAD = 48'h008C00780064;
    localparam [15:0] EXPECTED_X = 16'd100;
    localparam [15:0] EXPECTED_Y = 16'd120;
    localparam [15:0] EXPECTED_Z = 16'd140;
    localparam integer USE_IRQ = 0;

    wire rst_n;
    wire rx_ready;
    wire [47:0] rx_payload;
    wire payload_ready_pulse;
    wire [15:0] x_axis;
    wire [15:0] y_axis;
    wire [15:0] z_axis;
    wire packet_ready;

    reg [19:0] reset_counter;
    reg reset_sync;

    always @(posedge clk) begin
        if (reset_btn) begin
            reset_counter <= 20'h00000;
            reset_sync <= 1'b0;
        end else if (reset_counter < RESET_FILTER_MAX) begin
            reset_counter <= reset_counter + 1'b1;
        end else begin
            reset_sync <= 1'b1;
        end
    end

    assign rst_n = reset_sync;

    reg start_rx_reg;
    reg [8:0] startup_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_rx_reg <= 1'b0;
            startup_counter <= 9'd0;
        end else if (startup_counter < STARTUP_DELAY_CYCLES) begin
            startup_counter <= startup_counter + 1'b1;
            start_rx_reg <= 1'b0;
        end else if (startup_counter == STARTUP_DELAY_CYCLES) begin
            start_rx_reg <= 1'b1;
            startup_counter <= startup_counter + 1'b1;
        end else begin
            start_rx_reg <= 1'b0;
        end
    end

    nrf24l01_rx_controller #(
        .USE_IRQ(USE_IRQ)
    ) nrf_controller (
        .clk(clk),
        .rst_n(rst_n),
        .start_rx(start_rx_reg),
        .rx_ready(rx_ready),
        .nrf_ce(nrf_ce),
        .nrf_csn(nrf_csn),
        .nrf_irq(nrf_irq),
        .spi_sck(nrf_sck),
        .spi_mosi(nrf_mosi),
        .spi_miso(nrf_miso),
        .rx_payload(rx_payload),
        .payload_ready(payload_ready_pulse)
    );

    assign payload_ready = payload_ready_pulse;

    payload_assembler payload_assembler_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rx_payload_in(rx_payload),
        .data_valid(payload_ready_pulse),
        .x_axis_out(x_axis),
        .y_axis_out(y_axis),
        .z_axis_out(z_axis),
        .packet_ready(packet_ready)
    );

    reg [3:0] led_state;
    reg led0_latched;
    reg led1_latched;
    reg led2_latched;
    reg led3_latched;
    reg [26:0] led0_counter;
    reg [26:0] led1_counter;
    reg [26:0] led2_counter;
    reg [26:0] led3_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            led0_counter <= 27'd0;
            led1_counter <= 27'd0;
            led2_counter <= 27'd0;
            led3_counter <= 27'd0;
            led0_latched <= 1'b0;
            led1_latched <= 1'b0;
            led2_latched <= 1'b0;
            led3_latched <= 1'b0;
        end else begin
            if (led0_counter != 0) begin
                led0_counter <= led0_counter - 1'b1;
                if (led0_counter == 1)
                    led0_latched <= 1'b0;
            end

            if (led1_counter != 0) begin
                led1_counter <= led1_counter - 1'b1;
                if (led1_counter == 1)
                    led1_latched <= 1'b0;
            end

            if (led2_counter != 0) begin
                led2_counter <= led2_counter - 1'b1;
                if (led2_counter == 1)
                    led2_latched <= 1'b0;
            end

            if (led3_counter != 0) begin
                led3_counter <= led3_counter - 1'b1;
                if (led3_counter == 1)
                    led3_latched <= 1'b0;
            end

            if (payload_ready_pulse) begin
                led0_latched <= 1'b1;
                led0_counter <= LED_HOLD_TICKS;

                if (x_axis == EXPECTED_X) begin
                    led1_latched <= 1'b1;
                    led1_counter <= LED_HOLD_TICKS;
                end

                if (y_axis == EXPECTED_Y) begin
                    led2_latched <= 1'b1;
                    led2_counter <= LED_HOLD_TICKS;
                end

                if (z_axis == EXPECTED_Z) begin
                    led3_latched <= 1'b1;
                    led3_counter <= LED_HOLD_TICKS;
                end
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            led_state <= 4'b0000;
        end else begin
            led_state[0] <= led0_latched || (nrf_controller.current_state == 6'd44);
            led_state[1] <= led1_latched;
            led_state[2] <= led2_latched;
            led_state[3] <= led3_latched;
        end
    end

    assign leds = led_state;

    // =========================================================================
    // Payload Assembler - Extract X, Y, Z axis from 48-bit payload
    // =========================================================================
    wire [15:0] gesture_x;
    wire [15:0] gesture_y;
    wire [15:0] gesture_z;
    wire packet_ready;
    
    payload_assembler assembler_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rx_payload_in(rx_payload),
        .data_valid(payload_ready_pulse),
        .x_axis_out(gesture_x),
        .y_axis_out(gesture_y),
        .z_axis_out(gesture_z),
        .packet_ready(packet_ready)
    );

    // =========================================================================
    // Motor Controller - Control motors based on gesture data
    // =========================================================================
    motor_controller #(
        .CLK_FREQ(100_000_000),    // 100MHz Arty Z7 clock
        .PWM_FREQ(1000),           // 1kHz PWM frequency
        .TIMEOUT_MS(2000)          // 2 second timeout
    ) motor_ctrl_inst (
        .clk(clk),
        .rst_n(rst_n),
        .x_axis(gesture_x),
        .y_axis(gesture_y),
        .z_axis(gesture_z),
        .data_valid(packet_ready),
        .motor_a1(motor_a1),
        .motor_a2(motor_a2),
        .motor_b1(motor_b1),
        .motor_b2(motor_b2),
        .pwm_ena(pwm_ena),
        .pwm_enb(pwm_enb)
    );

endmodule
