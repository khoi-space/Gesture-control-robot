module nrf24l01_rx_controller #(
    parameter USE_IRQ = 1,
    parameter [23:0] RX_POLL_INTERVAL = 24'd12_500_000,
    parameter [39:0] PIPE0_ADDRESS = 40'hE7E7E7E7E7,
    parameter [7:0] RF_CHANNEL = 8'h4C,
    parameter [7:0] RF_SETUP_VALUE = 8'h26,
    parameter integer PAYLOAD_BYTES = 6,
    parameter integer INITIAL_DELAY_COUNT = 27'd625000,
    parameter integer POWERUP_DELAY_COUNT = 27'd625000,
    parameter integer WATCHDOG_TIMEOUT_COUNT = 32'd375000000 // ~3 s at 125 MHz; adjust if clk differs
) (
    input clk,
    input rst_n,
    input start_rx,
    output reg rx_ready,
    output reg nrf_ce,
    output reg nrf_csn,
    input nrf_irq,
    output wire spi_sck,
    output wire spi_mosi,
    input spi_miso,
    output reg [(PAYLOAD_BYTES*8)-1:0] rx_payload,
    output reg payload_ready
);

`include "nrf24l01_rx_defines.v"

    reg [5:0] current_state;
    reg [5:0] next_after_csn;
    reg [26:0] delay_counter;
    reg [2:0] addr_byte_index;
    reg [3:0] payload_byte_index;
    reg spi_busy;
    reg [7:0] fifo_status_reg;
    reg [23:0] rx_poll_counter;
    reg [31:0] watchdog_counter;
    reg watchdog_active;

    reg spi_start;
    reg [7:0] spi_data_in;
    wire spi_done;
    wire [7:0] spi_data_out;

    function [7:0] pipe0_addr_byte;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: pipe0_addr_byte = PIPE0_ADDRESS[7:0];
                3'd1: pipe0_addr_byte = PIPE0_ADDRESS[15:8];
                3'd2: pipe0_addr_byte = PIPE0_ADDRESS[23:16];
                3'd3: pipe0_addr_byte = PIPE0_ADDRESS[31:24];
                default: pipe0_addr_byte = PIPE0_ADDRESS[39:32];
            endcase
        end
    endfunction

    spi_master spi_master_inst (
        .clk(clk),
        .rst_n(rst_n),
        .spi_clk_div(8'd250),
        .start_transfer(spi_start),
        .data_in(spi_data_in),
        .transfer_done(spi_done),
        .data_out(spi_data_out),
        .spi_sck(spi_sck),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= STATE_IDLE;
            next_after_csn <= STATE_IDLE;
            delay_counter <= 27'd0;
            addr_byte_index <= 3'd0;
            payload_byte_index <= 4'd0;
            spi_busy <= 1'b0;
            fifo_status_reg <= 8'h00;
            rx_poll_counter <= 24'd0;
            spi_start <= 1'b0;
            spi_data_in <= 8'h00;
            rx_payload <= {(PAYLOAD_BYTES*8){1'b0}};
            payload_ready <= 1'b0;
            rx_ready <= 1'b0;
            nrf_ce <= 1'b0;
            nrf_csn <= 1'b1;
            watchdog_counter <= 32'd0;
            watchdog_active <= 1'b0;
        end else begin
            spi_start <= 1'b0;
            payload_ready <= 1'b0;

            if (current_state == STATE_READY) begin
                watchdog_active <= 1'b1;
            end else if (current_state == STATE_FORCE_RESET || current_state == STATE_IDLE) begin
                watchdog_active <= 1'b0;
            end

            if (watchdog_active) begin
                if (watchdog_counter < WATCHDOG_TIMEOUT_COUNT) begin
                    watchdog_counter <= watchdog_counter + 1'b1;
                end
            end else begin
                watchdog_counter <= 32'd0;
            end

            case (current_state)
                STATE_IDLE: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b1;
                    rx_ready <= 1'b0;
                    rx_poll_counter <= 24'd0;
                    if (start_rx) begin
                        delay_counter <= 27'd0;
                        current_state <= STATE_INIT_DELAY;
                    end
                end

                STATE_INIT_DELAY: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b1;
                    if (delay_counter < INITIAL_DELAY_COUNT) begin
                        delay_counter <= delay_counter + 1'b1;
                    end else begin
                        current_state <= STATE_WRITE_CONFIG0_CMD;
                    end
                end

                STATE_WRITE_CONFIG0_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_CONFIG;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_CONFIG0_DATA;
                    end
                end

                STATE_WRITE_CONFIG0_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_CONFIG_BASE;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_SETUP_RETR_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_SETUP_RETR_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_SETUP_RETR;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_SETUP_RETR_DATA;
                    end
                end

                STATE_WRITE_SETUP_RETR_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_SETUP_RETR;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_RF_SETUP_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_RF_SETUP_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_RF_SETUP;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_RF_SETUP_DATA;
                    end
                end

                STATE_WRITE_RF_SETUP_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= RF_SETUP_VALUE;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_TOGGLE_FEATURES_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_TOGGLE_FEATURES_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_ACTIVATE;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_TOGGLE_FEATURES_DATA;
                    end
                end

                STATE_TOGGLE_FEATURES_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= 8'h73;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_FEATURE_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_FEATURE_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_FEATURE;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_FEATURE_DATA;
                    end
                end

                STATE_WRITE_FEATURE_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= 8'h00;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_DYNPD_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_DYNPD_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_DYNPD;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_DYNPD_DATA;
                    end
                end

                STATE_WRITE_DYNPD_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= 8'h00;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_EN_AA_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_EN_AA_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_EN_AA;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_EN_AA_DATA;
                    end
                end

                STATE_WRITE_EN_AA_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_EN_AA_NONE;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_EN_RXADDR_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_EN_RXADDR_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_EN_RXADDR;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_EN_RXADDR_DATA;
                    end
                end

                STATE_WRITE_EN_RXADDR_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_EN_RXADDR_P0;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_SETUP_AW_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_SETUP_AW_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_SETUP_AW;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_SETUP_AW_DATA;
                    end
                end

                STATE_WRITE_SETUP_AW_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_SETUP_AW_5B;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_RF_CH_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_RF_CH_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_RF_CH;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_RF_CH_DATA;
                    end
                end

                STATE_WRITE_RF_CH_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= RF_CHANNEL;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_RX_PW_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_RX_PW_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_RX_PW_P0;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_RX_PW_DATA;
                    end
                end

                STATE_WRITE_RX_PW_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= PAYLOAD_BYTES[7:0];
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_RX_ADDR_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_RX_ADDR_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_RX_ADDR_P0;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        addr_byte_index <= 3'd0;
                        current_state <= STATE_WRITE_RX_ADDR_BYTE;
                    end
                end

                STATE_WRITE_RX_ADDR_BYTE: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= pipe0_addr_byte(addr_byte_index);
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        if (addr_byte_index == 3'd4) begin
                            next_after_csn <= STATE_WRITE_STATUS_CMD;
                            current_state <= STATE_RAISE_CSN;
                        end else begin
                            addr_byte_index <= addr_byte_index + 1'b1;
                        end
                    end
                end

                STATE_WRITE_STATUS_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_STATUS;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_STATUS_DATA;
                    end
                end

                STATE_WRITE_STATUS_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_STATUS_CLEAR;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_FLUSH_RX_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_FLUSH_RX_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_FLUSH_RX;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_FLUSH_TX_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_FLUSH_TX_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_FLUSH_TX;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_CONFIG_PWR_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_CONFIG_PWR_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_CONFIG;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_CONFIG_PWR_DATA;
                    end
                end

                STATE_WRITE_CONFIG_PWR_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_CONFIG_PWRUP;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        delay_counter <= 27'd0;
                        next_after_csn <= STATE_POWERUP_DELAY;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_POWERUP_DELAY: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b1;
                    if (delay_counter < POWERUP_DELAY_COUNT) begin
                        delay_counter <= delay_counter + 1'b1;
                    end else begin
                        current_state <= STATE_WRITE_CONFIG_RX_CMD;
                    end
                end

                STATE_WRITE_CONFIG_RX_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_CONFIG;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_CONFIG_RX_DATA;
                    end
                end

                STATE_WRITE_CONFIG_RX_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_CONFIG_RX;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_WRITE_STATUS_RX_CMD;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_WRITE_STATUS_RX_CMD: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_STATUS;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_WRITE_STATUS_RX_DATA;
                    end
                end

                STATE_WRITE_STATUS_RX_DATA: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_STATUS_CLEAR;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        rx_poll_counter <= 24'd0;
                        next_after_csn <= STATE_READY;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_READY: begin
                    nrf_ce <= 1'b1;
                    nrf_csn <= 1'b1;
                    rx_ready <= 1'b1;
                    if (watchdog_counter >= WATCHDOG_TIMEOUT_COUNT) begin
                        nrf_ce <= 1'b0;
                        rx_ready <= 1'b0;
                        rx_poll_counter <= 24'd0;
                        current_state <= STATE_FORCE_RESET;
                    end else begin
                        rx_poll_counter <= USE_IRQ ? 24'd0 : rx_poll_counter + 1'b1;
                        if (USE_IRQ) begin
                            if (!nrf_irq) begin
                                current_state <= STATE_READ_PAYLOAD_CMD;
                            end
                        end else begin
                            if (rx_poll_counter >= RX_POLL_INTERVAL) begin
                                rx_poll_counter <= 24'd0;
                                current_state <= STATE_POLL_FIFO_CMD;
                            end
                        end
                    end
                end

                STATE_POLL_FIFO_CMD: begin
                    nrf_ce <= 1'b1;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_R_REGISTER | REG_FIFO_STATUS;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_POLL_FIFO_DATA;
                    end
                end

                STATE_POLL_FIFO_DATA: begin
                    nrf_ce <= 1'b1;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_NOP;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        fifo_status_reg <= spi_data_out;
                        if (!spi_data_out[0]) begin
                            next_after_csn <= STATE_READ_PAYLOAD_CMD;
                        end else begin
                            next_after_csn <= STATE_READY;
                        end
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_READ_PAYLOAD_CMD: begin
                    nrf_ce <= 1'b1;
                    nrf_csn <= 1'b0;
                    rx_poll_counter <= 24'd0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_R_RX_PAYLOAD;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        payload_byte_index <= 4'd0;
                        current_state <= STATE_READ_PAYLOAD_BYTE;
                    end
                end

                STATE_READ_PAYLOAD_BYTE: begin
                    nrf_ce <= 1'b1;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_NOP;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        rx_payload[(payload_byte_index*8)+:8] <= spi_data_out;
                        if (payload_byte_index == PAYLOAD_BYTES - 1) begin
                            payload_ready <= 1'b1;
                            watchdog_counter <= 32'd0;
                            next_after_csn <= STATE_CLEAR_IRQ_CMD;
                            current_state <= STATE_RAISE_CSN;
                        end else begin
                            payload_byte_index <= payload_byte_index + 1'b1;
                        end
                    end
                end

                STATE_CLEAR_IRQ_CMD: begin
                    nrf_ce <= 1'b1;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= CMD_W_REGISTER | REG_STATUS;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        current_state <= STATE_CLEAR_IRQ_DATA;
                    end
                end

                STATE_CLEAR_IRQ_DATA: begin
                    nrf_ce <= 1'b1;
                    nrf_csn <= 1'b0;
                    if (!spi_busy) begin
                        spi_data_in <= VAL_STATUS_RX_DR;
                        spi_start <= 1'b1;
                        spi_busy <= 1'b1;
                    end else if (spi_done) begin
                        spi_busy <= 1'b0;
                        next_after_csn <= STATE_READY;
                        current_state <= STATE_RAISE_CSN;
                    end
                end

                STATE_FORCE_RESET: begin
                    nrf_ce <= 1'b0;
                    nrf_csn <= 1'b1;
                    rx_ready <= 1'b0;
                    spi_busy <= 1'b0;
                    rx_poll_counter <= 24'd0;
                    delay_counter <= 27'd0;
                    addr_byte_index <= 3'd0;
                    payload_byte_index <= 4'd0;
                    current_state <= STATE_INIT_DELAY;
                end

                STATE_RAISE_CSN: begin
                    nrf_csn <= 1'b1;
                    spi_busy <= 1'b0;
                    current_state <= next_after_csn;
                end

                default: begin
                    current_state <= STATE_IDLE;
                end
            endcase

            if (watchdog_active && watchdog_counter >= WATCHDOG_TIMEOUT_COUNT && current_state != STATE_FORCE_RESET) begin
                watchdog_active <= 1'b0;
                watchdog_counter <= 32'd0;
                nrf_ce <= 1'b0;
                nrf_csn <= 1'b1;
                rx_ready <= 1'b0;
                spi_busy <= 1'b0;
                rx_poll_counter <= 24'd0;
                current_state <= STATE_FORCE_RESET;
            end

            if (current_state != STATE_READY) begin
                rx_ready <= 1'b0;
            end

            if (current_state != STATE_READ_PAYLOAD_BYTE && current_state != STATE_READY) begin
                rx_poll_counter <= USE_IRQ ? 24'd0 : rx_poll_counter;
            end
        end
    end
endmodule