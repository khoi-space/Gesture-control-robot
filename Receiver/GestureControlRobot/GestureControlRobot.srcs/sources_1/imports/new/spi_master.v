module spi_master (
    input clk,
    input rst_n,
    input [7:0] spi_clk_div,
    input start_transfer,
    output reg transfer_done,
    input [7:0] data_in,
    output reg [7:0] data_out,
    output reg spi_sck,
    output reg spi_mosi,
    input spi_miso
);

    localparam STATE_IDLE     = 2'd0;
    localparam STATE_TRANSFER = 2'd1;

    reg [1:0] state;
    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    reg [7:0] clk_div_counter;
    reg [3:0] bit_counter;

    wire [7:0] effective_div = (spi_clk_div == 8'd0) ? 8'd1 : spi_clk_div;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_IDLE;
            transfer_done <= 1'b0;
            tx_shift <= 8'h00;
            rx_shift <= 8'h00;
            data_out <= 8'h00;
            clk_div_counter <= 8'h00;
            bit_counter <= 4'd0;
            spi_sck <= 1'b0;
            spi_mosi <= 1'b0;
        end else begin
            transfer_done <= 1'b0;

            case (state)
                STATE_IDLE: begin
                    spi_sck <= 1'b0;
                    spi_mosi <= 1'b0;
                    clk_div_counter <= 8'h00;
                    bit_counter <= 4'd0;

                    if (start_transfer) begin
                        tx_shift <= data_in;
                        rx_shift <= 8'h00;
                        spi_mosi <= data_in[7];
                        state <= STATE_TRANSFER;
                    end
                end

                STATE_TRANSFER: begin
                    if (clk_div_counter == (effective_div - 1)) begin
                        clk_div_counter <= 8'h00;

                        if (spi_sck == 1'b0) begin
                            spi_sck <= 1'b1;
                            rx_shift <= {rx_shift[6:0], spi_miso};
                            if (bit_counter == 4'd7) begin
                                data_out <= {rx_shift[6:0], spi_miso};
                            end
                            bit_counter <= bit_counter + 1'b1;
                        end else begin
                            spi_sck <= 1'b0;
                            if (bit_counter == 4'd8) begin
                                transfer_done <= 1'b1;
                                spi_mosi <= 1'b0;
                                state <= STATE_IDLE;
                            end else begin
                                tx_shift <= {tx_shift[6:0], 1'b0};
                                spi_mosi <= tx_shift[6];
                            end
                        end
                    end else begin
                        clk_div_counter <= clk_div_counter + 1'b1;
                    end
                end

                default: begin
                    state <= STATE_IDLE;
                end
            endcase
        end
    end
endmodule