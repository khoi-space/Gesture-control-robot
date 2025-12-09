`ifndef NRF24L01_RX_DEFINES_V
`define NRF24L01_RX_DEFINES_V

	// SPI command opcodes
	localparam [7:0] CMD_R_REGISTER   = 8'h00;
	localparam [7:0] CMD_W_REGISTER   = 8'h20;
	localparam [7:0] CMD_R_RX_PAYLOAD = 8'h61;
	localparam [7:0] CMD_FLUSH_TX     = 8'hE1;
	localparam [7:0] CMD_FLUSH_RX     = 8'hE2;
	localparam [7:0] CMD_NOP          = 8'hFF;
	localparam [7:0] CMD_ACTIVATE     = 8'h50;

	// Register addresses
	localparam [7:0] REG_CONFIG       = 8'h00;
	localparam [7:0] REG_EN_AA        = 8'h01;
	localparam [7:0] REG_EN_RXADDR    = 8'h02;
	localparam [7:0] REG_SETUP_AW     = 8'h03;
	localparam [7:0] REG_SETUP_RETR   = 8'h04;
	localparam [7:0] REG_RF_CH        = 8'h05;
	localparam [7:0] REG_RF_SETUP     = 8'h06;
	localparam [7:0] REG_STATUS       = 8'h07;
	localparam [7:0] REG_RX_ADDR_P0   = 8'h0A;
	localparam [7:0] REG_RX_PW_P0     = 8'h11;
	localparam [7:0] REG_FIFO_STATUS  = 8'h17;
	localparam [7:0] REG_DYNPD        = 8'h1C;
	localparam [7:0] REG_FEATURE      = 8'h1D;

	// Register default values
	localparam [7:0] VAL_CONFIG_BASE  = 8'h0C;
	localparam [7:0] VAL_CONFIG_PWRUP = 8'h0E;
	localparam [7:0] VAL_CONFIG_RX    = 8'h0F;
	localparam [7:0] VAL_STATUS_CLEAR = 8'h70;
	localparam [7:0] VAL_STATUS_RX_DR = 8'h40;
	localparam [7:0] VAL_EN_AA_NONE   = 8'h00;
	localparam [7:0] VAL_EN_RXADDR_P0 = 8'h01;
	localparam [7:0] VAL_SETUP_AW_5B  = 8'h03;
	localparam [7:0] VAL_SETUP_RETR   = 8'h5F;

	// State encoding
	localparam [5:0] STATE_IDLE                 = 6'd0;
	localparam [5:0] STATE_INIT_DELAY           = 6'd1;
	localparam [5:0] STATE_WRITE_CONFIG0_CMD    = 6'd2;
	localparam [5:0] STATE_WRITE_CONFIG0_DATA   = 6'd3;
	localparam [5:0] STATE_WRITE_SETUP_RETR_CMD = 6'd4;
	localparam [5:0] STATE_WRITE_SETUP_RETR_DATA= 6'd5;
	localparam [5:0] STATE_WRITE_RF_SETUP_CMD   = 6'd6;
	localparam [5:0] STATE_WRITE_RF_SETUP_DATA  = 6'd7;
	localparam [5:0] STATE_TOGGLE_FEATURES_CMD  = 6'd8;
	localparam [5:0] STATE_TOGGLE_FEATURES_DATA = 6'd9;
	localparam [5:0] STATE_WRITE_FEATURE_CMD    = 6'd10;
	localparam [5:0] STATE_WRITE_FEATURE_DATA   = 6'd11;
	localparam [5:0] STATE_WRITE_DYNPD_CMD      = 6'd12;
	localparam [5:0] STATE_WRITE_DYNPD_DATA     = 6'd13;
	localparam [5:0] STATE_WRITE_EN_AA_CMD      = 6'd14;
	localparam [5:0] STATE_WRITE_EN_AA_DATA     = 6'd15;
	localparam [5:0] STATE_WRITE_EN_RXADDR_CMD  = 6'd16;
	localparam [5:0] STATE_WRITE_EN_RXADDR_DATA = 6'd17;
	localparam [5:0] STATE_WRITE_SETUP_AW_CMD   = 6'd18;
	localparam [5:0] STATE_WRITE_SETUP_AW_DATA  = 6'd19;
	localparam [5:0] STATE_WRITE_RF_CH_CMD      = 6'd20;
	localparam [5:0] STATE_WRITE_RF_CH_DATA     = 6'd21;
	localparam [5:0] STATE_WRITE_RX_PW_CMD      = 6'd22;
	localparam [5:0] STATE_WRITE_RX_PW_DATA     = 6'd23;
	localparam [5:0] STATE_WRITE_RX_ADDR_CMD    = 6'd24;
	localparam [5:0] STATE_WRITE_RX_ADDR_BYTE   = 6'd25;
	localparam [5:0] STATE_WRITE_STATUS_CMD     = 6'd26;
	localparam [5:0] STATE_WRITE_STATUS_DATA    = 6'd27;
	localparam [5:0] STATE_FLUSH_RX_CMD         = 6'd28;
	localparam [5:0] STATE_FLUSH_TX_CMD         = 6'd29;
	localparam [5:0] STATE_WRITE_CONFIG_PWR_CMD = 6'd30;
	localparam [5:0] STATE_WRITE_CONFIG_PWR_DATA= 6'd31;
	localparam [5:0] STATE_POWERUP_DELAY        = 6'd32;
	localparam [5:0] STATE_WRITE_CONFIG_RX_CMD  = 6'd33;
	localparam [5:0] STATE_WRITE_CONFIG_RX_DATA = 6'd34;
	localparam [5:0] STATE_WRITE_STATUS_RX_CMD  = 6'd35;
	localparam [5:0] STATE_WRITE_STATUS_RX_DATA = 6'd36;
	localparam [5:0] STATE_READY                = 6'd37;
	localparam [5:0] STATE_POLL_FIFO_CMD        = 6'd38;
	localparam [5:0] STATE_POLL_FIFO_DATA       = 6'd39;
	localparam [5:0] STATE_READ_PAYLOAD_CMD     = 6'd40;
	localparam [5:0] STATE_READ_PAYLOAD_BYTE    = 6'd41;
	localparam [5:0] STATE_CLEAR_IRQ_CMD        = 6'd42;
	localparam [5:0] STATE_CLEAR_IRQ_DATA       = 6'd43;
	localparam [5:0] STATE_FORCE_RESET          = 6'd44;
	localparam [5:0] STATE_RAISE_CSN            = 6'd63;

`endif // NRF24L01_RX_DEFINES_V