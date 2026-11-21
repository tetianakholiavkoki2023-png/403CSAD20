`timescale 1ns/1ps

// IEEE 1364-2001 I2C master wrapper.
// It starts the Tx/Slave-address phase and either performs one write byte or
// starts Rx for one read byte. SDA and SCL are open-drain controls.
module i2c_master (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_i,
    input  wire [6:0] address_i,
    input  wire       rw_i,
    input  wire [7:0] data_i,
    output reg        busy_o,
    output reg        done_o,
    output reg        nack_o,
    output reg [7:0]  data_o,
    output reg        data_valid_o,
    input  wire       scl_i,
    input  wire       sda_i,
    output wire       scl_o,
    output wire       sda_o
);
    parameter CLK_DIV = 500;

    localparam [2:0] M_IDLE      = 3'd0;
    localparam [2:0] M_TX_START  = 3'd1;
    localparam [2:0] M_TX_WAIT   = 3'd2;
    localparam [2:0] M_RX_START  = 3'd3;
    localparam [2:0] M_RX_WAIT   = 3'd4;
    localparam [2:0] M_DONE      = 3'd5;

    reg [2:0] state;
    reg tx_start;
    reg rx_start;

    wire tx_busy;
    wire tx_done;
    wire tx_nack;
    wire tx_scl;
    wire tx_sda;

    wire rx_busy;
    wire rx_done;
    wire rx_nack;
    wire [7:0] rx_data;
    wire rx_valid;
    wire rx_scl;
    wire rx_sda;

    i2c_tx #(.CLK_DIV(CLK_DIV)) u_tx (
        .clk(clk),
        .rst_n(rst_n),
        .start_i(tx_start),
        .address_i(address_i),
        .rw_i(rw_i),
        .data_i(data_i),
        .busy_o(tx_busy),
        .done_o(tx_done),
        .nack_o(tx_nack),
        .scl_i(scl_i),
        .sda_i(sda_i),
        .scl_o(tx_scl),
        .sda_o(tx_sda)
    );

    i2c_rx #(.CLK_DIV(CLK_DIV)) u_rx (
        .clk(clk),
        .rst_n(rst_n),
        .start_i(rx_start),
        .slave_mode_i(1'b0),
        .own_address_i(address_i),
        .ack_enable_i(1'b0),
        .busy_o(rx_busy),
        .done_o(rx_done),
        .data_o(rx_data),
        .data_valid_o(rx_valid),
        .nack_o(rx_nack),
        .scl_i(scl_i),
        .sda_i(sda_i),
        .scl_o(rx_scl),
        .sda_o(rx_sda)
    );

    // Only the active engine controls the open-drain bus. Idle release is 1.
    assign scl_o = tx_busy ? tx_scl : (rx_busy ? rx_scl : 1'b1);
    assign sda_o = tx_busy ? tx_sda : (rx_busy ? rx_sda : 1'b1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= M_IDLE;
            tx_start <= 1'b0;
            rx_start <= 1'b0;
            busy_o <= 1'b0;
            done_o <= 1'b0;
            nack_o <= 1'b0;
            data_o <= 8'd0;
            data_valid_o <= 1'b0;
        end else begin
            done_o <= 1'b0;
            data_valid_o <= 1'b0;

            case (state)
                M_IDLE: begin
                    tx_start <= 1'b0;
                    rx_start <= 1'b0;
                    busy_o <= 1'b0;
                    nack_o <= 1'b0;
                    if (start_i) begin
                        busy_o <= 1'b1;
                        tx_start <= 1'b1;
                        state <= M_TX_START;
                    end
                end

                M_TX_START: begin
                    tx_start <= 1'b0;
                    state <= M_TX_WAIT;
                end

                M_TX_WAIT: begin
                    if (tx_nack)
                        nack_o <= 1'b1;

                    if (tx_done) begin
                        if (tx_nack) begin
                            busy_o <= 1'b0;
                            done_o <= 1'b1;
                            state <= M_DONE;
                        end else if (rw_i) begin
                            rx_start <= 1'b1;
                            state <= M_RX_START;
                        end else begin
                            busy_o <= 1'b0;
                            done_o <= 1'b1;
                            state <= M_DONE;
                        end
                    end
                end

                M_RX_START: begin
                    rx_start <= 1'b0;
                    state <= M_RX_WAIT;
                end

                M_RX_WAIT: begin
                    if (rx_valid) begin
                        data_o <= rx_data;
                        data_valid_o <= 1'b1;
                    end
                    if (rx_done) begin
                        busy_o <= 1'b0;
                        done_o <= 1'b1;
                        state <= M_DONE;
                    end
                end

                M_DONE: begin
                    if (!start_i)
                        state <= M_IDLE;
                end

                default: begin
                    state <= M_IDLE;
                    tx_start <= 1'b0;
                    rx_start <= 1'b0;
                    busy_o <= 1'b0;
                end
            endcase
        end
    end
endmodule
