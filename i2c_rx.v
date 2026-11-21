`timescale 1ns/1ps

// IEEE 1364-2001 I2C receiver.
// slave_mode_i = 1: receive 7-bit addressed writes and generate ACK/NACK.
// slave_mode_i = 0: master-reader mode; receive one byte, NACK it, and issue STOP.
// SDA/SCL outputs are open-drain controls: 0 = drive low, 1 = release.
module i2c_rx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_i,
    input  wire       slave_mode_i,
    input  wire [6:0] own_address_i,
    input  wire       ack_enable_i,
    output reg        busy_o,
    output reg        done_o,
    output reg [7:0]  data_o,
    output reg        data_valid_o,
    output reg        nack_o,
    input  wire       scl_i,
    input  wire       sda_i,
    output reg        scl_o,
    output reg        sda_o
);
    parameter CLK_DIV = 500;

    localparam [3:0] RX_IDLE       = 4'd0;
    localparam [3:0] RX_DATA       = 4'd1;
    localparam [3:0] RX_ACK        = 4'd2;
    localparam [3:0] RX_STOP       = 4'd3;
    localparam [3:0] RX_DONE       = 4'd4;

    // Slave states are kept separate from the master receive protocol.
    localparam [3:0] SL_IDLE       = 4'd8;
    localparam [3:0] SL_ADDRESS    = 4'd9;
    localparam [3:0] SL_ADDR_ACK   = 4'd10;
    localparam [3:0] SL_DATA       = 4'd11;
    localparam [3:0] SL_DATA_ACK   = 4'd12;

    reg [3:0]  state;
    reg [3:0]  phase;
    reg [31:0] div_count;
    reg [2:0]  bit_count;
    reg [7:0]  shift_reg;
    reg        scl_d1;
    reg        scl_d2;
    reg        sda_d1;
    reg        sda_d2;
    reg        address_match;

    wire tick;
    wire scl_rising;
    wire scl_falling;
    wire start_detect;
    wire stop_detect;

    assign tick = (div_count == ((CLK_DIV / 2) - 1));
    assign scl_rising  = (scl_d1 == 1'b1) && (scl_d2 == 1'b0);
    assign scl_falling = (scl_d1 == 1'b0) && (scl_d2 == 1'b1);
    assign start_detect = (scl_d1 == 1'b1) && (sda_d1 == 1'b0) && (sda_d2 == 1'b1);
    assign stop_detect  = (scl_d1 == 1'b1) && (sda_d1 == 1'b1) && (sda_d2 == 1'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= (slave_mode_i ? SL_IDLE : RX_IDLE);
            phase       <= 4'd0;
            div_count   <= 32'd0;
            bit_count   <= 3'd7;
            shift_reg   <= 8'd0;
            data_o      <= 8'd0;
            busy_o      <= 1'b0;
            done_o      <= 1'b0;
            data_valid_o <= 1'b0;
            nack_o      <= 1'b0;
            scl_o       <= 1'b1;
            sda_o       <= 1'b1;
            scl_d1      <= 1'b1;
            scl_d2      <= 1'b1;
            sda_d1      <= 1'b1;
            sda_d2      <= 1'b1;
            address_match <= 1'b0;
        end else begin
            done_o <= 1'b0;
            data_valid_o <= 1'b0;

            // Synchronizers for start/stop and SCL edge detection.
            scl_d2 <= scl_d1;
            scl_d1 <= scl_i;
            sda_d2 <= sda_d1;
            sda_d1 <= sda_i;

            if (tick)
                div_count <= 32'd0;
            else
                div_count <= div_count + 32'd1;

            if (slave_mode_i) begin
                case (state)
                    SL_IDLE: begin
                        scl_o <= 1'b1;
                        sda_o <= 1'b1;
                        busy_o <= 1'b0;
                        if (start_detect) begin
                            busy_o <= 1'b1;
                            shift_reg <= 8'd0;
                            bit_count <= 3'd7;
                            state <= SL_ADDRESS;
                        end
                    end

                    SL_ADDRESS: begin
                        if (stop_detect) begin
                            busy_o <= 1'b0;
                            state <= SL_IDLE;
                        end else if (scl_rising) begin
                            shift_reg <= {shift_reg[6:0], sda_i};
                            if (bit_count == 3'd0)
                                state <= SL_ADDR_ACK;
                            else
                                bit_count <= bit_count - 3'd1;
                        end
                    end

                    SL_ADDR_ACK: begin
                        if (scl_falling) begin
                            address_match <= (shift_reg[7:1] == own_address_i);
                            if ((shift_reg[7:1] == own_address_i) && (shift_reg[0] == 1'b0))
                                sda_o <= 1'b0; // ACK
                            else
                                sda_o <= 1'b1; // R/W=1 or address mismatch: NACK
                        end
                        if (scl_rising) begin
                            if (!address_match)
                                state <= SL_IDLE;
                            else if (shift_reg[0] == 1'b1)
                                state <= SL_IDLE;
                            else begin
                                shift_reg <= 8'd0;
                                bit_count <= 3'd7;
                                state <= SL_DATA;
                            end
                        end
                    end

                    SL_DATA: begin
                        sda_o <= 1'b1;
                        if (stop_detect) begin
                            busy_o <= 1'b0;
                            state <= SL_IDLE;
                        end else if (scl_rising) begin
                            shift_reg <= {shift_reg[6:0], sda_i};
                            if (bit_count == 3'd0)
                                state <= SL_DATA_ACK;
                            else
                                bit_count <= bit_count - 3'd1;
                        end
                    end

                    SL_DATA_ACK: begin
                        if (scl_falling) begin
                            if (ack_enable_i)
                                sda_o <= 1'b0;
                            else
                                sda_o <= 1'b1;
                        end
                        if (scl_rising) begin
                            if (ack_enable_i) begin
                                data_o <= shift_reg;
                                data_valid_o <= 1'b1;
                                shift_reg <= 8'd0;
                                bit_count <= 3'd7;
                                state <= SL_DATA;
                            end else begin
                                nack_o <= 1'b1;
                                data_o <= shift_reg;
                                data_valid_o <= 1'b1;
                                state <= SL_IDLE;
                            end
                        end
                    end

                    default: begin
                        state <= SL_IDLE;
                        busy_o <= 1'b0;
                        sda_o <= 1'b1;
                    end
                endcase
            end else begin
                // Master receiver mode.
                if (tick)
                    div_count <= 32'd0;

                case (state)
                    RX_IDLE: begin
                        scl_o <= 1'b1;
                        sda_o <= 1'b1;
                        busy_o <= 1'b0;
                        div_count <= 32'd0;
                        if (start_i) begin
                            busy_o <= 1'b1;
                            nack_o <= 1'b0;
                            shift_reg <= 8'd0;
                            bit_count <= 3'd7;
                            phase <= 4'd0;
                            state <= RX_DATA;
                        end
                    end

                    // Receive eight bits. SDA is captured on the rising SCL edge.
                    RX_DATA: begin
                        if (tick) begin
                            case (phase)
                                4'd0: begin
                                    scl_o <= 1'b0;
                                    sda_o <= 1'b1;
                                    phase <= 4'd1;
                                end
                                4'd1: begin
                                    scl_o <= 1'b1;
                                    phase <= 4'd2;
                                end
                                4'd2: begin
                                    shift_reg <= {shift_reg[6:0], sda_i};
                                    phase <= 4'd3;
                                end
                                4'd3: begin
                                    scl_o <= 1'b0;
                                    if (bit_count == 3'd0) begin
                                        phase <= 4'd0;
                                        state <= RX_ACK;
                                    end else begin
                                        bit_count <= bit_count - 3'd1;
                                        phase <= 4'd0;
                                    end
                                end
                                default: phase <= 4'd0;
                            endcase
                        end
                    end

                    // Master receiver issues NACK for the last received byte.
                    RX_ACK: begin
                        if (tick) begin
                            case (phase)
                                4'd0: begin
                                    scl_o <= 1'b0;
                                    sda_o <= 1'b1; // release SDA: NACK
                                    phase <= 4'd1;
                                end
                                4'd1: begin
                                    scl_o <= 1'b1;
                                    phase <= 4'd2;
                                end
                                4'd2: begin
                                    phase <= 4'd3;
                                end
                                4'd3: begin
                                    scl_o <= 1'b0;
                                    sda_o <= 1'b0;
                                    data_o <= shift_reg;
                                    data_valid_o <= 1'b1;
                                    phase <= 4'd0;
                                    state <= RX_STOP;
                                end
                                default: phase <= 4'd0;
                            endcase
                        end
                    end

                    // STOP: SDA is low before SCL rises, then SDA is released high.
                    RX_STOP: begin
                        if (tick) begin
                            case (phase)
                                4'd0: begin
                                    scl_o <= 1'b0;
                                    sda_o <= 1'b0;
                                    phase <= 4'd1;
                                end
                                4'd1: begin
                                    scl_o <= 1'b1;
                                    phase <= 4'd2;
                                end
                                4'd2: begin
                                    sda_o <= 1'b1;
                                    phase <= 4'd3;
                                end
                                4'd3: begin
                                    busy_o <= 1'b0;
                                    done_o <= 1'b1;
                                    nack_o <= 1'b1;
                                    phase <= 4'd0;
                                    state <= RX_DONE;
                                end
                                default: phase <= 4'd0;
                            endcase
                        end
                    end

                    RX_DONE: begin
                        scl_o <= 1'b1;
                        sda_o <= 1'b1;
                        if (!start_i)
                            state <= RX_IDLE;
                    end

                    default: begin
                        state <= RX_IDLE;
                        busy_o <= 1'b0;
                        scl_o <= 1'b1;
                        sda_o <= 1'b1;
                    end
                endcase
            end
        end
    end
endmodule
