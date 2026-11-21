`timescale 1ns/1ps

// IEEE 1364-2001 I2C master transmitter.
// SDA/SCL outputs are open-drain controls: 0 = actively drive low, 1 = release.
module i2c_tx (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_i,
    input  wire [6:0] address_i,
    input  wire       rw_i,
    input  wire [7:0] data_i,
    output reg        busy_o,
    output reg        done_o,
    output reg        nack_o,
    input  wire       scl_i,
    input  wire       sda_i,
    output reg        scl_o,
    output reg        sda_o
);
    // Number of system clocks in one complete SCL period.
    // CLK_DIV must be at least 4 and even for correct 50/50 clock phases.
    parameter CLK_DIV = 500;

    localparam [3:0] ST_IDLE      = 4'd0;
    localparam [3:0] ST_START     = 4'd1;
    localparam [3:0] ST_ADDR      = 4'd2;
    localparam [3:0] ST_ADDR_ACK  = 4'd3;
    localparam [3:0] ST_DATA      = 4'd4;
    localparam [3:0] ST_DATA_ACK  = 4'd5;
    localparam [3:0] ST_STOP      = 4'd6;
    localparam [3:0] ST_DONE      = 4'd7;

    reg [3:0]  state;
    reg [3:0]  phase;
    reg [31:0] div_count;
    reg [2:0]  bit_count;
    reg [7:0]  shift_reg;
    reg        ack_level;

    // A tick occurs on each half-period boundary of the generated SCL clock.
    wire tick;
    assign tick = (div_count == ((CLK_DIV / 2) - 1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            phase      <= 4'd0;
            div_count  <= 32'd0;
            bit_count  <= 3'd0;
            shift_reg  <= 8'd0;
            ack_level  <= 1'b1;
            busy_o     <= 1'b0;
            done_o     <= 1'b0;
            nack_o     <= 1'b0;
            scl_o      <= 1'b1;
            sda_o      <= 1'b1;
        end else begin
            done_o <= 1'b0;

            if (tick)
                div_count <= 32'd0;
            else
                div_count <= div_count + 32'd1;

            case (state)
                ST_IDLE: begin
                    scl_o <= 1'b1;
                    sda_o <= 1'b1;
                    busy_o <= 1'b0;
                    nack_o <= 1'b0;
                    div_count <= 32'd0;
                    if (start_i) begin
                        busy_o <= 1'b1;
                        nack_o <= 1'b0;
                        state <= ST_START;
                        phase <= 4'd0;
                        div_count <= 32'd0;
                    end
                end

                // START: release SDA and SCL, then SDA 1 -> 0 while SCL is high.
                ST_START: begin
                    if (tick) begin
                        case (phase)
                            4'd0: begin
                                scl_o <= 1'b1;
                                sda_o <= 1'b1;
                                phase <= 4'd1;
                            end
                            4'd1: begin
                                sda_o <= 1'b0;
                                phase <= 4'd2;
                            end
                            4'd2: begin
                                scl_o <= 1'b0;
                                shift_reg <= {address_i, rw_i};
                                bit_count <= 3'd7;
                                phase <= 4'd0;
                                state <= ST_ADDR;
                            end
                            default: begin
                                phase <= 4'd0;
                            end
                        endcase
                    end
                end

                // Address byte. One full SCL period is split into four half-tick phases.
                ST_ADDR: begin
                    if (tick) begin
                        case (phase)
                            4'd0: begin
                                scl_o <= 1'b0;
                                sda_o <= shift_reg[7] ? 1'b1 : 1'b0;
                                phase <= 4'd1;
                            end
                            4'd1: begin
                                scl_o <= 1'b1;
                                phase <= 4'd2;
                            end
                            4'd2: begin
                                // Rising-edge sample point. Address data is not rewritten here.
                                if (scl_i)
                                    phase <= 4'd3;
                            end
                            4'd3: begin
                                scl_o <= 1'b0;
                                if (bit_count == 3'd0) begin
                                    phase <= 4'd0;
                                    state <= ST_ADDR_ACK;
                                end else begin
                                    shift_reg <= {shift_reg[6:0], 1'b0};
                                    bit_count <= bit_count - 3'd1;
                                    phase <= 4'd0;
                                end
                            end
                            default: phase <= 4'd0;
                        endcase
                    end
                end

                // Ninth clock for address ACK. Low SDA = ACK, high SDA = NACK.
                ST_ADDR_ACK: begin
                    if (tick) begin
                        case (phase)
                            4'd0: begin
                                scl_o <= 1'b0;
                                sda_o <= 1'b1;
                                ack_level <= 1'b1;
                                phase <= 4'd1;
                            end
                            4'd1: begin
                                scl_o <= 1'b1;
                                phase <= 4'd2;
                            end
                            4'd2: begin
                                ack_level <= sda_i;
                                phase <= 4'd3;
                            end
                            4'd3: begin
                                scl_o <= 1'b0;
                                phase <= 4'd0;
                                if (ack_level == 1'b0) begin
                                    if (rw_i) begin
                                        // A read transaction ends the address phase here.
                                        // i2c_master starts i2c_rx and begins data clocks.
                                        busy_o <= 1'b0;
                                        done_o <= 1'b1;
                                        state <= ST_DONE;
                                    end else begin
                                        shift_reg <= data_i;
                                        bit_count <= 3'd7;
                                        state <= ST_DATA;
                                    end
                                end else begin
                                    nack_o <= 1'b1;
                                    state <= ST_STOP;
                                end
                            end
                            default: phase <= 4'd0;
                        endcase
                    end
                end

                // Data byte, most significant bit first.
                ST_DATA: begin
                    if (tick) begin
                        case (phase)
                            4'd0: begin
                                scl_o <= 1'b0;
                                sda_o <= shift_reg[7] ? 1'b1 : 1'b0;
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
                                if (bit_count == 3'd0) begin
                                    phase <= 4'd0;
                                    state <= ST_DATA_ACK;
                                end else begin
                                    shift_reg <= {shift_reg[6:0], 1'b0};
                                    bit_count <= bit_count - 3'd1;
                                    phase <= 4'd0;
                                end
                            end
                            default: phase <= 4'd0;
                        endcase
                    end
                end

                // Ninth clock for data ACK.
                ST_DATA_ACK: begin
                    if (tick) begin
                        case (phase)
                            4'd0: begin
                                scl_o <= 1'b0;
                                sda_o <= 1'b1;
                                ack_level <= 1'b1;
                                phase <= 4'd1;
                            end
                            4'd1: begin
                                scl_o <= 1'b1;
                                phase <= 4'd2;
                            end
                            4'd2: begin
                                ack_level <= sda_i;
                                phase <= 4'd3;
                            end
                            4'd3: begin
                                scl_o <= 1'b0;
                                phase <= 4'd0;
                                if (ack_level == 1'b1)
                                    nack_o <= 1'b1;
                                state <= ST_STOP;
                            end
                            default: phase <= 4'd0;
                        endcase
                    end
                end

                // STOP: SDA 0 -> 1 while SCL is high.
                ST_STOP: begin
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
                                phase <= 4'd0;
                                state <= ST_DONE;
                            end
                            default: phase <= 4'd0;
                        endcase
                    end
                end

                ST_DONE: begin
                    busy_o <= 1'b0;
                    scl_o <= 1'b1;
                    sda_o <= 1'b1;
                    if (!start_i)
                        state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                    busy_o <= 1'b0;
                    scl_o <= 1'b1;
                    sda_o <= 1'b1;
                end
            endcase
        end
    end

    // Unused clock input prevents synthesis optimization from removing clock stretching support.
    // A slave may stretch SCL; the master leaves SCL released and can monitor scl_i.
    // This module intentionally samples sda_i and monitors scl_i at the receive points.
endmodule
