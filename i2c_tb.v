`timescale 1ns/1ps

// Testbench for i2c_tx.v, i2c_rx.v and i2c_master.v.
// Open-drain bus is modelled with tri1 wires and active-low drivers.
module i2c_tb;
    reg clk;
    reg rst_n;

    tri1 scl;
    tri1 sda;

    reg tx_enable;
    reg rx_slave_enable;
    reg master_enable;

    reg tx_start;
    reg [6:0] tx_address;
    reg tx_rw;
    reg [7:0] tx_data;

    reg rx_start;
    reg [7:0] rx_received;

    reg master_start;
    reg [6:0] master_address;
    reg master_rw;
    reg [7:0] master_data;

    wire tx_busy;
    wire tx_done;
    wire tx_nack;
    wire tx_scl;
    wire tx_sda;

    wire rx_busy;
    wire rx_done;
    wire [7:0] rx_data;
    wire rx_valid;
    wire rx_nack;
    wire rx_scl;
    wire rx_sda;

    wire master_busy;
    wire master_done;
    wire master_nack;
    wire [7:0] master_rx_data;
    wire master_data_valid;
    wire master_scl;
    wire master_sda;

    // Bus multiplexing: only one active engine at a time controls each line.
    assign scl = ((tx_enable && (tx_scl == 1'b0)) ||
                  (rx_slave_enable && (rx_scl == 1'b0)) ||
                  (master_enable && (master_scl == 1'b0))) ? 1'b0 : 1'bz;

    assign sda = ((slave_bfm_enable && (bfm_sda == 1'b0)) ||
                  (tx_enable && (tx_sda == 1'b0)) ||
                  (rx_slave_enable && (rx_sda == 1'b0)) ||
                  (master_enable && (master_sda == 1'b0))) ? 1'b0 : 1'bz;

    i2c_tx #(.CLK_DIV(20)) dut_tx (
        .clk(clk),
        .rst_n(rst_n),
        .start_i(tx_start),
        .address_i(tx_address),
        .rw_i(tx_rw),
        .data_i(tx_data),
        .busy_o(tx_busy),
        .done_o(tx_done),
        .nack_o(tx_nack),
        .scl_i(scl),
        .sda_i(sda),
        .scl_o(tx_scl),
        .sda_o(tx_sda)
    );

    i2c_rx #(.CLK_DIV(20)) dut_rx_slave (
        .clk(clk),
        .rst_n(rst_n),
        .start_i(rx_start),
        .slave_mode_i(1'b1),
        .own_address_i(7'h50),
        .ack_enable_i(1'b1),
        .busy_o(rx_busy),
        .done_o(rx_done),
        .data_o(rx_data),
        .data_valid_o(rx_valid),
        .nack_o(rx_nack),
        .scl_i(scl),
        .sda_i(sda),
        .scl_o(rx_scl),
        .sda_o(rx_sda)
    );

    i2c_master #(.CLK_DIV(20)) dut_master (
        .clk(clk),
        .rst_n(rst_n),
        .start_i(master_start),
        .address_i(master_address),
        .rw_i(master_rw),
        .data_i(master_data),
        .busy_o(master_busy),
        .done_o(master_done),
        .nack_o(master_nack),
        .data_o(master_rx_data),
        .data_valid_o(master_data_valid),
        .scl_i(scl),
        .sda_i(sda),
        .scl_o(master_scl),
        .sda_o(master_sda)
    );

    // -----------------------------------------------------------------
    // Emulated slave for the Master write/read tests.
    // slave_bfm_enable selects this optional BFM driver.
    // -----------------------------------------------------------------
    reg slave_bfm_enable;
    reg slave_ack_drive;
    reg slave_read_drive;
    reg [7:0] slave_read_byte;
    reg bfm_phase;          // 0=address, 1=write data
    reg [3:0] bfm_bit_count;
    reg [7:0] bfm_shift;
    reg bfm_wait_ack;
    reg scl_d1;
    reg scl_d2;

    tri1 bfm_sda;

    assign bfm_sda = slave_ack_drive ? 1'b0 :
                     slave_read_drive ? ((slave_read_byte[7-bfm_bit_count] == 1'b0) ? 1'b0 : 1'bz) :
                     1'bz;

    always @(posedge clk) begin
        scl_d2 <= scl_d1;
        scl_d1 <= scl;

        if (!rst_n) begin
            slave_ack_drive <= 1'b0;
            slave_read_drive <= 1'b0;
            bfm_phase <= 1'b0;
            bfm_bit_count <= 4'd0;
            bfm_shift <= 8'd0;
            bfm_wait_ack <= 1'b0;
            scl_d1 <= 1'b1;
            scl_d2 <= 1'b1;
        end else if (slave_bfm_enable) begin
            // Falling SCL: prepare ACK or the next read-data bit.
            if ((scl_d1 == 1'b0) && (scl_d2 == 1'b1)) begin
                if (bfm_wait_ack) begin
                    slave_ack_drive <= 1'b0;
                    bfm_wait_ack <= 1'b0;
                    if (bfm_phase == 1'b0) begin
                        if (bfm_shift[0] == 1'b1) begin
                            slave_read_drive <= 1'b1;
                            bfm_phase <= 1'b1;
                            bfm_bit_count <= 4'd0;
                        end else begin
                            slave_read_drive <= 1'b0;
                            bfm_phase <= 1'b1;
                            bfm_bit_count <= 4'd0;
                        end
                    end else begin
                        bfm_phase <= 1'b1;
                        bfm_bit_count <= 4'd0;
                        slave_read_drive <= 1'b0;
                    end
                end
            end

            // Rising SCL: sample address/write data or prepare read value.
            if ((scl_d1 == 1'b1) && (scl_d2 == 1'b0)) begin
                if (bfm_phase == 1'b0) begin
                    bfm_shift <= {bfm_shift[6:0], sda};
                    if (bfm_bit_count == 4'd7) begin
                        bfm_bit_count <= 4'd8;
                    end else begin
                        bfm_bit_count <= bfm_bit_count + 4'd1;
                    end

                    if (bfm_bit_count == 4'd7)
                        slave_ack_drive <= 1'b1;
                end else if (slave_read_drive) begin
                    if (bfm_bit_count == 4'd7) begin
                        slave_read_drive <= 1'b0;
                        bfm_bit_count <= 4'd0;
                    end else begin
                        bfm_bit_count <= bfm_bit_count + 4'd1;
                    end
                end else begin
                    bfm_shift <= {bfm_shift[6:0], sda};
                    if (bfm_bit_count == 4'd7) begin
                        bfm_bit_count <= 4'd8;
                        slave_ack_drive <= 1'b1;
                    end else begin
                        bfm_bit_count <= bfm_bit_count + 4'd1;
                    end
                end
            end

            // ACK driver is held low while SCL is low for the ninth clock.
            if ((scl_d1 == 1'b0) && (scl_d2 == 1'b1)) begin
                if ((bfm_bit_count == 4'd8) && !slave_ack_drive)
                    slave_ack_drive <= 1'b1;

                if ((bfm_bit_count == 4'd8) && slave_ack_drive)
                    bfm_wait_ack <= 1'b1;
            end
        end
    end

    // 50 MHz system clock; CLK_DIV=20 gives a 2.5 MHz test clock.
    initial begin
        clk = 1'b0;
        forever #10 clk = ~clk;
    end

    // Global watchdog prevents the simulation from hanging on a protocol fault.
    initial begin
        repeat (50000) @(posedge clk);
        $display("[%0t] WATCHDOG: simulation did not complete", $time);
        $display("Tx=%0d Rx=%0d Master=%0d SCL=%b SDA=%b", dut_tx.state, dut_rx_slave.state,
                 dut_master.state, scl, sda);
        $finish;
    end

    initial begin
        rst_n = 1'b0;
        tx_enable = 1'b0;
        rx_slave_enable = 1'b0;
        master_enable = 1'b0;
        slave_bfm_enable = 1'b0;
        tx_start = 1'b0;
        tx_address = 7'h50;
        tx_rw = 1'b0;
        tx_data = 8'hA5;
        rx_start = 1'b0;
        master_start = 1'b0;
        master_address = 7'h50;
        master_rw = 1'b0;
        master_data = 8'h5A;

        $dumpfile("i2c_wave.vcd");
        $dumpvars(0, i2c_tb);

        $display("[%0t] Reset asserted", $time);
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        $display("[%0t] Reset released", $time);

        // -----------------------------------------------------------------
        // Scenario 1: standalone Tx writes to standalone Rx Slave.
        // -----------------------------------------------------------------
        $display("[%0t] TEST 1: Tx write address 0x50 + data 0xA5", $time);
        tx_enable = 1'b1;
        rx_slave_enable = 1'b1;
        tx_address = 7'h50;
        tx_rw = 1'b0;
        tx_data = 8'hA5;
        repeat (2) @(posedge clk);
        tx_start = 1'b1;
        rx_start = 1'b1;
        @(posedge clk);
        tx_start = 1'b0;
        rx_start = 1'b0;

        wait (tx_done === 1'b1);
        $display("[%0t] Tx done: NACK=%b Tx-state=%0d", $time, tx_nack, dut_tx.state);
        repeat (500) @(posedge clk);
        if (rx_valid) begin
            $display("[%0t] Rx received 0x%02X; Rx-state=%0d", $time, rx_data, dut_rx_slave.state);
            rx_received = rx_data;
        end else begin
            $display("[%0t] Rx timeout: no valid byte; Rx-state=%0d", $time, dut_rx_slave.state);
        end

        repeat (20) @(posedge clk);
        tx_enable = 1'b0;
        rx_slave_enable = 1'b0;

        // -----------------------------------------------------------------
        // Scenario 2: Master write with an ACK from the emulated slave.
        // -----------------------------------------------------------------
        $display("[%0t] TEST 2: Master write address 0x50 + data 0x5A", $time);
        slave_bfm_enable = 1'b1;
        slave_ack_drive = 1'b0;
        slave_read_drive = 1'b0;
        bfm_phase = 1'b0;
        bfm_bit_count = 4'd0;
        bfm_shift = 8'd0;
        bfm_wait_ack = 1'b0;
        master_enable = 1'b1;
        master_address = 7'h50;
        master_rw = 1'b0;
        master_data = 8'h5A;
        repeat (2) @(posedge clk);
        master_start = 1'b1;
        @(posedge clk);
        master_start = 1'b0;

        wait (master_done === 1'b1);
        $display("[%0t] Master write done: NACK=%b Master-state=%0d Tx-state=%0d", $time,
                 master_nack, dut_master.state, dut_tx.state);

        repeat (20) @(posedge clk);
        master_enable = 1'b0;
        slave_bfm_enable = 1'b0;

        // -----------------------------------------------------------------
        // Scenario 3: Master read with an address ACK and final-byte NACK.
        // -----------------------------------------------------------------
        $display("[%0t] TEST 3: Master read address 0x50, slave returns 0xC3", $time);
        slave_bfm_enable = 1'b1;
        slave_ack_drive = 1'b0;
        slave_read_drive = 1'b1;
        slave_read_byte = 8'hC3;
        bfm_phase = 1'b0;
        bfm_bit_count = 4'd0;
        bfm_shift = 8'd0;
        bfm_wait_ack = 1'b0;
        master_enable = 1'b1;
        master_address = 7'h50;
        master_rw = 1'b1;
        master_data = 8'h00;
        repeat (2) @(posedge clk);
        master_start = 1'b1;
        @(posedge clk);
        master_start = 1'b0;

        wait (master_done === 1'b1);
        $display("[%0t] Master read done: data_valid=%b data=0x%02X NACK=%b", $time,
                 master_data_valid, master_rx_data, master_nack);

        repeat (20) @(posedge clk);
        master_enable = 1'b0;
        slave_bfm_enable = 1'b0;

        $display("[%0t] All I2C test scenarios completed", $time);
        $finish;
    end

    // Diagnostic logging is emitted only for test checkpoints above.
    // Detailed per-clock values are available in i2c_wave.vcd.
endmodule
