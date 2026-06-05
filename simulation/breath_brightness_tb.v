`timescale 1ns/1ps

module breath_brightness_tb;
    reg clk;
    reg rst_n;
    reg uart_rx;
    wire led_out;

    integer csv_file;
    integer sim_ms;
    reg [3:0] last_bright;

    top_rgb_bluetooth dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .uart_rx (uart_rx),
        .led_out(led_out)
    );

    initial begin
        clk = 1'b0;
        forever #10 clk = ~clk; // 50 MHz
    end

    initial begin
        csv_file = $fopen("simulation/breath_brightness.csv", "w");
        $fwrite(csv_file, "time_ms,brightness,direction\n");
        $dumpfile("simulation/breath_brightness.vcd");
        $dumpvars(0, breath_brightness_tb);

        rst_n = 1'b0;
        uart_rx = 1'b1;
        sim_ms = 0;
        last_bright = 4'hf;

        #200;
        rst_n = 1'b1;

        // Put the DUT into breath mode directly; this testbench is only for
        // reading the brightness sequence, not for testing the UART receiver.
        dut.sys_mode = 2'd2;
        dut.sys_color = 2'd1;
        dut.breath_cnt = 5'd0;
        dut.dynamic_bright = 4'd1;
        dut.breath_dir = 1'b0;

        // Long enough to see several 25 ms brightness steps.
        repeat (260) tick_1ms_fast();

        $fclose(csv_file);
        $finish;
    end

    always @(posedge clk) begin
        #1;
        if (rst_n && (dut.dynamic_bright !== last_bright)) begin
            last_bright <= dut.dynamic_bright;
            $display("%0d ms: brightness=%0d dir=%0d",
                     sim_ms, dut.dynamic_bright, dut.breath_dir);
            $fwrite(csv_file, "%0d,%0d,%0d\n",
                    sim_ms, dut.dynamic_bright, dut.breath_dir);
        end
    end

    task tick_1ms_fast;
        begin
            @(negedge clk);
            force dut.cnt_1ms = 16'd49_999;
            @(posedge clk);
            sim_ms = sim_ms + 1;
            release dut.cnt_1ms;
        end
    endtask
endmodule

module ws2812 (
    input        clk,
    input        rst_n,
    input  [7:0] led_data_in32,
    input  [7:0] led_data_in10,
    input        mode,
    input  [3:0] led_brightness,
    output       led_out
);
    assign led_out = 1'b0;
endmodule
