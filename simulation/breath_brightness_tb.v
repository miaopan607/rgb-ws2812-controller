`timescale 1ns/1ps

//==============================================================================
// 模块名称: breath_brightness_tb
// 功能说明: top_rgb_bluetooth 呼吸亮度序列仿真测试平台。
// 设计说明:
//   1. 生成 50 MHz 仿真时钟和复位。
//   2. 直接强制 DUT 进入呼吸模式，绕过 UART 接收流程。
//   3. 通过 force cnt_1ms 快速产生 1 ms tick，缩短仿真时间。
//   4. 将亮度变化写入 CSV，并生成 VCD 波形文件。
//==============================================================================
module breath_brightness_tb;
    reg clk;        // 50 MHz 仿真时钟。
    reg rst_n;      // 仿真复位信号，低电平有效。
    reg uart_rx;    // UART 输入，测试中保持空闲高电平。
    wire led_out;   // DUT 输出的 WS2812 数据线。

    integer csv_file;      // CSV 输出文件句柄。
    integer sim_ms;        // 仿真推进的毫秒计数。
    reg [7:0] last_bright; // 上一次记录的亮度值，用于检测变化。

    //--------------------------------------------------------------------------
    // 被测顶层模块。
    //--------------------------------------------------------------------------
    top_rgb_bluetooth dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .uart_rx (uart_rx),
        .led_out(led_out)
    );

    //--------------------------------------------------------------------------
    // 50 MHz 时钟生成，周期 20 ns。
    //--------------------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #10 clk = ~clk;
    end

    //--------------------------------------------------------------------------
    // 仿真主流程: 初始化文件、释放复位、配置 DUT、推进若干毫秒节拍。
    //--------------------------------------------------------------------------
    initial begin
        csv_file = $fopen("simulation/breath_brightness.csv", "w");
        $fwrite(csv_file, "time_ms,brightness\n");
        $dumpfile("simulation/breath_brightness.vcd");
        $dumpvars(0, breath_brightness_tb);

        rst_n = 1'b0;
        uart_rx = 1'b1;
        sim_ms = 0;
        last_bright = 8'hff;

        #200;
        rst_n = 1'b1;

        // 直接进入呼吸模式；该测试仅观察亮度序列，不验证 UART 接收器。
        dut.sys_mode = 2'd2;
        dut.cfg_r = 8'h00;
        dut.cfg_g = 8'h00;
        dut.cfg_b = 8'hff;
        dut.cfg_brightness = 8'h40;
        dut.cfg_period_100ms = 8'd20;
        dut.breath_step_cnt = 32'd0;
        dut.breath_bright = 8'd0;

        // 推进足够长的时间，以观察多个亮度台阶变化。
        repeat (2200) tick_1ms_fast();

        $fclose(csv_file);
        $finish;
    end

    //--------------------------------------------------------------------------
    // 亮度变化记录: breath_bright 改变时打印并写入 CSV。
    //--------------------------------------------------------------------------
    always @(posedge clk) begin
        #1;
        if (rst_n && (dut.breath_bright !== last_bright)) begin
            last_bright <= dut.breath_bright;
            $display("%0d ms: brightness=%0d", sim_ms, dut.breath_bright);
            $fwrite(csv_file, "%0d,%0d\n", sim_ms, dut.breath_bright);
        end
    end

    //--------------------------------------------------------------------------
    // 任务: tick_1ms_fast
    // 功能: 强制 DUT 的 1 ms 计数器到终值，使下一个时钟产生 tick_1ms。
    //--------------------------------------------------------------------------
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

//==============================================================================
// 模块名称: ws2812
// 功能说明: 仿真占位模块。
// 设计说明:
//   1. 用于兼容可能存在的旧模块引用。
//   2. 本测试平台实际实例化的是 ws2812_fast，因此这里固定输出低电平。
//==============================================================================
module ws2812 (
    input        clk,              // 占位时钟输入。
    input        rst_n,            // 占位复位输入。
    input [191:0] led_rgb_data,    // 占位 RGB 输入。
    input        mode,             // 占位模式输入。
    input  [7:0] led_brightness,   // 占位亮度输入。
    output       led_out           // 固定为低电平的占位输出。
);
    // 保留未使用端口，避免旧测试或旧顶层引用缺失模块。
    assign led_out = 1'b0;
endmodule
