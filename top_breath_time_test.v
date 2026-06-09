//==============================================================================
// 模块名称: top_breath_time_test
// 功能说明: WS2812 亮度变化速度板级测试。
// 设计说明:
//   1. 不使用 UART 输入，仅保留端口以匹配顶层引脚。
//   2. 所有 LED 固定为绿色，只周期性改变 led_brightness。
//   3. 修改 CHANGE_MS 后重新编译烧录，可观察实际亮度变化速度。
//==============================================================================
module top_breath_time_test(
    input  clk,      // 50 MHz 系统时钟。
    input  rst_n,    // 异步低电平复位。
    input  uart_rx,  // 未使用的 UART 输入，仅用于保持工程端口一致。
    output led_out   // WS2812 串行数据输出。
);

//------------------------------------------------------------------------------
// 亮度测试参数。
//------------------------------------------------------------------------------
localparam integer CLK_HZ    = 50_000_000; // 系统时钟频率。
localparam integer CHANGE_MS = 13;         // 亮度台阶切换周期，单位 ms。

localparam [7:0] BRIGHT_MIN   = 8'h11;                       // 测试最小亮度。
localparam [7:0] BRIGHT_MAX   = 8'h33;                       // 测试最大亮度。
localparam integer CHANGE_TICKS = (CLK_HZ / 1000) * CHANGE_MS; // 切换周期对应的时钟数。

//------------------------------------------------------------------------------
// 测试状态信号。
//------------------------------------------------------------------------------
reg [31:0] tick_cnt;   // 毫秒级切换计数器。
reg [7:0]  brightness; // 当前输出给 WS2812 驱动的亮度。

wire unused_uart_rx = uart_rx; // 显式连接未使用输入，避免综合工具告警。

//------------------------------------------------------------------------------
// 周期性递增亮度，达到最大值后回到最小值。
//------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tick_cnt <= 32'd0;
        brightness <= BRIGHT_MIN;
    end else if (tick_cnt == CHANGE_TICKS - 1) begin
        tick_cnt <= 32'd0;
        if (brightness == BRIGHT_MAX)
            brightness <= BRIGHT_MIN;
        else
            brightness <= brightness + 8'h11;
    end else begin
        tick_cnt <= tick_cnt + 1'b1;
    end
end

//------------------------------------------------------------------------------
// WS2812 驱动实例。
//------------------------------------------------------------------------------
ws2812_fast u_ws2812_fast(
    .clk            (clk),
    .rst_n          (rst_n),
    .led_data_in32  (8'b0101_0101), // LED2、3、6、7 全部显示绿色。
    .led_data_in10  (8'b0101_0101), // LED0、1、4、5 全部显示绿色。
    .mode           (1'b1),
    .led_brightness (brightness),
    .led_out        (led_out)
);

endmodule
