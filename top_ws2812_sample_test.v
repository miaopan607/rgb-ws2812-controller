// WS2812 快速驱动刷新速度测试。
// 本测试不改变 led_brightness，只在全绿和全灭之间切换颜色数据。
// 如果这个版本变化明显变快，说明原来的灰盒输出刷新很慢。
module top_ws2812_sample_test(
    input  clk,
    input  rst_n,
    input  uart_rx,
    output led_out
);

localparam integer CLK_HZ = 50_000_000;
localparam integer CHANGE_MS = 1; // 每隔多少毫秒切换一次全亮/全灭。
localparam integer CHANGE_TICKS = (CLK_HZ / 1000) * CHANGE_MS;

reg [31:0] tick_cnt;
reg        led_enable;

wire unused_uart_rx = uart_rx;
wire [7:0] test_data = led_enable ? 8'b0101_0101 : 8'b0000_0000;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tick_cnt <= 32'd0;
        led_enable <= 1'b0;
    end else if (tick_cnt == CHANGE_TICKS - 1) begin
        tick_cnt <= 32'd0;
        led_enable <= ~led_enable;
    end else begin
        tick_cnt <= tick_cnt + 1'b1;
    end
end

ws2812_fast u_ws2812_fast(
    .clk            (clk),
    .rst_n          (rst_n),
    .led_data_in32  (test_data),
    .led_data_in10  (test_data),
    .mode           (1'b1),
    .led_brightness (8'h33),
    .led_out        (led_out)
);

endmodule
