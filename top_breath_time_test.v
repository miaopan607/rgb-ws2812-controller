// 简单烧录测试：WS2812 亮度每隔 CHANGE_MS 改变一次。
// 修改下面的 CHANGE_MS，重新编译并烧录到 FPGA，即可检查实际变化速度。
module top_breath_time_test(
    input  clk,
    input  rst_n,
    input  uart_rx,
    output led_out
);

localparam integer CLK_HZ = 50_000_000;
localparam integer CHANGE_MS = 13; // 每隔多少毫秒改变一次亮度。

localparam [3:0] BRIGHT_MIN = 4'd1;
localparam [3:0] BRIGHT_MAX = 4'd3;
localparam integer CHANGE_TICKS = (CLK_HZ / 1000) * CHANGE_MS;

reg [31:0] tick_cnt;
reg [3:0]  brightness;

wire unused_uart_rx = uart_rx;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tick_cnt <= 32'd0;
        brightness <= BRIGHT_MIN;
    end else if (tick_cnt == CHANGE_TICKS - 1) begin
        tick_cnt <= 32'd0;
        if (brightness == BRIGHT_MAX)
            brightness <= BRIGHT_MIN;
        else
            brightness <= brightness + 1'b1;
    end else begin
        tick_cnt <= tick_cnt + 1'b1;
    end
end

ws2812 u_ws2812(
    .clk            (clk),
    .rst_n          (rst_n),
    .led_data_in32  (8'b0101_0101), // LED2、3、6、7 全部显示绿色。
    .led_data_in10  (8'b0101_0101), // LED0、1、4、5 全部显示绿色。
    .mode           (1'b1),
    .led_brightness (brightness),
    .led_out        (led_out)
);

endmodule
