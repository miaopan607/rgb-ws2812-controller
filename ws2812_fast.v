//==============================================================================
// 模块名称: ws2812_fast
// 功能说明: 基于 50 MHz 系统时钟生成 8 颗 WS2812 的单总线驱动时序。
// 设计说明:
//   1. 每帧固定发送 8 颗 LED，每颗 LED 发送 24 bit GRB 数据。
//   2. 上层按 LED0~LED7 提供 RGB 数据，本模块在发送时转换成 WS2812 的 GRB 顺序。
//   3. 在帧间复位结束时锁存输入，保证一整帧输出期间颜色和亮度稳定。
//   4. led_brightness 是 16 bit 全局亮度缩放参数，0 为全灭，65535 接近原始颜色。
//==============================================================================
module ws2812_fast(
    input          clk,              // 50 MHz 系统时钟。
    input          rst_n,            // 异步低电平复位。
    input  [191:0] led_rgb_data,     // 8 颗 LED 的 RGB 数据，LEDn 位段为 [n*24 +: 24] = {R,G,B}。
    input          mode,             // 彩色输出使能；为 0 时所有 LED 熄灭。
    input  [15:0]  led_brightness,   // 全局亮度缩放值，0=灭，65535=最大。
    output reg     led_out           // WS2812 串行数据输出。
);

//------------------------------------------------------------------------------
// WS2812 时序参数。
//------------------------------------------------------------------------------
localparam integer BIT_TOTAL_CYCLES = 63;   // 单 bit 总周期，约 1.26 us。
localparam integer T0H_CYCLES       = 20;   // 发送 bit0 时高电平宽度，约 0.40 us。
localparam integer T1H_CYCLES       = 40;   // 发送 bit1 时高电平宽度，约 0.80 us。
localparam integer RESET_CYCLES     = 3000; // 帧间低电平复位时间，约 60 us。

//------------------------------------------------------------------------------
// 帧发送状态寄存器。
//------------------------------------------------------------------------------
reg [5:0]   bit_cnt;                 // 当前 bit 内的时钟计数，范围 0~62。
reg [4:0]   bit_index;               // 当前 LED 的 bit 序号，范围 0~23。
reg [2:0]   led_index;               // 当前正在发送的 LED 序号，范围 0~7。
reg [11:0]  reset_cnt;               // 帧间复位低电平计数器。
reg         reset_state;             // 复位间隔状态标志，1=输出低电平复位。
reg [191:0] led_rgb_data_latched;    // 帧起始处锁存的 RGB 数据。
reg         mode_latched;            // 帧起始处锁存的模式控制。
reg [15:0]  led_brightness_latched;  // 帧起始处锁存的亮度值。

//------------------------------------------------------------------------------
// 当前发送 bit 的组合计算。
//------------------------------------------------------------------------------
wire [23:0] current_color = get_color(led_index);             // 当前 LED 的 24 bit GRB 数据。
wire        current_bit   = current_color[23 - bit_index];    // WS2812 按高位优先发送。
wire [5:0]  high_cycles   = current_bit ? T1H_CYCLES[5:0] : T0H_CYCLES[5:0]; // 当前 bit 高电平宽度。

//------------------------------------------------------------------------------
// 函数: scale_channel
// 功能: 对单个 RGB 通道做 16 bit 全局亮度缩放。
//------------------------------------------------------------------------------
function [7:0] scale_channel;
    input [7:0] channel_value; // 原始通道值。
    input [15:0] bright_value; // 全局亮度值。
    reg [24:0] product;        // 乘积最大为 255*65536，保留 25 bit 避免溢出。
    begin
        // 使用 (brightness + 1) / 65536 近似缩放，使 65535 时能输出原始通道值。
        product = channel_value * ({1'b0, bright_value} + 17'd1);
        scale_channel = product[23:16];
    end
endfunction

//------------------------------------------------------------------------------
// 函数: get_rgb
// 功能: 按 LED 序号从锁存的 192 bit 总线中取出 {R,G,B}。
//------------------------------------------------------------------------------
function [23:0] get_rgb;
    input [2:0] index; // LED 序号，0~7。
    begin
        case(index)
            3'd0: get_rgb = led_rgb_data_latched[23:0];
            3'd1: get_rgb = led_rgb_data_latched[47:24];
            3'd2: get_rgb = led_rgb_data_latched[71:48];
            3'd3: get_rgb = led_rgb_data_latched[95:72];
            3'd4: get_rgb = led_rgb_data_latched[119:96];
            3'd5: get_rgb = led_rgb_data_latched[143:120];
            3'd6: get_rgb = led_rgb_data_latched[167:144];
            default: get_rgb = led_rgb_data_latched[191:168];
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: get_color
// 功能: 将上层 RGB 数据缩放亮度后转换为 WS2812 需要的 GRB 数据。
//------------------------------------------------------------------------------
function [23:0] get_color;
    input [2:0] index;      // LED 序号，0~7。
    reg [23:0] rgb_value;   // 当前 LED 的 {R,G,B} 原始数据。
    reg [7:0]  scaled_r;    // 缩放后的红色通道。
    reg [7:0]  scaled_g;    // 缩放后的绿色通道。
    reg [7:0]  scaled_b;    // 缩放后的蓝色通道。
    begin
        rgb_value = mode_latched ? get_rgb(index) : 24'd0;
        scaled_r = scale_channel(rgb_value[23:16], led_brightness_latched);
        scaled_g = scale_channel(rgb_value[15:8], led_brightness_latched);
        scaled_b = scale_channel(rgb_value[7:0], led_brightness_latched);
        get_color = {scaled_g, scaled_r, scaled_b};
    end
endfunction

//------------------------------------------------------------------------------
// 主时序过程:
//   1. reset_state=1 时输出低电平复位码，并在复位结束锁存下一帧输入。
//   2. reset_state=0 时逐 bit 输出 WS2812 高低电平编码。
//   3. 8 颗 LED 全部发送完成后回到 reset_state，开始下一次刷新。
//------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        led_out <= 1'b0;
        bit_cnt <= 6'd0;
        bit_index <= 5'd0;
        led_index <= 3'd0;
        reset_cnt <= 12'd0;
        reset_state <= 1'b1;
        led_rgb_data_latched <= 192'd0;
        mode_latched <= 1'b0;
        led_brightness_latched <= 16'd0;
    end else if (reset_state) begin
        // 帧间复位阶段: 数据线持续拉低，达到复位时间后准备发送新帧。
        led_out <= 1'b0;
        if (reset_cnt == RESET_CYCLES - 1) begin
            reset_cnt <= 12'd0;
            reset_state <= 1'b0;
            bit_cnt <= 6'd0;
            bit_index <= 5'd0;
            led_index <= 3'd0;
            led_rgb_data_latched <= led_rgb_data;
            mode_latched <= mode;
            led_brightness_latched <= led_brightness;
        end else begin
            reset_cnt <= reset_cnt + 1'b1;
        end
    end else begin
        // 数据发送阶段: bit_cnt 小于目标高电平宽度时输出 1，否则输出 0。
        led_out <= (bit_cnt < high_cycles);
        if (bit_cnt == BIT_TOTAL_CYCLES - 1) begin
            bit_cnt <= 6'd0;
            if (bit_index == 5'd23) begin
                bit_index <= 5'd0;
                if (led_index == 3'd7) begin
                    led_index <= 3'd0;
                    reset_state <= 1'b1;
                end else begin
                    led_index <= led_index + 1'b1;
                end
            end else begin
                bit_index <= bit_index + 1'b1;
            end
        end else begin
            bit_cnt <= bit_cnt + 1'b1;
        end
    end
end

endmodule
