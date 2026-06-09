//==============================================================================
// 模块名称: ws2812_fast
// 功能说明: 基于 50 MHz 系统时钟生成 WS2812 单总线驱动时序。
// 设计说明:
//   1. 每帧固定发送 8 颗 LED，每颗 LED 发送 24 bit GRB 数据。
//   2. 帧发送完成后保持低电平 RESET_CYCLES 个时钟，形成 WS2812 复位间隔。
//   3. 在复位间隔结束时锁存输入数据，保证一整帧输出期间颜色和亮度稳定。
//   4. 输入颜色编码为每颗 LED 2 bit: 00=灭, 01=绿, 10=红, 11=蓝。
//==============================================================================
module ws2812_fast(
    input        clk,              // 50 MHz 系统时钟。
    input        rst_n,            // 异步低电平复位。
    input  [7:0] led_data_in32,    // LED2/3/6/7 的 2 bit 颜色编码组合输入。
    input  [7:0] led_data_in10,    // LED0/1/4/5 的 2 bit 颜色编码组合输入。
    input        mode,             // 彩色输出使能；为 0 时所有 LED 熄灭。
    input  [7:0] led_brightness,   // RGB 单通道亮度值，作用于当前选中颜色通道。
    output reg   led_out           // WS2812 串行数据输出。
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
reg [5:0]  bit_cnt;                 // 当前 bit 内的时钟计数，范围 0~62。
reg [4:0]  bit_index;               // 当前 LED 的 bit 序号，范围 0~23。
reg [2:0]  led_index;               // 当前正在发送的 LED 序号，范围 0~7。
reg [11:0] reset_cnt;               // 帧间复位低电平计数器。
reg        reset_state;             // 复位间隔状态标志，1=输出低电平复位。
reg [7:0]  led_data_in32_latched;   // 帧起始处锁存的 led_data_in32。
reg [7:0]  led_data_in10_latched;   // 帧起始处锁存的 led_data_in10。
reg        mode_latched;            // 帧起始处锁存的模式控制。
reg [7:0]  led_brightness_latched;  // 帧起始处锁存的亮度值。

//------------------------------------------------------------------------------
// 当前发送 bit 的组合计算。
//------------------------------------------------------------------------------
wire [7:0]  bright_value  = led_brightness_latched;           // 当前帧使用的亮度值。
wire [23:0] current_color = get_color(led_index);             // 当前 LED 的 24 bit GRB 数据。
wire        current_bit   = current_color[23 - bit_index];    // WS2812 按高位优先发送。
wire [5:0]  high_cycles   = current_bit ? T1H_CYCLES[5:0] : T0H_CYCLES[5:0]; // 当前 bit 高电平宽度。

//------------------------------------------------------------------------------
// 函数: get_color_code
// 功能: 根据 LED 序号从灰盒端口映射中取出对应的 2 bit 颜色编码。
//------------------------------------------------------------------------------
function [1:0] get_color_code;
    input [2:0] index; // LED 序号，0~7。
    begin
        case (index)
            3'd0: get_color_code = led_data_in10_latched[1:0];  // LED0。
            3'd1: get_color_code = led_data_in10_latched[5:4];  // LED1。
            3'd2: get_color_code = led_data_in32_latched[1:0];  // LED2。
            3'd3: get_color_code = led_data_in32_latched[5:4];  // LED3。
            3'd4: get_color_code = led_data_in10_latched[3:2];  // LED4。
            3'd5: get_color_code = led_data_in10_latched[7:6];  // LED5。
            3'd6: get_color_code = led_data_in32_latched[3:2];  // LED6。
            default: get_color_code = led_data_in32_latched[7:6]; // LED7。
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: get_color
// 功能: 将 2 bit 颜色编码转换为 WS2812 需要的 24 bit GRB 数据。
//------------------------------------------------------------------------------
function [23:0] get_color;
    input [2:0] index;      // LED 序号，0~7。
    reg [1:0] color_code;   // 当前 LED 的 2 bit 颜色编码。
    begin
        color_code = mode_latched ? get_color_code(index) : 2'b00;
        case (color_code)
            2'b01: get_color = {bright_value, 8'd0, 8'd0}; // 绿色: GRB 中 G 通道有效。
            2'b10: get_color = {8'd0, bright_value, 8'd0}; // 红色: GRB 中 R 通道有效。
            2'b11: get_color = {8'd0, 8'd0, bright_value}; // 蓝色: GRB 中 B 通道有效。
            default: get_color = 24'd0;                    // 关闭当前 LED。
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 主时序过程:
//   1. reset_state=1 时输出低电平复位码，并在复位结束锁存下一帧输入。
//   2. reset_state=0 时逐 bit 输出 WS2812 高低电平编码。
//   3. 8 颗 LED 全部发送完成后回到 reset_state，开始下一帧。
//------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 异步复位后先进入帧间复位状态，确保 WS2812 总线保持低电平。
        led_out <= 1'b0;
        bit_cnt <= 6'd0;
        bit_index <= 5'd0;
        led_index <= 3'd0;
        reset_cnt <= 12'd0;
        reset_state <= 1'b1;
        led_data_in32_latched <= 8'd0;
        led_data_in10_latched <= 8'd0;
        mode_latched <= 1'b0;
        led_brightness_latched <= 8'd0;
    end else if (reset_state) begin
        // 帧间复位阶段: 数据线持续拉低，达到复位时间后准备发送新帧。
        led_out <= 1'b0;
        if (reset_cnt == RESET_CYCLES - 1) begin
            reset_cnt <= 12'd0;
            reset_state <= 1'b0;
            bit_cnt <= 6'd0;
            bit_index <= 5'd0;
            led_index <= 3'd0;
            // 锁存输入，避免上层逻辑在一帧中途改变导致颜色撕裂。
            led_data_in32_latched <= led_data_in32;
            led_data_in10_latched <= led_data_in10;
            mode_latched <= mode;
            led_brightness_latched <= led_brightness;
        end else begin
            reset_cnt <= reset_cnt + 1'b1;
        end
    end else begin
        // 数据发送阶段: bit_cnt 小于目标高电平宽度时输出 1，否则输出 0。
        led_out <= (bit_cnt < high_cycles);
        if (bit_cnt == BIT_TOTAL_CYCLES - 1) begin
            // 当前 bit 周期结束，推进 bit 序号或 LED 序号。
            bit_cnt <= 6'd0;
            if (bit_index == 5'd23) begin
                bit_index <= 5'd0;
                if (led_index == 3'd7) begin
                    // 8 颗 LED 均发送完成，进入下一次帧间复位。
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
