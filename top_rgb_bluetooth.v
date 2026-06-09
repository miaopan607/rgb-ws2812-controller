//==============================================================================
// 模块名称: top_rgb_bluetooth
// 功能说明: 基于蓝牙 UART 固定帧协议控制的 8 路 WS2812 RGB 彩灯顶层系统。
// 设计说明:
//   1. UART 接收手机 App 通过蓝牙串口发送的 17 字节二进制控制帧。
//   2. 整帧校验通过后一次性更新模式、RGB、亮度、呼吸周期和流水灯序。
//   3. 支持静态、流水、呼吸、渐变四种模式，参数实时应用到下一次刷新。
//   4. 顶层统一生成 LED0~LED7 的 RGB 数据，再交给 ws2812_fast 输出 WS2812 时序。
//==============================================================================
module top_rgb_bluetooth(
    input               clk,         // 开发板 50 MHz 时钟。
    input               rst_n,       // 低电平复位按键。
    input               uart_rx,     // 蓝牙模块 TXD 引脚接到此 FPGA 引脚。
    output              led_out      // WS2812 彩灯数据输出。
);

//==================== UART 接收信号 ====================
wire [7:0] rx_data; // 接收到的 UART 字节。
wire       rx_done; // 接收完成标志，高电平持续一个 clk 周期。

//==================== 模式和协议常量 ====================
localparam [1:0] MODE_STATIC   = 2'd0; // 静态模式：所有 LED 同色常亮。
localparam [1:0] MODE_FLOW     = 2'd1; // 流水模式：单颗 LED 沿自定义灯序移动。
localparam [1:0] MODE_BREATH   = 2'd2; // 呼吸模式：所有 LED 同色亮度渐变。
localparam [1:0] MODE_GRADIENT = 2'd3; // 渐变模式：多色空间分布并旋转相位。

localparam [7:0] FRAME_HEAD0 = 8'hAA; // 固定帧头第 1 字节。
localparam [7:0] FRAME_HEAD1 = 8'h55; // 固定帧头第 2 字节。

localparam [1:0] RX_WAIT_HEAD0 = 2'd0; // 等待 0xAA。
localparam [1:0] RX_WAIT_HEAD1 = 2'd1; // 等待 0x55。
localparam [1:0] RX_PAYLOAD    = 2'd2; // 接收 mode 到 flow_order7。
localparam [1:0] RX_CHECKSUM   = 2'd3; // 接收 XOR 校验字节。

localparam [7:0] RUN_STEP_LAST      = 8'd249; // 250 ms 流动一次，基于 1 ms tick。
localparam [7:0] GRADIENT_STEP_LAST = 8'd249; // 250 ms 切换一次渐变相位，基于 1 ms tick。
localparam [7:0] DEFAULT_BRIGHTNESS = 8'h11;  // 默认亮度，避免复位后过亮。

//==================== 配置寄存器 ====================
reg [1:0] sys_mode;           // 当前系统模式。
reg [7:0] cfg_r;              // App 下发的红色通道。
reg [7:0] cfg_g;              // App 下发的绿色通道。
reg [7:0] cfg_b;              // App 下发的蓝色通道。
reg [7:0] cfg_brightness;     // 静态、流水、渐变模式使用的全局亮度。
reg [7:0] cfg_period_100ms;   // 呼吸完整周期，单位 100 ms，0 会按 1 处理。
reg [2:0] flow_order0;        // 流水第 0 步点亮的 LED 编号。
reg [2:0] flow_order1;        // 流水第 1 步点亮的 LED 编号。
reg [2:0] flow_order2;        // 流水第 2 步点亮的 LED 编号。
reg [2:0] flow_order3;        // 流水第 3 步点亮的 LED 编号。
reg [2:0] flow_order4;        // 流水第 4 步点亮的 LED 编号。
reg [2:0] flow_order5;        // 流水第 5 步点亮的 LED 编号。
reg [2:0] flow_order6;        // 流水第 6 步点亮的 LED 编号。
reg [2:0] flow_order7;        // 流水第 7 步点亮的 LED 编号。

//==================== 协议解析暂存寄存器 ====================
reg [1:0] rx_state;       // 固定帧接收状态机。
reg [3:0] payload_index;  // 当前接收的 payload 字节序号，0~13。
reg [7:0] checksum_accum; // payload 字节 XOR 累加值。
reg [7:0] tmp_mode;       // 暂存模式字段。
reg [7:0] tmp_r;          // 暂存红色通道。
reg [7:0] tmp_g;          // 暂存绿色通道。
reg [7:0] tmp_b;          // 暂存蓝色通道。
reg [7:0] tmp_brightness; // 暂存亮度字段。
reg [7:0] tmp_period;     // 暂存呼吸周期字段。
reg [7:0] tmp_order0;     // 暂存流水灯序第 0 项。
reg [7:0] tmp_order1;     // 暂存流水灯序第 1 项。
reg [7:0] tmp_order2;     // 暂存流水灯序第 2 项。
reg [7:0] tmp_order3;     // 暂存流水灯序第 3 项。
reg [7:0] tmp_order4;     // 暂存流水灯序第 4 项。
reg [7:0] tmp_order5;     // 暂存流水灯序第 5 项。
reg [7:0] tmp_order6;     // 暂存流水灯序第 6 项。
reg [7:0] tmp_order7;     // 暂存流水灯序第 7 项。

wire frame_checksum_ok = (rx_data == checksum_accum); // 当前校验字节是否匹配 payload XOR。
wire frame_mode_ok = (tmp_mode <= 8'd3);              // 模式字段是否合法。
wire frame_order_ok = flow_order_valid(               // 流水灯序必须为 0~7 的不重复排列。
    tmp_order0, tmp_order1, tmp_order2, tmp_order3,
    tmp_order4, tmp_order5, tmp_order6, tmp_order7
);
wire frame_accept = frame_checksum_ok && frame_mode_ok && frame_order_ok; // 整帧是否允许应用。

//==================== 时间基准计数器 ====================
reg [15:0] cnt_1ms;                            // 1 ms 分频计数器，50 MHz 下计数 0~49999。
wire       tick_1ms = (cnt_1ms == 16'd49_999); // 1 ms 节拍，高电平持续一个 clk 周期。

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) cnt_1ms <= 16'd0;
    else if(tick_1ms) cnt_1ms <= 16'd0;
    else cnt_1ms <= cnt_1ms + 1'b1;
end

//------------------------------------------------------------------------------
// 函数: flow_order_valid
// 功能: 检查 8 字节流水灯序是否恰好包含 0~7 且不重复。
//------------------------------------------------------------------------------
function flow_order_valid;
    input [7:0] order0;
    input [7:0] order1;
    input [7:0] order2;
    input [7:0] order3;
    input [7:0] order4;
    input [7:0] order5;
    input [7:0] order6;
    input [7:0] order7;
    reg [7:0] seen;
    begin
        seen = 8'd0;
        flow_order_valid = 1'b0;
        if((order0 < 8'd8) && (order1 < 8'd8) && (order2 < 8'd8) && (order3 < 8'd8) &&
           (order4 < 8'd8) && (order5 < 8'd8) && (order6 < 8'd8) && (order7 < 8'd8)) begin
            seen[order0[2:0]] = 1'b1;
            seen[order1[2:0]] = 1'b1;
            seen[order2[2:0]] = 1'b1;
            seen[order3[2:0]] = 1'b1;
            seen[order4[2:0]] = 1'b1;
            seen[order5[2:0]] = 1'b1;
            seen[order6[2:0]] = 1'b1;
            seen[order7[2:0]] = 1'b1;
            flow_order_valid = (seen == 8'hFF);
        end
    end
endfunction

//------------------------------------------------------------------------------
// 函数: get_flow_led
// 功能: 根据当前流水步号返回自定义灯序中的 LED 编号。
//------------------------------------------------------------------------------
function [2:0] get_flow_led;
    input [2:0] pos; // 当前流水位置。
    begin
        case(pos)
            3'd0: get_flow_led = flow_order0;
            3'd1: get_flow_led = flow_order1;
            3'd2: get_flow_led = flow_order2;
            3'd3: get_flow_led = flow_order3;
            3'd4: get_flow_led = flow_order4;
            3'd5: get_flow_led = flow_order5;
            3'd6: get_flow_led = flow_order6;
            default: get_flow_led = flow_order7;
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: gradient_rgb
// 功能: 根据逻辑位置返回渐变模式的基础 RGB 颜色。
//------------------------------------------------------------------------------
function [23:0] gradient_rgb;
    input [2:0] pos; // 渐变位置，3 bit 溢出自然形成循环。
    begin
        case(pos)
            3'd0, 3'd3, 3'd6: gradient_rgb = 24'h00FF00; // 绿色。
            3'd1, 3'd4, 3'd7: gradient_rgb = 24'hFF0000; // 红色。
            default:          gradient_rgb = 24'h0000FF; // 蓝色。
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: pack_rgb8
// 功能: 将 8 颗 LED 的 RGB 数据打包为 ws2812_fast 使用的 192 bit 总线。
//------------------------------------------------------------------------------
function [191:0] pack_rgb8;
    input [23:0] led0;
    input [23:0] led1;
    input [23:0] led2;
    input [23:0] led3;
    input [23:0] led4;
    input [23:0] led5;
    input [23:0] led6;
    input [23:0] led7;
    begin
        pack_rgb8 = {led7, led6, led5, led4, led3, led2, led1, led0};
    end
endfunction

//==================== 1. 蓝牙固定帧解析 ====================
// 通信协议: AA 55 mode R G B brightness period order0..order7 checksum。
// checksum 为 mode 到 order7 共 14 字节的 XOR；校验和灯序合法后才更新配置。
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rx_state <= RX_WAIT_HEAD0;
        payload_index <= 4'd0;
        checksum_accum <= 8'd0;
        tmp_mode <= 8'd0;
        tmp_r <= 8'd0;
        tmp_g <= 8'd0;
        tmp_b <= 8'd0;
        tmp_brightness <= 8'd0;
        tmp_period <= 8'd0;
        tmp_order0 <= 8'd0;
        tmp_order1 <= 8'd1;
        tmp_order2 <= 8'd2;
        tmp_order3 <= 8'd3;
        tmp_order4 <= 8'd4;
        tmp_order5 <= 8'd5;
        tmp_order6 <= 8'd6;
        tmp_order7 <= 8'd7;
    end else if(rx_done) begin
        case(rx_state)
            RX_WAIT_HEAD0: begin
                if(rx_data == FRAME_HEAD0) rx_state <= RX_WAIT_HEAD1;
            end
            RX_WAIT_HEAD1: begin
                if(rx_data == FRAME_HEAD1) begin
                    rx_state <= RX_PAYLOAD;
                    payload_index <= 4'd0;
                    checksum_accum <= 8'd0;
                end else if(rx_data == FRAME_HEAD0) begin
                    rx_state <= RX_WAIT_HEAD1;
                end else begin
                    rx_state <= RX_WAIT_HEAD0;
                end
            end
            RX_PAYLOAD: begin
                checksum_accum <= checksum_accum ^ rx_data;
                case(payload_index)
                    4'd0:  tmp_mode <= rx_data;
                    4'd1:  tmp_r <= rx_data;
                    4'd2:  tmp_g <= rx_data;
                    4'd3:  tmp_b <= rx_data;
                    4'd4:  tmp_brightness <= rx_data;
                    4'd5:  tmp_period <= rx_data;
                    4'd6:  tmp_order0 <= rx_data;
                    4'd7:  tmp_order1 <= rx_data;
                    4'd8:  tmp_order2 <= rx_data;
                    4'd9:  tmp_order3 <= rx_data;
                    4'd10: tmp_order4 <= rx_data;
                    4'd11: tmp_order5 <= rx_data;
                    4'd12: tmp_order6 <= rx_data;
                    default: tmp_order7 <= rx_data;
                endcase
                if(payload_index == 4'd13) begin
                    rx_state <= RX_CHECKSUM;
                end else begin
                    payload_index <= payload_index + 1'b1;
                end
            end
            default: begin
                // 校验字节处理完后无论成功与否都回到找帧头状态，等待下一帧。
                rx_state <= RX_WAIT_HEAD0;
            end
        endcase
    end
end

//==================== 2. 配置应用 ====================
// 整帧合法时立即更新配置；无效帧直接丢弃，保持当前灯效不变。
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sys_mode <= MODE_FLOW;
        cfg_r <= 8'h00;
        cfg_g <= 8'hFF;
        cfg_b <= 8'h00;
        cfg_brightness <= DEFAULT_BRIGHTNESS;
        cfg_period_100ms <= 8'd20;
        flow_order0 <= 3'd0;
        flow_order1 <= 3'd1;
        flow_order2 <= 3'd2;
        flow_order3 <= 3'd3;
        flow_order4 <= 3'd4;
        flow_order5 <= 3'd5;
        flow_order6 <= 3'd6;
        flow_order7 <= 3'd7;
    end else if(rx_done && (rx_state == RX_CHECKSUM) && frame_accept) begin
        sys_mode <= tmp_mode[1:0];
        cfg_r <= tmp_r;
        cfg_g <= tmp_g;
        cfg_b <= tmp_b;
        cfg_brightness <= tmp_brightness;
        cfg_period_100ms <= (tmp_period == 8'd0) ? 8'd1 : tmp_period;
        flow_order0 <= tmp_order0[2:0];
        flow_order1 <= tmp_order1[2:0];
        flow_order2 <= tmp_order2[2:0];
        flow_order3 <= tmp_order3[2:0];
        flow_order4 <= tmp_order4[2:0];
        flow_order5 <= tmp_order5[2:0];
        flow_order6 <= tmp_order6[2:0];
        flow_order7 <= tmp_order7[2:0];
    end
end

//==================== 3. 呼吸亮度控制 ====================
reg [31:0] breath_step_cnt; // 当前完整呼吸周期内的 1 ms 节拍计数。
reg [15:0] breath_bright;   // 呼吸模式 16 bit 内部亮度，输出给 ws2812_fast。

wire [15:0] breath_period_ms = {cfg_period_100ms, 2'b00} + cfg_period_100ms; // period * 5。
wire [15:0] breath_period_ms_x20 = {breath_period_ms, 4'b0000} + {breath_period_ms, 2'b00}; // period * 100。
wire [15:0] breath_period_safe = (breath_period_ms_x20 < 16'd2) ? 16'd2 : breath_period_ms_x20;
wire [15:0] breath_half_period = (breath_period_safe >> 1);
wire [15:0] breath_full_last = breath_period_safe - 1'b1;
wire [15:0] breath_rise_denom = (breath_half_period < 16'd2) ? 16'd1 : (breath_half_period - 1'b1);
wire [15:0] breath_fall_period = breath_period_safe - breath_half_period;
wire [15:0] breath_fall_denom = (breath_fall_period < 16'd2) ? 16'd1 : (breath_fall_period - 1'b1);
wire [15:0] breath_target_bright = {cfg_brightness, cfg_brightness}; // 8 bit 用户亮度扩展成 16 bit 内部亮度。
wire [31:0] breath_fall_step = (breath_step_cnt >= breath_half_period) ? (breath_step_cnt - breath_half_period) : 32'd0;
wire [47:0] breath_rise_numer = breath_target_bright * breath_step_cnt;
wire [47:0] breath_fall_numer = breath_target_bright * breath_fall_step;
wire [31:0] breath_rise_value = breath_rise_numer / breath_rise_denom;
wire [31:0] breath_fall_delta = breath_fall_numer / breath_fall_denom;
wire [15:0] breath_rise_limited = (breath_rise_value > breath_target_bright) ? breath_target_bright : breath_rise_value[15:0];
wire [15:0] breath_fall_limited = (breath_fall_delta > breath_target_bright) ? 16'd0 : (breath_target_bright - breath_fall_delta[15:0]);
wire [15:0] breath_calc_bright = (breath_step_cnt < breath_half_period) ? breath_rise_limited : breath_fall_limited;
wire        frame_applied = rx_done && (rx_state == RX_CHECKSUM) && frame_accept; // 合法新帧脉冲。

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        breath_step_cnt <= 32'd0;
        breath_bright <= 16'd0;
    end else if(frame_applied && (tmp_mode[1:0] == MODE_BREATH)) begin
        // 新呼吸参数下发后从最暗处重新开始，便于手机端看到参数立即生效。
        breath_step_cnt <= 32'd0;
        breath_bright <= 16'd0;
    end else if(tick_1ms) begin
        if(sys_mode == MODE_BREATH) begin
            // 使用 16 bit 内部亮度计算呼吸曲线，低亮度时也保留更细的相位精度。
            breath_bright <= breath_calc_bright;
            if(breath_step_cnt >= breath_full_last) begin
                breath_step_cnt <= 32'd0;
            end else begin
                breath_step_cnt <= breath_step_cnt + 1'b1;
            end
        end else begin
            breath_step_cnt <= 32'd0;
            breath_bright <= 16'd0;
        end
    end
end

//==================== 4. 颜色阵列与灯效逻辑 ====================
reg [7:0]   run_cnt;        // 流水速度计数器，基于 1 ms tick。
reg [2:0]   run_pos;        // 当前流水灯序位置。
reg [7:0]   gradient_cnt;   // 渐变速度计数器，基于 1 ms tick。
reg [2:0]   gradient_phase; // 当前渐变相位，3 bit 溢出自然循环。
reg [191:0] led_rgb_data;   // 输出给 ws2812_fast 的 8 颗 LED RGB 数据。

wire [23:0] cfg_rgb = {cfg_r, cfg_g, cfg_b};         // 当前 App 下发的基础 RGB 颜色。
wire [2:0]  active_flow_led = get_flow_led(run_pos); // 流水当前应点亮的 LED 编号。
wire [15:0] output_brightness = (sys_mode == MODE_BREATH) ? breath_bright : {cfg_brightness, cfg_brightness};

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        run_cnt <= 8'd0;
        run_pos <= 3'd0;
        gradient_cnt <= 8'd0;
        gradient_phase <= 3'd0;
        led_rgb_data <= 192'd0;
    end else begin
        if(frame_applied) begin
            // 模式或参数更新后重置动画相位，让手机端每次下发都从确定状态开始。
            run_cnt <= 8'd0;
            run_pos <= 3'd0;
            gradient_cnt <= 8'd0;
            gradient_phase <= 3'd0;
        end else if(tick_1ms) begin
            case(sys_mode)
                MODE_FLOW: begin
                    gradient_cnt <= 8'd0;
                    if(run_cnt == RUN_STEP_LAST) begin
                        run_cnt <= 8'd0;
                        run_pos <= run_pos + 1'b1;
                    end else begin
                        run_cnt <= run_cnt + 1'b1;
                    end
                end
                MODE_GRADIENT: begin
                    run_cnt <= 8'd0;
                    if(gradient_cnt == GRADIENT_STEP_LAST) begin
                        gradient_cnt <= 8'd0;
                        gradient_phase <= gradient_phase + 1'b1;
                    end else begin
                        gradient_cnt <= gradient_cnt + 1'b1;
                    end
                end
                default: begin
                    run_cnt <= 8'd0;
                    gradient_cnt <= 8'd0;
                end
            endcase
        end

        case(sys_mode)
            MODE_STATIC: begin
                led_rgb_data <= pack_rgb8(cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb);
            end
            MODE_FLOW: begin
                led_rgb_data <= pack_rgb8(
                    (active_flow_led == 3'd0) ? cfg_rgb : 24'd0,
                    (active_flow_led == 3'd1) ? cfg_rgb : 24'd0,
                    (active_flow_led == 3'd2) ? cfg_rgb : 24'd0,
                    (active_flow_led == 3'd3) ? cfg_rgb : 24'd0,
                    (active_flow_led == 3'd4) ? cfg_rgb : 24'd0,
                    (active_flow_led == 3'd5) ? cfg_rgb : 24'd0,
                    (active_flow_led == 3'd6) ? cfg_rgb : 24'd0,
                    (active_flow_led == 3'd7) ? cfg_rgb : 24'd0
                );
            end
            MODE_BREATH: begin
                led_rgb_data <= pack_rgb8(cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb);
            end
            default: begin
                led_rgb_data <= pack_rgb8(
                    gradient_rgb(gradient_phase + 3'd0),
                    gradient_rgb(gradient_phase + 3'd1),
                    gradient_rgb(gradient_phase + 3'd2),
                    gradient_rgb(gradient_phase + 3'd3),
                    gradient_rgb(gradient_phase + 3'd4),
                    gradient_rgb(gradient_phase + 3'd5),
                    gradient_rgb(gradient_phase + 3'd6),
                    gradient_rgb(gradient_phase + 3'd7)
                );
            end
        endcase
    end
end

//==================== 5. 实例化模块 ====================
uart_rx_module u_uart_rx(
    .clk     (clk),
    .rst_n   (rst_n),
    .rx      (uart_rx),
    .rx_data (rx_data),
    .rx_done (rx_done)
);

ws2812_fast u_ws2812_fast(
    .clk            (clk),
    .rst_n          (rst_n),
    .led_rgb_data   (led_rgb_data),
    .mode           (1'b1),
    .led_brightness (output_brightness),
    .led_out        (led_out)
);

endmodule
