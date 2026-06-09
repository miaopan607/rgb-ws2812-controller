//==============================================================================
// 模块名称: top_rgb_bluetooth
// 功能说明: 基于蓝牙 UART 指令控制的 8 路 WS2812 RGB 彩灯顶层系统。
// 设计说明:
//   1. UART 接收手机或蓝牙模块发送的单字节控制命令。
//   2. 高 4 bit 表示显示模式，低 4 bit 表示基础颜色。
//   3. 支持静态、流水、呼吸、渐变四种模式。
//   4. 使用 16 bit 逻辑颜色数组管理 LED0~LED7，再映射到 ws2812_fast 灰盒端口。
//==============================================================================
module top_rgb_bluetooth(
    input               clk,         // 开发板50MHz时钟
    input               rst_n,       // 低电平复位按键
    input               uart_rx,     // 蓝牙模块的TXD引脚接到此FPGA引脚
    output              led_out      // WS2812彩灯数据输出
);

//==================== 内部信号定义 ====================
wire [7:0]  rx_data;       // 接收到的蓝牙指令
wire        rx_done;       // 接收完成标志

reg [1:0]   sys_mode;      // 系统当前模式: 0=静态, 1=流水, 2=呼吸, 3=渐变
reg [1:0]   sys_color;     // 当前基础颜色: 1=绿, 2=红, 3=蓝

//==================== 模式、颜色和节拍参数 ====================
localparam [1:0] MODE_STATIC   = 2'd0; // 静态模式：所有 LED 同色常亮。
localparam [1:0] MODE_FLOW     = 2'd1; // 流水模式：单颗 LED 沿灯序移动。
localparam [1:0] MODE_BREATH   = 2'd2; // 呼吸模式：所有 LED 同色亮度渐变。
localparam [1:0] MODE_GRADIENT = 2'd3; // 渐变模式：多色空间分布并旋转相位。

localparam [1:0] COLOR_GREEN = 2'd1; // 颜色编码 01: 绿色。
localparam [1:0] COLOR_RED   = 2'd2; // 颜色编码 10: 红色。
localparam [1:0] COLOR_BLUE  = 2'd3; // 颜色编码 11: 蓝色。

localparam [7:0] BRIGHT_MIN = 8'd0;  // 呼吸模式最小亮度。
localparam [7:0] BRIGHT_MAX = 8'h11; // 常亮和呼吸模式最大亮度。

localparam [7:0] RUN_STEP_LAST      = 8'd249; // 250 ms 流动一次，基于 1 ms tick。
localparam [7:0] GRADIENT_STEP_LAST = 8'd249; // 250 ms 切换一次渐变相位，基于 1 ms tick。
localparam integer BREATH_PERIOD_MS      = 2000; // 完整呼吸周期，单位 ms，建议配置为 2 或更大。
localparam integer BREATH_HALF_PERIOD_MS = BREATH_PERIOD_MS / 2; // 上升半周期，单位 ms。
localparam integer BREATH_FALL_PERIOD_MS = BREATH_PERIOD_MS - BREATH_HALF_PERIOD_MS; // 下降半周期，单位 ms。
localparam integer BREATH_RISE_DENOM     = (BREATH_HALF_PERIOD_MS > 1) ? (BREATH_HALF_PERIOD_MS - 1) : 1; // 上升段归一化分母。
localparam integer BREATH_FALL_DENOM     = (BREATH_FALL_PERIOD_MS > 1) ? (BREATH_FALL_PERIOD_MS - 1) : 1; // 下降段归一化分母。
localparam integer BREATH_RANGE          = BRIGHT_MAX - BRIGHT_MIN; // 呼吸亮度变化范围。

//==================== 命令译码辅助信号 ====================
wire cmd_is_flow     = rx_done && (rx_data[7:4] == 4'h1); // 收到流水模式命令。
wire cmd_is_breath   = rx_done && (rx_data[7:4] == 4'h2); // 收到呼吸模式命令。
wire cmd_is_gradient = rx_done && (rx_data[7:4] == 4'h3); // 收到渐变模式命令。
wire cmd_has_color   = (rx_data[3:0] >= 4'h1) && (rx_data[3:0] <= 4'h3); // 命令低 4 bit 是否为有效颜色。

//------------------------------------------------------------------------------
// 函数: flow_data_in10
// 功能: 流水模式下，根据当前位置生成 led_data_in10 的灰盒端口数据。
//------------------------------------------------------------------------------
function [7:0] flow_data_in10;
    input [2:0] pos;    // 当前流水位置，范围 0~7。
    input [1:0] color;  // 当前流水颜色编码。
    begin
        flow_data_in10 = 8'd0;
        case(pos)
            3'd0: flow_data_in10[1:0] = color; // 灯0
            3'd1: flow_data_in10[5:4] = color; // 灯1
            3'd4: flow_data_in10[3:2] = color; // 灯4
            3'd5: flow_data_in10[7:6] = color; // 灯5
            default: flow_data_in10 = 8'd0;
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: flow_data_in32
// 功能: 流水模式下，根据当前位置生成 led_data_in32 的灰盒端口数据。
//------------------------------------------------------------------------------
function [7:0] flow_data_in32;
    input [2:0] pos;    // 当前流水位置，范围 0~7。
    input [1:0] color;  // 当前流水颜色编码。
    begin
        flow_data_in32 = 8'd0;
        case(pos)
            3'd2: flow_data_in32[1:0] = color; // 灯2
            3'd3: flow_data_in32[5:4] = color; // 灯3
            3'd6: flow_data_in32[3:2] = color; // 灯6
            3'd7: flow_data_in32[7:6] = color; // 灯7
            default: flow_data_in32 = 8'd0;
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: gradient_color
// 功能: 根据逻辑位置返回渐变模式的颜色编码。
//------------------------------------------------------------------------------
function [1:0] gradient_color;
    input [2:0] pos; // 渐变位置，3 bit 溢出自然形成循环。
    begin
        case(pos)
            3'd0, 3'd3, 3'd6: gradient_color = COLOR_GREEN;
            3'd1, 3'd4, 3'd7: gradient_color = COLOR_RED;
            default:          gradient_color = COLOR_BLUE;
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: gradient_pattern
// 功能: 根据渐变相位生成 8 颗 LED 的 16 bit 颜色数组。
//------------------------------------------------------------------------------
function [15:0] gradient_pattern;
    input [2:0] phase; // 当前渐变相位，3 bit 溢出自然形成循环。
    begin
        gradient_pattern[1:0]   = gradient_color(phase + 3'd0); // LED0。
        gradient_pattern[3:2]   = gradient_color(phase + 3'd1); // LED1。
        gradient_pattern[5:4]   = gradient_color(phase + 3'd2); // LED2。
        gradient_pattern[7:6]   = gradient_color(phase + 3'd3); // LED3。
        gradient_pattern[9:8]   = gradient_color(phase + 3'd4); // LED4。
        gradient_pattern[11:10] = gradient_color(phase + 3'd5); // LED5。
        gradient_pattern[13:12] = gradient_color(phase + 3'd6); // LED6。
        gradient_pattern[15:14] = gradient_color(phase + 3'd7); // LED7。
    end
endfunction

//==================== 时间基准计数器 ====================
reg [15:0]  cnt_1ms;                          // 1 ms 分频计数器，50 MHz 下计数 0~49999。
wire        tick_1ms = (cnt_1ms == 16'd49_999); // 1 ms 节拍，高电平持续一个 clk 周期。

// 由 50 MHz 时钟分频产生 1 ms tick，供流水、呼吸和渐变节奏复用。
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) cnt_1ms <= 16'd0;
    else if(tick_1ms) cnt_1ms <= 16'd0;
    else cnt_1ms <= cnt_1ms + 1'b1;
end

//==================== 1. 蓝牙指令解析 ====================
// 通信协议定义（手机APP发送单字节Hex）：
// 0x01: 静态绿   0x02: 静态红   0x03: 静态蓝
// 0x11: 流水绿   0x12: 流水红   0x13: 流水蓝
// 0x21: 呼吸绿   0x22: 呼吸红   0x23: 呼吸蓝
// 0x30: 渐变
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sys_mode  <= MODE_FLOW;   // 默认流水模式
        sys_color <= COLOR_GREEN; // 默认绿色
    end else if(rx_done) begin
        case(rx_data[7:4]) // 高4位代表模式。
            4'h0: sys_mode <= MODE_STATIC;   // 静态
            4'h1: sys_mode <= MODE_FLOW;     // 流水
            4'h2: sys_mode <= MODE_BREATH;   // 呼吸
            4'h3: sys_mode <= MODE_GRADIENT; // 渐变
        endcase
        if(cmd_has_color) begin
            sys_color <= rx_data[1:0]; // 低4位代表颜色 (1=绿,2=红,3=蓝)。
        end
    end
end

//==================== 1. 暴力测试版指令解析（调试专用） ====================
// always @(posedge clk or negedge rst_n) begin
//     if(!rst_n) begin
//         sys_mode  <= 2'd0; // 固定使用静态模式（方便看颜色）
//         sys_color <= 2'd1; // 默认绿色
//     end else if(rx_done) begin
//         // 只要串口接收完成一次，不论收到什么数据，颜色都切换一次！
//         // 顺序：绿 -> 红 -> 蓝 -> 绿...循环
//         if(sys_color == 2'd3) 
//             sys_color <= 2'd1;
//         else 
//             sys_color <= sys_color + 1'b1;
//     end
// end

//==================== 2. 呼吸灯亮度控制 (解耦设计) ====================
reg [31:0] breath_step_cnt;  // 当前半周期内的 1 ms 节拍计数。
reg [31:0] breath_accum;     // 亮度台阶误差累加器，用于在固定周期内均匀分布亮度变化。
reg [7:0]  dynamic_bright;   // 动态亮度，输出给 ws2812_fast。
reg        breath_dir;       // 呼吸方向: 0=变亮, 1=变暗。
wire [31:0] breath_rise_accum_next = breath_accum + BREATH_RANGE; // 上升段下一拍累加值。
wire [31:0] breath_fall_accum_next = breath_accum + BREATH_RANGE; // 下降段下一拍累加值。
wire        breath_rise_step_en = (breath_rise_accum_next >= BREATH_RISE_DENOM); // 上升段是否调整 1 档亮度。
wire        breath_fall_step_en = (breath_fall_accum_next >= BREATH_FALL_DENOM); // 下降段是否调整 1 档亮度。
wire [31:0] breath_rise_accum_reload = breath_rise_step_en ? (breath_rise_accum_next - BREATH_RISE_DENOM) : breath_rise_accum_next; // 上升段累加器回写值。
wire [31:0] breath_fall_accum_reload = breath_fall_step_en ? (breath_fall_accum_next - BREATH_FALL_DENOM) : breath_fall_accum_next; // 下降段累加器回写值。

// 呼吸模式下以 BREATH_PERIOD_MS 为固定完整周期生成亮度三角波。
// 非呼吸模式将亮度恢复到 BRIGHT_MAX，保证静态/流水/渐变模式常亮。
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        breath_step_cnt <= 32'd0;
        breath_accum <= 32'd0;
        dynamic_bright <= BRIGHT_MAX;
        breath_dir <= 1'b0;
    end else if(cmd_is_breath) begin
        breath_step_cnt <= 32'd0;
        breath_accum <= 32'd0;
        dynamic_bright <= BRIGHT_MIN;
        breath_dir <= 1'b0;
    end else if(tick_1ms) begin
        if(sys_mode == MODE_BREATH) begin // 仅在呼吸模式下生效
            if(!breath_dir) begin
                // 上升半周期固定持续 BREATH_HALF_PERIOD_MS，期间均匀提升亮度。
                if(breath_step_cnt == BREATH_HALF_PERIOD_MS - 1) begin
                    breath_step_cnt <= 32'd0;
                    breath_accum <= 32'd0;
                    dynamic_bright <= BRIGHT_MAX;
                    breath_dir <= 1'b1;
                end else begin
                    breath_step_cnt <= breath_step_cnt + 1'b1;
                    breath_accum <= breath_rise_accum_reload;
                    if(breath_rise_step_en && (dynamic_bright < BRIGHT_MAX)) begin
                        dynamic_bright <= dynamic_bright + 1'b1;
                    end
                end
            end else begin
                // 下降半周期固定持续 BREATH_FALL_PERIOD_MS，期间均匀降低亮度。
                if(breath_step_cnt == BREATH_FALL_PERIOD_MS - 1) begin
                    breath_step_cnt <= 32'd0;
                    breath_accum <= 32'd0;
                    dynamic_bright <= BRIGHT_MIN;
                    breath_dir <= 1'b0;
                end else begin
                    breath_step_cnt <= breath_step_cnt + 1'b1;
                    breath_accum <= breath_fall_accum_reload;
                    if(breath_fall_step_en && (dynamic_bright > BRIGHT_MIN)) begin
                        dynamic_bright <= dynamic_bright - 1'b1;
                    end
                end
            end
        end else begin
            breath_step_cnt <= 32'd0;
            breath_accum <= 32'd0;
            breath_dir <= 1'b0;
            dynamic_bright <= BRIGHT_MAX; // 非呼吸模式使用限幅后的常亮亮度
        end
    end
end

//==================== 3. 颜色阵列与流水效果逻辑 ====================
// 使用 16bit 寄存器统一管理8个灯的颜色，每2bit代表一个灯 (00=灭, 01=绿, 10=红, 11=蓝)
// [1:0]=LED0, [3:2]=LED1 ... [15:14]=LED7
reg [15:0] led_colors;      // 逻辑颜色数组，每 2 bit 对应一颗 LED。
reg [7:0]  run_cnt;         // 流水速度计数器，基于 1 ms tick。
reg [2:0]  run_pos;         // 当前流水灯位置，3 bit 溢出自然回到 LED0。
reg [7:0]  gradient_cnt;    // 渐变速度计数器，基于 1 ms tick。
reg [2:0]  gradient_phase;  // 当前渐变相位，3 bit 溢出自然循环。
wire [2:0] run_pos_next = run_pos + 3'd1;                 // 下一颗流水灯位置。
wire [2:0] gradient_phase_next = gradient_phase + 3'd1;   // 下一帧渐变相位。

// 根据当前系统模式更新 led_colors、run_pos 和 gradient_phase。
// 流水模式走 ws2812_fast 原始灰盒端口顺序，因此 led_colors 保持清零。
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        led_colors <= 16'd0;
        run_cnt <= 8'd0;
        run_pos <= 3'd0;
        gradient_cnt <= 8'd0;
        gradient_phase <= 3'd0;
    end else if(cmd_is_flow) begin
        run_cnt <= 8'd0;
        run_pos <= 3'd0;
        gradient_cnt <= 8'd0;
        led_colors <= 16'd0;
    end else if(cmd_is_gradient) begin
        // 收到渐变命令时重置相位，避免从上一次渐变中途继续。
        run_cnt <= 8'd0;
        gradient_cnt <= 8'd0;
        gradient_phase <= 3'd0;
        led_colors <= gradient_pattern(3'd0);
    end else if(tick_1ms) begin
        case(sys_mode)
            MODE_STATIC: begin // 静态模式：所有灯同色
                run_cnt <= 8'd0;
                gradient_cnt <= 8'd0;
                led_colors <= {8{sys_color}};
            end
            MODE_FLOW: begin // 流水模式：按参考工程的灰盒端口顺序直接输出
                gradient_cnt <= 8'd0;
                if(run_cnt == RUN_STEP_LAST) begin
                    run_cnt <= 8'd0;
                    run_pos <= run_pos_next;
                end else begin
                    run_cnt <= run_cnt + 1'b1;
                end
                led_colors <= 16'd0;
            end
            MODE_BREATH: begin // 单色呼吸模式：颜色固定，亮度由上面控制
                run_cnt <= 8'd0;
                gradient_cnt <= 8'd0;
                led_colors <= {8{sys_color}};
            end
            MODE_GRADIENT: begin // 渐变模式：绿/红/蓝空间渐变并旋转
                run_cnt <= 8'd0;
                if(gradient_cnt == GRADIENT_STEP_LAST) begin
                    gradient_cnt <= 8'd0;
                    gradient_phase <= gradient_phase_next;
                    led_colors <= gradient_pattern(gradient_phase_next);
                end else begin
                    gradient_cnt <= gradient_cnt + 1'b1;
                    led_colors <= gradient_pattern(gradient_phase);
                end
            end
        endcase
    end
end

//==================== 4. 适配灰盒引脚的组合逻辑映射 ====================
// 利用组合逻辑将符合人类直觉的 led_colors 数组，打乱映射到 data10 和 data32 中
wire [7:0] led_data_in10;         // 输入 ws2812_fast 的 LED0/1/4/5 灰盒颜色端口。
wire [7:0] led_data_in32;         // 输入 ws2812_fast 的 LED2/3/6/7 灰盒颜色端口。
wire [7:0] mapped_led_data_in10;  // 非流水模式下由 led_colors 映射得到的 data10。
wire [7:0] mapped_led_data_in32;  // 非流水模式下由 led_colors 映射得到的 data32。

// 原始灰盒映射关系: 
// data10: [7:6]=LED5, [5:4]=LED1, [3:2]=LED4, [1:0]=LED0
// data32: [7:6]=LED7, [5:4]=LED3, [3:2]=LED6, [1:0]=LED2
assign mapped_led_data_in10 = {led_colors[11:10], led_colors[3:2], led_colors[9:8], led_colors[1:0]};
assign mapped_led_data_in32 = {led_colors[15:14], led_colors[7:6], led_colors[13:12], led_colors[5:4]};
// 流水模式直接生成灰盒端口数据；其他模式先生成直观 led_colors 后再映射。
assign led_data_in10 = (sys_mode == MODE_FLOW) ? flow_data_in10(run_pos, sys_color) : mapped_led_data_in10;
assign led_data_in32 = (sys_mode == MODE_FLOW) ? flow_data_in32(run_pos, sys_color) : mapped_led_data_in32;

//==================== 5. 实例化模块 ====================

// 串口接收模块 (接收蓝牙数据)
uart_rx_module u_uart_rx(
    .clk        (clk),
    .rst_n      (rst_n),
    .rx         (uart_rx),
    .rx_data    (rx_data),
    .rx_done    (rx_done)
);

// 自定义快速 WS2812 驱动 (固定为Mode 1: 彩色模式)
ws2812_fast u_ws2812_fast(
    .clk            (clk),
    .rst_n          (rst_n),
    .led_data_in32  (led_data_in32),
    .led_data_in10  (led_data_in10),
    .mode           (1'b1),            // 强制启用数码管彩色模式
    .led_brightness (dynamic_bright),  // 动态亮度输入（实现平滑呼吸效果）
    .led_out        (led_out)
);

endmodule
