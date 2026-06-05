// 实验3：基于蓝牙控制的RGB彩灯系统
module top_rgb_bluetooth(
    input               clk,         // 开发板50MHz时钟
    input               rst_n,       // 低电平复位按键
    input               uart_rx,     // 蓝牙模块的TXD引脚接到此FPGA引脚
    output              led_out      // WS2812彩灯数据输出
);

//==================== 内部信号定义 ====================
wire [7:0]  rx_data;       // 接收到的蓝牙指令
wire        rx_done;       // 接收完成标志

reg [1:0]   sys_mode;      // 系统当前模式: 0=静态, 1=流水, 2=呼吸
reg [1:0]   sys_color;     // 当前基础颜色: 1=绿, 2=红, 3=蓝

localparam [1:0] MODE_STATIC = 2'd0;
localparam [1:0] MODE_FLOW   = 2'd1;
localparam [1:0] MODE_BREATH = 2'd2;

localparam [1:0] COLOR_GREEN = 2'd1;
localparam [1:0] COLOR_RED   = 2'd2;
localparam [1:0] COLOR_BLUE  = 2'd3;

localparam [3:0] BRIGHT_MIN = 4'd1;
localparam [3:0] BRIGHT_MAX = 4'd6;

localparam [7:0] RUN_STEP_LAST    = 8'd99; // 100ms流动一次
localparam [4:0] BREATH_STEP_LAST = 5'd24; // 25ms改变一次亮度

wire cmd_is_flow   = rx_done && (rx_data[7:4] == 4'h1);
wire cmd_is_breath = rx_done && (rx_data[7:4] == 4'h2);
wire cmd_has_color = (rx_data[3:0] >= 4'h1) && (rx_data[3:0] <= 4'h3);
wire [1:0] cmd_color = cmd_has_color ? rx_data[1:0] : sys_color;

function [1:0] next_rgb_color;
    input [1:0] color;
    begin
        case(color)
            COLOR_GREEN: next_rgb_color = COLOR_RED;
            COLOR_RED:   next_rgb_color = COLOR_BLUE;
            default:     next_rgb_color = COLOR_GREEN;
        endcase
    end
endfunction

function [15:0] flow_pattern;
    input [2:0] pos;
    input [1:0] color;
    begin
        flow_pattern = 16'd0;
        case(pos)
            3'd0: flow_pattern[1:0]   = color;
            3'd1: flow_pattern[3:2]   = color;
            3'd2: flow_pattern[5:4]   = color;
            3'd3: flow_pattern[7:6]   = color;
            3'd4: flow_pattern[9:8]   = color;
            3'd5: flow_pattern[11:10] = color;
            3'd6: flow_pattern[13:12] = color;
            3'd7: flow_pattern[15:14] = color;
        endcase
    end
endfunction

// 时间基准计数器 (产生1ms tick)
reg [15:0]  cnt_1ms;
wire        tick_1ms = (cnt_1ms == 16'd49_999);

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) cnt_1ms <= 16'd0;
    else if(tick_1ms) cnt_1ms <= 16'd0;
    else cnt_1ms <= cnt_1ms + 1'b1;
end

//==================== 1. 蓝牙指令解析 ====================
// 通信协议定义（手机APP发送单字节Hex）：
// 0x01: 静态绿   0x02: 静态红   0x03: 静态蓝
// 0x11: 流水绿   0x12: 流水红   0x13: 流水蓝
// 0x20: 彩色呼吸
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        sys_mode  <= MODE_FLOW;   // 默认流水模式
        sys_color <= COLOR_GREEN; // 默认绿色
    end else if(rx_done) begin
        case(rx_data[7:4]) // 高4位代表模式
            4'h0: sys_mode <= MODE_STATIC; // 静态
            4'h1: sys_mode <= MODE_FLOW;   // 流水
            4'h2: sys_mode <= MODE_BREATH; // 呼吸
        endcase
        if(cmd_has_color) begin
            sys_color <= rx_data[1:0]; // 低4位代表颜色 (1=绿,2=红,3=蓝)
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
reg [4:0]  breath_cnt;    // 呼吸节奏计数器
reg [3:0]  dynamic_bright;// 动态亮度 (1~6)
reg        breath_dir;    // 0:变亮, 1:变暗
reg [1:0]  breath_color;  // 彩色呼吸当前颜色

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        breath_cnt <= 5'd0;
        dynamic_bright <= BRIGHT_MAX;
        breath_dir <= 1'b0;
        breath_color <= COLOR_GREEN;
    end else if(cmd_is_breath) begin
        breath_cnt <= 5'd0;
        dynamic_bright <= BRIGHT_MIN;
        breath_dir <= 1'b0;
        breath_color <= COLOR_GREEN;
    end else if(tick_1ms) begin
        if(sys_mode == MODE_BREATH) begin // 仅在呼吸模式下生效
            breath_cnt <= breath_cnt + 1'b1;
            if(breath_cnt == BREATH_STEP_LAST) begin
                breath_cnt <= 5'd0;
                if(!breath_dir) begin
                    if(dynamic_bright < BRIGHT_MAX) dynamic_bright <= dynamic_bright + 1'b1;
                    else breath_dir <= 1'b1;
                end else begin
                    if(dynamic_bright > BRIGHT_MIN) dynamic_bright <= dynamic_bright - 1'b1;
                    else begin
                        breath_dir <= 1'b0;
                        breath_color <= next_rgb_color(breath_color);
                    end
                end
            end
        end else begin
            breath_cnt <= 5'd0;
            breath_dir <= 1'b0;
            dynamic_bright <= BRIGHT_MAX; // 非呼吸模式使用限幅后的常亮亮度
        end
    end
end

//==================== 3. 颜色阵列与流水效果逻辑 ====================
// 使用 16bit 寄存器统一管理8个灯的颜色，每2bit代表一个灯 (00=灭, 01=绿, 10=红, 11=蓝)
// [1:0]=LED0, [3:2]=LED1 ... [15:14]=LED7
reg [15:0] led_colors;
reg [7:0]  run_cnt;       // 流水速度计数器
reg [2:0]  run_pos;       // 当前流水灯位置
wire [2:0] run_pos_next = run_pos + 3'd1;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        led_colors <= 16'd0;
        run_cnt <= 8'd0;
        run_pos <= 3'd0;
    end else if(cmd_is_flow) begin
        run_cnt <= 8'd0;
        run_pos <= 3'd0;
        led_colors <= flow_pattern(3'd0, cmd_color);
    end else if(tick_1ms) begin
        case(sys_mode)
            MODE_STATIC: begin // 静态模式：所有灯同色
                run_cnt <= 8'd0;
                led_colors <= {8{sys_color}}; 
            end
            MODE_FLOW: begin // 流水模式：按位置直接生成，避免切换模式时残留旧颜色
                if(run_cnt == RUN_STEP_LAST) begin
                    run_cnt <= 8'd0;
                    run_pos <= run_pos_next;
                    led_colors <= flow_pattern(run_pos_next, sys_color);
                end else begin
                    run_cnt <= run_cnt + 1'b1;
                    led_colors <= flow_pattern(run_pos, sys_color);
                end
            end
            MODE_BREATH: begin // 彩色呼吸模式：颜色循环，亮度由上面控制
                run_cnt <= 8'd0;
                led_colors <= {8{breath_color}};
            end
        endcase
    end
end

//==================== 4. 适配灰盒引脚的组合逻辑映射 ====================
// 利用组合逻辑将符合人类直觉的 led_colors 数组，打乱映射到 data10 和 data32 中
wire [7:0] led_data_in10;
wire [7:0] led_data_in32;

// 原始灰盒映射关系: 
// data10: [7:6]=LED5, [5:4]=LED1, [3:2]=LED4, [1:0]=LED0
// data32: [7:6]=LED7, [5:4]=LED3, [3:2]=LED6, [1:0]=LED2
assign led_data_in10 = {led_colors[11:10], led_colors[3:2], led_colors[9:8], led_colors[1:0]};
assign led_data_in32 = {led_colors[15:14], led_colors[7:6], led_colors[13:12], led_colors[5:4]};

//==================== 5. 实例化模块 ====================

// 串口接收模块 (接收蓝牙数据)
uart_rx_module u_uart_rx(
    .clk        (clk),
    .rst_n      (rst_n),
    .rx         (uart_rx),
    .rx_data    (rx_data),
    .rx_done    (rx_done)
);

// 原始提供的 WS2812 灰盒模块 (固定为Mode 1: 彩色模式)
ws2812 u_ws2812(
    .clk            (clk),
    .rst_n          (rst_n),
    .led_data_in32  (led_data_in32),
    .led_data_in10  (led_data_in10),
    .mode           (1'b1),            // 强制启用数码管彩色模式
    .led_brightness (dynamic_bright),  // 动态亮度输入（实现呼吸效果）
    .led_out        (led_out)
);

endmodule
