//==============================================================================
// 模块名称: top_rgb_bluetooth
// 功能说明: 基于蓝牙 UART 可变长度帧协议控制的 8 路 WS2812 RGB 彩灯顶层系统。
// 设计说明:
//   1. UART 接收手机 App 通过蓝牙串口发送的二进制控制帧。
//   2. 整帧校验通过后一次性更新模式、RGB、亮度、呼吸周期和流水画面。
//   3. 支持静态、流水、呼吸、Disco、渐变、流动渐变六种模式，参数实时应用到下一次刷新。
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
localparam [2:0] MODE_STATIC        = 3'd0; // 静态模式：所有 LED 同色常亮。
localparam [2:0] MODE_FLOW          = 3'd1; // 流水模式：按自定义画面掩码循环点亮。
localparam [2:0] MODE_BREATH        = 3'd2; // 呼吸模式：所有 LED 同色亮度渐变。
localparam [2:0] MODE_DISCO         = 3'd3; // Disco 模式：板载离散三色循环。
localparam [2:0] MODE_GRADIENT      = 3'd4; // 普通渐变：8 颗灯同色做平滑 RGB 环渐变。
localparam [2:0] MODE_FLOW_GRADIENT = 3'd5; // 流动渐变：8 颗灯错相分布并整体流动。

localparam [7:0] FRAME_HEAD0 = 8'hAA; // 固定帧头第 1 字节。
localparam [7:0] FRAME_HEAD1 = 8'h55; // 固定帧头第 2 字节。

localparam [1:0] RX_WAIT_HEAD0 = 2'd0; // 等待 0xAA。
localparam [1:0] RX_WAIT_HEAD1 = 2'd1; // 等待 0x55。
localparam [1:0] RX_PAYLOAD    = 2'd2; // 接收 mode 到最后一个流水画面。
localparam [1:0] RX_CHECKSUM   = 2'd3; // 接收 XOR 校验字节。

localparam [7:0] DISCO_STEP_LAST    = 8'd249; // Disco 仍按 250 ms 切换一次，基于 1 ms tick。
localparam [7:0] DEFAULT_BRIGHTNESS = 8'h11;  // 默认亮度，避免复位后过亮。
localparam [3:0] BASE_PAYLOAD_LAST  = 4'd6;   // payload 第 6 字节为 flow_count。
localparam [3:0] FLOW_FRAME_MAX     = 4'd8;   // 高级流水最多 8 个画面。
localparam [10:0] GRADIENT_PHASE_COUNT = 11'd1536; // 一轮六段平滑渐变共 1536 个相位。
localparam [10:0] FLOW_GRADIENT_OFFSET = 11'd192;  // 8 颗灯均匀错相。

//==================== 配置寄存器 ====================
reg [2:0] sys_mode;           // 当前系统模式。
reg [7:0] cfg_r;              // App 下发的红色通道。
reg [7:0] cfg_g;              // App 下发的绿色通道。
reg [7:0] cfg_b;              // App 下发的蓝色通道。
reg [7:0] cfg_brightness;     // 静态、流水、渐变模式使用的全局亮度。
reg [7:0] cfg_period_units;   // 周期字段原始值；流水单位 10 ms，呼吸单位 20 ms，渐变模式单位 50 ms。
reg [3:0] cfg_flow_count;     // 当前流水画面数量，范围 1~8。
reg [7:0] flow_frame0;        // 流水第 0 个画面，bit0~bit7 对应 LED0~LED7。
reg [7:0] flow_frame1;        // 流水第 1 个画面，bit0~bit7 对应 LED0~LED7。
reg [7:0] flow_frame2;        // 流水第 2 个画面，bit0~bit7 对应 LED0~LED7。
reg [7:0] flow_frame3;        // 流水第 3 个画面，bit0~bit7 对应 LED0~LED7。
reg [7:0] flow_frame4;        // 流水第 4 个画面，bit0~bit7 对应 LED0~LED7。
reg [7:0] flow_frame5;        // 流水第 5 个画面，bit0~bit7 对应 LED0~LED7。
reg [7:0] flow_frame6;        // 流水第 6 个画面，bit0~bit7 对应 LED0~LED7。
reg [7:0] flow_frame7;        // 流水第 7 个画面，bit0~bit7 对应 LED0~LED7。

//==================== 协议解析暂存寄存器 ====================
reg [1:0] rx_state;            // 可变长度帧接收状态机。
reg [3:0] payload_index;       // 当前接收的 payload 字节序号。
reg [7:0] checksum_accum;      // payload 字节 XOR 累加值。
reg [7:0] tmp_mode;            // 暂存模式字段。
reg [7:0] tmp_r;               // 暂存红色通道。
reg [7:0] tmp_g;               // 暂存绿色通道。
reg [7:0] tmp_b;               // 暂存蓝色通道。
reg [7:0] tmp_brightness;      // 暂存亮度字段。
reg [7:0] tmp_period;          // 暂存周期字段。
reg [7:0] tmp_flow_count;      // 暂存流水画面数量。
reg [7:0] tmp_flow_frame0;     // 暂存流水第 0 个画面。
reg [7:0] tmp_flow_frame1;     // 暂存流水第 1 个画面。
reg [7:0] tmp_flow_frame2;     // 暂存流水第 2 个画面。
reg [7:0] tmp_flow_frame3;     // 暂存流水第 3 个画面。
reg [7:0] tmp_flow_frame4;     // 暂存流水第 4 个画面。
reg [7:0] tmp_flow_frame5;     // 暂存流水第 5 个画面。
reg [7:0] tmp_flow_frame6;     // 暂存流水第 6 个画面。
reg [7:0] tmp_flow_frame7;     // 暂存流水第 7 个画面。

wire frame_checksum_ok = (rx_data == checksum_accum);                         // 当前校验字节是否匹配 payload XOR。
wire frame_mode_ok = (tmp_mode <= 8'd5);                                       // 模式字段是否合法。
wire frame_flow_count_ok = (tmp_flow_count >= 8'd1) && (tmp_flow_count <= 8'd8); // 流水画面数量必须为 1~8。
wire frame_accept = frame_checksum_ok && frame_mode_ok && frame_flow_count_ok;  // 整帧是否允许应用。

//==================== 时间基准计数器 ====================
reg [15:0] cnt_1ms;                            // 1 ms 分频计数器，50 MHz 下计数 0~49999。
wire       tick_1ms = (cnt_1ms == 16'd49_999); // 1 ms 节拍，高电平持续一个 clk 周期。

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) cnt_1ms <= 16'd0;
    else if(tick_1ms) cnt_1ms <= 16'd0;
    else cnt_1ms <= cnt_1ms + 1'b1;
end

//------------------------------------------------------------------------------
// 函数: get_flow_frame
// 功能: 根据当前流水位置返回 8 bit 画面掩码。
//------------------------------------------------------------------------------
function [7:0] get_flow_frame;
    input [2:0] pos; // 当前流水画面位置。
    begin
        case(pos)
            3'd0: get_flow_frame = flow_frame0;
            3'd1: get_flow_frame = flow_frame1;
            3'd2: get_flow_frame = flow_frame2;
            3'd3: get_flow_frame = flow_frame3;
            3'd4: get_flow_frame = flow_frame4;
            3'd5: get_flow_frame = flow_frame5;
            3'd6: get_flow_frame = flow_frame6;
            default: get_flow_frame = flow_frame7;
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: disco_rgb
// 功能: 保留原 mode=3 的离散三色循环，用于 Disco 模式。
//------------------------------------------------------------------------------
function [23:0] disco_rgb;
    input [2:0] pos; // 逻辑位置，3 bit 溢出自然形成循环。
    begin
        case(pos)
            3'd0, 3'd3, 3'd6: disco_rgb = 24'h00FF00; // 绿色。
            3'd1, 3'd4, 3'd7: disco_rgb = 24'hFF0000; // 红色。
            default:          disco_rgb = 24'h0000FF; // 蓝色。
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: gradient_cycle_rgb
// 功能: 根据 0~1535 相位生成平滑 RGB 环渐变颜色。
//------------------------------------------------------------------------------
function [23:0] gradient_cycle_rgb;
    input [10:0] phase; // 完整渐变环相位。
    reg [7:0] offset;
    begin
        offset = phase[7:0];
        case(phase / 11'd256)
            3'd0: gradient_cycle_rgb = {8'hFF, offset, 8'h00};
            3'd1: gradient_cycle_rgb = {8'hFF - offset, 8'hFF, 8'h00};
            3'd2: gradient_cycle_rgb = {8'h00, 8'hFF, offset};
            3'd3: gradient_cycle_rgb = {8'h00, 8'hFF - offset, 8'hFF};
            3'd4: gradient_cycle_rgb = {offset, 8'h00, 8'hFF};
            default: gradient_cycle_rgb = {8'hFF, 8'h00, 8'hFF - offset};
        endcase
    end
endfunction

//------------------------------------------------------------------------------
// 函数: add_gradient_phase
// 功能: 对渐变相位做回绕加法。
//------------------------------------------------------------------------------
function [10:0] add_gradient_phase;
    input [10:0] base_phase;
    input [10:0] offset_phase;
    reg [11:0] sum_phase;
    begin
        sum_phase = {1'b0, base_phase} + {1'b0, offset_phase};
        if(sum_phase >= GRADIENT_PHASE_COUNT) begin
            add_gradient_phase = sum_phase - GRADIENT_PHASE_COUNT;
        end else begin
            add_gradient_phase = sum_phase[10:0];
        end
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

//==================== 1. 蓝牙可变长度帧解析 ====================
// 通信协议: AA 55 mode R G B brightness period flow_count frame0..frameN-1 checksum。
// checksum 为 mode 到最后一个 frame 共 7+N 字节的 XOR；校验和画面数量合法后才更新配置。
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
        tmp_flow_count <= 8'd8;
        tmp_flow_frame0 <= 8'b0000_0001;
        tmp_flow_frame1 <= 8'b0000_0010;
        tmp_flow_frame2 <= 8'b0000_0100;
        tmp_flow_frame3 <= 8'b0000_1000;
        tmp_flow_frame4 <= 8'b0001_0000;
        tmp_flow_frame5 <= 8'b0010_0000;
        tmp_flow_frame6 <= 8'b0100_0000;
        tmp_flow_frame7 <= 8'b1000_0000;
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
                    4'd6:  tmp_flow_count <= rx_data;
                    4'd7:  tmp_flow_frame0 <= rx_data;
                    4'd8:  tmp_flow_frame1 <= rx_data;
                    4'd9:  tmp_flow_frame2 <= rx_data;
                    4'd10: tmp_flow_frame3 <= rx_data;
                    4'd11: tmp_flow_frame4 <= rx_data;
                    4'd12: tmp_flow_frame5 <= rx_data;
                    4'd13: tmp_flow_frame6 <= rx_data;
                    default: tmp_flow_frame7 <= rx_data;
                endcase
                if(payload_index == BASE_PAYLOAD_LAST) begin
                    if((rx_data < 8'd1) || (rx_data > 8'd8)) begin
                        // 画面数量非法时立即丢弃本帧，避免继续等待未知长度数据。
                        rx_state <= RX_WAIT_HEAD0;
                        payload_index <= 4'd0;
                    end else begin
                        payload_index <= payload_index + 1'b1;
                    end
                end else if((payload_index >= BASE_PAYLOAD_LAST) &&
                            (payload_index == (BASE_PAYLOAD_LAST + tmp_flow_count[3:0]))) begin
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
        cfg_period_units <= 8'd25;
        cfg_flow_count <= FLOW_FRAME_MAX;
        flow_frame0 <= 8'b0000_1000;
        flow_frame1 <= 8'b0000_0100;
        flow_frame2 <= 8'b0000_0010;
        flow_frame3 <= 8'b0000_0001;
        flow_frame4 <= 8'b0001_0000;
        flow_frame5 <= 8'b0010_0000;
        flow_frame6 <= 8'b0100_0000;
        flow_frame7 <= 8'b1000_0000;
    end else if(rx_done && (rx_state == RX_CHECKSUM) && frame_accept) begin
        sys_mode <= tmp_mode[2:0];
        cfg_r <= tmp_r;
        cfg_g <= tmp_g;
        cfg_b <= tmp_b;
        cfg_brightness <= tmp_brightness;
        cfg_period_units <= (tmp_period == 8'd0) ? 8'd1 : tmp_period;
        cfg_flow_count <= tmp_flow_count[3:0];
        flow_frame0 <= tmp_flow_frame0;
        flow_frame1 <= tmp_flow_frame1;
        flow_frame2 <= tmp_flow_frame2;
        flow_frame3 <= tmp_flow_frame3;
        flow_frame4 <= tmp_flow_frame4;
        flow_frame5 <= tmp_flow_frame5;
        flow_frame6 <= tmp_flow_frame6;
        flow_frame7 <= tmp_flow_frame7;
    end
end

//==================== 3. 呼吸亮度控制 ====================
reg [31:0] breath_step_cnt; // 当前完整呼吸周期内的 1 ms 节拍计数。
reg [15:0] breath_bright;   // 呼吸模式 16 bit 内部亮度，输出给 ws2812_fast。

wire [15:0] breath_period_ms = {cfg_period_units, 4'b0000} + {cfg_period_units, 2'b00}; // period * 20。
wire [15:0] breath_period_safe = (breath_period_ms < 16'd2) ? 16'd2 : breath_period_ms;
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
    end else if(frame_applied && (tmp_mode[2:0] == MODE_BREATH)) begin
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
reg [11:0]  run_cnt;        // 流水速度计数器，基于 1 ms tick。
reg [2:0]   run_pos;        // 当前流水画面位置。
reg [31:0]  gradient_step_cnt; // 当前完整渐变周期内的 1 ms 节拍计数。
reg [7:0]   disco_cnt;         // Disco 速度计数器，基于 1 ms tick。
reg [2:0]   disco_phase;       // Disco 当前离散相位。
reg [10:0]  gradient_phase;    // 当前渐变相位，范围 0~1535。
reg [191:0] led_rgb_data;   // 输出给 ws2812_fast 的 8 颗 LED RGB 数据。

wire [23:0] cfg_rgb = {cfg_r, cfg_g, cfg_b};               // 当前 App 下发的基础 RGB 颜色。
wire [7:0]  active_flow_frame = get_flow_frame(run_pos);   // 流水当前画面的 8 bit 灯掩码。
wire [3:0]  flow_last_pos = cfg_flow_count - 1'b1;         // 当前流水最后一个有效画面位置。
wire [15:0] output_brightness = (sys_mode == MODE_BREATH) ? breath_bright : {cfg_brightness, cfg_brightness};
wire [15:0] flow_period_ms = {cfg_period_units, 3'b000} + {cfg_period_units, 1'b0}; // period * 10。
wire [15:0] flow_period_safe = (flow_period_ms < 16'd1) ? 16'd1 : flow_period_ms;
wire [15:0] gradient_period_ms = {cfg_period_units, 5'b0} + {cfg_period_units, 4'b0} + {cfg_period_units, 1'b0}; // period * 50。
wire [15:0] gradient_period_safe = (gradient_period_ms < 16'd1) ? 16'd1 : gradient_period_ms;
wire [31:0] gradient_step_next = gradient_step_cnt + 1'b1;
wire [31:0] gradient_phase_numer = gradient_step_next * GRADIENT_PHASE_COUNT;
wire [10:0] gradient_phase_calc = gradient_phase_numer / gradient_period_safe;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        run_cnt <= 12'd0;
        run_pos <= 3'd0;
        gradient_step_cnt <= 32'd0;
        disco_cnt <= 8'd0;
        disco_phase <= 3'd0;
        gradient_phase <= 11'd0;
        led_rgb_data <= 192'd0;
    end else begin
        if(frame_applied) begin
            // 模式或参数更新后重置动画相位，让手机端每次下发都从确定状态开始。
            run_cnt <= 12'd0;
            run_pos <= 3'd0;
            gradient_step_cnt <= 32'd0;
            disco_cnt <= 8'd0;
            disco_phase <= 3'd0;
            gradient_phase <= 11'd0;
        end else if(tick_1ms) begin
            case(sys_mode)
                MODE_FLOW: begin
                    gradient_step_cnt <= 32'd0;
                    disco_cnt <= 8'd0;
                    disco_phase <= 3'd0;
                    gradient_phase <= 11'd0;
                    if(run_cnt >= (flow_period_safe - 1'b1)) begin
                        run_cnt <= 12'd0;
                        // 按已下发画面数量回绕，基础流水和高级流水共用同一套底层协议。
                        if({1'b0, run_pos} >= flow_last_pos) begin
                            run_pos <= 3'd0;
                        end else begin
                            run_pos <= run_pos + 1'b1;
                        end
                    end else begin
                        run_cnt <= run_cnt + 1'b1;
                    end
                end
                MODE_DISCO: begin
                    run_cnt <= 12'd0;
                    gradient_step_cnt <= 32'd0;
                    gradient_phase <= 11'd0;
                    if(disco_cnt == DISCO_STEP_LAST) begin
                        disco_cnt <= 8'd0;
                        disco_phase <= disco_phase + 1'b1;
                    end else begin
                        disco_cnt <= disco_cnt + 1'b1;
                    end
                end
                MODE_GRADIENT,
                MODE_FLOW_GRADIENT: begin
                    run_cnt <= 12'd0;
                    disco_cnt <= 8'd0;
                    disco_phase <= 3'd0;
                    if(gradient_step_cnt >= (gradient_period_safe - 1'b1)) begin
                        gradient_step_cnt <= 32'd0;
                        gradient_phase <= 11'd0;
                    end else begin
                        gradient_step_cnt <= gradient_step_cnt + 1'b1;
                        gradient_phase <= gradient_phase_calc;
                    end
                end
                default: begin
                    run_cnt <= 12'd0;
                    gradient_step_cnt <= 32'd0;
                    disco_cnt <= 8'd0;
                    disco_phase <= 3'd0;
                    gradient_phase <= 11'd0;
                end
            endcase
        end

        case(sys_mode)
            MODE_STATIC: begin
                led_rgb_data <= pack_rgb8(cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb);
            end
            MODE_FLOW: begin
                led_rgb_data <= pack_rgb8(
                    active_flow_frame[0] ? cfg_rgb : 24'd0,
                    active_flow_frame[1] ? cfg_rgb : 24'd0,
                    active_flow_frame[2] ? cfg_rgb : 24'd0,
                    active_flow_frame[3] ? cfg_rgb : 24'd0,
                    active_flow_frame[4] ? cfg_rgb : 24'd0,
                    active_flow_frame[5] ? cfg_rgb : 24'd0,
                    active_flow_frame[6] ? cfg_rgb : 24'd0,
                    active_flow_frame[7] ? cfg_rgb : 24'd0
                );
            end
            MODE_BREATH: begin
                led_rgb_data <= pack_rgb8(cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb, cfg_rgb);
            end
            MODE_DISCO: begin
                led_rgb_data <= pack_rgb8(
                    disco_rgb(disco_phase + 3'd0),
                    disco_rgb(disco_phase + 3'd1),
                    disco_rgb(disco_phase + 3'd2),
                    disco_rgb(disco_phase + 3'd3),
                    disco_rgb(disco_phase + 3'd4),
                    disco_rgb(disco_phase + 3'd5),
                    disco_rgb(disco_phase + 3'd6),
                    disco_rgb(disco_phase + 3'd7)
                );
            end
            MODE_GRADIENT: begin
                led_rgb_data <= pack_rgb8(
                    gradient_cycle_rgb(gradient_phase),
                    gradient_cycle_rgb(gradient_phase),
                    gradient_cycle_rgb(gradient_phase),
                    gradient_cycle_rgb(gradient_phase),
                    gradient_cycle_rgb(gradient_phase),
                    gradient_cycle_rgb(gradient_phase),
                    gradient_cycle_rgb(gradient_phase),
                    gradient_cycle_rgb(gradient_phase)
                );
            end
            default: begin
                led_rgb_data <= pack_rgb8(
                    gradient_cycle_rgb(add_gradient_phase(gradient_phase, 11'd0)),
                    gradient_cycle_rgb(add_gradient_phase(gradient_phase, FLOW_GRADIENT_OFFSET)),
                    gradient_cycle_rgb(add_gradient_phase(gradient_phase, FLOW_GRADIENT_OFFSET * 2)),
                    gradient_cycle_rgb(add_gradient_phase(gradient_phase, FLOW_GRADIENT_OFFSET * 3)),
                    gradient_cycle_rgb(add_gradient_phase(gradient_phase, FLOW_GRADIENT_OFFSET * 4)),
                    gradient_cycle_rgb(add_gradient_phase(gradient_phase, FLOW_GRADIENT_OFFSET * 5)),
                    gradient_cycle_rgb(add_gradient_phase(gradient_phase, FLOW_GRADIENT_OFFSET * 6)),
                    gradient_cycle_rgb(add_gradient_phase(gradient_phase, FLOW_GRADIENT_OFFSET * 7))
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
