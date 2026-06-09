//==============================================================================
// 模块名称: uart_rx_module
// 功能说明: UART 串口接收器。
// 设计说明:
//   1. 默认使用 50 MHz 系统时钟。
//   2. rx 输入先经过两级寄存器同步，再做下降沿检测。
//   3. 检测到起始位下降沿后进入接收状态，依次采样 8 bit 数据位。
//   4. rx_done 在一帧数据结束后输出一个时钟周期的完成标志。
//==============================================================================
module uart_rx_module(
    input            clk,      // 50 MHz 系统时钟。
    input            rst_n,    // 异步低电平复位。
    input            rx,       // UART 串行接收输入，空闲状态为高电平。
    output reg [7:0] rx_data,  // 接收到的 8 bit 数据，低位先接收。
    output reg       rx_done   // 接收完成脉冲，高电平持续一个 clk 周期。
);

//------------------------------------------------------------------------------
// 波特率参数。
// 注意: 13'd5208 对应 50 MHz / 9600 baud；若使用 115200 baud，应改为约 434。
//------------------------------------------------------------------------------
parameter BAUD_CNT_MAX = 13'd5208; // 一个 UART bit 周期对应的系统时钟数。

//------------------------------------------------------------------------------
// 内部状态信号。
//------------------------------------------------------------------------------
reg [12:0] baud_cnt;     // 波特率分频计数器。
reg [3:0]  bit_cnt;      // UART 位计数: 起始位、8 个数据位、停止位。
reg        rx_reg1;      // rx 第一级同步寄存器。
reg        rx_reg2;      // rx 第二级同步寄存器，也是采样数据源。
wire       rx_fall;      // 起始位下降沿检测标志。
reg        rx_flag;      // 接收进行中标志。

//------------------------------------------------------------------------------
// rx 同步与起始位下降沿检测。
//------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rx_reg1 <= 1'b1;
        rx_reg2 <= 1'b1;
    end else begin
        rx_reg1 <= rx;
        rx_reg2 <= rx_reg1;
    end
end
assign rx_fall = (~rx_reg1) & rx_reg2;

//------------------------------------------------------------------------------
// 接收状态控制。
// rx_fall 置位后进入接收状态，停止位中点后退出接收状态。
//------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rx_flag <= 1'b0;
    end else if(rx_fall) begin
        rx_flag <= 1'b1;
    end else if((bit_cnt == 4'd9) && (baud_cnt == BAUD_CNT_MAX/2)) begin
        rx_flag <= 1'b0;
    end
end

//------------------------------------------------------------------------------
// 数据采样与完成标志生成。
// 采样顺序为 bit0 到 bit7；bit_cnt=9 表示停止位阶段。
//------------------------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        baud_cnt <= 13'd0;
        bit_cnt <= 4'd0;
        rx_data <= 8'd0;
        rx_done <= 1'b0;
    end else if(rx_flag) begin
        if(baud_cnt < BAUD_CNT_MAX - 1) begin
            baud_cnt <= baud_cnt + 1'b1;
            rx_done <= 1'b0;
        end else begin
            baud_cnt <= 13'd0;
            bit_cnt <= bit_cnt + 1'b1;
            if(bit_cnt > 0 && bit_cnt < 9) begin
                // bit_cnt=1~8 时分别写入 rx_data[0]~rx_data[7]。
                rx_data[bit_cnt-1] <= rx_reg2;
            end
        end
    end else begin
        baud_cnt <= 13'd0;
        bit_cnt <= 4'd0;
        rx_done <= (bit_cnt == 4'd9) ? 1'b1 : 1'b0;
    end
end
endmodule
