// 标准UART接收器 (波特率9600 @ 50MHz时钟)
module uart_rx_module(
    input            clk,
    input            rst_n,
    input            rx,
    output reg [7:0] rx_data,
    output reg       rx_done
);

parameter BAUD_CNT_MAX = 13'd5208; // 50MHz / 115200 = 434

reg [12:0] baud_cnt;
reg [3:0]  bit_cnt;
reg        rx_reg1, rx_reg2;
wire       rx_fall;
reg        rx_flag;

// 边沿检测与同步防亚稳态
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

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        rx_flag <= 1'b0;
    end else if(rx_fall) begin
        rx_flag <= 1'b1;
    end else if((bit_cnt == 4'd9) && (baud_cnt == BAUD_CNT_MAX/2)) begin
        rx_flag <= 1'b0;
    end
end

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