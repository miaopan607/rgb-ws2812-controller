// 简单快速 WS2812 驱动。
// 按 50MHz 时钟生成 WS2812 时序，每帧发送 8 个灯，帧间复位时间约 60us。
module ws2812_fast(
    input        clk,
    input        rst_n,
    input  [7:0] led_data_in32,
    input  [7:0] led_data_in10,
    input        mode,
    input  [3:0] led_brightness,
    output reg   led_out
);

localparam integer BIT_TOTAL_CYCLES = 63;   // 约 1.26us
localparam integer T0H_CYCLES       = 20;   // 约 0.40us
localparam integer T1H_CYCLES       = 40;   // 约 0.80us
localparam integer RESET_CYCLES     = 3000; // 约 60us

reg [5:0]  bit_cnt;
reg [4:0]  bit_index;
reg [2:0]  led_index;
reg [11:0] reset_cnt;
reg        reset_state;

wire [7:0] bright_value = {led_brightness, led_brightness};
wire [23:0] current_color = get_color(led_index);
wire        current_bit = current_color[23 - bit_index];
wire [5:0]  high_cycles = current_bit ? T1H_CYCLES[5:0] : T0H_CYCLES[5:0];

function [1:0] get_color_code;
    input [2:0] index;
    begin
        case (index)
            3'd0: get_color_code = led_data_in10[1:0];
            3'd1: get_color_code = led_data_in10[5:4];
            3'd2: get_color_code = led_data_in32[1:0];
            3'd3: get_color_code = led_data_in32[5:4];
            3'd4: get_color_code = led_data_in10[3:2];
            3'd5: get_color_code = led_data_in10[7:6];
            3'd6: get_color_code = led_data_in32[3:2];
            default: get_color_code = led_data_in32[7:6];
        endcase
    end
endfunction

function [23:0] get_color;
    input [2:0] index;
    reg [1:0] color_code;
    begin
        color_code = mode ? get_color_code(index) : 2'b00;
        case (color_code)
            2'b01: get_color = {bright_value, 8'd0, 8'd0}; // 绿
            2'b10: get_color = {8'd0, bright_value, 8'd0}; // 红
            2'b11: get_color = {8'd0, 8'd0, bright_value}; // 蓝
            default: get_color = 24'd0;
        endcase
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        led_out <= 1'b0;
        bit_cnt <= 6'd0;
        bit_index <= 5'd0;
        led_index <= 3'd0;
        reset_cnt <= 12'd0;
        reset_state <= 1'b1;
    end else if (reset_state) begin
        led_out <= 1'b0;
        if (reset_cnt == RESET_CYCLES - 1) begin
            reset_cnt <= 12'd0;
            reset_state <= 1'b0;
            bit_cnt <= 6'd0;
            bit_index <= 5'd0;
            led_index <= 3'd0;
        end else begin
            reset_cnt <= reset_cnt + 1'b1;
        end
    end else begin
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
