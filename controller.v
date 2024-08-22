`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/07/17 10:12:08
// Design Name: 
// Module Name: controller
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module fnd_cntr(
    input clk, reset_p,
    input [15:0] value,
    output [3:0] com,
    output [7:0] seg_7);
    
    ring_counter_asd rc(.clk(clk), .reset_p(reset_p), .com(com));
    
    reg [3:0] hex_value;
    always @(posedge clk)begin
       case(com)
          4'b1110: hex_value = value[3:0];
          4'b1101: hex_value = value[7:4];
          4'b1011: hex_value = value[11:8];
          4'b0111: hex_value = value[15:12];
       endcase 
    end
    
    decoder_7seg a(.hex_value(hex_value), .seg_7(seg_7));
endmodule

module button_cntr(
    input clk, reset_p,
    input btn,
    output btn_pedge, btn_nedge);
    
    reg [20:0] clk_div;
    always @(posedge clk)clk_div = clk_div + 1;
    
    wire clk_div_nedge;
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(clk_div[16]), .n_edge(clk_div_nedge));
    
    reg debounced_btn;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)debounced_btn = 0;
        else if(clk_div_nedge)debounced_btn = btn;
    end
    
    edge_detector_n ed_btn(.clk(clk), .reset_p(reset_p), .cp(debounced_btn), .n_edge(btn_nedge), .p_edge(btn_pedge));
    
endmodule

module key_pad_cntr(
    input clk, reset_p,
    input [3:0] row,
    output reg [3:0] col,
    output reg [3:0] key_value,
    output reg key_valid);
 
    reg [19:0] clk_div;
    always @(posedge clk)clk_div = clk_div + 1;
    wire clk_8msec_p, clk_8msec_n;
    edge_detector_n ed_btn(.clk(clk), .reset_p(reset_p), .cp(clk_div[19]), .n_edge(clk_8msec_n), .p_edge(clk_8msec_p));        
 
    always @(posedge clk or posedge reset_p)begin
        if(reset_p) col = 4'b0001;
        else if(clk_8msec_p && !key_valid)begin
            case(col)
                4'b0001 : col = 4'b0010;
                4'b0010 : col = 4'b0100;
                4'b0100 : col = 4'b1000;
                4'b1000 : col = 4'b0001;
                default : col = 4'b0001;
            endcase
        end
    end
    
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            key_value = 0;
            key_valid  = 0;
        end
        else begin
          if(clk_8msec_n)begin
                if(row)begin
                    key_valid = 1;
                    case({col, row})
                        8'b0001_0001: key_value = 4'h0;
                        8'b0001_0010: key_value = 4'h1;
                        8'b0001_0100: key_value = 4'h2;
                        8'b0001_1000: key_value = 4'h3;
                        8'b0010_0001: key_value = 4'h4;
                        8'b0010_0010: key_value = 4'h5;
                        8'b0010_0100: key_value = 4'h6;
                        8'b0010_1000: key_value = 4'h7;
                        8'b0100_0001: key_value = 4'h8;
                        8'b0100_0010: key_value = 4'h9;
                        8'b0100_0100: key_value = 4'ha;
                        8'b0100_1000: key_value = 4'hb;
                        8'b1000_0001: key_value = 4'hc;
                        8'b1000_0010: key_value = 4'hd;
                        8'b1000_0100: key_value = 4'he;
                        8'b1000_1000: key_value = 4'hf;
                    endcase
                end
                else begin
                    key_valid  = 0;
         //           key_value = 0;
          end
        end
    end
    end
 endmodule
 
 module keypad_cntr_FSM(
    input clk, reset_p,
    input [3:0] row,
    output reg [3:0] col,
    output reg [3:0] key_value,
    output reg key_valid);
    
    parameter SCAN0 =           5'b00001;
    parameter SCAN1 =           5'b00010;
    parameter SCAN2 =           5'b00100;
    parameter SCAN3 =           5'b01000;
    parameter KEY_PROCESS =  5'b10000;
    
     reg [19:0] clk_div;
    always @(posedge clk)clk_div = clk_div + 1;
    wire clk_8msec_p, clk_8msec_n;
    edge_detector_n ed_btn(.clk(clk), .reset_p(reset_p), .cp(clk_div[19]), .p_edge(clk_8msec_p), .n_edge(clk_8msec_n));
    
    reg[4:0] state, next_state;
    
    // ÃÊ±â »óÅÂ °áÁ¤
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)state = SCAN0;
        else if(clk_8msec_n)state = next_state;
     end
     
     always @* begin
        case(state)
                SCAN0: begin
                if(row == 0)next_state = SCAN1;
                else next_state = KEY_PROCESS;
                end            
                SCAN1: begin
                if(row == 0)next_state = SCAN2;
                else next_state = KEY_PROCESS;
                end            
                SCAN2: begin
                if(row == 0)next_state = SCAN3;
                else next_state = KEY_PROCESS;
                end            
                SCAN3: begin
                if(row == 0)next_state = SCAN0;
                else next_state = KEY_PROCESS;
                end            
                KEY_PROCESS: begin
                if(row == 0)next_state = SCAN0;
                else next_state = KEY_PROCESS;
                end
                default: next_state = SCAN0;
        endcase
     end
     
     // ´ÙÀ½»óÅÂ °áÁ¤À» À§ÇÑ Á¶ÇÕÈ¸·Î ºí·Ï
     always @(negedge clk or posedge reset_p) begin
        if(reset_p)begin
            key_value = 0;
            key_valid = 0;
            col = 0;
        end
        else if(clk_8msec_p)begin
            case(state)
                SCAN0 : begin col = 4'b0001; key_valid = 0; end
                SCAN1 : begin col = 4'b0010; key_valid = 0; end
                SCAN2 : begin col = 4'b0100; key_valid = 0; end
                SCAN3 : begin col = 4'b1000; key_valid = 0; end
                KEY_PROCESS : begin
                    key_valid = 1;
                    case({col, row})
                        8'b0001_0001: key_value = 4'h0;
                        8'b0001_0010: key_value = 4'h1;
                        8'b0001_0100: key_value = 4'h2;
                        8'b0001_1000: key_value = 4'h3;
                        8'b0010_0001: key_value = 4'h4;
                        8'b0010_0010: key_value = 4'h5;
                        8'b0010_0100: key_value = 4'h6;
                        8'b0010_1000: key_value = 4'h7;
                        8'b0100_0001: key_value = 4'h8;
                        8'b0100_0010: key_value = 4'h9;
                        8'b0100_0100: key_value = 4'ha;
                        8'b0100_1000: key_value = 4'hb;
                        8'b1000_0001: key_value = 4'hc;
                        8'b1000_0010: key_value = 4'hd;
                        8'b1000_0100: key_value = 4'he;
                        8'b1000_1000: key_value = 4'hf;
                    endcase
                    end
                    endcase
                    end
                    end
//        else if(clk_8msec)begin
//              if(row)begin
//                    key_valid = 1;
//                    case({col, row})
//                        8'b0001_0001: key_value = 4'h0;
//                        8'b0001_0010: key_value = 4'h1;
//                        8'b0001_0100: key_value = 4'h2;
//                        8'b0001_1000: key_value = 4'h3;
//                        8'b0010_0001: key_value = 4'h4;
//                        8'b0010_0010: key_value = 4'h5;
//                        8'b0010_0100: key_value = 4'h6;
//                        8'b0010_1000: key_value = 4'h7;
//                        8'b0100_0001: key_value = 4'h8;
//                        8'b0100_0010: key_value = 4'h9;
//                        8'b0100_0100: key_value = 4'ha;
//                        8'b0100_1000: key_value = 4'hb;
//                        8'b1000_0001: key_value = 4'hc;
//                        8'b1000_0010: key_value = 4'hd;
//                        8'b1000_0100: key_value = 4'he;
//                        8'b1000_1000: key_value = 4'hf;
//                    endcase
//                end
//                else key_valid = 0;
//                end
//        else begin
//            case(state)
//                4'b0001:col = 4'b0001;
//                4'b0010:col = 4'b0010;
//                4'b0100:col = 4'b0100;
//                4'b1000:col = 4'b1000;
//            endcase
//        end
//     end       
endmodule

module dht11_cntr(
    input clk, reset_p,
    inout dht11_data,
    output reg [7:0] humidity, temperature,
    output [15:0] led_debug);
    parameter S_IDLE           = 6'b00_0001;
    parameter S_LOW_18MS = 6'b00_0010;
    parameter S_HIGH_20US = 6'b00_0100;
    parameter S_LOW_80US  = 6'b00_1000;
    parameter S_HIGH_80US  = 6'b01_0000;
    parameter S_READ_DATA  = 6'b10_0000;
    
    parameter S_WAIT_PEDGE  = 2'b01;
    parameter S_WAIT_NEDGE = 2'b10;
    
    reg [5:0] state, next_state;
    reg [1:0] read_state;
    
    wire clk_microsec;
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_microsec));
    
    reg [21:0] count_microsec;
    reg count_microsec_e;
    
    assign led_debug [5:0] = state;
    
    always @(negedge clk or posedge reset_p)begin
        if(reset_p)count_microsec = 0;
        else if(clk_microsec && count_microsec_e)count_microsec = count_microsec + 1;
        else if(!count_microsec_e)count_microsec = 0;
    end
    
    always @(negedge clk or posedge reset_p)begin
        if(reset_p)state = S_IDLE;
        else state = next_state;
    end
    
    reg dht11_buffer;
    assign dht11_data = dht11_buffer;
    
    wire dht_pedge, dht_nedge;
    edge_detector_n ed(.clk(clk), .reset_p(reset_p), .cp(dht11_data),
    .p_edge(dht_pedge), .n_edge(dht_nedge));
    
    reg [39:0] temp_data;
    reg [5:0] data_count;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            next_state =  S_IDLE;
            read_state = S_WAIT_PEDGE;
            temp_data = 0;
            data_count = 0;
        end
        else begin
            case(state)
                S_IDLE:begin
                    if(count_microsec < 22'd3_000_000)begin // 3s  22'd3_000_000
                        count_microsec_e = 1;
                        dht11_buffer = 'bz;
                    end
                    else begin
                        count_microsec_e = 0;
                        next_state = S_LOW_18MS;
                    end
                end
                S_LOW_18MS:begin
                    if(count_microsec < 22'd20_000)begin // ÃÖ¼Ò 18ms
                        dht11_buffer = 0;
                        count_microsec_e = 1;
                    end
                    else begin
                        count_microsec_e = 0;
                        next_state = S_HIGH_20US;
                        dht11_buffer = 'bz;

                    end
                end
                S_HIGH_20US:begin
//                    if(count_microsec < 22'd3)begin
//                            count_microsec_e = 1;
//                            dht11_buffer = 'bz;   // ÀÓÇÇ´ø½º Ãâ·Â 
//                    end
//                    else
                        count_microsec_e = 1;
                        if(count_microsec > 22'd100_000)begin
                            next_state = S_IDLE;
                            count_microsec_e = 0;
                        end
                        if(dht_nedge)begin
                        count_microsec_e = 0;
                        next_state = S_LOW_80US;
                        
                    end
                end
                S_LOW_80US:begin
                        count_microsec_e = 1;
                        if(count_microsec > 22'd100_000)begin
                            next_state = S_IDLE;
                            count_microsec_e = 0;    
                            end            
                    if(dht_pedge)begin
                        next_state = S_HIGH_80US;
                        count_microsec_e = 0;
                    end
                end
                S_HIGH_80US:begin
                        count_microsec_e = 1;
                        if(count_microsec > 22'd100_000)begin
                            next_state = S_IDLE;
                            count_microsec_e = 0;    
                            end   
                    if(dht_nedge)begin
                        next_state = S_READ_DATA;
                        count_microsec_e = 0;
                    end
                end
                S_READ_DATA:begin
                    count_microsec_e = 1;
                        if(count_microsec > 22'd100_000)begin
                            next_state = S_IDLE;
                            count_microsec_e = 0;  
                            data_count = 0;
                            read_state = S_WAIT_PEDGE;
                        end
                        else begin    
                    case(read_state)
                        S_WAIT_PEDGE:begin
                            if(dht_pedge)read_state = S_WAIT_NEDGE;
                        end
                        S_WAIT_NEDGE:begin
                            if(dht_nedge)begin
                                if(count_microsec < 95)begin
                                    temp_data = {temp_data[38:0], 1'b0};
                                end
                                else begin
                                    temp_data = {temp_data[38:0], 1'b1};                                    
                                end
                                data_count = data_count + 1;
                                read_state = S_WAIT_PEDGE;  
                                count_microsec_e = 0;                               
                            end
                            else begin
                                count_microsec_e = 1;
                            end
                        end
                    endcase    
                    if(data_count >= 40)begin
                        data_count = 0;
                        next_state = S_IDLE;
                        count_microsec_e = 0;  
                        read_state = S_WAIT_PEDGE;        
                        if(temp_data[39:32] + temp_data[31:24] + temp_data[23:16] + temp_data[15:8] == temp_data[7:0])begin
                        humidity = temp_data[39:32];
                        temperature = temp_data[23:16];
                        end
                    end    
                end
                end
            endcase
        end
    end
endmodule

module hcsr04_cntr(
    input clk, 
    input reset_p,
    output reg trigger,
    input echo,
    output reg [15:0] distance,
    output [15:0] led_debug
);

    // »óÅÂ Á¤ÀÇ (3ºñÆ®)
    parameter S_IDLE          = 3'b001;
    parameter S_TRIG          = 3'b010;
    parameter S_WAIT_ECHO     = 3'b100;

    // »óÅÂ ¹× Ä«¿îÅÍ º¯¼ö
    reg [2:0] state, next_state;
    reg [22:0] count_microsec;
    reg count_microsec_e;

    wire clk_microsec;
    wire echo_pedge, echo_nedge;

    // Å¬·° ºÐÁÖ±â ¸ðµâ ÀÎ½ºÅÏ½º
    clock_div_100 microsec_clk(
        .clk(clk),
        .reset_p(reset_p),
        .clk_div_100_nedge(clk_microsec)
    );

    // ¿§Áö °¨Áö ¸ðµâ ÀÎ½ºÅÏ½º
    edge_detector_n ed(
        .clk(clk),
        .reset_p(reset_p),
        .cp(echo),
        .p_edge(echo_pedge),
        .n_edge(echo_nedge)
    );
    
     // distance °Å¸® °è»êÇÏ±âÀ§ÇØ 58ºÐÁÖ±â 
    reg cnt_e;
    wire [11:0] cm;
    hcsr04_58 div5(.clk(clk), .reset_p(reset_p), .clk_microsec(clk_microsec), .cnt_enable(cnt_e), .cm(cm));
    
    
    // µð¹ö±ë LED »óÅÂ
    assign led_debug[2:0] = state; // »óÅÂ¸¦ LED¿¡ Ãâ·Â

    // Ä«¿îÅÍ Áõ°¡ ¹× ¸®¼Â
    always @(negedge clk or posedge reset_p) begin
        if (reset_p)
            count_microsec <= 0;
        else if (clk_microsec && count_microsec_e)
            count_microsec <= count_microsec + 1;
        else if (!count_microsec_e)
            count_microsec <= 0;
    end

    // »óÅÂ ¾÷µ¥ÀÌÆ®
    always @(negedge clk or posedge reset_p) begin
        if (reset_p)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    // HC-SR04 µ¿ÀÛ ·ÎÁ÷
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
        next_state <= S_IDLE;
        trigger <= 0;
        distance <= 0;
        count_microsec_e <= 0;
        cnt_e <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (count_microsec < 22'd1_000_000) begin // 1ÃÊ ´ë±â
                        count_microsec_e <= 1;
                        trigger <= 0;
                    end else begin
                        count_microsec_e <= 0;
                        next_state <= S_TRIG;
                    end
                end
                
                S_TRIG: begin
                    if (count_microsec < 22'd10) begin // 10us Æ®¸®°Å ½ÅÈ£
                        trigger <= 1;
                        count_microsec_e <= 1;
                    end else begin
                        trigger <= 0;
                        count_microsec_e <= 0;
                        next_state <= S_WAIT_ECHO;
                    end
                end
                
                S_WAIT_ECHO: begin
                    if (echo_pedge) begin
                        cnt_e = 1; // Echo 
                    end
                    if (echo_nedge) begin
                        distance = cm;
                        cnt_e = 0; 
                        next_state <= S_IDLE;
                    end
                end
                default: next_state <= S_IDLE;
            endcase
        end
    end
endmodule

module pwm_100step(
    input clk, reset_p,
    input [6:0] duty,
    output pwm);
    
   parameter sys_clk_freq = 100_000_000;
   parameter pwm_freq = 10_000;
   parameter duty_step = 100; 
   parameter temp = sys_clk_freq / duty_step / pwm_freq;
   parameter temp_half = temp / 2;
    
   integer cnt_sysclk;
   wire pwm_freqX100;
   always @(negedge clk or posedge reset_p)begin
        if(reset_p)cnt_sysclk = 0;
        else begin
        if(cnt_sysclk >= temp-1) cnt_sysclk = 0; 
        else cnt_sysclk = cnt_sysclk + 1;
        end
   end

   assign pwm_freqX100 = (cnt_sysclk < temp_half) ? 1 : 0;
   wire pwm_freqX100_nedge;
   edge_detector_n n(
    .clk(clk), .reset_p(reset_p),
    .cp(pwm_freqX100),
    .n_edge(pwm_freqX100_nedge));
    
   reg [6:0] cnt_duty;
   
   always @(negedge clk or posedge reset_p)begin
        if(reset_p)cnt_duty = 0;
        else if(pwm_freqX100_nedge)begin
            cnt_duty = cnt_duty + 1;
        end
   end

   assign pwm = (cnt_duty < duty) ? 1 : 0;
   
endmodule

module led_pwm_top(
    input clk, reset_p,
    output pwm, led_r, led_g, led_b
);
    reg [31:0] clk_div;
    always @(posedge clk)clk_div = clk_div + 1;
    
    pwm_100step pwm_inst(.clk(clk), .reset_p(reset_p), .duty(clk_div[27:21]), .pwm(pwm));
    pwm_nstep_freq #(.duty_step(77)) pwm_r( .clk(clk), .reset_p(reset_p), .duty(clk_div[28:23]), .pwm(led_r));
    pwm_nstep_freq #(.duty_step(93)) pwm_g(.clk(clk), .reset_p(reset_p), .duty(clk_div[27:22]), .pwm(led_g));
    pwm_nstep_freq #(.duty_step(103)) pwm_b(.clk(clk), .reset_p(reset_p), .duty(clk_div[26:21]), .pwm(led_b));

endmodule

module pwm_nstep_freq
#(
   parameter sys_clk_freq = 100_000_000,
   parameter pwm_freq = 10_000,
   parameter duty_step = 100,
   parameter temp = sys_clk_freq / duty_step / pwm_freq,
   parameter temp_half = temp / 2)
(
    input clk, reset_p,
    input [31:0] duty,
    output pwm);
    
   integer cnt_sysclk;
   wire pwm_freqXstep;
   always @(negedge clk or posedge reset_p)begin
        if(reset_p)cnt_sysclk = 0;
        else begin
        if(cnt_sysclk >= temp-1) cnt_sysclk = 0; 
        else cnt_sysclk = cnt_sysclk + 1;
        end
   end

   assign pwm_freqXstep = (cnt_sysclk < temp_half) ? 1 : 0;
   wire pwm_freqXstep_nedge;
   edge_detector_n n(
    .clk(clk), .reset_p(reset_p),
    .cp(pwm_freqXstep),
    .n_edge(pwm_freqXstep_nedge));
    
   integer cnt_duty;
   
   always @(negedge clk or posedge reset_p)begin
        if(reset_p)cnt_duty = 0;
        else if(pwm_freqXstep_nedge)begin
            if(cnt_duty >= (duty_step-1))cnt_duty = 0;
            cnt_duty = cnt_duty + 1;
        end
   end

   assign pwm = (cnt_duty < duty) ? 1 : 0;
   
endmodule

module led_abc_top(
    input clk, reset_p,
    input btn,
    output [15:0] led_debug
);
    
    reg [1:0] brightness_level; // 2비트 밝기 레벨
    reg [6:0] duty_cycle;
    wire pwm_signal;
    reg btn1_last_state;

    // 밝기 단계에 따른 duty cycle 설정
    always @(*) begin
        case (brightness_level)
            2'b00: duty_cycle = 7'd0;   // LED 꺼짐
            2'b01: duty_cycle = 7'd25;  // LED 약한 밝기
            2'b10: duty_cycle = 7'd50;  // LED 중간 밝기
            2'b11: duty_cycle = 7'd75;  // LED 최대 밝기
            default: duty_cycle = 7'd0;
        endcase
    end

    // 밝기 레벨 변경 - btn[1]이 눌릴 때마다 변경
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            brightness_level <= 2'b00; // 초기 밝기 레벨
            btn1_last_state <= 1'b0;
        end else begin
            // btn[1]이 눌렸을 때 (rising edge 감지)
            if (btn && !btn1_last_state) begin
                brightness_level <= brightness_level + 1;
            end
            btn1_last_state <= btn;
        end
    end

    // PWM 모듈 인스턴스
    pwm_100step pwm_inst(
        .clk(clk), 
        .reset_p(reset_p), 
        .duty(duty_cycle), 
        .pwm(pwm_signal)
    );

    // 모든 LED에 동일한 PWM 신호를 적용
    assign led_debug = {16{pwm_signal}};

endmodule


module I2C_controller(
    input clk, reset_p,
    input [6:0] addr, // 주소 전송
    input rd_wr, // read가 1이고 write가 0임
    input [7:0] data,
    input comm_go, // 통신 스타트 신호 comm_go에 1이들어오면 I2C를통해 SCL과 SDA와 통신
    output reg scl, sda, // scl = 클락, sda = 데이터선
    output reg [15:0] led_debug // 디버깅을 위한 led 
);

    parameter IDLE       = 7'b000_0001;
    parameter COMM_START = 7'b000_0010;
    parameter SEND_ADDR  = 7'b000_0100;       
    parameter RD_ACK     = 7'b000_1000;           
    parameter SEND_DATA  = 7'b001_0000;
    parameter SCL_STOP   = 7'b010_0000;
    parameter COMM_STOP  = 7'b100_0000;

    wire [7:0] addr_rw;
    assign addr_rw = {addr, rd_wr};

    wire clk_microsec;
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_microsec));

    reg [2:0] count_microsec5;
    reg scl_e;

    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            count_microsec5 = 0;
            scl = 1;
        end
        else if(scl_e)begin
            if(clk_microsec)begin
                if(count_microsec5 >= 4)begin
                    count_microsec5 = 0;
                    scl = ~scl;
                end 
                else count_microsec5 = count_microsec5 + 1;
            end
        end
        else if(!scl_e)begin
            scl = 1;
            count_microsec5 = 0;
        end
    end

    wire scl_nedge, scl_pedge;
    edge_detector_n na(
    .clk(clk), .reset_p(reset_p), .cp(scl),
    .n_edge(scl_nedge), .p_edge(scl_pedge));

    wire comm_go_pedge;
    edge_detector_n ns(
    .clk(clk), .reset_p(reset_p), .cp(comm_go), .p_edge(comm_go_pedge));

    reg [6:0] state, next_state;
    always @(negedge clk or posedge reset_p)begin
        if(reset_p)state = IDLE;
        else state = next_state;
    end

    reg [2:0] cnt_bit;
    reg stop_flag;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            next_state = IDLE;
            scl_e      = 0;
            sda        = 1;
            cnt_bit    = 7;
            stop_flag  = 0;
        end 
        else begin
             case(state)
                IDLE:begin
                    scl_e = 0;
                    sda = 1;
                    if(comm_go_pedge)next_state = COMM_START;
                end
                COMM_START:begin
                    sda = 0;
                    scl_e = 1;
                    next_state = SEND_ADDR;
                end
                SEND_ADDR:begin
                    if(scl_nedge)sda = addr_rw[cnt_bit];
                    if(scl_pedge)begin
                        if(cnt_bit == 0)begin
                            cnt_bit = 7;
                            next_state = RD_ACK;
                        end
                        else cnt_bit = cnt_bit - 1;
                    end
                end       
                RD_ACK:begin
                    if(scl_nedge)sda = 'bz;
                    else if(scl_pedge)begin
                        if(stop_flag)begin
                        stop_flag = 0;
                        next_state = SCL_STOP;
                        end
                        else begin
                            stop_flag = 1;
                            next_state = SEND_DATA;
                        end              
                    end
                end       
                SEND_DATA:begin
                    if(scl_nedge)sda = data[cnt_bit];
                    if(scl_pedge)begin
                        if(cnt_bit == 0)begin
                            cnt_bit = 7;
                            next_state = RD_ACK;
                        end
                        else cnt_bit = cnt_bit - 1;
                    end
                end
                SCL_STOP:begin
                    if(scl_nedge)sda = 0;
                    else if(scl_pedge)next_state = COMM_STOP;
                end
                COMM_STOP:begin
                    if(count_microsec5 >= 3)begin
                        scl_e = 0;
                        sda = 1;
                        next_state = IDLE;
                    end
                end 
             endcase
        end
    end
endmodule

module i2c_lcd_send_byte(
    input clk, reset_p,
    input [6:0] addr,
    input [7:0] send_buffer,
    input rs, send,
    output scl, sda,
    output reg busy,
    output [15:0] led_debug);
 
    parameter IDLE                     = 6'b00_0001;
    parameter SEND_HIGH_NIBBLE_DISABLE = 6'b00_0010;
    parameter SEND_HIGH_NIBBLE_ENABLE  = 6'b00_0100;
    parameter SEND_LOW_NIBBLE_DISABLE  = 6'b00_1000;
    parameter SEND_LOW_NIBBLE_ENABLE   = 6'b01_0000;
    parameter SEND_DISABLE             = 6'b10_0000;

    reg [7:0] data;
    reg comm_go;

    wire send_pedge;
    edge_detector_n send_inst(
    .clk(clk), .reset_p(reset_p), .cp(send), .p_edge(send_pedge));

    wire clk_microsec;
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_microsec));


    reg [21:0] count_microsec;
    reg count_microsec_e;

    always @(negedge clk or posedge reset_p)begin
        if(reset_p)count_microsec = 0;
        else if(clk_microsec && count_microsec_e)count_microsec = count_microsec + 1;
        else if(!count_microsec_e)count_microsec = 0;
    end
    
    reg [5:0] state, next_state;
    always @(negedge clk or posedge reset_p)begin
        if(reset_p)state = IDLE;
        else state = next_state;
    end

    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            next_state       = IDLE;
            busy             = 0;
            comm_go          = 0;
            data             = 0;
            count_microsec_e = 0;
        end
        else begin
            case(state)
                IDLE:begin
                    if(send_pedge)begin
                        next_state = SEND_HIGH_NIBBLE_DISABLE;
                        busy = 1;
                    end
                end
                SEND_HIGH_NIBBLE_DISABLE:begin
                    if(count_microsec <= 22'd200 )begin
                        data             = {send_buffer[7:4],3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS
                        comm_go          = 1;
                        count_microsec_e = 1;
                    end
                    else begin
                        count_microsec_e = 0;
                        comm_go          = 0;
                        next_state       = SEND_HIGH_NIBBLE_ENABLE;
                    end
                end
                SEND_HIGH_NIBBLE_ENABLE:begin
                    if(count_microsec <= 22'd200 )begin
                        data             = {send_buffer[7:4],3'b110, rs}; // [d7 d6 d5 d4], BT, E, RW, RS
                        comm_go          = 1;
                        count_microsec_e = 1;
                    end
                    else begin
                        count_microsec_e = 0;
                        comm_go          = 0;
                        next_state       = SEND_LOW_NIBBLE_DISABLE;
                    end
                end
                SEND_LOW_NIBBLE_DISABLE:begin
                    if(count_microsec <= 22'd200 )begin
                        data             = {send_buffer[3:0],3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS
                        comm_go          = 1;
                        count_microsec_e = 1;
                    end
                    else begin
                        count_microsec_e = 0;
                        comm_go          = 0;
                        next_state       = SEND_LOW_NIBBLE_ENABLE;
                    end                    
                end
                SEND_LOW_NIBBLE_ENABLE:begin
                     if(count_microsec <= 22'd200 )begin
                        data             = {send_buffer[3:0],3'b110, rs}; // [d7 d6 d5 d4], BT, E, RW, RS
                        comm_go          = 1;
                        count_microsec_e = 1;
                    end
                    else begin
                        count_microsec_e = 0;
                        comm_go          = 0;
                        next_state       = SEND_DISABLE;
                    end                       
                end
                SEND_DISABLE:begin
                    if(count_microsec <= 22'd200 )begin
                        data             = {send_buffer[3:0],3'b100, rs}; // [d7 d6 d5 d4], BT, E, RW, RS
                        comm_go          = 1;
                        count_microsec_e = 1;
                    end
                    else begin
                        count_microsec_e = 0;
                        comm_go          = 0;
                        next_state       = IDLE;
                        busy             = 0;
                    end    
                end
            endcase
        end
    end

    I2C_controller master(.clk(clk), .reset_p(reset_p), .addr(addr),
    .rd_wr(0), .data(data), .comm_go(comm_go), .scl(scl), .sda(sda), .led_debug(led_debug));

endmodule