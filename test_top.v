`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/07/16 14:15:35
// Design Name: 
// Module Name: test_top
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


module board_led_switch_test_top(
    input [15:0] switch,
    output [15:0] led);
    
    assign led = switch;
    
endmodule

module fnd_test_top(
    input clk, reset_p,
    input [15:0] switch,
    output [3:0] com,
    output [7:0] seg_7);
    
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(switch), .com(com), .seg_7(seg_7));

endmodule

module watch_top(
    input clk, reset_p,
    input [2:0] btn,
    output [3:0] com,
    output [7:0] seg_7);
    
    wire btn_mode;
    wire btn_sec;
    wire btn_min;
    wire set_watch;
    wire inc_sec, inc_min;
    wire clk_microsec, clk_millisec, clk_sec, clk_min;
    wire [3:0] sec1, sec10, min1, min10;
    wire [15:0] value;    
    
    // button debounce circuit
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_mode));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_sec));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_min));
    
//    edge_detector_n ed_btn0(
//        .clk(clk), .reset_p(reset_p), 
//        .cp(btn[0]), .p_edge(btn_mode ) );

//    edge_detector_n ed_btn1(
//        .clk(clk), .reset_p(reset_p), 
//        .cp(btn[1]), .p_edge(btn_sec ) );

//    edge_detector_n ed_btn2(
//        .clk(clk), .reset_p(reset_p), 
//        .cp(btn[2]), .p_edge(btn_min ) );
    
    T_flip_flop_p t_mode(.clk(clk), .reset_p(reset_p), .t(btn_mode), .q(set_watch));
    
    // add mux
    assign inc_sec = set_watch ? btn_sec : clk_sec;
    assign inc_min = set_watch ? btn_min : clk_min;
    
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_microsec));
    clock_div_1000 millisec_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(clk_microsec), .clk_div_1000(clk_millisec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(clk_millisec), .clk_div_1000_nedge(clk_sec));    
    clock_div_60 min_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(inc_sec), .clk_div_60_nedge(clk_min));
 
    counter_bcd_60 counter_sec(.clk(clk), .reset_p(reset_p), .clk_time(inc_sec),
    .bcd1(sec1), .bcd10(sec10));
    counter_bcd_60 counter_min(.clk(clk), .reset_p(reset_p), .clk_time(inc_min),
    .bcd1(min1), .bcd10(min10));
    
    assign value = {min10, min1, sec10, sec1};
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
    
endmodule



module loadable_watch_top(
    input clk, reset_p,
    input [2:0] btn,
    output [3:0] com,
    output [7:0] seg_7);
    
    wire btn_mode;
    wire btn_sec;
    wire btn_min;
    wire set_watch;
    wire inc_sec, inc_min;
    wire clk_microsec, clk_millisec, clk_sec, clk_min;
    wire [3:0] watch_sec1, watch_sec10, watch_min1, watch_min10;
    wire [3:0] set_sec1, set_sec10, set_min1, set_min10;
    wire [15:0] watch_value, set_value, value;

    // button debounce circuit
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_mode));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_sec));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_min));

    
    T_flip_flop_p t_mode(.clk(clk), .reset_p(reset_p), .t(btn_mode), .q(set_watch));
    
    wire watch_load_en, set_load_en;
    edge_detector_n ed_loaded(
    .clk(clk), .reset_p(reset_p), .cp(set_watch),
    .n_edge(watch_load_en), .p_edge(set_load_en));
    
    // add mux
    assign inc_sec = set_watch ? btn_sec : clk_sec;
    assign inc_min = set_watch ? btn_min : clk_min;
    
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_microsec));
    clock_div_1000 millisec_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(clk_microsec), .clk_div_1000(clk_millisec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(clk_millisec), .clk_div_1000_nedge(clk_sec));    
    clock_div_60 min_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(inc_sec), .clk_div_60_nedge(clk_min));
 
    loadable_counter_bcd_60 sec_watch(
    .clk(clk), .reset_p(reset_p), .clk_time(clk_sec),
    .load_enable(watch_load_en), .load_bcd1(set_sec1), .load_bcd10(set_sec10),
    .bcd1(watch_sec1), .bcd10(watch_sec10));
    loadable_counter_bcd_60 min_watch(
    .clk(clk), .reset_p(reset_p), .clk_time(clk_min),
    .load_enable(watch_load_en), .load_bcd1(set_min1), .load_bcd10(set_min10),
    .bcd1(watch_min1), .bcd10(watch_min10));
    
    loadable_counter_bcd_60 sec_set(
    .clk(clk), .reset_p(reset_p), .clk_time(btn_sec),
    .load_enable(set_load_en), .load_bcd1(watch_sec1), .load_bcd10(watch_sec10),
    .bcd1(set_sec1), .bcd10(set_sec10));
    loadable_counter_bcd_60 min_set(
    .clk(clk), .reset_p(reset_p), .clk_time(btn_min),
    .load_enable(set_load_en), .load_bcd1(watch_min1), .load_bcd10(watch_min10),
    .bcd1(set_min1), .bcd10(set_min10));
    
    assign set_value = {set_min10, set_min1, set_sec10, set_sec1};
    assign watch_value = {watch_min10, watch_min1, watch_sec10, watch_sec1};
    assign value = set_watch ? set_value : watch_value;
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
    
endmodule

module stop_watch_top(
    input clk, reset_p,
    input [2:0] btn,
    output [3:0] com,
    output [7:0] seg_7,
    output led_start, led_lap);
    
    wire btn_start, btn_clear;
    wire start_stop; 
    reg lap;
    wire clk_start, btn_lap;
    wire clk_microsec, clk_millisec, clk_sec, clk_min;
    wire [3:0] min10, min1, sec10, sec1;
    wire [15:0] value;
    wire reset_start;
    
    assign clk_start = start_stop ? clk : 0; 
    
    clock_div_100 microsec_clk(.clk(clk_start), .reset_p(reset_start), .clk_div_100(clk_microsec));
    clock_div_1000 millisec_clk(.clk(clk_start), .reset_p(reset_start),
     .clk_source(clk_microsec), .clk_div_1000(clk_millisec));
    clock_div_1000 sec_clk(.clk(clk_start), .reset_p(reset_start),
     .clk_source(clk_millisec), .clk_div_1000_nedge(clk_sec));    
    clock_div_60 min_clk(.clk(clk_start), .reset_p(reset_start),
     .clk_source(clk_sec), .clk_div_60_nedge(clk_min));

     // button cntr 
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_start));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_lap));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_clear));
    
    assign reset_start = reset_p | btn_clear;

    // Â½Ãƒ?Ã› ÂµÃ‡Â´Ã‚ Â»Ã³Ã…Ã‚Â°Â¡ Â½ÂºÃ…Â¾Â»Ã³Ã…Ã‚
    T_flip_flop_p t_start(.clk(clk), .reset_p(reset_start), .t(btn_start), .q(start_stop));
    assign led_start = start_stop; // T Ã‡ÃƒÂ¸Â³Ã‡ÃƒÂ·Ã“ ?Ã‡ Â»Ã³Ã…Ã‚Â¸Â¦ led Â·ÃŽ ÃˆÂ®?ÃŽ

    // T Ã‡ÃƒÂ¸Â³Ã‡ÃƒÂ·Ã“ ?Â» Â»Ã§Â¿Ã«Ã‡Ã˜ system Â¸Â®Â¼Ã‚?ÃŒ Â¾Ã†Â´Ã‘ clearÂ±Ã¢Â´Ã‰Â¸Â¸?Â» Â»Ã§Â¿Ã«Ã‡ÃÂ±Ã¢ ?Â§Ã‡Ã˜ Â»Ã§Â¿Ã« Â¿Â¹Â½Ãƒ 
        always @(posedge clk or posedge reset_p)begin
        if(reset_p)lap = 0;
        else begin
            if(btn_lap) lap = ~lap;
            else if(btn_clear) lap = 0;
            end
       end   
    assign led_lap = lap;
    
    counter_bcd_60_clear counter_sec(.clk(clk), .reset_p(reset_p), .clk_time(clk_sec),
    .bcd1(sec1), .bcd10(sec10), .clear(btn_clear));
    counter_bcd_60_clear counter_min(.clk(clk), .reset_p(reset_p), .clk_time(clk_min),
    .bcd1(min1), .bcd10(min10), .clear(btn_clear));
    
    reg [15:0] lap_time; 
    wire [15:0] cur_time;
    assign cur_time = {min10, min1, sec10, sec1};
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)lap_time = 0;
        else if(btn_lap)lap_time = cur_time;
        else if(btn_clear) lap_time = 0;
    end

    assign value = lap ? lap_time : cur_time;
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
endmodule

module stop_watch_milli_top(
    input clk, reset_p,
    input [2:0] btn,
    output [3:0] com,
    output [7:0] seg_7,
    output led_start, led_lap);
    
    wire btn_start, btn_clear;
    wire start_stop; 
    reg lap;
    wire clk_start, btn_lap;
    wire clk_microsec, clk_millisec, clk_sec, clk_milli;
    wire [3:0] sec10, sec1, milli10, milli1;
    wire [15:0] value;
    wire reset_start;
    
    assign clk_start = start_stop ? clk : 0; 
    
    clock_div_100 microsec_clk(.clk(clk_start), .reset_p(reset_start), .clk_div_100(clk_microsec));
    clock_div_1000 millisec_clk(.clk(clk_start), .reset_p(reset_start),
     .clk_source(clk_microsec), .clk_div_1000(clk_millisec));
    clock_div_1000 sec_clk(.clk(clk_start), .reset_p(reset_start),
     .clk_source(clk_millisec), .clk_div_1000_nedge(clk_sec));    
    clock_div_10 ms(.clk(clk), .reset_p(reset_start), // 0.01
     .clk_source(clk_millisec), .clk_div_10(clk_milli));
     // button cntr 
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_start));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_lap));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_clear));
    
    assign reset_start = reset_p | btn_clear;

    // Â½Ãƒ?Ã› ÂµÃ‡Â´Ã‚ Â»Ã³Ã…Ã‚Â°Â¡ Â½ÂºÃ…Â¾Â»Ã³Ã…Ã‚
    T_flip_flop_p t_start(.clk(clk), .reset_p(reset_start), .t(btn_start), .q(start_stop));
    assign led_start = start_stop; // T Ã‡ÃƒÂ¸Â³Ã‡ÃƒÂ·Ã“ ?Ã‡ Â»Ã³Ã…Ã‚Â¸Â¦ led Â·ÃŽ ÃˆÂ®?ÃŽ

    // T Ã‡ÃƒÂ¸Â³Ã‡ÃƒÂ·Ã“ ?Â» Â»Ã§Â¿Ã«Ã‡Ã˜ system Â¸Â®Â¼Ã‚?ÃŒ Â¾Ã†Â´Ã‘ clearÂ±Ã¢Â´Ã‰Â¸Â¸?Â» Â»Ã§Â¿Ã«Ã‡ÃÂ±Ã¢ ?Â§Ã‡Ã˜ Â»Ã§Â¿Ã« Â¿Â¹Â½Ãƒ 
        always @(posedge clk or posedge reset_p)begin
        if(reset_p)lap = 0;
        else begin
            if(btn_lap) lap = ~lap;
            else if(btn_clear) lap = 0;
            end
       end   
    assign led_lap = lap;
    
    counter_bcd_60_clear counter_sec(.clk(clk), .reset_p(reset_start), .clk_time(clk_sec),
    .bcd1(sec1), .bcd10(sec10), .clear(btn_clear));
   counter_bcd_100_clear counter_milli(.clk(clk), .reset_p(reset_start), .clk_time(clk_milli),
    .bcd10(milli1), .bcd100(milli10), .clear(btn_clear));
    
    reg [15:0] lap_time; 
    wire [15:0] cur_time;
    assign cur_time = {sec10, sec1, milli10, milli1};
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)lap_time = 0;
        else if(btn_lap)lap_time = cur_time;
        else if(btn_clear) lap_time = 0;
    end

    assign value = lap ? lap_time : cur_time;
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
endmodule

module cook_timer_top(
    input clk, reset_p,
    input [4:0] btn,
    output [3:0] com,
    output [7:0] seg_7,
    output led_alarm, led_start, buzz);
    
    wire clk_microsec, clk_millisec, clk_sec, clk_min;
    wire btn_start, btn_sec, btn_min, btn_alarm_off;    
    wire [3:0] set_sec1, set_sec10, set_min1, set_min10;
    wire [3:0] cur_sec1, cur_sec10, cur_min1, cur_min10;     
    reg start_set, alarm;
    wire [15:0] value, cur_time, set_time;
    wire dec_clk;
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_microsec));
    clock_div_1000 millisec_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(clk_microsec), .clk_div_1000(clk_millisec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(clk_millisec), .clk_div_1000_nedge(clk_sec));    
    clock_div_60 min_clk(.clk(clk), .reset_p(reset_p),
     .clk_source(clk_sec), .clk_div_60_nedge(clk_min));

    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_start));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_sec));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_min));
    button_cntr btn3(.clk(clk), .reset_p(reset_p), .btn(btn[4]), .btn_pedge(btn_alarm_off));
    
    counter_bcd_60 counter_sec(.clk(clk), .reset_p(reset_p), .clk_time(btn_sec), .bcd1(set_sec1), .bcd10(set_sec10));
    counter_bcd_60 counter_min(.clk(clk), .reset_p(reset_p), .clk_time(btn_min), .bcd1(set_min1), .bcd10(set_min10));
    
    loadable_down_counter_bcd_60 cur_sec(.clk(clk), .reset_p(reset_p), .clk_time(clk_sec), .load_enable(btn_start),
     .load_bcd1(set_sec1), .load_bcd10(set_sec10), .bcd1(cur_sec1), .bcd10(cur_sec10), .dec_clk(dec_clk));   
   
    loadable_down_counter_bcd_60 cur_min(.clk(clk), .reset_p(reset_p), .clk_time(dec_clk), .load_enable(btn_start),
     .load_bcd1(set_min1), .load_bcd10(set_min10), .bcd1(cur_min1), .bcd10(cur_min10));
    
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            start_set = 0;
            alarm = 0;
        end
        else begin
            if(btn_start)start_set = ~start_set;
            else if(cur_time == 0 && start_set)begin
                start_set = 0;
                alarm = 1;
            end
            else if(btn_alarm_off) alarm = 0;
        end
    end

    assign led_alarm = alarm;
    assign led_start = start_set;
    assign buzz = alarm;
    assign set_time = {set_min10, set_min1, set_sec10, set_sec1};
    assign cur_time = {cur_min10, cur_min1, cur_sec10, cur_sec1};
    assign value = start_set ? cur_time : set_time;
    fnd_cntr fnd(.clk(clk), .reset_p(reset_p), .value(value), .com(com), .seg_7(seg_7));
endmodule

module keypad_test_top(
    input clk, reset_p,
    input [3:0] row,
    output [3:0] col,
    output [3:0] com,
    output [7:0] seg_7,
    output led_key_valid
 );
    
        wire [3:0] key_value;
        wire key_valid;
        keypad_cntr_FSM keypd(.clk(clk), .reset_p(reset_p),
         .row(row), .col(col),
          .key_value(key_value),
          .key_valid(key_valid));
       assign led_key_valid = key_valid;
       
        wire key_valid_p;
        edge_detector_n ed_btn0(
        .clk(clk), .reset_p(reset_p), 
        .cp(key_valid), .p_edge(key_valid_p ) );
        
        reg [15:0] key_count;
        always @(posedge clk or posedge reset_p)begin
        if (reset_p)key_count = 0;
        else if (key_valid_p)begin
               if(key_value == 1)key_count = key_count + 1;
               else if(key_value == 2)key_count = key_count - 1;
               else if(key_value == 3)key_count = key_count + 2;
        end
        end
        
//        fnd_cntr fnd(.clk(clk), .reset_p(reset_p),
//         .value({12'b0, key_value}), .com(com), .seg_7(seg_7));
        fnd_cntr fnd(.clk(clk), .reset_p(reset_p),
         .value({key_count}), .com(com), .seg_7(seg_7));
endmodule

module dht11_test_top(
    input clk, reset_p,
    inout dht11_data,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);
   
        wire [7:0]  humidity, temperature;
        dht11_cntr dht11(.clk(clk), .reset_p(reset_p),
                                .dht11_data(dht11_data),
                                .humidity(humidity), .temperature(temperature), .led_debug(led_debug));
        
        wire [15:0] humidity_bcd, temperature_bcd;
        bin_to_dec bcd_humi(.bin({4'b0, humidity}), .bcd(humidity_bcd));
        bin_to_dec bcd_tmpr(.bin({4'b0, temperature}), .bcd(temperature_bcd));
        
        wire [15:0] value;
        assign value = {humidity_bcd[7:0], temperature_bcd[7:0]};
        fnd_cntr fnd(.clk(clk), .reset_p(reset_p),
         .value(value), .com(com), .seg_7(seg_7));
endmodule

module hcsr04_test_top(
    input clk, reset_p,
    input echo,
    output trigger,
    output [3:0] com,
    output [7:0] seg_7,
    output [15:0] led_debug);

        wire [22:0] distance;
        hcsr04_cntr hcsr04(.clk(clk), .reset_p(reset_p), .echo(echo),
                                  .trigger(trigger), .distance(distance), .led_debug(led_debug));

        wire [15:0] distance_bcd;
        bin_to_dec bcd_distance(.bin(distance[11:0]), .bcd(distance_bcd));

        fnd_cntr fnd(.clk(clk), .reset_p(reset_p),
         .value(distance_bcd), .com(com), .seg_7(seg_7));
endmodule


module clock_module (
    input clk, reset_p,
    input btn_mode,
    input [4:0] btn,
    output [7:0] seg_7,
    output [3:0] com,
    output led_stopwatch, led_cookstart, led_lap, led_alarm, buzz);
    
    wire debounced_btn;
    wire [1:0] mode;
    
    button_cntr btnmode(.clk(clk), .reset_p(reset_p), .btn(btn_mode), .btn_pedge(debounced_btn));
    button_ring_counter brc ( .clk(clk), .reset_p(reset_p), .btn_change(debounced_btn),  .count(mode));
    
    wire [2:0] btn_watch = (mode == 2'b00) ? btn : 3'b000;
    wire [2:0] btn_stopwatch = (mode == 2'b01) ? btn : 3'b000;
    wire [4:0] btn_cook = (mode == 2'b10) ? btn : {btn[4] ,4'b0000};
    
    wire [7:0] clock_seg7, stopwatch_seg7, cook_seg7;
    loadable_watch_top watch_inst ( .clk(clk), .reset_p(reset_p), .btn(btn_watch), .com(com), .seg_7(clock_seg7));
    stop_watch_milli_top stop_inst( .clk(clk), .reset_p(reset_p), .btn(btn_stopwatch), .com(com), .seg_7(stopwatch_seg7),
     .led_start(led_stopwatch), .led_lap(led_lap));
    cook_timer_top cooktimer_inst ( .clk(clk), .reset_p(reset_p), .btn(btn_cook), .com(com), .seg_7(cook_seg7),
     .led_alarm(led_alarm), .led_start(led_cookstart), .buzz(buzz));
    
    assign seg_7 = reset_p ? 8'b0 : (mode == 2'b10) ? cook_seg7 : (mode == 2'b01) ? stopwatch_seg7 : clock_seg7;
endmodule

module dc_motor_pwm_top(
    input clk, reset_p,
    output motor_pwm,
    output [3:0] com,
    output [7:0] seg_7);

    reg [31:0] clk_div;
    always @(posedge clk)clk_div = clk_div + 1;
    
    wire clk_div_26_nedge;
        edge_detector_n ed1(
        .clk(clk), .reset_p(reset_p), 
        .cp(clk_div[26]), .n_edge(clk_div_26_nedge) );
    
    reg [5:0] duty;
    always @(posedge clk or posedge reset_p) begin
            if(reset_p) duty = 20;
            else if(clk_div_26_nedge)begin
            if(duty >= 99)duty = 20;
            else duty = duty + 1;
        end
    end  
    
     pwm_nstep_freq #(
        .duty_step(100),
        .pwm_freq(100))
     pwm_motor(
        .clk(clk),
        .reset_p(reset_p),
        .duty(duty),
        .pwm(motor_pwm));
        
        wire [15:0] duty_bcd;
        bin_to_dec bcd_distance(.bin({6'b0, clk_div[31:26]}), .bcd(duty_bcd));

        fnd_cntr fnd(.clk(clk), .reset_p(reset_p),
         .value(duty_bcd), .com(com), .seg_7(seg_7));
endmodule

module sv_motor_pwm_top(
    input clk,            // Â½ÃƒÂ½ÂºÃ…Ã› Ã…Â¬Â·Â°
    input reset_p,        // Â¸Â®Â¼Ã‚ Â½Ã…ÃˆÂ£ (ÃˆÂ°Â¼Âº Ã‡Ã?ÃŒ)
    input [2:0] btn,
    output sv_pwm,        // Â¼Â­ÂºÂ¸ Â¸Ã°Ã…Ã ÃÂ¦Â¾Ã®Â¿Ã« PWM ÃƒÃ¢Â·Ã‚
    output [3:0] com,    // 7Â¼Â¼Â±Ã—Â¸Ã•Ã†Â® ÂµÃ°Â½ÂºÃ‡ÃƒÂ·Â¹?ÃŒ?Ã‡ Â°Ã¸Ã…Ã« Ã‡Ã‰
    output [7:0] seg_7   // 7Â¼Â¼Â±Ã—Â¸Ã•Ã†Â® ÂµÃ°Â½ÂºÃ‡ÃƒÂ·Â¹?ÃŒ?Ã‡ Â¼Â¼Â±Ã—Â¸Ã•Ã†Â® Ã‡Ã‰
);
    
    wire btn_0, btn_1, btn_2;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_0));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_1));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_2));
   
    reg [5:0] duty;
    always @(posedge clk or posedge reset_p) begin
            if(reset_p) duty    = 12;
            else if(btn_0)duty = 12;
            else if(btn_1)duty = 32;
            else if(btn_2)duty = 49;
        end
   
     pwm_nstep_freq #(
        .duty_step(400),
        .pwm_freq(50))
     pwm_motor(
        .clk(clk),
        .reset_p(reset_p),
        .duty(duty),
        .pwm(sv_pwm));
        
        wire [15:0] duty_bcd;
        bin_to_dec bcd_distance(.bin({6'b0, duty}), .bcd(duty_bcd));

        fnd_cntr fnd(.clk(clk), .reset_p(reset_p),
         .value(duty_bcd), .com(com), .seg_7(seg_7));
endmodule

module adc_ch6_top(
    input clk, reset_p,
    input vauxp6, vauxn6,
    output [3:0]com,
    output [7:0]seg_7,
    output led_pwm
);
    wire [4:0]   channel_out;
    wire [15:0]  do_out;
    wire eoc_out;
    xadc_wiz_0 inst_adc_6
        (
            .daddr_in    ({2'b0, channel_out}),
            .dclk_in     (clk),
            .den_in      (eoc_out),
            .reset_in    (reset_p),
            .vauxp6      (vauxp6),
            .vauxn6      (vauxn6),
            .channel_out (channel_out),
            .do_out      (do_out),
            .eoc_out     (eoc_out)
        );


//--------------------------
     pwm_nstep_freq #(
        .duty_step(256),
        .pwm_freq(10000))
     pwm_backlight(
        .clk(clk),
        .reset_p(reset_p),
        .duty(do_out[15:8]),
        .pwm(led_pwm));
//-------------------------


        wire [15:0] adc_value;
        bin_to_dec bcd_distance(.bin({2'b0, do_out[15:6]}), .bcd(adc_value)); // ?ƒ?œ„ 12ë¹„íŠ¸ë§? ?‚¬?š© 

        fnd_cntr fnd(.clk(clk), .reset_p(reset_p),
         .value(adc_value), .com(com), .seg_7(seg_7));

endmodule

module adc_sequence2_top(
    input clk, reset_p,   // Clock
    input vauxp6, vauxn6, vauxp15, vauxn15,
    output [3:0]com,
    output [7:0]seg_7,
    output led_r, led_g);

    wire [4:0]   channel_out;
    wire [15:0]  do_out;
    wire eoc_out;
    
    xadc_wiz_1 inst_adc_seq2
        (
            .daddr_in    ({2'b0, channel_out}),
            .dclk_in     (clk),
            .den_in      (eoc_out),
            .reset_in    (reset_p),
            .vauxp6      (vauxp6),
            .vauxn6      (vauxn6),
            .vauxp15     (vauxp15),
            .vauxn15     (vauxn15),
            .channel_out (channel_out),
            .do_out      (do_out),
            .eoc_out     (eoc_out)
        );

    wire eoc_out_pedge;
        edge_detector_n ed(
        .clk(clk), .reset_p(reset_p), 
        .cp(eoc_out), .p_edge(eoc_out_pedge));


    reg [11:0] adc_value_x, adc_value_y;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p) begin
            adc_value_x = 0;
            adc_value_y = 0;
        end
        else if(eoc_out_pedge)begin
            case(channel_out[3:0])
                6: adc_value_x = do_out[15:4];
                15: adc_value_y = do_out[15:4];
            endcase
        end
    end

//--------------------------
     pwm_nstep_freq #(
        .duty_step(256),
        .pwm_freq(10000))
     pwm_red(
        .clk(clk),
        .reset_p(reset_p),
        .duty(adc_value_x[11:4]),
        .pwm(led_r));
//-------------------------
//--------------------------
     pwm_nstep_freq #(
        .duty_step(256),
        .pwm_freq(10000))
     pwm_green(
        .clk(clk),
        .reset_p(reset_p),
        .duty(adc_value_y[11:4]),
        .pwm(led_g));
//-------------------------

        wire [15:0] bcd_x, bcd_y, value;
        bin_to_dec bcd_adc_x(.bin({6'b0, adc_value_x[11:6]}), .bcd(bcd_x));
        bin_to_dec bcd_adc_y(.bin({6'b0, adc_value_y[11:6]}), .bcd(bcd_y));
        
        assign value = {bcd_x[7:0], bcd_y[7:0]};
        fnd_cntr fnd(.clk(clk), .reset_p(reset_p),
         .value(value), .com(com), .seg_7(seg_7));
endmodule

module i2c_master_top (
     input clk, reset_p,
     input [1:0] btn,
     output scl, sda,
     output [15:0] led_debug);

     reg [7:0] data;
     reg comm_go;

     wire [1:0] btn_pedge;
     button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_pedge[0]));
     button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_pedge[1]));

     always @(posedge clk or posedge reset_p)begin
         if(reset_p)begin
             data = 0;
             comm_go = 0;
         end
         else begin
             if(btn_pedge[0])begin
                 data = 8'b0000_0000;
                 comm_go = 1;
             end
             else if(btn_pedge[1])begin
                 data = 8'b1111_1111;
                 comm_go = 1;
             end
             else comm_go = 0;
         end
     end

     I2C_controller master(.clk(clk), .reset_p(reset_p), .addr(7'h27),
     .rd_wr(0), .data(data), .comm_go(comm_go), .scl(scl), .sda(sda), .led_debug(led_debug));



endmodule

module i2c_txtlcd_top(
    input clk, reset_p,
    input [3:0]btn,
    output scl, sda,
    output [15:0]led_debug);
    
    parameter IDLE      = 6'b00_0001;
    parameter INIT      = 6'b00_0010;
    parameter SEND_BYTE = 6'b00_0100;
        // parameter IDLE      = 6'b00_1000;
        // parameter IDLE      = 6'b01_0000;
        // parameter IDLE      = 6'b10_0000;

    wire clk_microsec;
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100_nedge(clk_microsec));


    reg [21:0] count_microsec;
    reg count_microsec_e;

    always @(negedge clk or posedge reset_p)begin
        if(reset_p)count_microsec = 0;
        else if(clk_microsec && count_microsec_e)count_microsec = count_microsec + 1;
        else if(!count_microsec_e)count_microsec = 0;
    end

    wire [3:0] btn_pedge;
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_pedge[0]));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_pedge[1]));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_pedge[2]));
    button_cntr btn3(.clk(clk), .reset_p(reset_p), .btn(btn[3]), .btn_pedge(btn_pedge[3]));

    reg [7:0] send_buffer;
    reg rs, send;
    wire busy;
    i2c_lcd_send_byte txtlcd(.clk(clk), .reset_p(reset_p),
    .addr(7'h27), .send_buffer(send_buffer),
    .rs(rs), .send(send), .scl(scl), .sda(sda),
    .busy(busy), .led_debug(led_debug));

    reg [5:0] state, next_state;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)state = IDLE;
        else state       = next_state;
    end

    reg init_flag;
    reg [3:0] cnt_data;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            next_state = IDLE;
            init_flag  = 0;
            cnt_data   = 0;
            rs = 0;
        end
        else begin
            case(state)
                IDLE:begin
                    if(init_flag)begin
                        if(!busy)begin
                            if(btn_pedge[0])next_state = SEND_BYTE;
                        end
                    end 
                    else begin
                        if(count_microsec <= 22'd80_000)begin
                            count_microsec_e = 1;
                        end
                        else begin
                            next_state       = INIT;
                            count_microsec_e = 0;
                        end
                    end
                end
                INIT:begin
                    if(busy)begin
                        send = 0;
                        if(cnt_data >= 6)begin
                            next_state = IDLE; 
                            init_flag  = 1;
                            cnt_data   = 0;
                        end
                    end
                    else if(!send)begin
                        /*busy가 0이고 send가 0일때만 들어가게함 엣지잡아서 한클럭뒤에 들어가기때문*/
                        case(cnt_data)
                            0:send_buffer = 8'h33;
                            1:send_buffer = 8'h32;
                            2:send_buffer = 8'h28;
                            3:send_buffer = 8'h0f; // display on/off
                            4:send_buffer = 8'h01; // display clr  null로 덮어씀
                            5:send_buffer = 8'h06; // entry mode set
                        endcase
                        rs       = 0;
                        send     = 1;
                        cnt_data = cnt_data + 1;
                    end
                   /*
                   init은 처음 한번만 가고 0으로 꺼지지 않음. 
                   카운트 데이터가 6이되면 아이들상태로 감 */
                end
                SEND_BYTE:begin
                    if(busy)begin
                        next_state  = IDLE;
                        send        = 0;
                    end
                    else begin
                        send_buffer = "A";
                        rs          = 1;
                        send        = 1;
                    end
                end
            endcase
        end
    end
endmodule