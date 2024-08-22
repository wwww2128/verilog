`timescale 1ns / 1ps

module fan_pwm_top(
    input clk, 
    input reset_p,
    input [3:0] btn,
    input ir_input,          // IR 센서 입력

    output motor_pwm,
    output [3:0] com,
    output [7:0] seg_7,
    output reg led_r, led_g, led_b
);

    wire btn_start, btn_1, btn_2, btn_mode;
    wire no_object_detected;      // 물체 감지가 되지 않음을 나타내는 신호
    reg reset_counter;            // IR 센서 타이머 리셋 신호
    
    // 버튼 처리
    button_cntr btn0(.clk(clk), .reset_p(reset_p), .btn(btn[0]), .btn_pedge(btn_start));
    button_cntr btn1(.clk(clk), .reset_p(reset_p), .btn(btn[1]), .btn_pedge(btn_1));
    button_cntr btn2(.clk(clk), .reset_p(reset_p), .btn(btn[2]), .btn_pedge(btn_2));
    button_cntr btn3(.clk(clk), .reset_p(reset_p), .btn(btn[3]), .btn_pedge(btn_mode));


    wire air_speed_edge;
    edge_detector_n ed1(
        .clk(clk), 
        .reset_p(reset_p), 
        .cp(btn_start), 
        .n_edge(air_speed_edge)
    );
    
    reg [5:0] duty;
    reg [3:0] fnd_value;
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            duty <= 0;
            reset_counter <= 1'b1;  // 리셋 시 타이머 초기화
        end else if (air_speed_edge) begin
            if (duty == 0) begin
                duty <= 21;  // 버튼을 누르면 기본 속도로 시작
                reset_counter <= 1'b1;  // 타이머 리셋
            end else if (duty >= 63) begin
                duty <= 0;   // 다시 버튼을 누르면 팬을 끔
                reset_counter <= 1'b0;  // 팬이 꺼질 때는 타이머 리셋 비활성화
            end else begin
                duty <= duty + 21;  // 팬 속도 증가
                reset_counter <= 1'b1;  // 타이머 리셋
            end
        end else if (duty != 0 && no_object_detected) begin
            // 팬이 동작 중이고, 물체가 5초 동안 감지되지 않으면 팬을 끔
            duty <= 0;  // 팬을 멈추기 위해 duty를 0으로 설정
            reset_counter <= 1'b0;  // 팬이 꺼질 때는 타이머 리셋 비활성화
        end else begin
            reset_counter <= 1'b0;  // 기본 상태에서 타이머 리셋 비활성화
        end
    end  
    

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            // 리셋 시 RGB LED를 초기화
            fnd_value <= 4'b0;
            led_r <= 0;
            led_g <= 0;
            led_b <= 0;
        end else begin
            if (duty == 0) begin
                fnd_value <= 4'b0;
                led_r <= 0;
                led_g <= 0;
                led_b <= 0;
            end else if (duty == 21) begin
                fnd_value <= 4'd1;
                led_r <= 1;
                led_g <= 0;
                led_b <= 0;
            end else if (duty == 42) begin
                fnd_value <= 4'd2;
                led_r <= 0;
                led_g <= 1;
                led_b <= 0;
            end else if (duty == 63) begin
                fnd_value <= 4'd3;
                led_r <= 0;
                led_g <= 0;
                led_b <= 1;            
            end
        end
    end

    // IR 센서 모듈 인스턴스화
    ir_sensor ir_sensor_inst (
        .clk(clk),
        .reset_p(reset_p),  // 시스템 리셋 신호
        .ir_input(ir_input),
        .count_enable(duty != 0),  // 팬이 켜졌을 때만 카운트 동작
        .reset_counter(reset_counter), // 버튼을 누를 때마다 타이머 리셋
        .no_object_detected(no_object_detected)  // 감지되지 않음을 나타내는 신호 연결
    );

    // 팬 제어를 위해 PWM 모듈 사용
    pwm_nstep_freq #(
        .duty_step(100),
        .pwm_freq(100)
    ) pwm_motor(
        .clk(clk),
        .reset_p(reset_p),
        .duty(duty),  // duty 값을 그대로 사용하여 PWM 신호 생성
        .pwm(motor_pwm)
    );
        
    wire [15:0] fan_bcd;
    bin_to_dec bcd_distance(
        .bin({12'b0, fnd_value}), 
        .bcd(fan_bcd)
    );

    fnd_cntr fnd(
        .clk(clk), 
        .reset_p(reset_p),
        .value(fan_bcd), 
        .com(com), 
        .seg_7(seg_7)
    );

endmodule

module ir_sensor (
    input clk, 
    input reset_p,
    input ir_input,         // FC-51 센서의 디지털 출력 입력
    input count_enable,     // 카운트 동작 제어 신호 (팬이 켜진 상태에서만 활성화)
    input reset_counter,    // 타이머 리셋 신호 (팬의 상태 변화 시 사용)
    output reg no_object_detected // 물체 감지 여부 출력
);
    // 5초 타이머를 위한 카운터 (100 MHz 클럭 가정, 5초 = 500,000,000 사이클)
    reg [28:0] no_object_counter; // 29비트 카운터 (2^29 > 500,000,000)
    localparam TIMEOUT = 29'd500_000_000; // 5초 (100MHz 클럭 기준)

    always @(posedge clk or posedge reset_p or posedge reset_counter) begin
        if (reset_p || reset_counter) begin
            no_object_counter <= 29'd0;
            no_object_detected <= 1'b0;
        end else if (!count_enable) begin
            // 카운트가 비활성화 상태라면 카운터와 감지 상태 초기화
            no_object_counter <= 29'd0;
            no_object_detected <= 1'b0;
        end else begin
            if (ir_input == 1'b0) begin
                // 물체가 감지되면 카운터 초기화
                no_object_counter <= 29'd0;
                no_object_detected <= 1'b0;
            end else if (no_object_counter < TIMEOUT) begin
                // 물체가 감지되지 않으면 카운터 증가
                no_object_counter <= no_object_counter + 1;
            end else begin
                // 5초 동안 물체가 감지되지 않으면 감지 신호 출력
                no_object_detected <= 1'b1;
            end
        end
    end
endmodule

module fan_timer(
    input clk, reset_p,
    input btn,  // btn[2] 사용
    output [15:0] value     // 현재 타이머 값을 전달
);

    wire clk_microsec, clk_millisec, clk_sec;
    reg [3:0] set_sec1, set_sec10;
    wire [3:0] cur_sec1, cur_sec10;     
    wire dec_clk;
    reg [1:0] state;

    // Clocks
    clock_div_100 microsec_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_microsec));
    clock_div_1000 millisec_clk(.clk(clk), .reset_p(reset_p), 
        .clk_source(clk_microsec), .clk_div_1000(clk_millisec));
    clock_div_1000 sec_clk(.clk(clk), .reset_p(reset_p), 
        .clk_source(clk_millisec), .clk_div_1000_nedge(clk_sec));    
    
    // Edge detector for button press
    wire btn_nedge;
    button_cntr btn_debounce(.clk(clk), .reset_p(reset_p), .btn(btn), .btn_nedge(btn_nedge));

    // Loadable down counters
    fan_down_counter cur_sec(
        .clk(clk), 
        .reset_p(reset_p), 
        .clk_time(clk_sec), 
        .load_enable(btn_nedge), 
        .load_bcd1(set_sec1), 
        .load_bcd10(set_sec10), 
        .bcd1(cur_sec1), 
        .bcd10(cur_sec10), 
        .dec_clk(dec_clk)
    );

    // Timer setting states
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            state <= 2'b00;
            set_sec1 <= 4'b0011; // 초기값 3초
            set_sec10 <= 4'b0000;
        end else begin
            if (btn_nedge) begin
                // 버튼을 누르거나 타이머가 0이 되면 상태를 변경
                case (state)
                    2'b00: begin
                        set_sec1 <= 4'b0101; // 다음 상태: 5 seconds
                        set_sec10 <= 4'b0000;
                        state <= 2'b01;
                    end
                    2'b01: begin
                        set_sec1 <= 4'b0000; // 다음 상태: 10 seconds
                        set_sec10 <= 4'b0001;
                        state <= 2'b10;
                    end
                    2'b10: begin
                        set_sec1 <= 4'b0011; // 다음 상태: 3 seconds
                        set_sec10 <= 4'b0000;
                        state <= 2'b00;
                    end
                    default: begin
                        state <= 2'b00;
                    end
                endcase
            end
        end
    end

    // 타이머 값을 상위 비트에 할당하여 출력
    assign value = {cur_sec10, cur_sec1, 8'b0};  // 16비트 출력으로 전달

endmodule


module fan_down_counter(
    input clk, reset_p,
    input clk_time,
    input load_enable,
    input [3:0] load_bcd1, load_bcd10,
    output reg [3:0] bcd1, bcd10,
    output reg dec_clk
);

    wire clk_time_nedge;
    edge_detector_n ed_clk(
        .clk(clk), .reset_p(reset_p), .cp(clk_time),
        .n_edge(clk_time_nedge)
    );

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            bcd1    <= 0;
            bcd10   <= 0;
            dec_clk <= 0;
        end else begin
            if (load_enable) begin
                bcd1    <= load_bcd1;
                bcd10   <= load_bcd10;
                dec_clk <= 0; // Load 시에는 dec_clk를 리셋
            end else if (clk_time_nedge) begin
                if (bcd1 == 0 && bcd10 == 0) begin
                    dec_clk <= 1; // 카운터가 0에 도달하면 dec_clk를 1로 설정하여 종료 신호로 사용
                end else begin
                    if (bcd1 == 0) begin                
                        bcd1 <= 9;
                        if (bcd10 > 0) begin
                            bcd10 <= bcd10 - 1;
                        end
                    end else begin
                        bcd1 <= bcd1 - 1;
                    end
                    dec_clk <= 0; // 아직 카운트 중이므로 dec_clk를 0으로 유지
                end
            end
        end
    end
endmodule
