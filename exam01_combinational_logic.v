`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2024/06/12 15:04:26
// Design Name: 
// Module Name: exam01_combinational_logic
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

module and_gate(
    output reg q,
    input a, b);
    
    always @(a, b)begin
        case({a, b})
        2'b00: q = 0; // 4'b0001: q = 0
        2'b01: q = 0; // 8'b00000000
        2'b10: q = 0;
        2'b11: q = 1;
        endcase
        end
endmodule

module half_adder_structural(
    input a, b,
    output s, c);
    
    and carry(c, a, b);     // 
    xor (s, a, b);          // s = a xor b
endmodule

module half_adder_behavioral(
    input a, b,
    output reg s, c);
    
    always @(a, b)begin                     //if Sensitivity List value change
        case({a, b})
            2'b00: begin s = 0; c = 0; end  // binary = 2'b00 
            2'b01: begin s = 1; c = 0; end
            2'b10: begin s = 1; c = 0; end
            2'b11: begin s = 0; c = 1; end
        endcase
    end
endmodule

module half_adder_dataflow(
    input a, b,
    output s, c);
    
    wire [1:0] sum_value;       // 2bit 
    
    assign sum_value = a + b;   // assgin = join wire
    
    assign s = sum_value[0];    // 0 = sum
    assign c = sum_value[1];    // 1 = carry
    
    
endmodule

module full_adder_structural(
    input a, b, c,
    output sum, carry);
    
    wire sum_0, carry_0, carry_1;
    
    half_adder_structural ha0(.a(a), .b(b), .s(sum_0), .c(carry_0));
    half_adder_structural ha1(.a(sum_0), .b(c), .s(sum), .c(carry_1));
    
    or (carry, carry_0, carry_1);

endmodule

module full_adder_behavioral(
    input a, b, c,
    output reg sum, carry);
    
    always @(*)begin                     //if Sensitivity List value change
        case({a, b, c})
            3'b000: begin sum = 0; carry = 0; end  // binary = 2'b00 
            3'b001: begin sum = 1; carry = 0; end  // binary = 2'b00 
            3'b010: begin sum = 1; carry = 0; end  // binary = 2'b00 
            3'b011: begin sum = 0; carry = 1; end  // binary = 2'b00 
            3'b100: begin sum = 1; carry = 0; end  // binary = 2'b00 
            3'b101: begin sum = 0; carry = 1; end  // binary = 2'b00 
            3'b110: begin sum = 0; carry = 1; end  // binary = 2'b00 
            3'b111: begin sum = 1; carry = 1; end  // binary = 2'b00 
            default: begin sum = 0; carry = 0; end  // binary = 2'b00 
        endcase
        end
endmodule

module full_adder_dataflow(
    input a, b, c,
    output sum, carry);
    
    wire[1:0] sum_value;
    
    assign sum_value = a + b + c;   // assgin = join wire
    assign sum = sum_value[0];    // 0 = sum
    assign carry = sum_value[1];    // 1 = carry

endmodule

module parallel_4bits_s(
    input [3:0] a, b,
    input cin,
    output [3:0] sum,
    output carry);
   
    wire [2:0] carry_w;
    
    full_adder_behavioral fa0(.a(a[0]), .b(b[0]), .c(cin), .sum(sum[0]), .carry(carry_w[0]));
    full_adder_behavioral fa1(.a(a[1]), .b(b[1]), .c(carry_w[0]), .sum(sum[1]), .carry(carry_w[1]));
    full_adder_behavioral fa2(.a(a[2]), .b(b[2]), .c(carry_w[1]), .sum(sum[2]), .carry(carry_w[2]));
    full_adder_behavioral fa3(.a(a[3]), .b(b[3]), .c(carry_w[2]), .sum(sum[3]), .carry(carry));

endmodule

module parallel_4bits_dataflow(
    input [3:0] a, b,
    input cin,
    output [3:0] sum, 
    output carry);
    
    wire[5:0] sum_value;
    
    assign sum_value = a + b + cin;   // assgin = join wire
    assign sum = sum_value[3:0];    // 0 = sum
    assign carry = sum_value[4];    // 1 = carry

endmodule


module parallel_add_sub_4bits_structural(
    input [3:0] a, b,
    input s,                     // s +-
    output [3:0] sum,
    output carry);

    wire[2:0] carry_w;
    wire[3:0] b_w;
    
    xor(b_w[0], b[0], s);
    xor(b_w[1], b[1], s);
    xor(b_w[2], b[2], s);
    xor(b_w[3], b[3], s);
    
    full_adder_behavioral fa0(.a(a[0]), .b(b_w[0]), .c(s), .sum(sum[0]), .carry(carry_w[0]));
    full_adder_behavioral fa1(.a(a[1]), .b(b_w[1]), .c(carry_w[0]), .sum(sum[1]), .carry(carry_w[1]));
    full_adder_behavioral fa2(.a(a[2]), .b(b_w[2]), .c(carry_w[1]), .sum(sum[2]), .carry(carry_w[2]));
    full_adder_behavioral fa3(.a(a[3]), .b(b_w[3]), .c(carry_w[2]), .sum(sum[3]), .carry(carry));

endmodule

module parallel_add_sub_4bits_dataflow(
    input [3:0] a, b,
    input s,
    output [3:0] sum,
    output carry);
    
    wire [5:0] result;
    
    assign result = s ? a - b : a + b;
    assign sum = result[3:0];
    assign carry = s ? ~result[4] : result[4];

endmodule


module comparrator_dataflow(
    input a, b,
    output equal, greater, less);

    assign equal = (a == b) ? 1'b1 : 1'b0;
    assign greater = (a > b) ? 1'b1 : 1'b0;
    assign less = (a < b) ? 1'b1 : 1'b0;

endmodule

module comparator#(parameter N = 8)(
    input [N-1:0]a, b,
    output equal, greater, less);

    assign equal = (a == b) ? 1'b1 : 1'b0;
    assign greater = (a > b) ? 1'b1 : 1'b0;
    assign less = (a < b) ? 1'b1 : 1'b0;

endmodule

module comparator_n_bits_test(
    input [1:0]a, b,
    output equal,greater, less);

    comparator_n_bits_behavioral #(.N(2)) comp_2bit(a, b, equal, greater, less);
    
    endmodule

module comparator_n_bits_behavioral #(parameter N = 8)(
    input [N-1:0]a, b,
    output reg equal,greater, less);
    
    always @(a, b)begin
        equal = 0;
        greater = 0;
        less = 0;
        if (a == b)equal = 1;
        else if (a > b)greater = 1;
        else if (a < b)less = 1;
    end
endmodule

module half_add2 (input a, input b, output sum, output cout);
    assign cout = a&b;
    assign sum  = a^b;
endmodule

module full_add (
input a, b, cin,
output sum, cout);

wire w1,w2,w3;

half_add2 U1(.a(a), .b(b), .sum(w1), .cout(w2));
half_add2 U2(w1, cin, sum, w3);
or        U3(cout, w2, w3);
endmodule

module decoder_2x4_b(
    input [1:0] code,  
    output reg [3:0] signal);
    
//    always @(code)begin
//        if(code == 2'b00) signal = 4'b0001;
//        else if(code == 2'b01) signal = 4'b0010;
//        else if(code == 2'b10) signal = 4'b0100;
//        else if(code == 2'b11) signal = 4'b1000;
//    end

      always @(code)begin
          case(code)
          2'b00 : signal = 4'b0001;
          2'b01 : signal = 4'b0010;
          2'b10 : signal = 4'b0100;
          2'b11 : signal = 4'b1000;
          endcase
      end
endmodule

module decoder_2x4_d(
    input [1:0] code,  
    output [3:0] signal);
    
    assign signal = (code == 2'b00) ? 4'b0001 : 
                    ((code == 2'b01) ? 4'b0010 :
                    ((code == 2'b10) ? 4'b0100 : 4'b1000));
endmodule

module decoder_7seg(
    input [3:0] hex_value,
    output reg [7:0] seg_7);
    
    always @(hex_value)begin
        case(hex_value)
          //                 8'babcd_efgp
            0: seg_7 = 8'b0000_0011; // 0
            1: seg_7 = 8'b1001_1111; // 1
            2: seg_7 = 8'b0010_0101; // 2
            3: seg_7 = 8'b0000_1101; // 3
            4: seg_7 = 8'b1001_1001; // 4
            5: seg_7 = 8'b0100_1001; // 5
            6: seg_7 = 8'b0100_0001; // 6
            7: seg_7 = 8'b0001_1011; // 7
            8: seg_7 = 8'b0000_0001; // 8
            9: seg_7 = 8'b0001_1001; // 9
            10: seg_7 = 8'b0001_0001; // A
            11: seg_7 = 8'b1100_0001; // B
            12: seg_7 = 8'b0110_0011; // C
            13: seg_7 = 8'b1000_0101; // d
            14: seg_7 = 8'b0110_0001; // E
            15: seg_7 = 8'b0111_0001; // F
        endcase
    end
    
endmodule

module encoder_4x2_b(
    input [3:0] signal,
    output reg [1:0] code);
           
           //    always @(signal)begin
//        if(code == 4'b0001) code = 2'b00;
//        else if(code == 4'b0010) code = 2'b01;
//        else if(code == 4'b0100) code = 2'b10;
//        else if(code == 4'b1000) code = 2'b11;
//        else code = 2'b00;
//    end
            always@(signal)begin
            case(signal)
            4'b0001 : code = 2'b00;
            4'b0010 : code = 2'b01;
            4'b0100 : code = 2'b10;
            4'b1000 : code = 2'b11;
            default : code = 2'b00;
            endcase
            end
endmodule

module encoder_4x2_d(
    input [3:0] signal,
    output[1:0] code);
    
    assign code = (signal == 4'b0001) ? 2'b00 :
                    ((signal == 4'b0010) ? 2'b01 :
                    ((signal == 4'b0100) ? 2'b10 : 2'b11));
     
endmodule

module mux_2_1(
    input [1:0] d,
    input s,
    output f);
    
    assign f = s ? d[1] : d[0];


endmodule

module mux_4_1(
    input [3:0] d,
    input [1:0]s,
    output f);
    
    
    // If 's' is 00, then d[0] is assigned to 'f'.
    // If 's' is 01, then d[1] is assigned to 'f'.
    // If 's' is 10, then d[2] is assigned to 'f'.
    // If 's' is 11, then d[3] is assigned to 'f'.
    assign f = d[s];


endmodule

module demux_1_4(
    input d,
    input [1:0] s,
    output [3:0] f);
    
    assign f = (s == 2'b00) ? {3'd000,d} :
               ((s == 2'b01) ? {2'b00, d, 1'b0} :
               ((s == 2'b10) ? {1'b0, d, 2'b00} : {d, 3'b000}));
               
endmodule

module mux_demux_test(
    input [3:0] d,
    input [1:0]mux_s, demux_s,
    output [3:0] f);
    
    wire line;
    
    mux_4_1 mux(.d(d), .s(mux_s), .f(line));
    demux_1_4 demux(.d(line), .s(demux_s), .f(f));

endmodule

module bin_to_dec(
        input [11:0] bin,
        output reg [15:0] bcd
    );

    reg [3:0] i;

    always @(bin) begin
        bcd = 0;
        for (i=0;i<12;i=i+1)begin
            bcd = {bcd[14:0], bin[11-i]};
            if(i < 11 && bcd[3:0] > 4) bcd[3:0] = bcd[3:0] + 3;
            if(i < 11 && bcd[7:4] > 4) bcd[7:4] = bcd[7:4] + 3;
            if(i < 11 && bcd[11:8] > 4) bcd[11:8] = bcd[11:8] + 3;
            if(i < 11 && bcd[15:12] > 4) bcd[15:12] = bcd[15:12] + 3;
        end
    end
endmodule