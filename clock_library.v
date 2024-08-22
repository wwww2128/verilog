`timescale 1ns / 1ps

module clock_div_10( 
   input clk, reset_p,
   input clk_source,
   output clk_div_10,
   output clk_div_10_nedge);
   
   reg [3:0] cnt_clk_source; // 10
   
   wire clk_source_nedge;
   edge_detector_n ed_source(
        .clk(clk), .reset_p(reset_p), .cp(clk_source),
        .n_edge(clk_source_nedge));
   
   always @(negedge clk or posedge reset_p)begin
        if(reset_p)cnt_clk_source = 0;
        else if(clk_source_nedge)begin
        if(cnt_clk_source >= 9) cnt_clk_source = 0; 
        else cnt_clk_source = cnt_clk_source + 1;
        end
   end

   assign clk_div_10 = (cnt_clk_source < 5) ? 0 : 1;
   
   edge_detector_n a(
    .clk(clk), .reset_p(reset_p),
    .cp(clk_div_10),
    .n_edge(clk_div_10_nedge));
endmodule

module clock_div_100(
   input clk, reset_p,
   output clk_div_100,
   output clk_div_100_nedge);
   
   reg [6:0] cnt_sysclk;
   
   always @(negedge clk or posedge reset_p)begin
        if(reset_p)cnt_sysclk = 0;
        else begin
        if(cnt_sysclk >= 99) cnt_sysclk = 0; 
        else cnt_sysclk = cnt_sysclk + 1;
        end
   end

   assign clk_div_100 = (cnt_sysclk < 50) ? 0 : 1;
   
   edge_detector_n n(
    .clk(clk), .reset_p(reset_p),
    .cp(clk_div_100),
    .n_edge(clk_div_100_nedge));
endmodule

module clock_div_1000( // micro sec input nsyc 
   input clk, reset_p,
   input clk_source, // micro second
   output clk_div_1000,
   output clk_div_1000_nedge);
   
   reg [9:0] cnt_clk_source; // 1000 
   
   wire clk_source_nedge;
   edge_detector_n ed_source(
        .clk(clk), .reset_p(reset_p), .cp(clk_source),
        .n_edge(clk_source_nedge));
   
   always @(negedge clk or posedge reset_p)begin
        if(reset_p)cnt_clk_source = 0;
        else if(clk_source_nedge)begin
        if(cnt_clk_source >= 999) cnt_clk_source = 0; 
        else cnt_clk_source = cnt_clk_source + 1;
        end
   end

   assign clk_div_1000 = (cnt_clk_source < 500) ? 0 : 1;
   
   edge_detector_n a(
    .clk(clk), .reset_p(reset_p),
    .cp(clk_div_1000),
    .n_edge(clk_div_1000_nedge));
endmodule

module clock_div_60( // micro sec input nsyc 
   input clk, reset_p,
   input clk_source, // micro second
   output clk_div_60,
   output clk_div_60_nedge);
   
   reg [5:0] cnt_clk_source; // 1000 
   
   wire clk_source_nedge;
   edge_detector_n ed_source(
        .clk(clk), .reset_p(reset_p), .cp(clk_source),
        .n_edge(clk_source_nedge));
   
   always @(negedge clk or posedge reset_p)begin
        if(reset_p)cnt_clk_source = 0;
        else if(clk_source_nedge)begin
        if(cnt_clk_source >= 59) cnt_clk_source = 0; 
        else cnt_clk_source = cnt_clk_source + 1;
        end
   end

   assign clk_div_60 = (cnt_clk_source < 30) ? 0 : 1;
   
   edge_detector_n a(
    .clk(clk), .reset_p(reset_p),
    .cp(clk_div_60),
    .n_edge(clk_div_60_nedge));
endmodule

module counter_bcd_60(
    input clk, reset_p,
    input clk_time,
    output reg [3:0] bcd1, bcd10);

   wire clk_time_nedge;
   edge_detector_n ed_clk(
    .clk(clk), .reset_p(reset_p), .cp(clk_time),
    .n_edge(clk_time_nedge));

    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            bcd1 = 0;
            bcd10 = 0;
        end
        else if(clk_time_nedge)begin
            if(bcd1 >= 9)begin                
                bcd1 = 0;
                if(bcd10 >= 5)bcd10 = 0;
                else bcd10 = bcd10 + 1; 
            end
            else bcd1 = bcd1 + 1;
        end
    end

endmodule

module loadable_counter_bcd_60(
    input clk, reset_p,
    input clk_time,
    input load_enable,
    input [3:0] load_bcd1, load_bcd10,
    output reg [3:0] bcd1, bcd10);

   wire clk_time_nedge;
   edge_detector_n ed_clk(
    .clk(clk), .reset_p(reset_p), .cp(clk_time),
    .n_edge(clk_time_nedge));

    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            bcd1 = 0;
            bcd10 = 0;
        end
        else begin;
            if(load_enable)begin
                bcd1 = load_bcd1;
                bcd10 = load_bcd10;
            end
            else if(clk_time_nedge)begin
                if(bcd1 >= 9)begin                
                bcd1 = 0;
                if(bcd10 >= 5)bcd10 = 0;
                else bcd10 = bcd10 + 1; 
            end
            else bcd1 = bcd1 + 1;
        end
    end
end
endmodule

module counter_bcd_60_clear(
    input clk, reset_p,
    input clk_time,
    input clear, 
    output reg [3:0] bcd1, bcd10);

   wire clk_time_nedge;
   edge_detector_n ed_clk(
    .clk(clk), .reset_p(reset_p), .cp(clk_time),
    .n_edge(clk_time_nedge));

    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            bcd1 = 0;
            bcd10 = 0;
        end
        else begin
            if(clear)begin
                bcd1 = 0;
                bcd10 = 0;
            end
        else if(clk_time_nedge)begin
            if(bcd1 >= 9)begin                
                bcd1 = 0;
                if(bcd10 >= 5)bcd10 = 0;
                else bcd10 = bcd10 + 1; 
            end
            else bcd1 = bcd1 + 1;
        end
    end
end
endmodule

module loadable_counter_bcd_100(
    input clk, reset_p,
    input clk_time,
    input load_enable,
    input [6:0] load_bcd10, load_bcd100,
    output reg [6:0] bcd10, bcd100);

   wire clk_time_nedge;
   edge_detector_n ed_clk(
    .clk(clk), .reset_p(reset_p), .cp(clk_time),
    .n_edge(clk_time_nedge));

    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            bcd10 = 0;
            bcd100 = 0;
        end
        else begin;
            if(load_enable)begin
                bcd10 = load_bcd10;
                bcd100 = load_bcd100;
            end
            else if(clk_time_nedge)begin
                if(bcd10 >= 99)begin                
                bcd10 = 0;
                if(bcd100 >= 9)bcd100 = 0;
                else bcd100 = bcd100 + 1; 
            end
            else bcd10 = bcd10 + 1;
        end
    end
end
endmodule

module counter_bcd_100_clear(
    input clk, reset_p,
    input clk_time,
    input clear, 
    output reg [3:0] bcd10, bcd100);

   wire clk_time_nedge;
   edge_detector_n ed_clk(
    .clk(clk), .reset_p(reset_p), .cp(clk_time),
    .n_edge(clk_time_nedge));

    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            bcd10 = 0;
            bcd100 = 0;
        end
        else begin
            if(clear)begin
                bcd10 = 0;
                bcd100 = 0;
            end
        else if(clk_time_nedge)begin
            if(bcd10 >= 9)begin                
                bcd10 = 0;
                if(bcd100 >= 9)bcd100 = 0;
                else bcd100 = bcd100 + 1; 
            end
            else bcd10 = bcd10 + 1;
        end
    end
end
endmodule

module loadable_down_counter_bcd_60(
    input clk, reset_p,
    input clk_time,
    input load_enable,
    input [3:0] load_bcd1, load_bcd10,
    output reg [3:0] bcd1, bcd10,
    output reg dec_clk);

   wire clk_time_nedge;
   edge_detector_n ed_clk(
    .clk(clk), .reset_p(reset_p), .cp(clk_time),
    .n_edge(clk_time_nedge));

    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            bcd1 = 0;
            bcd10 = 0;
            dec_clk = 0;
        end
        else begin
            if(load_enable)begin
                bcd1 = load_bcd1;
                bcd10 = load_bcd10;
            end
            else if(clk_time_nedge)begin
                if(bcd1 == 0)begin                
                    bcd1 = 9;
                if(bcd10 == 0)begin
                    bcd10 = 5;
                    dec_clk = 1;
                    end
                else bcd10 = bcd10 - 1;
            end
            else bcd1 = bcd1 - 1;
        end
        else dec_clk = 0;
    end
end
endmodule

module hcsr04_58( // micro sec input nsyc 
   input clk, reset_p,
   input clk_microsec, cnt_enable,
   output reg[11:0] cm);
   
   reg [5:0] cnt; // 1000 
   

   always @(negedge clk or posedge reset_p)begin
        if(reset_p)begin
        cnt = 0;
        cm = 0;
        end
        else if(clk_microsec)begin
            if(cnt_enable) begin
                if(cnt >= 57)begin
                cnt = 0;
                cm = cm + 1;
                end
                else cnt = cnt + 1;
              end
        end
        else if(!cnt_enable)begin
               cnt = 0;
               cm = 0;
   end
   end
endmodule