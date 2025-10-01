// test buffer
`timescale 1ns / 1ps

module test_buffer (
    input  logic       fclk,
    input  logic       reset,
    input  logic       btn,
    input  logic [8:0] data,
    output logic [8:0] led
);
logic btn_rise;

button_detector #(
    .DIV(2)
    )
    U_BTN_Detector(
    .fclk(fclk),
    .reset(reset),
    .in_button(~btn),
    .rising_edge(btn_rise)
);
    logic [7:0] mem[0:8];
    logic [10:0] counter;
    logic active;
    
    //assign sw_f=sw_a & ~sw_b

    always_ff @(posedge fclk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < 9; i++) begin
                mem[i] <= '0;
            end
            counter <= 0;
            led <= 1'b0;
            active <= 1'b0;
        end else begin
            if (btn_rise) begin
                active <= 1'b1;
                counter <= 0;
                for (int i = 0; i < 9; i++) begin
                    mem[i] <= '0;
                end
            end
        
         if (active) begin
            for (int i = 0; i < 9; i++) begin 
                mem[i] <= mem[i] + data[i];
            end
            if (counter == 90 - 1) begin
                counter <= 0;
                active <= 1'b0;
                for (int j = 0; j < 9; j++) begin
                    if (mem[j] > 20) begin
                        led[j] <= 1;
                    end else begin
                        led[j] <= 0;
                    end
                end
            end else begin
                counter <= counter + 1;
            end
        end
        end
    end

endmodule
