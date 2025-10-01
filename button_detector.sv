// button_detector
`timescale 1ns / 1ps

module button_detector #(
    parameter int DIV = 2    // 샘플 간격: 매 DIV개의 fclk마다 1틱 샘플(1이면 매 프레임)
)(
    input  logic fclk,        // ← 여기로 fclk 연결
    input  logic reset,
    input  logic in_button,
    output logic rising_edge
);
    // 분주 → 1틱 샘플 펄스
    logic        sample_pulse;
    logic [7:0]  div_counter; // DIV가 작으니 8비트면 충분

    always_ff @(posedge fclk or posedge reset) begin
        if (reset) begin
            div_counter  <= '0;
            sample_pulse <= 1'b0;
        end else begin
            if (div_counter == DIV-1) begin
                div_counter  <= '0;
                sample_pulse <= 1'b1;
            end else begin
                div_counter  <= div_counter + 1;
                sample_pulse <= 1'b0;
            end
        end
    end

    // 8탭 쉬프트(디바운스용) — 기존 모듈 그대로 사용
    logic [7:0] sh_reg;
    shift_register U_Shift_Register (
        .clk     (sample_pulse), // 1틱 펄스를 클럭으로 사용(원래 구조 유지)
        .reset   (reset),
        .in_data (in_button),
        .out_data(sh_reg)
    );

    // 8샘플 모두 1일 때 눌림 확정
    wire debounce = &sh_reg;

    // 에지 검출은 fclk 도메인에서 1틱 펄스
    logic [1:0] edge_reg;
    always_ff @(posedge fclk or posedge reset) begin
        if (reset) edge_reg <= '0;
        else begin
            edge_reg[0] <= debounce;
            edge_reg[1] <= edge_reg[0];
        end
    end

    assign rising_edge = edge_reg[0] & ~edge_reg[1];

endmodule




module shift_register (
    input  logic       clk,
    input  logic       reset,
    input  logic       in_data,
    output logic [7:0] out_data
);
    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            out_data <= 0;
        end else begin
            out_data <= {in_data, out_data[7:1]};  // right shift
            //out_data <= {out_data[6:0], in_data}; // left shift
        end
    end
endmodule
