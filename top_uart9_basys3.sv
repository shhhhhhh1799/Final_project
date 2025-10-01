//uart top
module top_uart9_basys3 (
  input  logic       clk,     // 100MHz
  input  logic       fclk,
  input  logic       reset,
  input  logic [8:0] data_detect,      // Basys3 슬라이드 스위치 9개 사용
  input  logic       btnU,    // 중앙 버튼: 송신 트리거
  output logic       uart_tx
);

logic btn_rise;

  logic busy;
  uart9_tx #(.CLK_HZ(100_000_000), .BAUD(115200)) UTX (
    .clk, .reset,
    .start(btn_rise),
    .data9(data_detect),
    .tx(uart_tx),
    .busy
  );

  button_detector #(
    .DIV(2)
    )
    U_BTN_Detector(
    .fclk(fclk),
    .reset(reset),
    .in_button(~btnU),
    .rising_edge(btn_rise)
);
endmodule

//tx
module uart9_tx #(
  parameter int CLK_HZ = 100_000_000,
  parameter int BAUD   = 115200
)(
  input  logic       clk, reset,
  input  logic       start,           // 1클록 펄스
  input  logic [8:0] data9,
  output logic       tx,              // idle=1
  output logic       busy
);
  localparam int DIV = CLK_HZ / BAUD; // 100e6/115200 ≈ 868
  logic [$clog2(DIV)-1:0] baud_cnt;
  logic [3:0] bit_idx;
  logic [10:0] shifter; // {stop(1), 9 data, start(0)}

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      tx <= 1'b1; busy <= 1'b0; baud_cnt <= '0; bit_idx <= '0;
    end else begin
      if (!busy) begin
        if (start) begin
          shifter <= {1'b1, data9, 1'b0}; // stop, data[8:0], start
          busy    <= 1'b1;
          baud_cnt<= '0; bit_idx <= 0;
          tx      <= 1'b0;                // start bit
        end
      end else begin
        if (baud_cnt == DIV-1) begin
          baud_cnt <= '0;
          bit_idx  <= bit_idx + 1;
          tx       <= shifter[bit_idx+1];
          if (bit_idx == 10) begin
            busy <= 1'b0;
            tx   <= 1'b1;                 // idle
          end
        end else baud_cnt <= baud_cnt + 1;
      end
    end
  end
endmodule

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
