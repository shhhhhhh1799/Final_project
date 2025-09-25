`timescale 1ns/1ps

module top_uart9_USB (
    input  logic       clk,           // 100MHz
    input  logic       fclk,
    input  logic       reset,
    input  logic [8:0] data_detect,   // 탑 모듈과 일치하는 포트 이름
    input  logic       btnU,          // 사용하지 않음
    output logic       uart_tx,
    output logic [15:0] led           // 디버그용 LED (선택사항)
);

    // 입력 데이터에서 관심있는 비트만 추출
    logic [1:0] sw_select;
    logic [1:0] sw_select_reg;
    logic [1:0] sw_select_prev;
    logic sw_changed;
    logic first_time;
    logic start_tx;
    logic busy;
    logic tx_ready;

    assign sw_select = data_detect[1:0];  // 하위 2비트만 사용

    // 스위치 상태 변화 감지 및 초기 전송 처리
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            sw_select_reg  <= 2'b00;
            sw_select_prev <= 2'b00;
            first_time     <= 1'b1;
        end else begin
            sw_select_reg <= sw_select;
            sw_select_prev <= sw_select_reg;
            
            // 첫 전송이 완료되면 first_time 플래그 해제
            if (start_tx && !busy) begin
                first_time <= 1'b0;
            end
        end
    end

    // 스위치 값이 변했거나 초기 상태일 때 전송 조건
    assign sw_changed = (sw_select_reg != sw_select_prev) || first_time;
    
    // UART가 준비되고 변화가 감지되었을 때만 전송 시작
    assign tx_ready = !busy && sw_changed;
    
    // 전송 시작 신호 생성 (1클럭 펄스, busy 상태 고려)
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            start_tx <= 1'b0;
        end else begin
            start_tx <= tx_ready;
        end
    end

    // 전송할 데이터 생성 (ASCII 코드)
    logic [7:0] tx_data;
    always_comb begin
        case (sw_select_reg)
            2'b00: tx_data = 8'h30;  // ASCII '0' - 기본 화면
            2'b01: tx_data = 8'h31;  // ASCII '1' - 흑백 필터
            2'b10: tx_data = 8'h32;  // ASCII '2' - GUI 오버레이
            2'b11: tx_data = 8'h33;  // ASCII '3' - 예비 모드
            default: tx_data = 8'h30;
        endcase
    end

    // 디버그용 LED 출력 (선택사항)
    always_comb begin
        led[15:14] = sw_select_reg;      // 현재 선택된 모드
        led[13]    = busy;               // 전송 중 표시
        led[12]    = start_tx;           // 전송 시작 신호
        led[11]    = sw_changed;         // 변화 감지
        led[10]    = first_time;         // 초기 전송 플래그
        led[9:0]   = 10'b0;             // 나머지는 0
    end

    // 개선된 8비트 UART 전송 모듈
    uart8_tx_USB #(
        .CLK_HZ(100_000_000), 
        .BAUD(9600)
    ) UTX (
        .clk     (clk), 
        .reset   (reset),
        .start   (start_tx),
        .data8   (tx_data),
        .tx      (uart_tx),
        .busy    (busy)
    );

endmodule

// 개선된 8비트 UART 전송 모듈
module uart8_tx_USB #(
    parameter int CLK_HZ = 100_000_000,
    parameter int BAUD   = 9600
)(
    input  logic       clk, 
    input  logic       reset,
    input  logic       start,           // 1클록 펄스
    input  logic [7:0] data8,
    output logic       tx,              // idle=1, active=0
    output logic       busy
);
    
    localparam int DIV = CLK_HZ / BAUD; // 100e6/9600 ≈ 10417
    localparam int BITS_TOTAL = 10;     // start(1) + data(8) + stop(1)
    
    logic [$clog2(DIV)-1:0] baud_cnt;
    logic [$clog2(BITS_TOTAL+1)-1:0] bit_idx;
    logic [BITS_TOTAL-1:0] shifter;     // {stop(1), data[7:0], start(0)}
    
    typedef enum logic [1:0] {
        IDLE = 2'b00,
        LOAD = 2'b01, 
        TRANSMIT = 2'b10
    } state_t;
    
    state_t state, next_state;

    // 상태 레지스터
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    // 상태 전이 로직
    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (start) 
                    next_state = LOAD;
            end
            LOAD: begin
                next_state = TRANSMIT;
            end
            TRANSMIT: begin
                if (bit_idx == BITS_TOTAL && baud_cnt == DIV-1)
                    next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end

    // 데이터패스 로직
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            tx       <= 1'b1;      // UART idle state
            busy     <= 1'b0;
            baud_cnt <= '0;
            bit_idx  <= '0;
            shifter  <= '0;
        end else begin
            case (state)
                IDLE: begin
                    tx       <= 1'b1;  // idle high
                    busy     <= 1'b0;
                    baud_cnt <= '0;
                    bit_idx  <= '0;
                end
                
                LOAD: begin
                    // 전송할 데이터 로드: start(0) + data[7:0] + stop(1)
                    shifter  <= {1'b1, data8, 1'b0};
                    busy     <= 1'b1;
                    baud_cnt <= '0;
                    bit_idx  <= '0;
                    tx       <= 1'b0;  // start bit
                end
                
                TRANSMIT: begin
                    if (baud_cnt == DIV-1) begin
                        baud_cnt <= '0;
                        bit_idx  <= bit_idx + 1;
                        
                        // 다음 비트 출력 (비트 인덱스에 따라)
                        if (bit_idx < BITS_TOTAL-1) begin
                            tx <= shifter[bit_idx + 1];
                        end else begin
                            tx <= 1'b1;  // 전송 완료 후 idle로
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end
                
                default: begin
                    tx       <= 1'b1;
                    busy     <= 1'b0;
                    baud_cnt <= '0;
                    bit_idx  <= '0;
                end
            endcase
        end
    end

endmodule
/*
// 개선된 버튼 디텍터 모듈 
module button_detector_USB #(
    parameter int DIV = 2
)(
    input  logic fclk,
    input  logic reset,
    input  logic in_button,
    output logic rising_edge,
    output logic falling_edge,    // falling edge도 감지
    output logic debounced_out    // 디바운스된 출력
);
    logic        sample_pulse;
    logic [$clog2(DIV)-1:0] div_counter;

    // 샘플링 클럭 생성
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

    // 시프트 레지스터를 이용한 디바운싱
    logic [7:0] sh_reg;
    shift_register_USB U_Shift_Register (
        .clk     (sample_pulse),
        .reset   (reset),
        .in_data (in_button),
        .out_data(sh_reg)
    );

    // 모든 비트가 1이면 안정적인 HIGH, 모든 비트가 0이면 안정적인 LOW
    logic debounce_high = &sh_reg;
    logic debounce_low  = ~|sh_reg;
    
    assign debounced_out = debounce_high;

    // 엣지 검출
    logic [1:0] edge_reg;
    always_ff @(posedge fclk or posedge reset) begin
        if (reset) begin
            edge_reg <= '0;
        end else begin
            edge_reg[0] <= debounced_out;
            edge_reg[1] <= edge_reg[0];
        end
    end

    assign rising_edge  = edge_reg[0] & ~edge_reg[1];
    assign falling_edge = ~edge_reg[0] & edge_reg[1];

endmodule

// 시프트 레지스터 모듈 (동일)
module shift_register_USB (
    input  logic       clk,
    input  logic       reset,
    input  logic       in_data,
    output logic [7:0] out_data
);
    always_ff @(posedge clk, posedge reset) begin
        if (reset) begin
            out_data <= 8'b0;
        end else begin
            out_data <= {in_data, out_data[7:1]};
        end
    end
endmodule
*/