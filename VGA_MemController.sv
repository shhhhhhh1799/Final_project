`timescale 1ns / 1ps

// =======================
// VGA_MemController (No pipeline in MemController)
// - x_pixel, y_pixel, DE, rData를 즉시 사용
// - 왼쪽 가장자리 픽셀 미스얼라인은 GUI로 가린다는 전제
// =======================
module VGA_MemController (
    input  logic        pclk,
    input  logic        reset,
    input  logic btn_rise,
    input  logic        DE,          // 640x480 visible
    input  logic        sw,          // 1: Sobel, 0: original
    input  logic [9:0]  x_pixel,     // 0..639
    input  logic [9:0]  y_pixel,     // 0..479
    // frame buffer side (320x240 RGB565)
    output logic        den,
    output logic [16:0] rAddr,
    input  logic [15:0] rData,
    // VGA ports (Basys3 4:4:4)
    output logic [3:0]  r_port,
    output logic [3:0]  g_port,
    output logic [3:0]  b_port
);



//logic btn_rise;
logic [$clog2(12_500_000-1):0] counter;
logic [$clog2(6-1):0] counter2;
logic active;
logic screen;

//assign led1 = active; assign led2 = screen;
/*
button_detector #(
    .DIV(1000)
    )
    U_BTN_Detector(
    .fclk(pclk),
    .reset(reset),
    .in_button(~btn),
    .rising_edge(btn_rise)
);
*/
always_ff @(posedge pclk or posedge reset) begin
    if (reset) begin
        counter <= 0;
        screen <= 0;
        active <= 0;
        counter2 <= 0;
    end else if (!sw) begin
        if (btn_rise&&!active) begin
            counter <= 0;
            counter2 <= 0;
            active<=1;
        end 
        if (active) begin
            if (counter == 12_500_000-1) begin
                counter <= 0;
                if (counter2 == 6-1) begin
                    active   <= 0;
                    counter2 <= 0;
                    screen   <= 0;
                end else begin
                    screen   <= ~screen;
                    counter2 <= counter2 + 1;
                end
            end else begin
                counter <= counter + 1;
            end
        end
    end else if (sw) begin
        active   <= 0;
        counter2 <= 0;
        if (counter == 12_500_000-1) begin
            screen <= ~screen;
            counter <= 0;
        end else begin
            counter <= counter + 1;
        end
    end
end


    // ------------------------------
    // Params
    // ------------------------------
    localparam int SRC_W = 320;
    localparam int SRC_H = 240;

    // ------------------------------
    // Frame buffer read: no delay
    // ------------------------------
    assign den = 1'b1;           // continuous read
    wire de_vis = DE;

    // 2x 업스케일 좌표를 즉시 계산해 주소 생성
    wire [9:0] src_x = x_pixel >> 1;  // 0..319
    wire [9:0] src_y = y_pixel >> 1;  // 0..239
    assign rAddr = src_y * SRC_W + src_x;

    // ------------------------------
    // Sobel (기존 모듈 그대로 사용)
    // ------------------------------
    logic        de_sobel;
    logic [15:0] sobel_rgb16;

    SobelRedFilter #(
        .W(SRC_W),
        .H(SRC_H),
        .R_MIN(10),
        .G_MAX(25),
        .B_MAX(12),
        .RG_DELTA(64),
        .RB_DELTA(48),
        .THICK_H(1'b1),
        .THICK_V(1'b1),
        .SOBEL_TH(120)
        
        //.USE_MAJORITY(1'b0),
        //.K_RED(5)
    ) u_sobel (
        .pclk      (pclk),
        .rstn      (1'b1),        // 필요시 ~reset으로 교체 가능
        .de_in     (de_vis),      // ★ 지연 없이 바로 전달
        .rgb565_in (rData),       // ★ 지연 없이 바로 전달
        .x_in      (src_x),       // ★ 즉시 계산 좌표
        .y_in      (src_y),       // ★ 즉시 계산 좌표
        .de_out    (de_sobel),
        .rgb565_out(sobel_rgb16)
    );

    // ------------------------------
    // Output select (Sobel / Original)
    // ------------------------------
    logic [15:0] pix_out;
    logic        de_out;

    always_comb begin
        if (screen) begin
            pix_out = sobel_rgb16;
            de_out  = de_sobel;   // Sobel 내부 파이프라인에 정렬됨
        end else begin
            pix_out = rData;      // 원본은 지연 없음
            de_out  = de_vis;
        end
    end

    // RGB 4:4:4 출력
    assign r_port = de_out ? pix_out[15:12] : 4'h0;
    assign g_port = de_out ? pix_out[10:7 ] : 4'h0;
    assign b_port = de_out ? pix_out[ 4:1 ] : 4'h0;

endmodule



// =======================
// SobelRedFilter (강화된 빨강 마스크 + 엣지 두껍게)
// - 모듈 인터페이스 동일
// - 항상 pclk 동기, rstn=1 동작/0 클리어
// =======================
module SobelRedFilter #(
    parameter int W = 320,
    parameter int H = 240,
    // 빨강 판정(좀 더 타이트하게 기본값 설정)
    parameter int R_MIN     = 20,  // R5(0..31)
    parameter int G_MAX     = 16,  // G6(0..63)
    parameter int B_MAX     = 8,   // B5(0..31)
    // 빨강 지배도(dominance) 마진 (8b 확장값 기준)
    parameter int RG_DELTA  = 64,  // R8 >= G8 + 64
    parameter int RB_DELTA  = 48,  // R8 >= B8 + 48
    // 엣지 두께 옵션
    parameter bit THICK_H   = 1'b1, // 가로로 +1픽셀(이전 픽셀)
    parameter bit THICK_V   = 1'b1, // 세로로 +1픽셀(이전 라인 동일 x)
    // 소벨 임계
    parameter int SOBEL_TH  = 120
)(
    input  logic        pclk,
    input  logic        rstn,        // 1 동작, 0 클리어
    input  logic        de_in,
    input  logic [15:0] rgb565_in,
    input  logic [ 9:0] x_in,
    input  logic [ 9:0] y_in,
    output logic        de_out,
    output logic [15:0] rgb565_out
);
    // --- RGB565 → 8b 확장 ---
    logic [4:0] R5; logic [5:0] G6; logic [4:0] B5;
    assign R5 = rgb565_in[15:11];
    assign G6 = rgb565_in[10:5 ];
    assign B5 = rgb565_in[ 4:0 ];

    logic [7:0] R8, G8, B8;
    assign R8 = {R5, R5[4:2]};
    assign G8 = {G6, G6[5:4]};
    assign B8 = {B5, B5[4:2]};

    // --- 빨강 중앙 마스크(지배도 포함) ---
    logic [8:0] R9, G9, B9;
    assign R9 = {1'b0, R8};
    assign G9 = {1'b0, G8};
    assign B9 = {1'b0, B8};
    localparam [8:0] RG_DEL9 = RG_DELTA[8:0];
    localparam [8:0] RB_DEL9 = RB_DELTA[8:0];

    logic rmask_in;
    always_ff @(posedge pclk) begin
        if (!rstn) rmask_in <= 1'b0;
        else begin
            rmask_in <= (R5 >= R_MIN) && (G6 <= G_MAX) && (B5 <= B_MAX)
                        && (R9 >= (G9 + RG_DEL9))
                        && (R9 >= (B9 + RB_DEL9));
        end
    end

    // --- 그레이 Y ≈ (2R + 5G + B) >> 3 ---
    logic [10:0] sum_tmp;
    logic [ 7:0] Y_in;
    always_ff @(posedge pclk) begin
        if (!rstn) begin
            sum_tmp <= '0; Y_in <= '0;
        end else if (de_in) begin
            sum_tmp <= (R8<<1) + (G8*5) + B8;
            Y_in    <= sum_tmp[10:3];
        end else begin
            sum_tmp <= '0; Y_in <= '0;
        end
    end

    // --- 3x3 윈도우 (라인버퍼 2 + 시프트) ---
    (* ram_style = "block" *) logic [7:0] LB0 [0:W-1]; // y-1
    (* ram_style = "block" *) logic [7:0] LB1 [0:W-1]; // y-2
    (* ram_style = "block" *) logic       LB0m[0:W-1];
    (* ram_style = "block" *) logic       LB1m[0:W-1];

    logic [7:0] y0_in, y1_in;
    logic       m0_in, m1_in;

    logic [7:0] w00,w01,w02, w10,w11,w12, w20,w21,w22;
    logic       m00,m01,m02, m10,m11,m12, m20,m21,m22;

    // 원본 RGB 파이프라인(1단)
    logic [15:0] rgb_pipe0;
    always_ff @(posedge pclk) begin
        if (!rstn) rgb_pipe0 <= 16'h0000;
        else       rgb_pipe0 <= de_in ? rgb565_in : 16'h0000;
    end

    // 윈도우/마스크 이동
    always_ff @(posedge pclk) begin
        if (!rstn) begin
            {w00,w01,w02, w10,w11,w12, w20,w21,w22} <= '{default:0};
            {m00,m01,m02, m10,m11,m12, m20,m21,m22} <= '{default:0};
            y0_in <= 0; y1_in <= 0; m0_in <= 0; m1_in <= 0;
        end else if (de_in) begin
            // 과거 두 줄 읽기
            y1_in <= LB1[x_in];
            y0_in <= LB0[x_in];
            m1_in <= LB1m[x_in];
            m0_in <= LB0m[x_in];

            // 시프트
            {w00,w01} <= {w01,w02}; w02 <= y1_in;
            {w10,w11} <= {w11,w12}; w12 <= y0_in;
            {w20,w21} <= {w21,w22}; w22 <= Y_in;

            {m00,m01} <= {m01,m02}; m02 <= m1_in;
            {m10,m11} <= {m11,m12}; m12 <= m0_in;
            {m20,m21} <= {m21,m22}; m22 <= rmask_in;

            // 라인버퍼 갱신
            LB1 [x_in] <= LB0[x_in];
            LB0 [x_in] <= Y_in;
            LB1m[x_in] <= LB0m[x_in];
            LB0m[x_in] <= rmask_in;
        end
    end

    // --- Sobel |Gx|+|Gy| ---
    logic signed [11:0] gx, gy;
    logic [11:0]        mag;
    logic [11:0]        ax, ay;  // |gx|, |gy|
    always_comb begin
        gx  =  ($signed({1'b0,w02}) + ($signed({1'b0,w12})<<<1) + $signed({1'b0,w22}))
            -  ($signed({1'b0,w00}) + ($signed({1'b0,w10})<<<1) + $signed({1'b0,w20}));
        gy  =  ($signed({1'b0,w00}) + ($signed({1'b0,w01})<<<1) + $signed({1'b0,w02}))
            -  ($signed({1'b0,w20}) + ($signed({1'b0,w21})<<<1) + $signed({1'b0,w22}));
        ax  = (gx[11] ? -gx : gx);
        ay  = (gy[11] ? -gy : gy);
        mag = ax + ay;
    end

    // --- "빨강 ↔ 비빨강" 경계 판정 ---
    // 각 측(row/col) 3픽셀 합산 후 다수결로 '빨강측' 판정
    logic [2:0] sumL, sumR, sumU, sumD;
    logic       left_red, right_red, up_red, down_red;
    always_comb begin
        sumL = m00 + m10 + m20;
        sumR = m02 + m12 + m22;
        sumU = m00 + m01 + m02;
        sumD = m20 + m21 + m22;

        left_red  = (sumL >= 2);
        right_red = (sumR >= 2);
        up_red    = (sumU >= 2);
        down_red  = (sumD >= 2);
    end

    // 색 경계가 "명확"할 때만 통과 (한쪽은 빨강, 반대쪽은 비빨강)
    // 방향 무관하게 OR로 허용 (수직/수평/대각 포함)
    wire color_edge_ok = (left_red ^ right_red) | (up_red ^ down_red);

    // 최종 엣지 bin: 소벨 세기 + 색 경계 일치 + 경계부 보호
    logic edge_bin;
    always_ff @(posedge pclk) begin
        if (!rstn) edge_bin <= 1'b0;
        else       edge_bin <= (mag >= SOBEL_TH) && color_edge_ok
                               && (x_in > 1) && (y_in > 1);
    end

    // === 엣지 두께 증가 (가로/세로 +1) ===
    logic edge_prev;      // 이전 픽셀(가로)
    logic thick_edge;

    // 이전 라인 엣지 ping-pong 메모리
    logic bank_wr, bank_rd;
    (* ram_style = "distributed" *) logic E0 [0:W-1];
    (* ram_style = "distributed" *) logic E1 [0:W-1];

    // 라인 시작에서 bank 스왑
    always_ff @(posedge pclk) begin
        if (!rstn) begin
            bank_wr <= 1'b0;
            bank_rd <= 1'b0;
        end else if (de_in && (x_in == 0)) begin
            bank_wr <= ~bank_wr;     // 이번 라인 기록용
            bank_rd <= bank_wr;      // 이전 라인 읽기용
        end
    end

    // 이전 라인 같은 x에서의 엣지
    logic edge_up_rd;
    assign edge_up_rd = THICK_V ? (bank_rd ? E1[x_in] : E0[x_in]) : 1'b0;

    // 두께 합성
    always_ff @(posedge pclk) begin
        if (!rstn) begin
            edge_prev  <= 1'b0;
            thick_edge <= 1'b0;
        end else if (de_in) begin
            if (bank_wr) E1[x_in] <= edge_bin;
            else         E0[x_in] <= edge_bin;

            thick_edge <= edge_bin
                        | (THICK_H ? edge_prev  : 1'b0)
                        | edge_up_rd;

            edge_prev <= (x_in == 0) ? 1'b0 : edge_bin;
        end else begin
            edge_prev  <= 1'b0;
            thick_edge <= 1'b0;
        end
    end

    // --- DE 정렬 (1클럭) ---
    logic de_q;
    always_ff @(posedge pclk) begin
        if (!rstn) de_q <= 1'b0;
        else       de_q <= de_in;
    end
    assign de_out = de_q;

    // --- 출력 구성 ---
    logic [15:0] edge_rgb;
    always_comb begin
        edge_rgb = thick_edge ? 16'hFFE0 : rgb_pipe0; // 필요시 16'h0000로 변경
    end

    assign rgb565_out = edge_rgb;
endmodule



