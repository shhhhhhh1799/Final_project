/*
`timescale 1ns/1ps
module GUIMaker(
    input  logic [9:0] vga_x_pixel,
    input  logic [9:0] vga_y_pixel,
    input  logic [3:0] i_r,
    input  logic [3:0] i_g,
    input  logic [3:0] i_b,
    output logic [3:0] o_r,
    output logic [3:0] o_g,
    output logic [3:0] o_b
);

    // VGA 해상도 (640x480) 기준
    localparam H_CENTER_1 = 320; 
    localparam H_CENTER_2 = 213;
    localparam H_CENTER_3 = 426;
    localparam V_CENTER_1 = 240;
    localparam V_CENTER_2 = 160;
    localparam V_CENTER_3 = 320;
    localparam LINE_THICKNESS = 5;

    // GUI를 그릴지 여부를 결정하는 논리
    logic draw_gui;
    assign draw_gui =
        // 세로선 (가운데, 1/3, 2/3 지점)
        (vga_x_pixel >= H_CENTER_1 - LINE_THICKNESS/2 && vga_x_pixel < H_CENTER_1 + LINE_THICKNESS/2) ||
        (vga_x_pixel >= H_CENTER_2 - LINE_THICKNESS/2 && vga_x_pixel < H_CENTER_2 + LINE_THICKNESS/2) ||
        (vga_x_pixel >= H_CENTER_3 - LINE_THICKNESS/2 && vga_x_pixel < H_CENTER_3 + LINE_THICKNESS/2) ||
        // 가로선 (가운데, 1/3, 2/3 지점)
        (vga_y_pixel >= V_CENTER_1 - LINE_THICKNESS/2 && vga_y_pixel < V_CENTER_1 + LINE_THICKNESS/2) ||
        (vga_y_pixel >= V_CENTER_2 - LINE_THICKNESS/2 && vga_y_pixel < V_CENTER_2 + LINE_THICKNESS/2) ||
        (vga_y_pixel >= V_CENTER_3 - LINE_THICKNESS/2 && vga_y_pixel < V_CENTER_3 + LINE_THICKNESS/2);

    // 조건에 따라 출력 색상을 결정
    always_comb begin
        if (draw_gui) begin
            o_r = 4'h0;
            o_g = 4'hF; // Green
            o_b = 4'h0;
        end else begin
            // If not drawing GUI, pass through original video
            o_r = i_r;
            o_g = i_g;
            o_b = i_b;
        end
    end
endmodule
*/
`timescale 1ns/1ps
module GUIMaker(
    input  logic [9:0] vga_x_pixel,
    input  logic [9:0] vga_y_pixel,
    input  logic [3:0] i_r,
    input  logic [3:0] i_g,
    input  logic [3:0] i_b,
    output logic [3:0] o_r,
    output logic [3:0] o_g,
    output logic [3:0] o_b
);

    // VGA 해상도 (640x480) 기준 - 9분할
    localparam H_1_3 = 213;        // 640/3
    localparam H_2_3 = 427;        // 640*2/3
    localparam V_1_3 = 160;        // 480/3
    localparam V_2_3 = 320;        // 480*2/3
    localparam LINE_THICKNESS = 3;
    
    // 타겟 모양 동심원 파라미터
    localparam OUTER_RADIUS = 5;   // 바깥 원 반지름
    localparam INNER_RADIUS = 2;   // 안쪽 원 반지름
    localparam RING_THICKNESS = 1; // 고리 두께
    
    // 각 칸의 중심점 계산 (9개)
    localparam H_CENTER_1 = 106;   // (0 + 213) / 2 = 106
    localparam H_CENTER_2 = 320;   // (213 + 427) / 2 = 320
    localparam H_CENTER_3 = 533;   // (427 + 640) / 2 = 533
    localparam V_CENTER_1 = 80;    // (0 + 160) / 2 = 80
    localparam V_CENTER_2 = 240;   // (160 + 320) / 2 = 240
    localparam V_CENTER_3 = 400;   // (320 + 480) / 2 = 400

    // 신호 선언
    logic draw_grid;
    logic draw_target_rings;
    logic draw_target_centers;

    // 원의 거리 계산을 위한 중간 신호들
    logic [19:0] dx_sq, dy_sq, dist_sq, radius_sq;
    logic [9:0] abs_dx, abs_dy;

    // GUI 그리드 라인 그리기
    assign draw_grid =
        // 세로선 2개 (1/3, 2/3 지점)
        (vga_x_pixel >= H_1_3 - LINE_THICKNESS/2 && vga_x_pixel <= H_1_3 + LINE_THICKNESS/2) ||
        (vga_x_pixel >= H_2_3 - LINE_THICKNESS/2 && vga_x_pixel <= H_2_3 + LINE_THICKNESS/2) ||
        // 가로선 2개 (1/3, 2/3 지점)
        (vga_y_pixel >= V_1_3 - LINE_THICKNESS/2 && vga_y_pixel <= V_1_3 + LINE_THICKNESS/2) ||
        (vga_y_pixel >= V_2_3 - LINE_THICKNESS/2 && vga_y_pixel <= V_2_3 + LINE_THICKNESS/2);

    // 간단한 원 검사 (맨하탄 거리 근사)
    logic is_center_1, is_center_2, is_center_3, is_center_4, is_center_5;
    logic is_center_6, is_center_7, is_center_8, is_center_9;
    logic is_outer_1, is_outer_2, is_outer_3, is_outer_4, is_outer_5;
    logic is_outer_6, is_outer_7, is_outer_8, is_outer_9;

    // 중심 점들 (작은 원)
    assign is_center_1 = ((vga_x_pixel >= H_CENTER_1 - INNER_RADIUS && vga_x_pixel <= H_CENTER_1 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_1 - INNER_RADIUS && vga_y_pixel <= V_CENTER_1 + INNER_RADIUS));
    assign is_center_2 = ((vga_x_pixel >= H_CENTER_2 - INNER_RADIUS && vga_x_pixel <= H_CENTER_2 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_1 - INNER_RADIUS && vga_y_pixel <= V_CENTER_1 + INNER_RADIUS));
    assign is_center_3 = ((vga_x_pixel >= H_CENTER_3 - INNER_RADIUS && vga_x_pixel <= H_CENTER_3 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_1 - INNER_RADIUS && vga_y_pixel <= V_CENTER_1 + INNER_RADIUS));
    assign is_center_4 = ((vga_x_pixel >= H_CENTER_1 - INNER_RADIUS && vga_x_pixel <= H_CENTER_1 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_2 - INNER_RADIUS && vga_y_pixel <= V_CENTER_2 + INNER_RADIUS));
    assign is_center_5 = ((vga_x_pixel >= H_CENTER_2 - INNER_RADIUS && vga_x_pixel <= H_CENTER_2 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_2 - INNER_RADIUS && vga_y_pixel <= V_CENTER_2 + INNER_RADIUS));
    assign is_center_6 = ((vga_x_pixel >= H_CENTER_3 - INNER_RADIUS && vga_x_pixel <= H_CENTER_3 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_2 - INNER_RADIUS && vga_y_pixel <= V_CENTER_2 + INNER_RADIUS));
    assign is_center_7 = ((vga_x_pixel >= H_CENTER_1 - INNER_RADIUS && vga_x_pixel <= H_CENTER_1 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_3 - INNER_RADIUS && vga_y_pixel <= V_CENTER_3 + INNER_RADIUS));
    assign is_center_8 = ((vga_x_pixel >= H_CENTER_2 - INNER_RADIUS && vga_x_pixel <= H_CENTER_2 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_3 - INNER_RADIUS && vga_y_pixel <= V_CENTER_3 + INNER_RADIUS));
    assign is_center_9 = ((vga_x_pixel >= H_CENTER_3 - INNER_RADIUS && vga_x_pixel <= H_CENTER_3 + INNER_RADIUS) &&
                         (vga_y_pixel >= V_CENTER_3 - INNER_RADIUS && vga_y_pixel <= V_CENTER_3 + INNER_RADIUS));

    // 바깥 고리들 (큰 원에서 중간 원 제외)
    assign is_outer_1 = ((vga_x_pixel >= H_CENTER_1 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_1 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_1 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_1 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_1 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_1 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_1 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_1 + (OUTER_RADIUS-RING_THICKNESS))));
    assign is_outer_2 = ((vga_x_pixel >= H_CENTER_2 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_2 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_1 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_1 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_2 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_2 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_1 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_1 + (OUTER_RADIUS-RING_THICKNESS))));
    assign is_outer_3 = ((vga_x_pixel >= H_CENTER_3 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_3 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_1 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_1 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_3 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_3 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_1 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_1 + (OUTER_RADIUS-RING_THICKNESS))));
    assign is_outer_4 = ((vga_x_pixel >= H_CENTER_1 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_1 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_2 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_2 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_1 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_1 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_2 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_2 + (OUTER_RADIUS-RING_THICKNESS))));
    assign is_outer_5 = ((vga_x_pixel >= H_CENTER_2 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_2 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_2 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_2 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_2 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_2 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_2 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_2 + (OUTER_RADIUS-RING_THICKNESS))));
    assign is_outer_6 = ((vga_x_pixel >= H_CENTER_3 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_3 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_2 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_2 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_3 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_3 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_2 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_2 + (OUTER_RADIUS-RING_THICKNESS))));
    assign is_outer_7 = ((vga_x_pixel >= H_CENTER_1 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_1 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_3 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_3 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_1 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_1 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_3 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_3 + (OUTER_RADIUS-RING_THICKNESS))));
    assign is_outer_8 = ((vga_x_pixel >= H_CENTER_2 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_2 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_3 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_3 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_2 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_2 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_3 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_3 + (OUTER_RADIUS-RING_THICKNESS))));
    assign is_outer_9 = ((vga_x_pixel >= H_CENTER_3 - OUTER_RADIUS && vga_x_pixel <= H_CENTER_3 + OUTER_RADIUS) &&
                        (vga_y_pixel >= V_CENTER_3 - OUTER_RADIUS && vga_y_pixel <= V_CENTER_3 + OUTER_RADIUS) &&
                        !((vga_x_pixel >= H_CENTER_3 - (OUTER_RADIUS-RING_THICKNESS) && vga_x_pixel <= H_CENTER_3 + (OUTER_RADIUS-RING_THICKNESS)) &&
                          (vga_y_pixel >= V_CENTER_3 - (OUTER_RADIUS-RING_THICKNESS) && vga_y_pixel <= V_CENTER_3 + (OUTER_RADIUS-RING_THICKNESS))));

    // 최종 타겟 신호들
    assign draw_target_centers = is_center_1 || is_center_2 || is_center_3 || is_center_4 || is_center_5 ||
                               is_center_6 || is_center_7 || is_center_8 || is_center_9;
    
    assign draw_target_rings = is_outer_1 || is_outer_2 || is_outer_3 || is_outer_4 || is_outer_5 ||
                             is_outer_6 || is_outer_7 || is_outer_8 || is_outer_9;

    // 조건에 따라 출력 색상을 결정
    always_comb begin
        if (draw_target_rings || draw_target_centers) begin
            // 빨간색 타겟 (바깥 고리 + 중심 점)
            o_r = 4'hF;
            o_g = 4'h0;
            o_b = 4'h0;
        end else if (draw_grid) begin
            // 초록색 그리드 라인
            o_r = 4'h0;
            o_g = 4'hF;
            o_b = 4'h0;
        end else begin
            // 원본 이미지 통과
            o_r = i_r;
            o_g = i_g;
            o_b = i_b;
        end
    end
endmodule
