//top
`timescale 1ns / 1ps

module VGA_Camera_Display (
    input  logic       clk,
    input  logic       reset,
    input  logic [1:0] sw_select,
    input  logic       btn_run,
    input  logic       btnU,
    // ov7670 side
    output logic       ov7670_xclk,
    input  logic       ov7670_pclk,
    input  logic       ov7670_href,
    input  logic       ov7670_vsync,
    input  logic [7:0] ov7670_data,
    // export side
    output logic       h_sync,
    output logic       v_sync,
    output logic [3:0] r_port,
    output logic [3:0] g_port,
    output logic [3:0] b_port,
    output logic [8:0] led,           // 9구역 판정 결과
    output logic       uart_tx,
    //output logic       led_any,        // 하나라도 빨강이면 1
    output logic       SCL,
    inout  logic       SDA
);
    assign led_any = |led;
    logic        startSig;
    logic        ov7670_we;
    logic [16:0] ov7670_wAddr;
    logic [15:0] ov7670_wData;

    logic        vga_pclk;
    logic [ 9:0] vga_x_pixel;
    logic [ 9:0] vga_y_pixel;
    logic        vga_DE;

    logic        vga_den;
    logic [16:0] vga_rAddr;
    logic [15:0] vga_rData;

    assign ov7670_xclk = vga_pclk;

    logic [3:0] vga_r;
    logic [3:0] vga_g;
    logic [3:0] vga_b;
    logic [3:0] gray_r;
    logic [3:0] gray_g;
    logic [3:0] gray_b;
    logic [3:0] gui_r;
    logic [3:0] gui_g;
    logic [3:0] gui_b;

    logic fclk;
    logic [8:0] data;

    logic vsync_d;
    always_ff @(posedge ov7670_pclk or posedge reset) begin
        if (reset) begin
            vsync_d  <= 1'b0;
            startSig <= 1'b1;
        end else begin
            vsync_d  <= ov7670_vsync;
            startSig <= 1'b0;
        end
    end
    logic vsync_q;
    always_ff @(posedge ov7670_pclk or posedge reset) begin
        if (reset) vsync_q <= 1'b0;
        else vsync_q <= ov7670_vsync;
    end

    wire frame_start = (vsync_d & ~ov7670_vsync);  // falling edge에서 펄스
    wire cam_frame_end = (~vsync_q & ov7670_vsync);  // rising edge = frame end

    VGA_Decoder U_VGA_Decoder (
        .clk    (clk),
        .reset  (reset),
        .pclk   (vga_pclk),
        .fclk   (fclk),
        .h_sync (h_sync),
        .v_sync (v_sync),
        .x_pixel(vga_x_pixel),
        .y_pixel(vga_y_pixel),
        .DE     (vga_DE)
    );

    OV7670_MemController U_OV7670_MemController (
        .clk        (ov7670_pclk),
        .reset      (reset),
        .href       (ov7670_href),
        .vsync      (ov7670_vsync),
        .ov7670_data(ov7670_data),
        .we         (ov7670_we),
        .wAddr      (ov7670_wAddr),
        .wData      (ov7670_wData)
    );

    frame_buffer #(
        .W(320),
        .H(240),
        .RED_TH(18),
        .BLUE_TH(10),
        .GREEN_TH(13)
    ) U_FrameBuffer (
        .wclk (ov7670_pclk),
        .we   (ov7670_we),
        .wAddr(ov7670_wAddr),
        .wData(ov7670_wData),

        .rclk(vga_pclk),
        .oe(vga_den),
        .rAddr(vga_rAddr),
        .rData(vga_rData),
        .led_clear (frame_start),   // 프레임 시작시 1클럭 펄스 신호 (vsync에서 생성)
        .data(data)
    );

    test_buffer U_Test_Buffer (
        .fclk (cam_frame_end),
        .reset(reset),
        .btn  (btn_run),
        .data (data),
        .led  (led)
    );

    VGA_MemController U_VGAMemController (
        .DE     (vga_DE),
        .x_pixel(vga_x_pixel),
        .y_pixel(vga_y_pixel),
        .den    (vga_den),
        .rAddr  (vga_rAddr),
        .rData  (vga_rData),
        .r_port (vga_r),
        .g_port (vga_g),
        .b_port (vga_b)
    );

    GrayScaleFilter U_GRAY (
        .i_r(vga_r),
        .i_g(vga_g),
        .i_b(vga_b),
        .o_r(gray_r),
        .o_g(gray_g),
        .o_b(gray_b)
    );

    GUIMaker U_GUI (
        .vga_x_pixel(vga_x_pixel),
        .vga_y_pixel(vga_y_pixel),
        .i_r(vga_r),
        .i_g(vga_g),
        .i_b(vga_b),
        .o_r(gui_r),
        .o_g(gui_g),
        .o_b(gui_b)
    );

    mux_3x1 U_MUX (
        .sel     (sw_select),
        .vga_rgb ({vga_r, vga_g, vga_b}),
        .gray_rgb({gray_r, gray_g, gray_b}),
        .gui_rgb ({gui_r, gui_g, gui_b}),
        .rgb     ({r_port, g_port, b_port})
    );

    OV7670_Master U_SCCB (.*);

    top_uart9_basys3 U_UART (
        .clk        (clk),     // 100MHz
        .fclk       (cam_frame_end),
        .reset      (reset),
        .data_detect(led),    // Basys3 슬라이드 스위치 9개 사용
        .btnU       (btnU),    // 중앙 버튼: 송신 트리거
        .uart_tx    (uart_tx)
    );
endmodule
/*
module mux_2x1 (
    input logic sel,
    input logic [11:0] vga_rgb,
    input logic [11:0] gray_rgb,
    output logic [11:0] rgb
);
    always_comb begin
        case (sel)
            1'b0: rgb = vga_rgb;
            1'b1: rgb = gray_rgb;
        endcase
    end
endmodule
*/

module mux_3x1 (
    input  logic [ 1:0] sel,
    input  logic [11:0] vga_rgb,
    input  logic [11:0] gray_rgb,
    input  logic [11:0] gui_rgb,
    output logic [11:0] rgb
);
    always_comb begin
        case (sel)
            2'b00:   rgb = vga_rgb;  // Original video
            2'b01:   rgb = gray_rgb;  // Grayscale filter
            2'b10:   rgb = gui_rgb;  // GUI overlay
            default: rgb = vga_rgb;  // Default to original video for safety
        endcase
    end
endmodule
