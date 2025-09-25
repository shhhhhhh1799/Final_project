`timescale 1ns / 1ps

module frame_buffer #(
    parameter int W = 320,
    parameter int H = 240
) (
    // write side (camera pclk domain)
    input logic        wclk,
    input logic        we,
    input logic [16:0] wAddr,
    input logic [15:0] wData,

    // read side (VGA pclk domain)
    input  logic        rclk,
    input  logic        oe,
    input  logic [16:0] rAddr,
    output logic [15:0] rData,

    // frame control
    input  logic       frame_start,  // 프레임 시작에 1클럭
    output logic [8:0] data          // 9개 구역 결과
);

    parameter int R_MIN = 10;  // 0..31
    parameter int G_MAX = 32;  // 0..63
    parameter int B_MAX = 16;  // 0..31
    parameter int R_MINUS_G = 4;  // R - (G/2) >= 4
    parameter int R_MINUS_B = 4;  // R - B     >= 4

    localparam int SCALE = 25;

    //(* ram_style = "block" *)
    logic [15:0] mem[0:(W*H-1)];

    // write
    always_ff @(posedge wclk) begin
        if (we) mem[wAddr] <= wData;
    end

    // read (sync)
    always_ff @(posedge rclk) begin
        if (oe) rData <= mem[rAddr];
    end

    logic [4:0] R5;
    assign R5 = wData[15:11];  // 0..31
    logic [5:0] G6;
    assign G6 = wData[10:5];  // 0..63
    logic [4:0] B5;
    assign B5 = wData[4:0];  // 0..31

    // ----- 3×3 센터 좌표 (320×240 기준) -----
    localparam int X0 = 53, X1 = 160, X2 = 267;
    localparam int Y0 = 40, Y1 = 120, Y2 = 200;

    // 인덱스 0..8 에 대응하는 (X,Y) 배열
    localparam int X[9] = '{X0, X1, X2, X0, X1, X2, X0, X1, X2};
    localparam int Y[9] = '{Y0, Y0, Y0, Y1, Y1, Y1, Y2, Y2, Y2};

    function logic in_window(input int Xcenter, input int Ycenter,
                                     input logic [16:0] addr);
        int base, left, right;
        for (int dy = -2; dy <= 2; dy++) begin
            base  = (Ycenter + dy) * W;
            left  = base + (Xcenter - 2);
            right = base + (Xcenter + 2);
            if ((addr >= left) && (addr <= right)) begin
                return 1'b1;
            end
        end
    endfunction

    logic [10:0] sumR[9];  // 0..775
    logic [10:0] sumB[9];  // 0..775
    logic [11:0] sumG[9];  // 0..1575
    logic [4:0] cnt[9];  // 0..25 (5비트)

    integer i;
    always_ff @(posedge wclk) begin
        if (frame_start) begin
            for (i = 0; i < 9; i++) begin
                sumR[i] <= '0;
                sumG[i] <= '0;
                sumB[i] <= '0;
                cnt[i]  <= '0;
            end
            data <= '0;
        end else if (we) begin
            for (int k = 0; k < 9; k++) begin
                if (in_window(X[k], Y[k], wAddr)) begin
                    if (cnt[k] != SCALE[4:0]) begin
                        sumR[k] <= sumR[k] + R5;
                        sumG[k] <= sumG[k] + G6;
                        sumB[k] <= sumB[k] + B5;
                        cnt[k]  <= cnt[k] + 1'b1;
                    end
                    if (cnt[k] == SCALE - 1) begin
                        if (( (sumR[k] + R5) >= SCALE*R_MIN ) &&
                               ( (sumG[k] + G6) <= SCALE*G_MAX ) &&
                               ( (sumB[k] + B5) <= SCALE*B_MAX ) &&
                               ( ((sumR[k] + R5) - ((sumG[k] + G6)>>1)) >= SCALE*R_MINUS_G ) &&
                               ( ((sumR[k] + R5) -  (sumB[k] + B5)    ) >= SCALE*R_MINUS_B ))begin
                            data[k] <= 1;
                        end
                    end
                end
            end
        end
    end

endmodule