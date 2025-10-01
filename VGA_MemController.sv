module VGA_MemController (
    // VGA side
    input  logic        DE,
    input  logic [ 9:0] x_pixel,
    input  logic [ 9:0] y_pixel,
    // frame buffer side
    output logic        den,
    output logic [16:0] rAddr,
    input  logic [15:0] rData,
    // export side
    output logic [ 3:0] r_port,
    output logic [ 3:0] g_port,
    output logic [ 3:0] b_port
);

    // 픽셀 복제를 위해 VGA 좌표를 2로 나눔 (>> 1)
    logic [9:0] mem_x_addr;
    logic [9:0] mem_y_addr;

    assign mem_x_addr = x_pixel >> 1;  // 640 -> 320
    assign mem_y_addr = y_pixel >> 1;  // 480 -> 240

    // VGA 출력 유효 영역 (DE)에서만 데이터 읽기 활성화
    assign den = DE;

    // 복제된 주소로 프레임 버퍼에서 데이터 읽기
    assign rAddr = den ? (mem_y_addr * 320 + mem_x_addr) : 17'bz;

    // RGB565 데이터를 Basys3의 4비트 VGA 포트에 맞게 변환
    assign r_port = den ? rData[15:12] : 4'h0;
    assign g_port = den ? rData[10:7] : 4'h0;
    assign b_port = den ? rData[4:1] : 4'h0;

endmodule
