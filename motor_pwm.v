module motor_pwm(
    input clk,          // 100 MHz clock
    input [7:0] duty,   // Speed control (0 to 255)
    input fwd,          // Forward command
    input rev,          // Reverse command
    output reg in1,     // DRV8833 input 1
    output reg in2      // DRV8833 input 2
);
    reg [7:0] counter = 0;
    reg clk_div = 0;
    reg [11:0] div_counter = 0;

    // Divide 100MHz down to ~20kHz for smooth PWM
    always @(posedge clk) begin
        if (div_counter >= 2000) begin
            div_counter <= 0;
            clk_div <= ~clk_div;
        end else begin
            div_counter <= div_counter + 1;
        end
    end

    always @(posedge clk_div) begin
        counter <= counter + 1;
        if (counter < duty) begin
            in1 <= fwd;
            in2 <= rev;
        end else begin
            in1 <= 1'b0; // Coast/Brake when PWM cycle drops
            in2 <= 1'b0;
        end
    end
endmodule