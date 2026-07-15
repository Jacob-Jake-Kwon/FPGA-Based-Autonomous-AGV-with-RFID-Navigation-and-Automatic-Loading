module stepper_driver(
    input  wire clk,
    input  wire motor_en,
    input  wire motor_dir,

    output reg  motor_ain1,
    output reg  motor_ain2,
    output wire motor_pwma,
    output wire motor_stby
);

    // Keep TB6612FNG enabled
    assign motor_stby = 1'b1;

    // ================================================================
    // DIRECTION CONTROL
    // ================================================================

    always @(*) begin
        if (motor_en) begin
            if (motor_dir) begin
                // Forward: lift upward
                motor_ain1 = 1'b1;
                motor_ain2 = 1'b0;
            end
            else begin
                // Reverse: lower downward
                motor_ain1 = 1'b0;
                motor_ain2 = 1'b1;
            end
        end
        else begin
            // Coast
            motor_ain1 = 1'b0;
            motor_ain2 = 1'b0;
        end
    end

    // ================================================================
    // PWM GENERATOR
    // ================================================================

    reg [12:0] pwm_counter = 13'd0;

    always @(posedge clk) begin
        if (pwm_counter >= 13'd4999) begin
            pwm_counter <= 13'd0;
        end
        else begin
            pwm_counter <= pwm_counter + 1'b1;
        end
    end

    // Preserve the full-power behavior of the working test
    assign motor_pwma =
        motor_en && (pwm_counter < 13'd5000);

endmodule