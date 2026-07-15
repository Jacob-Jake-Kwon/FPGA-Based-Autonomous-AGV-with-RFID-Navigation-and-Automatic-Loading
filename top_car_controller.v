`timescale 1ns / 1ps

module top_car_controller(
    input  wire        clk,
    input  wire [3:0]  s,
    input  wire        echo,
    input  wire        JA_RX,

    output wire        trig,

    output wire        in1,
    output wire        in2,
    output wire        in3,
    output wire        in4,

    output wire        stepper_a1,
    output wire        stepper_a2,
    output wire        stepper_b1,
    output wire        stepper_b2,

    output wire [15:0] led
);

    // ================================================================
    // TIMING CONSTANTS — 100 MHz SYSTEM CLOCK
    // ================================================================

    // Lift actuator duration: 0.7 seconds
    localparam [26:0] LIFT_TIME =
        27'd70_000_000;

    // Wait for RFID for 7 seconds
    localparam [29:0] RFID_WAIT_TIME =
        30'd700_000_000;

    // Require line pattern 0000 continuously for 20 ms
    localparam [21:0] LINE_CONFIRM_TIME =
        22'd2_000_000;

    // PID update every 1 ms
    // 100 MHz × 0.001 s = 100,000 clocks
    localparam [16:0] PID_UPDATE_TIME =
        17'd100_000;

    // ================================================================
    // MOTOR SPEED CONSTANTS
    // ================================================================

    localparam [7:0] BASE_SPEED   = 8'd170;
    localparam [7:0] ROTATE_SPEED = 8'd250;
    localparam [7:0] STOP_SPEED   = 8'd0;

    // ================================================================
    // PID CONTROL CONSTANTS
    // ================================================================
    //
    // PID output:
    //
    // correction =
    //      KP × error
    //    + KI × integral
    //    + KD × derivative
    //
    // Start with KI = 0 for PD control.
    // Increase KI carefully only if needed.
    // ================================================================

    localparam signed [15:0] KP = 16'sd22;
    localparam signed [15:0] KI = 16'sd0;
    localparam signed [15:0] KD = 16'sd14;

    // Prevent integral windup
    localparam signed [15:0] INTEGRAL_MAX =
        16'sd100;

    localparam signed [15:0] INTEGRAL_MIN =
        -16'sd100;

    // ================================================================
    // AGV STATE DEFINITIONS
    // ================================================================

    localparam [3:0]
        OUTBOUND_TRACE   = 4'd0,
        DESTINATION_WAIT = 4'd1,
        LIFT_UP          = 4'd2,
        TURN_TO_HOME     = 4'd3,
        RETURN_TRACE     = 4'd4,
        HOME_WAIT        = 4'd5,
        TURN_TO_WORK     = 4'd6,
        LIFT_DOWN        = 4'd7,
        RECOVER_OUTBOUND = 4'd8,
        RECOVER_RETURN   = 4'd9;

    reg [3:0] state = OUTBOUND_TRACE;

    // ================================================================
    // STATE MACHINE TIMERS
    // ================================================================

    reg [26:0] lift_timer =
        27'd0;

    reg [29:0] rfid_wait_timer =
        30'd0;

    reg [21:0] line_confirm_timer =
        22'd0;

    reg endpoint_armed =
        1'b1;

    // ================================================================
    // ULTRASONIC SENSOR
    // ================================================================

    wire [15:0] distance;
    wire        obstacle_detected;

    ultrasonic_sensor US0 (
        .clk      (clk),
        .echo     (echo),
        .trig     (trig),
        .distance (distance)
    );

    // Ignore distance = 0 because it can represent no valid reading yet.
    assign obstacle_detected =
        (distance > 16'd0) &&
        (distance < 16'd15);

    // ================================================================
    // UART / RFID RECEIVER
    // ================================================================

    wire [7:0] uart_data;
    wire       uart_ready;

    uart_rx receiver (
        .clk      (clk),
        .rx       (JA_RX),
        .rx_data  (uart_data),
        .rx_ready (uart_ready)
    );

    reg [7:0] current_command =
        8'h00;

    always @(posedge clk) begin
        if (uart_ready) begin
            current_command <= uart_data;
        end
    end

    // ================================================================
    // MAIN AGV STATE MACHINE
    // ================================================================

    always @(posedge clk) begin
        case (state)

            // ========================================================
            // Follow line toward destination
            // ========================================================

            OUTBOUND_TRACE: begin
                lift_timer         <= 27'd0;
                rfid_wait_timer    <= 30'd0;
                line_confirm_timer <= 22'd0;

                // Rearm endpoint detection after leaving 1111.
                if (s != 4'b1111) begin
                    endpoint_armed <= 1'b1;
                end

                // End-of-line marker detected.
                if ((s == 4'b1111) &&
                    endpoint_armed) begin

                    endpoint_armed <= 1'b0;

                    // RFID data arrived at the same moment.
                    if (uart_ready) begin
                        state      <= LIFT_UP;
                        lift_timer <= 27'd0;
                    end
                    else begin
                        state           <= DESTINATION_WAIT;
                        rfid_wait_timer <= 30'd0;
                    end
                end
            end

            // ========================================================
            // Wait up to 7 seconds for destination RFID
            // ========================================================

            DESTINATION_WAIT: begin
                lift_timer         <= 27'd0;
                line_confirm_timer <= 22'd0;

                if (uart_ready) begin
                    state           <= LIFT_UP;
                    lift_timer      <= 27'd0;
                    rfid_wait_timer <= 30'd0;
                end
                else if (
                    rfid_wait_timer >=
                    RFID_WAIT_TIME - 1'b1
                ) begin
                    // No RFID detected.
                    // Treat this as an accidental track loss.
                    state              <= RECOVER_OUTBOUND;
                    rfid_wait_timer    <= 30'd0;
                    line_confirm_timer <= 22'd0;
                end
                else begin
                    rfid_wait_timer <=
                        rfid_wait_timer + 1'b1;
                end
            end

            // ========================================================
            // Lift cargo
            // ========================================================

            LIFT_UP: begin
                rfid_wait_timer    <= 30'd0;
                line_confirm_timer <= 22'd0;

                if (lift_timer >=
                    LIFT_TIME - 1'b1) begin

                    state              <= TURN_TO_HOME;
                    lift_timer         <= 27'd0;
                    line_confirm_timer <= 22'd0;
                end
                else begin
                    lift_timer <= lift_timer + 1'b1;
                end
            end

            // ========================================================
            // Rotate until the line is found again
            // ========================================================

            TURN_TO_HOME: begin
                lift_timer      <= 27'd0;
                rfid_wait_timer <= 30'd0;

                if (s == 4'b0000) begin
                    if (
                        line_confirm_timer >=
                        LINE_CONFIRM_TIME - 1'b1
                    ) begin
                        state              <= RETURN_TRACE;
                        line_confirm_timer <= 22'd0;
                        endpoint_armed     <= 1'b0;
                    end
                    else begin
                        line_confirm_timer <=
                            line_confirm_timer + 1'b1;
                    end
                end
                else begin
                    line_confirm_timer <= 22'd0;
                end
            end

            // ========================================================
            // Follow line back toward starting point
            // ========================================================

            RETURN_TRACE: begin
                lift_timer         <= 27'd0;
                rfid_wait_timer    <= 30'd0;
                line_confirm_timer <= 22'd0;

                if (s != 4'b1111) begin
                    endpoint_armed <= 1'b1;
                end

                if ((s == 4'b1111) &&
                    endpoint_armed) begin

                    endpoint_armed <= 1'b0;

                    if (uart_ready) begin
                        state <= TURN_TO_WORK;
                    end
                    else begin
                        state           <= HOME_WAIT;
                        rfid_wait_timer <= 30'd0;
                    end
                end
            end

            // ========================================================
            // Wait up to 7 seconds for home RFID
            // ========================================================

            HOME_WAIT: begin
                lift_timer         <= 27'd0;
                line_confirm_timer <= 22'd0;

                if (uart_ready) begin
                    state              <= TURN_TO_WORK;
                    rfid_wait_timer    <= 30'd0;
                    line_confirm_timer <= 22'd0;
                end
                else if (
                    rfid_wait_timer >=
                    RFID_WAIT_TIME - 1'b1
                ) begin
                    state              <= RECOVER_RETURN;
                    rfid_wait_timer    <= 30'd0;
                    line_confirm_timer <= 22'd0;
                end
                else begin
                    rfid_wait_timer <=
                        rfid_wait_timer + 1'b1;
                end
            end

            // ========================================================
            // Rotate at home until line is found again
            // ========================================================

            TURN_TO_WORK: begin
                lift_timer      <= 27'd0;
                rfid_wait_timer <= 30'd0;

                if (s == 4'b0000) begin
                    if (
                        line_confirm_timer >=
                        LINE_CONFIRM_TIME - 1'b1
                    ) begin
                        state              <= LIFT_DOWN;
                        line_confirm_timer <= 22'd0;
                    end
                    else begin
                        line_confirm_timer <=
                            line_confirm_timer + 1'b1;
                    end
                end
                else begin
                    line_confirm_timer <= 22'd0;
                end
            end

            // ========================================================
            // Lower cargo
            // ========================================================

            LIFT_DOWN: begin
                rfid_wait_timer    <= 30'd0;
                line_confirm_timer <= 22'd0;

                if (lift_timer >=
                    LIFT_TIME - 1'b1) begin

                    state          <= OUTBOUND_TRACE;
                    lift_timer     <= 27'd0;
                    endpoint_armed <= 1'b0;
                end
                else begin
                    lift_timer <= lift_timer + 1'b1;
                end
            end

            // ========================================================
            // Recover track while travelling outbound
            // ========================================================

            RECOVER_OUTBOUND: begin
                lift_timer      <= 27'd0;
                rfid_wait_timer <= 30'd0;

                if (s == 4'b0000) begin
                    if (
                        line_confirm_timer >=
                        LINE_CONFIRM_TIME - 1'b1
                    ) begin
                        state              <= OUTBOUND_TRACE;
                        line_confirm_timer <= 22'd0;
                        endpoint_armed     <= 1'b0;
                    end
                    else begin
                        line_confirm_timer <=
                            line_confirm_timer + 1'b1;
                    end
                end
                else begin
                    line_confirm_timer <= 22'd0;
                end
            end

            // ========================================================
            // Recover track while travelling home
            // ========================================================

            RECOVER_RETURN: begin
                lift_timer      <= 27'd0;
                rfid_wait_timer <= 30'd0;

                if (s == 4'b0000) begin
                    if (
                        line_confirm_timer >=
                        LINE_CONFIRM_TIME - 1'b1
                    ) begin
                        state              <= RETURN_TRACE;
                        line_confirm_timer <= 22'd0;
                        endpoint_armed     <= 1'b0;
                    end
                    else begin
                        line_confirm_timer <=
                            line_confirm_timer + 1'b1;
                    end
                end
                else begin
                    line_confirm_timer <= 22'd0;
                end
            end

            // ========================================================
            // Safety default
            // ========================================================

            default: begin
                state               <= OUTBOUND_TRACE;
                lift_timer          <= 27'd0;
                rfid_wait_timer     <= 30'd0;
                line_confirm_timer  <= 22'd0;
                endpoint_armed      <= 1'b1;
            end
        endcase
    end

    // ================================================================
    // LIFT ACTUATOR CONTROL
    // ================================================================

    wire actuator_enable;
    wire actuator_direction;

    assign actuator_enable =
        (state == LIFT_UP) ||
        (state == LIFT_DOWN);

    // 1 = raise, 0 = lower
    assign actuator_direction =
        (state == LIFT_UP);

    stepper_driver Actuator0 (
        .clk        (clk),
        .motor_en   (actuator_enable),
        .motor_dir  (actuator_direction),

        .motor_ain1 (stepper_a1),
        .motor_ain2 (stepper_a2),
        .motor_pwma (stepper_b1),
        .motor_stby (stepper_b2)
    );

    // ================================================================
    // SENSOR-PATTERN-TO-ERROR LOOKUP TABLE
    // ================================================================
    //
    // Negative error:
    //     Line is toward the left side.
    //
    // Positive error:
    //     Line is toward the right side.
    //
    // Zero error:
    //     Robot is centered.
    //
    // 1111 is handled separately as the endpoint marker.
    // ================================================================

    reg signed [7:0] current_error;
    reg signed [7:0] previous_error = 8'sd0;

    always @(*) begin
        case (s)

            // Far left
            4'b1000:
                current_error = -8'sd3;

            // Left
            4'b1100:
                current_error = -8'sd2;

            // Slightly left
            4'b0100:
                current_error = -8'sd1;

            // Centered patterns
            4'b0110,
            4'b1001,
            4'b0000:
                current_error = 8'sd0;

            // Slightly right
            4'b0010:
                current_error = 8'sd1;

            // Right
            4'b0011:
                current_error = 8'sd2;

            // Far right
            4'b0001:
                current_error = 8'sd3;

            // Ambiguous sensor combinations:
            // continue correcting in the previous direction.
            default:
                current_error = previous_error;
        endcase
    end

    // ================================================================
    // PID CONTROLLER
    // ================================================================

    reg [16:0] pid_update_counter =
        17'd0;

    reg signed [15:0] integral_error =
        16'sd0;

    reg signed [15:0] next_integral;

    reg signed [15:0] derivative_error;

    reg signed [31:0] proportional_term;
    reg signed [31:0] integral_term;
    reg signed [31:0] derivative_term;
    reg signed [31:0] pid_correction;

    reg signed [31:0] calculated_left_speed;
    reg signed [31:0] calculated_right_speed;

    reg [7:0] pid_speed_left =
        BASE_SPEED;

    reg [7:0] pid_speed_right =
        BASE_SPEED;

    // ------------------------------------------------
    // Integral clamping
    // ------------------------------------------------

    always @(*) begin
        if (
            integral_error +
            current_error >
            INTEGRAL_MAX
        ) begin
            next_integral =
                INTEGRAL_MAX;
        end
        else if (
            integral_error +
            current_error <
            INTEGRAL_MIN
        ) begin
            next_integral =
                INTEGRAL_MIN;
        end
        else begin
            next_integral =
                integral_error +
                current_error;
        end
    end

    // ------------------------------------------------
    // PID mathematical calculation
    // ------------------------------------------------

    always @(*) begin
        derivative_error =
            current_error -
            previous_error;

        proportional_term =
            KP * current_error;

        integral_term =
            KI * next_integral;

        derivative_term =
            KD * derivative_error;

        pid_correction =
            proportional_term +
            integral_term +
            derivative_term;

        // Negative correction steers one way.
        // Positive correction steers the other way.
        calculated_left_speed =
            $signed({1'b0, BASE_SPEED}) -
            pid_correction;

        calculated_right_speed =
            $signed({1'b0, BASE_SPEED}) +
            pid_correction;
    end

    // ------------------------------------------------
    // PID update timer and PWM saturation
    // ------------------------------------------------

    always @(posedge clk) begin

        if (
            (state == OUTBOUND_TRACE) ||
            (state == RETURN_TRACE)
        ) begin

            if (
                pid_update_counter >=
                PID_UPDATE_TIME - 1'b1
            ) begin
                pid_update_counter <= 17'd0;

                previous_error <= current_error;
                integral_error <= next_integral;

                // Clamp left speed to 0–255.
                if (calculated_left_speed < 0) begin
                    pid_speed_left <= 8'd0;
                end
                else if (
                    calculated_left_speed > 255
                ) begin
                    pid_speed_left <= 8'd255;
                end
                else begin
                    pid_speed_left <=
                        calculated_left_speed[7:0];
                end

                // Clamp right speed to 0–255.
                if (calculated_right_speed < 0) begin
                    pid_speed_right <= 8'd0;
                end
                else if (
                    calculated_right_speed > 255
                ) begin
                    pid_speed_right <= 8'd255;
                end
                else begin
                    pid_speed_right <=
                        calculated_right_speed[7:0];
                end
            end
            else begin
                pid_update_counter <=
                    pid_update_counter + 1'b1;
            end
        end
        else begin
            // Reset PID memory outside line-following states.
            pid_update_counter <= 17'd0;
            previous_error     <= 8'sd0;
            integral_error     <= 16'sd0;
            pid_speed_left     <= BASE_SPEED;
            pid_speed_right    <= BASE_SPEED;
        end
    end

    // ================================================================
    // WHEEL MOTOR PWM INSTANCES
    // ================================================================

    reg [7:0] speed_left;
    reg [7:0] speed_right;

    reg fwd_l;
    reg rev_l;
    reg fwd_r;
    reg rev_r;

    motor_pwm MotorLeft (
        .clk  (clk),
        .duty (speed_left),

        // Left motor wiring is physically reversed.
        .fwd  (rev_l),
        .rev  (fwd_l),

        .in1  (in1),
        .in2  (in2)
    );

    motor_pwm MotorRight (
        .clk  (clk),
        .duty (speed_right),

        .fwd  (fwd_r),
        .rev  (rev_r),

        .in1  (in3),
        .in2  (in4)
    );

    // ================================================================
    // WHEEL DIRECTION AND SPEED CONTROL
    // ================================================================

    always @(*) begin

        // Default: stop both wheel motors.
        fwd_l       = 1'b0;
        rev_l       = 1'b0;
        fwd_r       = 1'b0;
        rev_r       = 1'b0;

        speed_left  = STOP_SPEED;
        speed_right = STOP_SPEED;

        case (state)

            // ========================================================
            // PID line following
            // ========================================================

            OUTBOUND_TRACE,
            RETURN_TRACE: begin

                // 1111 is the end-of-line marker.
                if (s != 4'b1111) begin
                    fwd_l = 1'b1;
                    rev_l = 1'b0;

                    fwd_r = 1'b1;
                    rev_r = 1'b0;

                    speed_left  = pid_speed_left;
                    speed_right = pid_speed_right;
                end
            end

            // ========================================================
            // Rotate in place
            // ========================================================

            TURN_TO_HOME,
            TURN_TO_WORK,
            RECOVER_OUTBOUND,
            RECOVER_RETURN: begin

                // Left wheel forward.
                fwd_l = 1'b1;
                rev_l = 1'b0;

                // Right wheel reverse.
                fwd_r = 1'b0;
                rev_r = 1'b1;

                speed_left  = ROTATE_SPEED;
                speed_right = ROTATE_SPEED;
            end

            // Waiting and lifting states stop the wheels.
            default: begin
                fwd_l       = 1'b0;
                rev_l       = 1'b0;
                fwd_r       = 1'b0;
                rev_r       = 1'b0;

                speed_left  = STOP_SPEED;
                speed_right = STOP_SPEED;
            end
        endcase

        // ============================================================
        // ULTRASONIC EMERGENCY STOP
        // ============================================================

        if (
            (
                (state == OUTBOUND_TRACE) ||
                (state == RETURN_TRACE)
            ) &&
            obstacle_detected
        ) begin
            fwd_l       = 1'b0;
            rev_l       = 1'b0;
            fwd_r       = 1'b0;
            rev_r       = 1'b0;

            speed_left  = STOP_SPEED;
            speed_right = STOP_SPEED;
        end
    end

    // ================================================================
    // LED DEBUG OUTPUTS
    // ================================================================

    // RFID/UART received byte
    assign led[7:0] =
        current_command;

    // Current state number
    assign led[11:8] =
        state;

    // Lift actuator active
    assign led[12] =
        actuator_enable;

    // Waiting for RFID
    assign led[13] =
        (state == DESTINATION_WAIT) ||
        (state == HOME_WAIT);

    // Ultrasonic obstacle detected
    assign led[14] =
        obstacle_detected;

    // Track recovery active
    assign led[15] =
        (state == RECOVER_OUTBOUND) ||
        (state == RECOVER_RETURN);

endmodule