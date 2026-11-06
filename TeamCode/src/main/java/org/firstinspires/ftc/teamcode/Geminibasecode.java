package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_Mecanum", group = "TeleOp")
public class Geminibasecode extends LinearOpMode {

    // Servo Constants
    private static final double FEEDER_REST_POS = 0.0;
    private static final double FEEDER_PUSH_POS = 0.5;
    private static final double GATE_CLOSED_POS = 0.0;
    private static final double GATE_OPEN_POS   = 0.5;

    // Shooter Power
    private static final double SHOOTER_POWER = 0.80;

    // Hardware Declarations
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor shooterMotor = null;
    private Servo feederServo = null;
    private Servo gateServo = null;

    @Override
    public void runOpMode() {

        // Hardware Mapping
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        try {
            shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
            feederServo  = hardwareMap.get(Servo.class, "feeder_servo");
            gateServo    = hardwareMap.get(Servo.class, "gate_servo");
        } catch (Exception e) {
            telemetry.addData("Notice", "Optional hardware mechanisms missing.");
        }

        // Motor Directions
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run Without Encoders for TeleOp responsiveness
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initial Servo States
        if (gateServo != null) gateServo.setPosition(GATE_CLOSED_POS);
        if (feederServo != null) feederServo.setPosition(FEEDER_REST_POS);

        telemetry.addData("Status", "TeleOp Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -----------------------------------------------------------------
            // GAMEPAD 1: Mecanum Drive Controls
            // -----------------------------------------------------------------
            double y  = -gamepad1.left_stick_y;  // Forward/Backward (inverted stick)
            double x  =  gamepad1.left_stick_x;   // Strafe Left/Right
            double rx =  gamepad1.right_stick_x;  // Turn Left/Right

            // Calculate Mecanum power distribution
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            // -----------------------------------------------------------------
            // GAMEPAD 2: Mechanism Controls (Shooter, Gate, Feeder)
            // -----------------------------------------------------------------

            // Shooter Motor Toggle / Hold (Gamepad 2 - Button A or Right Trigger)
            if (shooterMotor != null) {
                if (gamepad2.a || gamepad2.right_trigger > 0.1) {
                    shooterMotor.setPower(SHOOTER_POWER);
                } else {
                    shooterMotor.setPower(0.0);
                }
            }

            // Gate Servo Control (Gamepad 2 - Right Bumper opens, otherwise holds closed)
            if (gateServo != null) {
                if (gamepad2.right_bumper) {
                    gateServo.setPosition(GATE_OPEN_POS);
                } else {
                    gateServo.setPosition(GATE_CLOSED_POS);
                }
            }

            // Feeder Servo Control (Gamepad 2 - Button X or Left Bumper pushes ball)
            if (feederServo != null) {
                if (gamepad2.x || gamepad2.left_bumper) {
                    feederServo.setPosition(FEEDER_PUSH_POS);
                } else {
                    feederServo.setPosition(FEEDER_REST_POS);
                }
            }

            // Telemetry Output
            telemetry.addData("Shooter Status", shooterMotor != null && shooterMotor.getPower() > 0 ? "SPINNING" : "OFF");
            telemetry.addData("Gate Status", gamepad2.right_bumper ? "OPEN" : "CLOSED");
            telemetry.addData("Drive Motors", "LF: %.2f | RF: %.2f", frontLeftPower, frontRightPower);
            telemetry.update();
        }
    }
}