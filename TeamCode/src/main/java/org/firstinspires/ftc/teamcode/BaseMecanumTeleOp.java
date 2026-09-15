package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Base Mecanum TeleOp", group = "Drive")
public class BaseMecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Mapping matching REV Control/Expansion Hub port names
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor backRight  = hardwareMap.get(DcMotor.class, "back_right");

        // Reverse left motors so forward stick inputs drive all wheels forward
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake mode to resist external pushing when sticks are centered
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized & Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Gamepad 1 Controls
            // Left Stick Y: Forward / Backward (Inverted because pushed UP returns -1.0)
            double y = -gamepad1.left_stick_y;

            // Left Stick X: Strafe Left / Strafe Right (1.1 compensates for strafe friction)
            double x = gamepad1.left_stick_x * 1.1;

            // Right Stick X: Rotate Left / Rotate Right
            double rx = gamepad1.right_stick_x;

            // Denominator normalizes motor power values to stay within -1.0 to 1.0 range
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // Write output power levels to drive motors
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
}