package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous - Gate Servo Only", group = "Autonomous")
public class Auto_Bucket_Shoot_Move extends LinearOpMode {

    // Hardware declarations
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo gateServo = null;

    // Servo positions (adjust these values based on your mechanical testing)
    private static final double GATE_CLOSED_POS = 0.0;
    private static final double GATE_OPEN_POS   = 0.5;

    @Override
    public void runOpMode() {

        // Initialize Hardware Map
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        gateServo  = hardwareMap.get(Servo.class, "gateServo");

        // Motor directions (adjust depending on drive train orientation)
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set initial gate state to CLOSED before start
        gateServo.setPosition(GATE_CLOSED_POS);

        telemetry.addData("Status", "Initialized & Gate Closed");
        telemetry.update();

        // Wait for Driver Station Start button
        waitForStart();

        if (opModeIsActive()) {

            // Step 1: Drive forward toward target
            telemetry.addData("Path", "Driving Forward");
            telemetry.update();
            setDrivePower(0.5, 0.5);
            sleep(1500); // Drive for 1.5 seconds

            // Step 2: Stop robot
            setDrivePower(0, 0);
            sleep(250);

            // Step 3: Open the gate to release element
            telemetry.addData("Path", "Opening Gate");
            telemetry.update();
            gateServo.setPosition(GATE_OPEN_POS);
            sleep(1000); // Wait 1 second for element to drop/dump

            // Step 4: Close the gate (optional)
            gateServo.setPosition(GATE_CLOSED_POS);
            sleep(500);

            // Step 5: Back away
            telemetry.addData("Path", "Backing Up");
            telemetry.update();
            setDrivePower(-0.4, -0.4);
            sleep(1000);

            // Step 6: Stop motion
            setDrivePower(0, 0);
            telemetry.addData("Status", "Complete");
            telemetry.update();
        }
    }

    /**
     * Helper method to set drive motor powers
     */
    private void setDrivePower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }
}