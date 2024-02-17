package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


//@Disabled
public class thirdAutocenterblue extends LinearOpMode {

    private VisionPortal visionPortal;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armmotor = null;
    private DcMotor slidemotor = null;
    private Servo clawmotor = null;


    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");
        armmotor = hardwareMap.get(DcMotor.class, "am");
        slidemotor = hardwareMap.get(DcMotor.class, "sm");
        clawmotor = hardwareMap.get(Servo.class, "csm");


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armmotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Preload");
        telemetry.update();
        clawmotor.setPosition(0.22);
        telemetry.addData("Status", "Preload Complete");
        telemetry.update();

        OpenCvWebcam webcam;


            /*
             * Instantiate an OpenCvCamera object for the camera we'll be using.
             * In this sample, we're using a webcam. Note that you will need to
             * make sure you have added the webcam to your configuration file and
             * adjusted the name here to match what you named it in said config file.
             *
             * We pass it the view that we wish to use for camera monitor (on
             * the RC phone). If no camera monitor is desired, use the alternate
             * single-parameter constructor instead (commented out below)
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

            // OR...  Do Not Activate the Camera Monitor View
            //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

            /*
             * Specify the image processing pipeline we wish to invoke upon receipt
             * of a frame from the camera. Note that switching pipelines on-the-fly
             * (while a streaming session is in flight) *IS* supported.
             */
            webcam.setPipeline(new VisionBlue());

            /*
             * Open the connection to the camera device. New in v1.4.0 is the ability
             * to open the camera asynchronously, and this is now the recommended way
             * to do it. The benefits of opening async include faster init time, and
             * better behavior when pressing stop during init (i.e. less of a chance
             * of tripping the stuck watchdog)
             *
             * If you really want to open synchronously, the old method is still available.
             */
            webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    /*
                     * Tell the webcam to start streaming images to us! Note that you must make sure
                     * the resolution you specify is supported by the camera. If it is not, an exception
                     * will be thrown.
                     *
                     * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                     * supports streaming from the webcam in the uncompressed YUV image format. This means
                     * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                     * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                     *
                     * Also, we specify the rotation that the webc am is used in. This is so that the image
                     * from the camera sensor can be rotated such that it is always displayed with the image upright.
                     * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                     * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                     * away from the user.
                     */
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

                    telemetry.addData("status", "cam on");
                    telemetry.update();
                }

                @Override
                public void onError(int errorCode) {



                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

            telemetry.addLine("Waiting for start");
            telemetry.update();

            /*
             * Wait for the user to press start on the Driver Station
             */

            waitForStart();


            telemetry.addData("Status", "Begin Autonomous");    //
            telemetry.update();
            sleep(1000);
            telemetry.addData("Status", "Arm Moving");
            telemetry.update();
            armmotor.setPower(-0.30);
            sleep(2000);
            armmotor.setPower(0);
            telemetry.addData("Status", "Arm Stopped");
            telemetry.update();


            telemetry.addData("status", "Pixle recognized");

            sleep(5000);
            telemetry.addData("Status", "Drive Forward");
            telemetry.update();
            leftBackDrive.setPower(0.25);
            leftFrontDrive.setPower(0.25);
            rightFrontDrive.setPower(0.25);
            rightBackDrive.setPower(0.25);
            sleep(1400);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            sleep(1000);
            telemetry.addData("Status", "Drive complete");
            telemetry.update();


            sleep(500);
            telemetry.addData("Status", "Claw Opening");
            telemetry.update();
            clawmotor.setPosition(0.45);
            sleep(1000);
            telemetry.addData("Status", "Claw Open pixle out");
            telemetry.update();
            sleep(1000);
            telemetry.addData("Status", "Claw Opening");

            telemetry.addData("Status", "Back UP");
            telemetry.update();
            leftBackDrive.setPower(-0.23);
            leftFrontDrive.setPower(-0.23);
            rightFrontDrive.setPower(-0.23);
            rightBackDrive.setPower(-0.23);
            sleep(700);
            armmotor.setPower(0.30);
            sleep(800);
            telemetry.addData("Status", "Drive complete");
            telemetry.update();

            sleep(1000);
            telemetry.addData("status", "strafe");
            rightFrontDrive.setPower(0.25);
            leftBackDrive.setPower(0.25);
            rightBackDrive.setPower(-0.25);
            leftFrontDrive.setPower(-0.25);
            sleep(4000);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }


    }   // end method telemetryTfod()




