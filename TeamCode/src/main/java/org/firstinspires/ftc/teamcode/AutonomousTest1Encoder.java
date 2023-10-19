package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@Autonomous(name="AutonomousTest1Encoder", group="Autonomous")
public class AutonomousTest1Encoder extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 6600 ;    // RPM for model am-2964
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;

    private Servo testServo = null;

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_rear_motor");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        testServo = hardwareMap.get(Servo.class, "test_servo");

        telemetry.addData("MotorPath0", "Start: %7d :%7d :%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition(),
                leftRearDrive.getCurrentPosition(),
                rightRearDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED, 5, 5, 1.0);   // Forward 5 Inches with 2 Second timeout
        encoderDrive(TURN_SPEED, -7.85, 7.85, 1.5);   // Turn Left 7.85 Inches with 1 Second timeout
        encoderDrive(DRIVE_SPEED, 36, 36, 8.0); // Forward 36 Inches with 5 Second timeout
        encoderDrive(TURN_SPEED, 15.7, -15.7, 3); // Turn Right 15.7 Inches with 2 Second timeout
        encoderDrive(DRIVE_SPEED, 36, 36, 8.0); // Forward 36 Inches with 5 Second timeout
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftFrontTarget  = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newLeftRearTarget   = leftRearDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightRearTarget  = rightRearDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                leftFrontDrive.setTargetPosition(newLeftFrontTarget);
                rightFrontDrive.setTargetPosition(newRightFrontTarget);
                leftRearDrive.setTargetPosition(newLeftRearTarget);
                rightRearDrive.setTargetPosition(newRightRearTarget);

                // Turn On RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                leftFrontDrive.setPower(Math.abs(speed));
                rightFrontDrive.setPower(Math.abs(speed));
                leftRearDrive.setPower(Math.abs(speed));
                rightRearDrive.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftRearDrive.isBusy() && rightRearDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("MotorPath1",  "Running to %7d :%7d :%7d :%7d",
                            newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                    telemetry.addData("MotorPath2",  "Running at %7d :%7d :%7d :%7d",
                            leftFrontDrive.getCurrentPosition(),
                            rightFrontDrive.getCurrentPosition(),
                            leftRearDrive.getCurrentPosition(),
                            rightRearDrive.getCurrentPosition());
                    telemetry.update();
                }

                // Stop all motion;
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftRearDrive.setPower(0);
                rightRearDrive.setPower(0);

                // Turn off RUN_TO_POSITION
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                sleep(250);   // optional pause after each move

            }
        }
}
