package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

//@Disabled
@TeleOp(name="TeleOpTest1", group="TeleOp")
public class TeleOpTest1 extends OpMode {

    private DcMotor leftFrontDrive  = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive   = null;
    private DcMotor rightRearDrive  = null;

    double strafe;
    double drive;

    private CRServo contServo   = null;

    /*
    private Servo   testServo   = null;
    private DcMotor sweeper     = null;

    double testServoPosition = 0;

    static final double MAX1 = 1;
    static final double MIN1 = 0;

    private DcMotor sweeper = null;
    */

    @Override
    public void init() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftRearDrive   = hardwareMap.get(DcMotor.class, "left_rear_motor");
        rightRearDrive  = hardwareMap.get(DcMotor.class, "right_rear_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        contServo = hardwareMap.get(CRServo.class, "cont_servo");

        /*
        testServo = hardwareMap.get(Servo.class, "test_servo");
        sweeper = hardwareMap.get(DcMotor.class, "sweeper");
        sweeper.setDirection(DcMotor.Direction.FORWARD);
        */
    }

    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftRearPower;
        double rightRearPower;

        double turn     = gamepad1.right_stick_x;
        double strafe   = gamepad1.left_stick_x;
        double drive    = gamepad1.left_stick_y;

        leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        rightFrontPower = Range.clip((drive - turn) - strafe, -1.0, 1.0);
        leftRearPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightRearPower = Range.clip((drive - turn) + strafe, -1.0, 1.0);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);

        telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f), leftRear (%.2f), rightRear (%.2f)",
                leftFrontDrive.getPower(),
                rightFrontDrive.getPower(),
                leftRearDrive.getPower(),
                rightRearDrive.getPower());

        /*
        if(testServoPosition < MAX1)
            testServoPosition += gamepad2.right_trigger/4;
        if(testServoPosition > MIN1)
            testServoPosition -= gamepad2.left_trigger/4;

        testServo.setPosition(testServoPosition);
        telemetry.addData("Servo", "Position (%.2f)", testServo.getPosition());
        */

        contServo.setPower(-gamepad2.left_stick_y);
        telemetry.addData("ContServo", "Moving (%.2f)", contServo.getPower());

        /*
        double sweepPower = gamepad2.right_trigger - gamepad2.left_trigger;
        sweeper.setPower(sweepPower);
        telemetry.addData("Sweep", "Speed", sweeper.getPower());
        */
    }

    @Override
    public void stop() {
    }
}
