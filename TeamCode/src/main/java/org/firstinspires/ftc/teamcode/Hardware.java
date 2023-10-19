package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    public DcMotor leftFrontDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor leftRearDrive = null;
    public DcMotor rightRearDrive = null;

    public Servo testServo = null;

    double testServoPosition = 0;

    static final double MAX1 = 1;
    static final double MIN1 = 0;

    HardwareMap hwM = null;

    public Hardware(){

    }

    public void init(Hardware ahwMap){

        leftFrontDrive = hwM.get(DcMotor.class, "left_front_motor");
        rightFrontDrive = hwM.get(DcMotor.class, "right_front_motor");
        leftRearDrive = hwM.get(DcMotor.class, "left_rear_motor");
        rightRearDrive = hwM.get(DcMotor.class, "right_rear_motor");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        testServo = hwM.get(Servo.class, "test_servo");
    }
}
