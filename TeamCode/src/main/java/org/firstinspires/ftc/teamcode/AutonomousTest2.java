package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name="AutonomousTest2", group="Autonomous")
public class AutonomousTest2 extends LinearOpMode {

    private DcMotor sweeper = null;
    private Servo contServo = null;
    private Servo testServo = null;

    @Override
    public void runOpMode() {

        sweeper     = hardwareMap.get(DcMotor.class, "sweeper");
        contServo   = hardwareMap.get(Servo.class, "cont_servo");
        testServo   = hardwareMap.get(Servo.class, "test_servo");

        waitForStart();
    }
}
