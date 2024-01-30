
package org.firstinspires.ftc.teamcode;

//import com.outoftheboxrobotics.photoncore.PhotonCore;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Gamepad;
        import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ClawTest_2")
public class ClawTest_1 extends OpMode {

    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;

    private Servo grabber_left_servo = null;
    private Servo grabber_right_servo = null;
    private Servo twist_wrist_servo = null;
    private Servo angle_wrist_servo = null;
    // Variables for controlling servo update rate
    // Variables for controlling servo update rate
    private int servoUpdateCounter = 0;
    private int servoUpdateInterval = 5;
    Servo test_servo = grabber_left_servo;
    Gamepad currentGamepad1;
    Gamepad previousGamepad1;


    static double MIN_SERVO = 0;
    static double MAX_SERVO = 1;


    private void increment_servo(Servo active_servo) {
        double at = active_servo.getPosition();
        if (at < MAX_SERVO) {
            at += .01;
            active_servo.setPosition(at);
        }
    }

    private void decrement_servo(Servo active_servo) {
        double at = active_servo.getPosition();
        if (at > MIN_SERVO) {
            at -= .01;
            active_servo.setPosition(at);
        }
    }

    @Override
    public void init() {

//            front_left   = hardwareMap.get(DcMotor.class, "front_left");
//            front_right  = hardwareMap.get(DcMotor.class, "front_right");
//            back_left    = hardwareMap.get(DcMotor.class, "back_left");
//            back_right   = hardwareMap.get(DcMotor.class, "back_right");
//
//            intake_servo = hardwareMap.get(CRServo.class, "intake_servo");
//            intake_tilt_servo = hardwareMap.get(Servo.class, "intake_tilt_servo");
//            front_claw_servo = hardwareMap.get(Servo.class, "front_claw_servo");
//            back_claw_servo = hardwareMap.get(Servo.class, "back_claw_servo");
//            front_wrist_servo = hardwareMap.get(Servo.class, "front_wrist_servo");
//            back_wrist_servo = hardwareMap.get(Servo.class, "back_wrist_servo");
//
//        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        front_right.setDirection(DcMotor.Direction.REVERSE);
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        grabber_left_servo = hardwareMap.get(Servo.class, "grabber_left");
        //grab = 0.0   release = 0.25
        grabber_right_servo = hardwareMap.get(Servo.class, "grabber_right");
        //grab = 0.8   release = 0.5
        twist_wrist_servo = hardwareMap.get(Servo.class, "twist_wrist");
        //grab = 0.0   release = 0.25
        angle_wrist_servo = hardwareMap.get(Servo.class, "angle_wrist");
        //grab = 0.8   release = 0.5

        twist_wrist_servo.setPosition(0.5);
        // max
        angle_wrist_servo.setPosition(0.74);
        // max 1.0
        // min 0.4
        // mid .74
    }

    @Override
    public void loop() {
//
//        telemetry.addData("front_claw_servo:",front_claw_servo.getPosition());
//        telemetry.addData("back_claw_servo:",front_claw_servo.getPosition());
//        telemetry.addData("front_wrist_servo:",front_claw_servo.getPosition());
//        telemetry.addData("back_wrist_servo:",front_claw_servo.getPosition());

        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;
        boolean intake_out = gamepad1.left_bumper; 
        boolean intake_in = gamepad1.right_bumper;
        boolean right_trigger = gamepad1.right_trigger > .1;
        boolean left_trigger = gamepad1.left_trigger > .1;


        // Only update the servo position every servoUpdateInterval iterations
        if (servoUpdateCounter >= servoUpdateInterval) {

            if (left_trigger) {
                decrement_servo(test_servo);
            } else {
                if (right_trigger) {
                    increment_servo(test_servo);
                }
            }
            // Reset the update counter
            servoUpdateCounter = 0;
        } else {
            // Increment the update counter
            servoUpdateCounter++;
        }

        if (gamepad1.a) {
         //   test_servo = grabber_left_servo;
            grabber_left_servo.setPosition(0.0);
            grabber_right_servo.setPosition(0.75);
        }

        if (gamepad1.b) {
         //   test_servo = grabber_right_servo;
            grabber_left_servo.setPosition(0.25);
            grabber_right_servo.setPosition(0.5);
        }
        if (gamepad1.x) {
            test_servo = twist_wrist_servo;
        }
        if (gamepad1.y) {
            test_servo = angle_wrist_servo;

        }
        telemetry.addData("Servos", "leftClaw (%.2f), rightClaw (%.2f), twist (%.2f), angle (%.2f), ",

                grabber_left_servo.getPosition(),
                grabber_right_servo.getPosition(),
                twist_wrist_servo.getPosition(),
                angle_wrist_servo.getPosition()
        );
//        if (gamepad1.x) {
//            test_servo = front_wrist_servo;
//        }
//
//        if (gamepad1.y) {
//            test_servo = back_wrist_servo;
//        }*/

        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
//
//        front_left.setPower(speeds[0]);
//        front_right.setPower(speeds[1]);
//        back_left.setPower(speeds[2]);
//        back_right.setPower(speeds[3]);
//
//        if (intake_in)
//        {
//            intake_servo.setPower(0.50);
//            intake_tilt_servo.setPosition(0.80);
//        }
//        else
//        {
//            if (intake_out)
//            {
//                intake_servo.setPower(-1);
//                intake_tilt_servo.setPosition(0.85);
//            }
//            else
//            {
//                intake_servo.setPower(0);
//                intake_tilt_servo.setPosition(0.91);
//            }
//        }


    }
}