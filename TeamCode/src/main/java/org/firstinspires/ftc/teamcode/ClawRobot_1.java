/*

        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.util.Range;

//@Disabled
@TeleOp(name="ClawRobot_1", group="TeleOp")
public class ClawRobot_1 extends OpMode {


    private DcMotor leftFrontDrive  = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive   = null;
    private DcMotor rightRearDrive  = null;
    private Servo grabber_left_servo = null;
    private Servo grabber_right_servo = null;
    double strafe;
    double drive;


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

        grabber_left_servo = hardwareMap.get(Servo.class, "grabber_left");
        //grab = 0.0   release = 0.25
        grabber_right_servo = hardwareMap.get(Servo.class, "grabber_right");
        //grab = 0.8   release = 0.5
        grabber_left_servo.setPosition(0.25);
        grabber_right_servo.setPosition(0.5);

    }

    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftRearPower;
        double rightRearPower;

        double turn     = -gamepad1.right_stick_x;
        double strafe   = -gamepad1.left_stick_x;
        double drive    = -gamepad1.left_stick_y;

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

    }

        if (gamepad1.a) {
            grabber_left_servo.setPosition(0.0);
    }

        if (gamepad1.b) {
            grabber_left_servo.setPosition(0.25);
    }

    @Override
    public void stop() {
    }
}
*/