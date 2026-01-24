package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="PID Op Mode")

public class DriveOpMode extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorExLeft = null;
    private DcMotorEx motorExRight = null;

    private CRServo servo = null;

    //left - up
    public static final double PLeft = 18.0;
    public static final double ILeft = 0.3;
    public static final double DLeft = 0.0;
    public static final double FLeft = 32.0;

    //right - down
    public static final double PRight = 18.0;
    public static final double IRight = 0.3;
    public static final double DRight = 0.0;
    public static final double FRight = 32.0;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "left_drive");
        motorExRight = (DcMotorEx)hardwareMap.get(DcMotor.class, "right_drive");

        servo = hardwareMap.get(CRServo.class, "servo");

        // Note: Change Directions to fix backwards motion
        motorExLeft.setDirection(DcMotor.Direction.FORWARD);
        motorExRight.setDirection(DcMotor.Direction.REVERSE);

        //starts and sets up motors to use encoders
        motorExLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //servo direction
        servo.setDirection(DcMotorSimple.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

        runtime.reset();

        //set left pidf coefficients
        PIDFCoefficients pidfCoELeft = new PIDFCoefficients(PLeft, ILeft, DLeft, FLeft);
        motorExLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoELeft);

        //read left pidf coefficients
        PIDFCoefficients pidfModifiedLeft = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        //set right pidf coefficients
        PIDFCoefficients pidfCoERight = new PIDFCoefficients(PRight, IRight, DRight, FRight);
        motorExRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoERight);

        //read right pid coefficients
        PIDFCoefficients pidfModifiedRight = motorExRight.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        //calculate power
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1, 1) * 1000;
        rightPower   = Range.clip(drive - turn, -1, 1) * 1000;

        // Send calculated power to wheels
        motorExLeft.setVelocity(leftPower);
        motorExRight.setVelocity(rightPower);

        if (gamepad1.a) {
            servo.setPower(10);
            telemetry.addData("Game[pad A", servo.getPower());
        } else {
            servo.setPower(0);
        }

        //calculate velocities
        double leftVelocity = motorExLeft.getVelocity();
        double rightVelocity = motorExRight.getVelocity();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Encoders: ", "left (%.3f), right (%.3f)", leftVelocity, rightVelocity);
    }

    @Override
    public void stop() {
    }

}