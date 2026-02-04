package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.DifferentialDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.RevIMU;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.kinematics.DifferentialOdometry;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "Solvers Gamepad Test")
public class SolversTest extends LinearOpMode {
    //FTC Dashboard init
    FtcDashboard dashboard = FtcDashboard.getInstance();

    //runtime var
    private ElapsedTime runtime = new ElapsedTime();

    //motor vars
    MotorEx  leftMotor;
    MotorEx rightMotor;

    //pose vars
    double poseX;
    double poseY;
    double poseR;

    // encoder vars
    MotorEx encoderLeft, encoderRight;

    //drive system var
    DifferentialDrive drive;

    //gamepad var
    GamepadEx gamepad;

    //imu var
    IMU imu;

    //field telemetry packet
    TelemetryPacket packet;

    // define our trackwidth
    static final double TRACKWIDTH = 13.7;

    // convert ticks to inches
    static final double TICKS_TO_INCHES = 15.3;

    //camera processor class
    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    //main execution block
    @Override
    public void runOpMode() {
        //code to run on start
        //telemetry init
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //gamepad init
        gamepad = new GamepadEx(gamepad1);

        //motors init
        leftMotor = new MotorEx(hardwareMap, "left_drive");
        rightMotor = new MotorEx(hardwareMap, "right_drive");

        //drive system init
        drive = new DifferentialDrive(leftMotor, rightMotor);

        //camera stream init
        final VisionPortalStreamingOpMode.CameraStreamProcessor processor = new VisionPortalStreamingOpMode.CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);

        //imu init
        imu = hardwareMap.get(IMU.class, "imu");

        //set orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        //update imu with orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //odometry init
        DifferentialOdometry diffOdom = new DifferentialOdometry(
                () -> leftMotor.getCurrentPosition() /  26.235,
                () -> rightMotor.getCurrentPosition() / 26.235,
                TRACKWIDTH
        );
        diffOdom.updatePose();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //code to run on loop
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            //drive run:
            drive.arcadeDrive(gamepad.getLeftY(), gamepad.getRightX());

            //hit a to test telemetry
            if (gamepad.getButton(GamepadKeys.Button.A)) {
                telemetry.addData("Button", "pressed");
            } else {
                telemetry.addData("Button", "not pressed");
            }

            //odometry update
            diffOdom.updatePose();

            poseX = diffOdom.getPose().getX();
            poseY = diffOdom.getPose().getY();
            poseR = diffOdom.getPose().getHeading();

            telemetry.addData("X", poseX);
            telemetry.addData("Y", poseY);
            telemetry.addData("R", poseR);

            //update field telemetry
            packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .drawImage("/images/ftc.jpg", 18, 18, poseX, poseY);
            dashboard.sendTelemetryPacket(packet);

            //update telemetry every loop
            telemetry.update();
        }
    }
}
