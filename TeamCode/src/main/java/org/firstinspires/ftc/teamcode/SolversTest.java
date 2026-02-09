package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.drivebase.DifferentialDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

enum Color {
    GREEN,
    PURPLE
}

@TeleOp(name = "Solvers Gamepad Test")
public class SolversTest extends LinearOpMode {
    //FTC Dashboard init
    FtcDashboard dashboard = FtcDashboard.getInstance();

    //runtime var
    private final ElapsedTime runtime = new ElapsedTime();

    //motor vars
    MotorEx  leftMotor;
    MotorEx rightMotor;
    MotorEx collectorMotor;

    //flipper servos
    ServoEx flipperServo1;
    ServoEx flipperServo2;
    ServoEx flipperServo3;

    //servo stats
    Boolean servoState1 = false;
    Boolean servoState2 = false;
    Boolean servoState3 = false;

    //color sensors
    SensorRevColorV3 colorSensor1;
    SensorRevColorV3 colorSensor2;
    SensorRevColorV3 colorSensor3;

    //pose vars
    double poseX;
    double poseY;
    double poseR;

    //april tag processor
    AprilTagProcessor aprilTagProcessor;
    List<AprilTagDetection> aprilTagDetections;

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

    static Color getColor(SensorRevColorV3 colorSensor) {
        //g:r
        //green r<.5g or not
        //no ball <175

        double a = colorSensor.getARGB()[0];
        double r = colorSensor.getARGB()[1];
        double g = colorSensor.getARGB()[2];
        double b = colorSensor.getARGB()[3];

        Color color;

        if ((a + r + g + b) > 180) {
            if (r < 0.33 * g) {
                color = Color.GREEN;
            } else {
                color = Color.PURPLE;
            }
        } else {
            color = null;
        }

        return color;
    }

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

        //create april tag processor
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        //gamepad initd
        gamepad = new GamepadEx(gamepad1);

        //motors init
        leftMotor = new MotorEx(hardwareMap, "left_drive");
        rightMotor = new MotorEx(hardwareMap, "right_drive");
        collectorMotor = new MotorEx(hardwareMap, "CollectorCoreHex");

        //servo init
        flipperServo1 = new ServoEx(hardwareMap, "flipper1");
        flipperServo2 = new ServoEx(hardwareMap, "flipper2");
        flipperServo3 = new ServoEx(hardwareMap, "flipper3");

        //color sensor init
        colorSensor1 = new SensorRevColorV3(hardwareMap, "color1");
        colorSensor2 = new SensorRevColorV3(hardwareMap, "color2");
        colorSensor3 = new SensorRevColorV3(hardwareMap, "color3");

        //drive system init
        drive = new DifferentialDrive(leftMotor, rightMotor);

        //camera stream init
        final CameraStreamProcessor processor = new CameraStreamProcessor();

        new VisionPortal.Builder()
                .addProcessor(processor)
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //code to run on loop
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            //drive run:
            drive.arcadeDrive(gamepad.getLeftY() * .6, gamepad.getRightX() * .6);

            //Ball Collector controls
            if (gamepad1.right_trigger != 0.0) {
                collectorMotor.set(1.0);
            } else {
                collectorMotor.set(0.0);
            }

            if (gamepad1.x) { servoState1 = !servoState1; }
            if (gamepad1.y) { servoState2 = !servoState2; }
            if (gamepad1.a) { servoState3 = !servoState3; }

            if (servoState1) {
                flipperServo1.set(0.0);
            } else {
                flipperServo1.set(1.0);
            }
            if (servoState2) {
                flipperServo2.set(0.0);
            } else {
                flipperServo2.set(1.0);
            }
            if (servoState3) {
                flipperServo3.set(0.0);
            } else {
                flipperServo3.set(1.0);
            }

            Color color1 = getColor(colorSensor1);
            Color color2 = getColor(colorSensor2);
            Color color3 = getColor(colorSensor3);

            if (color1 != null) telemetry.addData("Color Sensor 1", color1.toString());
            if (color2 != null) telemetry.addData("Color Sensor 2", color2.toString());
            if (color3 != null) telemetry.addData("Color Sensor 3", color3.toString());

            //april tag detections
            aprilTagDetections = aprilTagProcessor.getDetections();
            for (AprilTagDetection detection : aprilTagDetections) {
                if (detection.metadata != null) {
                    telemetry.addData("Tag", detection.id);
                    telemetry.addData("Pose X", detection.ftcPose.x);
                    telemetry.addData("Pose Y", detection.ftcPose.y);
                    telemetry.addData("Pose Z", detection.ftcPose.z);
                }
            }

            //update field telemetry
            /*packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .drawImage("/images/ftc.jpg", 18, 18, poseX, poseY);
            dashboard.sendTelemetryPacket(packet);*/

            //update telemetry every loop
            telemetry.update();
        }
    }
}
