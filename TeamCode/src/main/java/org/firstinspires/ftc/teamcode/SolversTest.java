package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
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
    ServoEx flipperServo0;
    ServoEx flipperServo1;
    ServoEx flipperServo2;

    //servo stats
    Boolean servoState0 = false;
    Boolean servoState1 = false;
    Boolean servoState2 = false;

    //color sensors
    SensorRevColorV3 colorSensor0;
    SensorRevColorV3 colorSensor1;
    SensorRevColorV3 colorSensor2;

    //nothing threshold
    Double sensorThreshold0 = 180.0;
    Double sensorThreshold1 = 220.0;
    Double sensorThreshold2 = 175.0;

    //red threshold
    Double redThreshold0 = 0.5;
    Double redThreshold1 = 0.5;
    Double redThreshold2 = 0.5;

    //pose vars
    double poseX;
    double poseY;
    double poseR;

    //april tag processor
    AprilTagProcessor aprilTagProcessor;
    List<AprilTagDetection> aprilTagDetections;

    //drive system var
    DifferentialDrive drive;

    //gamepad var
    GamepadEx gamepad;

    //field telemetry packet
    TelemetryPacket packet;

    static Color getColor(SensorRevColorV3 colorSensor, Double threshold, Double redThreshold) {
        //g:r
        //green r<.5g or not
        //no ball <175

        double a = colorSensor.getARGB()[0];
        double r = colorSensor.getARGB()[1];
        double g = colorSensor.getARGB()[2];
        double b = colorSensor.getARGB()[3];

        Color color;

        if ((a + r + g + b) > threshold) {
            if (r < redThreshold * g) {
                color = Color.GREEN;
            } else {
                color = Color.PURPLE;
            }
        } else {
            color = null;
        }

        return color;
    }

    static Double getColorSum(SensorRevColorV3 colorSensor) {
        double a = colorSensor.getARGB()[0];
        double r = colorSensor.getARGB()[1];
        double g = colorSensor.getARGB()[2];
        double b = colorSensor.getARGB()[3];

        return a + r + g +b;
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
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        //gamepad init
        gamepad = new GamepadEx(gamepad1);

        //motors init
        leftMotor = new MotorEx(hardwareMap, "left_drive");
        rightMotor = new MotorEx(hardwareMap, "right_drive");
        collectorMotor = new MotorEx(hardwareMap, "CollectorCoreHex");

        //servo init
        flipperServo0 = new ServoEx(hardwareMap, "flipper0");
        flipperServo1 = new ServoEx(hardwareMap, "flipper1");
        flipperServo2 = new ServoEx(hardwareMap, "flipper2");

        //color sensor init
        colorSensor0 = new SensorRevColorV3(hardwareMap, "color0");
        colorSensor1 = new SensorRevColorV3(hardwareMap, "color1");
        colorSensor2 = new SensorRevColorV3(hardwareMap, "color2");

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
            telemetry.addData("Status", "Run Time: " + runtime);

            //drive run:
            drive.arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);

            //Ball Collector controls
            if (gamepad1.right_trigger != 0.0) {
                collectorMotor.set(1.0);
            } else {
                collectorMotor.set(0.0);
            }

            if (gamepad1.b) { servoState0 = !servoState0; }
            if (gamepad1.x) { servoState1 = !servoState1; }
            if (gamepad1.a) { servoState2 = !servoState2; }

            if (servoState0) {
                flipperServo0.set(0.0);
            } else {
                flipperServo0.set(1.0);
            }
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

            //get colors
            Color color0 = getColor(colorSensor0, sensorThreshold0, redThreshold0);
            Color color1 = getColor(colorSensor1, sensorThreshold1, redThreshold1);
            Color color2 = getColor(colorSensor2, sensorThreshold2, redThreshold2);

            if (color0 != null) telemetry.addData("Color Sensor 0", color0.toString());
            if (color1 != null) telemetry.addData("Color Sensor 1", color1.toString());
            if (color2 != null) telemetry.addData("Color Sensor 2", color2.toString());

            //color sum
            /*Double colorSum0 = getColorSum(colorSensor0);
            Double colorSum1 = getColorSum(colorSensor1);
            Double colorSum2 = getColorSum(colorSensor2);

            telemetry.addData("Color Sum 0", colorSum0);
            telemetry.addData("Color Sum 1", colorSum1);
            telemetry.addData("Color Sum 2", colorSum2);

            telemetry.addData("A0", colorSensor0.getARGB()[0]);
            telemetry.addData("R0", colorSensor0.getARGB()[1]);
            telemetry.addData("G0", colorSensor0.getARGB()[2]);
            telemetry.addData("B0", colorSensor0.getARGB()[3]);

            telemetry.addData("A1", colorSensor1.getARGB()[0]);
            telemetry.addData("R1", colorSensor1.getARGB()[1]);
            telemetry.addData("G1", colorSensor1.getARGB()[2]);
            telemetry.addData("B1", colorSensor1.getARGB()[3]);

            telemetry.addData("A2", colorSensor2.getARGB()[0]);
            telemetry.addData("R2", colorSensor2.getARGB()[1]);
            telemetry.addData("G2", colorSensor2.getARGB()[2]);
            telemetry.addData("B2", colorSensor2.getARGB()[3]);*/

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
