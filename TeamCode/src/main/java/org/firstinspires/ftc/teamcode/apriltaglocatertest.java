package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag Seek Drive", group = "Vision")
public class apriltaglocatertest extends OpMode {

    // Drive motors
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;

    // Vision objects
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // The target AprilTag ID
    private static final int TARGET_TAG_ID = 1;

    @Override
    public void init() {

        // --- Initialize Motors ---
        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Initialize AprilTag Processor ---
        aprilTag = new AprilTagProcessor.Builder().build();

        // --- Initialize Vision Portal (FIXED) ---
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized - ready to find AprilTag");
    }

    @Override
    public void loop() {

        // Look for the target tag
        AprilTagDetection target = getTargetTag();

        if (target == null) {
            // No tag detected → rotate in place
            leftDrive.setPower(0.25);
            rightDrive.setPower(-0.25);

            telemetry.addLine("Searching for AprilTag...");
            return;
        }

        // Tag detected → track it
        double yaw = target.ftcPose.yaw;       // Angle to tag
        double range = target.ftcPose.range;   // Distance in inches

        // --- Simple proportional control ---
        double turn = clip(yaw * 0.02, -0.3, 0.3);
        double drive = clip((range - 8.0) * 0.05, -0.4, 0.4); // Stop ~8 inches away

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Telemetry for debugging
        telemetry.addData("Tag ID", target.id);
        telemetry.addData("Yaw (deg)", yaw);
        telemetry.addData("Range (in)", range);
    }

    private AprilTagDetection getTargetTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == TARGET_TAG_ID) {
                return detection;
            }
        }
        return null;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
