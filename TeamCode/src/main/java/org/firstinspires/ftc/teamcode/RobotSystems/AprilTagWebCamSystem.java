package org.firstinspires.ftc.teamcode.RobotSystems;

import static android.os.SystemClock.sleep;
import static org.opencv.android.Utils.matToBitmap;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Util.Enums.Artifacts;
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

@Config
public class AprilTagWebCamSystem {
//     final CameraStreamProcessor processor = new CameraStreamProcessor();
    public static int manualExposure = 30;  // milliseconds
    public static int manualGain = 10;
    public static int whiteBalanceTemp = 5000;
    private AprilTagProcessor aprilTagProcessor;

    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();


    Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    private Telemetry telemetry;

    private FtcDashboard dashboard;

    private Telemetry dashboardTelemetry;

    public Pose2d pose;

    public AprilTagWebCamSystem(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard, Pose2d startPose){
        this.telemetry = telemetry;
        this.dashboard = dashboard;
        this.dashboardTelemetry = this.dashboard.getTelemetry();
        this.pose = startPose;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)

                .setDrawTagID(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(481.985, 481.985, 334.203, 241.948)
                .build();
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 2");

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(camera);
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
//        builder.addProcessor(processor);
        builder.enableLiveView(true);


        visionPortal = builder.build();
        setManualCamera();







    }

    private void setManualCamera() {
        if (visionPortal == null) return;

        // המתנה עד שהמצלמה מתחילה לזרום
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();
            sleep(20);
        }

        telemetry.addData("Camera", "Ready in manual mode");
        telemetry.update();

        // --- Exposure ---
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null && exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        if (exposureControl != null) {
            exposureControl.setExposure(manualExposure, TimeUnit.MILLISECONDS);  // ערך קבוע לדוגמה
            sleep(20);
        }

        // --- Gain ---
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(manualGain);  // ערך קבוע לדוגמה
            sleep(20);
        }

        // --- White Balance ---
        WhiteBalanceControl wbControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
        if (wbControl != null && wbControl.getMode() != WhiteBalanceControl.Mode.MANUAL) {
            wbControl.setMode(WhiteBalanceControl.Mode.MANUAL);
            sleep(50);
        }
        if (wbControl != null) {
            wbControl.setWhiteBalanceTemperature(whiteBalanceTemp);  // ערך קבוע לדוגמה
            sleep(20);
        }
    }

    public void update(Pose2d currentPose){
        this.pose = currentPose;
        detectedTags = aprilTagProcessor.getDetections();
        telemetry.update();
        dashboardTelemetry.update();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public AprilTagDetection getDetectionByID(GoalColor goalColor){
        for (AprilTagDetection detection : detectedTags){
            if (detection.id == goalColor.getValue()){
                return detection;
            }
        }
        return null;
    }

    public void stop(){
        if (visionPortal != null){
            visionPortal.close();
        }
    }

    public void displayDetectionTelemtry(AprilTagDetection detectedId){
        if(detectedId == null) {return;}
        if(detectedId.metadata != null){
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));


            dashboardTelemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            dashboardTelemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            dashboardTelemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            dashboardTelemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));

            dashboardTelemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            dashboardTelemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    ///
    /// RET VALUE: x, y, rx
    ///
    public Pose2d getRobotPoseByID(GoalColor goalColor) {
        double[] retValue = new double[3];

        AprilTagDetection detection = this.getDetectionByID(goalColor);
        if (detection != null) {
            double x = Math.round(detection.robotPose.getPosition().x * 100) / 100.0;
            double y = Math.round(detection.robotPose.getPosition().y * 100) / 100.0;
            double heading = Math.round(detection.robotPose.getOrientation().getYaw() * 100) / 100.0;
            Pose2d pose = new Pose2d(x, y, heading);

            return pose;
        } else {
            return null;
        }
    }

    public double getDistanceFromTag(GoalColor goalColor){
        AprilTagDetection detection = getDetectionByID(goalColor);
        if(detection == null){
            return -1;
        }
        return Math.sqrt(Math.pow(detection.ftcPose.range, 2) -
                264.0625) + 8.16141732;
        // 870.25 is The height of the april tag squared in inch
    }

    public double getDistanceFromGoal(GoalColor goalColor){
        double goalX;
        double goalY;
        if(goalColor.getValue() == 24){
            goalX = -58.3727;
            goalY = 55.6425;
        }
        else{
            goalX = -58.3727;
            goalY = -55.6425;
        }
        double robotX = pose.position.x;
        double robotY = pose.position.y;

        double deltaX = robotX - goalX;
        double deltaY = robotY - goalY;

        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

    }

    public Artifacts[] getObelisk(){
        if(detectedTags == null){
            return null;
        }
        for (AprilTagDetection detection : detectedTags){
            if (detection.id == 21){
                return new Artifacts[] {Artifacts.GREEN, Artifacts.PURPLE, Artifacts.PURPLE};
            }
            if(detection.id == 22){
                return new Artifacts[] {Artifacts.PURPLE, Artifacts.GREEN, Artifacts.PURPLE};
            }
            if(detection.id == 23){
                return new Artifacts[] {Artifacts.PURPLE, Artifacts.PURPLE, Artifacts.GREEN};
            }

        }
        return null;

    }

//    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
//        private final AtomicReference<Bitmap> lastFrame =
//                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
//
//        @Override
//        public void init(int width, int height, CameraCalibration calibration) {
//            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
//        }
//
//        @Override
//        public Object processFrame(Mat frame, long captureTimeNanos) {
//            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
//            matToBitmap(frame, b);
//            lastFrame.set(b);
//            return null;
//        }
//
//        @Override
//        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
//                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
//                                Object userContext) {
//            // do nothing
//        }
//
//        @Override
//        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
//            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
//        }
//    }
}

