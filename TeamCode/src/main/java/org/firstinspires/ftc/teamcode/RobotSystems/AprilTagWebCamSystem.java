package org.firstinspires.ftc.teamcode.RobotSystems;

import static org.opencv.android.Utils.matToBitmap;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;


public class AprilTagWebCamSystem {
    // final CameraStreamProcessor processor = new CameraStreamProcessor();

    private AprilTagProcessor aprilTagProcessor;

    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 180, 0);

    private Telemetry telemetry;

    private FtcDashboard dashboard;

    private Telemetry dashboardTelemetry;

    public AprilTagWebCamSystem(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard){
        this.telemetry = telemetry;
        this.dashboard = dashboard;
        this.dashboardTelemetry = this.dashboard.getTelemetry();

        // dashboard.startCameraStream(processor, 30);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setDrawTagID(true)
                .setDrawTagID(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        // builder.addProcessor(processor);

        visionPortal = builder.build();
    }

    public void update(){
        detectedTags = aprilTagProcessor.getDetections();
        telemetry.update();
        dashboardTelemetry.update();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public AprilTagDetection getDetectionByID(int id){
        for (AprilTagDetection detection : detectedTags){
            if (detection.id == id){
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
    public double[] getRobotPoseByID(int id){
        double[] retValue = new double[3];

        AprilTagDetection detection = this.getDetectionByID(id);
        if(detection != null){

            retValue[0] = detection.robotPose.getPosition().x;
            retValue[1] = detection.robotPose.getPosition().y;
            retValue[2] = detection.robotPose.getOrientation().getYaw();
            return retValue;
        }
        else{
            return null;
        }

    }

    public double getDistanceFromTag(int id){
        AprilTagDetection detection = getDetectionByID(id);
        if(detection == null){
            return -1;
        }
        return Math.sqrt(Math.pow(detection.ftcPose.range, 2) -
                264.0625) + 8.16141732;
        // 870.25 is The height of the april tag squared in inch
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

