package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "")
@Disabled
public class ExposureTuning extends OpMode {

    private AprilTagProcessor aprilTagProcessor;

    private VisionPortal visionPortal;
    Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

    @Override
    public void init() {
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
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
//        telemetry.addData("Exposure");
    }
}
