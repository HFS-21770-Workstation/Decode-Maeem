package org.firstinspires.ftc.teamcode.RobotSystems;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
@Config
public class ArtifactIDSystem {

    private VisionPortal portal;

    private final PredominantColorProcessor slot1Processor;
    private final PredominantColorProcessor slot2Processor;
    private final PredominantColorProcessor slot3Processor;

    public ArtifactIDSystem(HardwareMap hardwareMap) {

        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        slot1Processor = createProcessor(100, 350, 325, 400);
        slot2Processor = createProcessor(0, 50, 100, 200);
        slot3Processor = createProcessor(300, 50, 500, 150);

        portal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(slot1Processor)
                .addProcessor(slot2Processor)
                .addProcessor(slot3Processor)
                .enableLiveView(false)
//                .setAutoStopLiveView(false)
//                .setLiveViewContainerId(2)
                .setCameraResolution(new Size(640, 480))

                .build();
    }

    private PredominantColorProcessor createProcessor(int left, int top, int right, int bottom) {
        return new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asImageCoordinates(left, top, right, bottom))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE
                )
                .build();
    }

    public void start() {
        if (portal != null) {
            portal.resumeStreaming();
        }
    }

    public void stop() {
        if (portal != null) {
            portal.stopStreaming();
            portal.close();
        }
    }

    public int[] getSlot1RGB() {
        return slot1Processor.getAnalysis().RGB;
    }

    public int[] getSlot2RGB() {
        return slot2Processor.getAnalysis().RGB;
    }

    public int[] getSlot3RGB() {
        return slot3Processor.getAnalysis().RGB;
    }
}