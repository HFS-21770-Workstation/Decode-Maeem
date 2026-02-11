package org.firstinspires.ftc.teamcode.RobotSystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import kotlin.NotImplementedError;

public class ArtifactIDSystem {
    
    public static final VisionProcessor[] processors = new VisionProcessor[]{
            /* Back Artifact */ getProcessor(ImageRegion.asImageCoordinates(/* arbitrary */ 10, 10, 10, 100))
            // TODO: Add Left and Right Artifacts
            
    };
    
    private final WebcamName camera;
    private final VisionPortal portal;

    public ArtifactIDSystem(WebcamName camera) {
        this.camera = camera;
        this.portal = VisionPortal.easyCreateWithDefaults(camera, ArtifactIDSystem.processors);
        
    }


    public void start() {
        throw new NotImplementedError();
    }
    
    public void stop() {
        throw new NotImplementedError();
    }
    
    
    private static PredominantColorProcessor getProcessor(ImageRegion region) {
        return new PredominantColorProcessor.Builder()
                .setRoi(region)
                .setSwatches(PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                             PredominantColorProcessor.Swatch.ARTIFACT_PURPLE)
                .build();
    }
}
