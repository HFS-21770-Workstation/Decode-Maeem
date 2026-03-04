package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystems.ArtifactIDSystem;

@TeleOp(name = "ColorDetectionCamTest")
public class ColorDetectionCamTest extends OpMode {

    private ArtifactIDSystem artifactIDSystem;

    @Override
    public void init() {
        try {
            artifactIDSystem = new ArtifactIDSystem(hardwareMap);
            telemetry.addLine("Camera Initialized");
        } catch (Exception e) {
            telemetry.addData("Init Error", e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void start() {
        artifactIDSystem.start();
    }

    @Override
    public void loop() {
        int[] rgb1 = artifactIDSystem.getSlot1RGB();
        int[] rgb2 = artifactIDSystem.getSlot2RGB();
        int[] rgb3 = artifactIDSystem.getSlot3RGB();

        telemetry.addData("Slot1 RGB", "R:%d G:%d B:%d", rgb1[0], rgb1[1], rgb1[2]);
        telemetry.addData("Slot2 RGB", "R:%d G:%d B:%d", rgb2[0], rgb2[1], rgb2[2]);
        telemetry.addData("Slot3 RGB", "R:%d G:%d B:%d", rgb3[0], rgb3[1], rgb3[2]);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (artifactIDSystem != null) {
            artifactIDSystem.stop();
        }
    }
}