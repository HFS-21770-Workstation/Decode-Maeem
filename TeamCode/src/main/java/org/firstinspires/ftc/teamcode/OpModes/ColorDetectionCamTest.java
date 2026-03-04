package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.RobotSystems.Storage.slot2RGB;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotSystems.ArtifactIDSystem;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.Util.Enums;

@Config
@TeleOp(name = "ColorDetectionCamTest")
public class ColorDetectionCamTest extends OpMode {
    Storage storage;
//    private ArtifactIDSystem artifactIDSystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Enums.Artifacts slots[];

    @Override
    public void init() {
        storage = new Storage(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        storage.initServos();




    }

    @Override
    public void start() {
        storage.start();
    }

    @Override
    public void loop() {
        storage.checkTime();

        if (!storage.waitingForDown) {
            storage.updateColorSensors();
        }

        if (gamepad1.aWasPressed()){
            storage.setOutPutArtifacts(Enums.Artifacts.PURPLE);

        }
        if (gamepad1.bWasPressed()){
            storage.setOutPutArtifacts(Enums.Artifacts.GREEN);

        }
        slots = storage.getArtifactsStorage();
        telemetry.addData("Slot1 RGB", slots[0]);
        telemetry.addData("Slot2 RGB", slots[1]);
        telemetry.addData("Slot3 RGB", slots[2]);
        telemetry.addData("s2","Slot2 RGB: R:" + slot2RGB[0] + " G:" + slot2RGB[1] + " B:" + slot2RGB[2]);
        telemetry.update();
    }


}