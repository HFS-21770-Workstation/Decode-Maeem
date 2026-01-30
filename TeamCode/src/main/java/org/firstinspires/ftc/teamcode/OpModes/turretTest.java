package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.AprilTagWebCamSystem;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;

@TeleOp
@Config
public class turretTest extends OpMode {

    MecanumDrive mecanumDrive;
    Turret turret;
    Shooter shooter;
    boolean startShot;

    // Dashboard variables
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Pose2d startPose = new Pose2d(61, 15, Math.toRadians(180));


    VoltageSensor voltageSensor;
    Storage storage;
//    AprilTagWebCamSystem aprilTagWebCamSystem;

    final double pos = 0.1;
    final double power = 0.04;

    double turretOffset = 0;

    @Override
    public void start(){

        turret.startFunction();
        shooter.startCal();
    }

    @Override
    public void init() {
        // Initialize MecanumDrive (This replaces your old 'Drive' class)
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);

        // Initialize Turret
        turret = new Turret(hardwareMap, telemetry, dashboard, startPose);

        shooter = new Shooter(hardwareMap);
        shooter.initPos();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        storage = new Storage(hardwareMap);
        storage.initServos();

//        aprilTagWebCamSystem = new AprilTagWebCamSystem(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);


        // This allows telemetry.addData to show up on both the Phone and the Dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {

        mecanumDrive.updatePoseEstimate();
        double distance = turret.aprilTagWebCamSystem.getDistanceFromGoal(24);


        Pose2d currentPose = mecanumDrive.localizer.getPose();

        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(x, y),
                rx
        ));

        // 4. Update Turret with the current pose
        turret.update(currentPose);
        turret.aprilTagWebCamSystem.update(currentPose);

        turret.updatePIDAlignment(24, turretOffset);

        if(gamepad2.dpadRightWasPressed()){
            turretOffset += 1;
        }
        if(gamepad2.dpadLeftWasPressed()){
            turretOffset -= 1;
        }

        if (gamepad1.yWasPressed()) {
            startShot = !startShot;
            if (!startShot) {
                shooter.StartShoot(0);
            }
        }


        if (startShot) {
            double currentPower = shooter.shootWithAutoPower(distance, voltageSensor.getVoltage(), 0);
            shooter.StartShoot(currentPower);
        }

//        if(gamepad1.yWasPressed()){
//            shooter.StartShoot(shooter.GetPower() + power);
//        }
//        if(gamepad1.aWasPressed()) {
//            shooter.StartShoot(shooter.GetPower() - power);
//        }
        if (gamepad1.right_trigger == 1){
            shooter.StopShoot();
        }
        shooter.ChangeAngle(shooter.getServoPositionWithDistance(distance),
                shooter.getServoPositionWithDistance(distance));

//        if (gamepad1.dpadUpWasPressed()) {
//            shooter.ChangeAngle(shooter.GetPosR() + pos, shooter.GetPosL() + pos);
//        }
//        if (gamepad1.dpadDownWasPressed()) {
//            shooter.ChangeAngle(shooter.GetPosR() - pos, shooter.GetPosL() - pos);
//        }

        storage.checkTime();

        if (!storage.waitingForDown) {
            storage.updateColorSensors();
        }
        if(gamepad1.xWasPressed()){
            storage.setOutPutArtifacts(Storage.Artifacts.PURPLE);


        }
        if(gamepad1.bWasPressed()){
            storage.setOutPutArtifacts(Storage.Artifacts.GREEN);

        }
//        telemetry.addData("slots1", storage.getArtifactsStorage()[0]);
//        telemetry.addData("slots2", storage.getArtifactsStorage()[1]);
//        telemetry.addData("slots3", storage.getArtifactsStorage()[2]);


        // 5. THE DRAWING LOGIC
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        // Draw the robot using the pose we just got from the localizer
        field.setStroke("#3F51B5");
        Drawing.drawRobot(field, currentPose);

        double turretAngle = Math.toRadians(turret.getCurrentAngleFromEncoder());
        double lineLength = 10.0;
        field.setStroke("#FF0000"); // Red for turret
        field.strokeLine(
                currentPose.position.x,
                currentPose.position.y,
                currentPose.position.x + Math.cos(turretAngle) * lineLength,
                currentPose.position.y + Math.sin(turretAngle) * lineLength
        );

        // 6. Send everything to Dashboard
        dashboard.sendTelemetryPacket(packet);

        // Optional: Keep your phone telemetry alive
//        telemetry.addData("X", currentPose.position.x);
//        telemetry.addData("Y", currentPose.position.y);
        telemetry.addData("target angle:", turret.getTargetAngleFromEncoder(24));
        telemetry.addData("error:", turret.pidController.getError());
//        telemetry.addData("output", turret.pidController.getOutput());
        telemetry.addData("Power:", shooter.GetPower());
        telemetry.addData("Volt:", voltageSensor.getVoltage());
        telemetry.addData("distance:", distance);
        telemetry.update();
    }

    @Override
    public void stop(){
        shooter.StopShoot();
    }
}
