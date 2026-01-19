package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.AprilTagWebCamSystem;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.IntakeOld;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;


@TeleOp(name = "Full Robot Test")
public class driveOpMode extends OpMode {
    Storage storage;
    Shooter shooter;
    Turret turret;
    final double pos = 0.1;
    final double power = 0.1;
    VoltageSensor voltageSensor;
    AprilTagWebCamSystem aprilTagWebCamSystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));


    MecanumDrive mecanumDrive;
//    Drive drive;

    int index = 0;

    IntakeOld intake;


    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        storage = new Storage(hardwareMap);

        shooter = new Shooter(hardwareMap);
        shooter.ChangeAngle(0,0);
        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        intake = IntakeOld.getInstance(hardwareMap);

        storage.initServos();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        mecanumDrive.updatePoseEstimate();
        Pose2d currentPose = mecanumDrive.localizer.getPose();

        aprilTagWebCamSystem.update(mecanumDrive.localizer.getPose());
        turret.update(currentPose);
        aprilTagWebCamSystem.update(currentPose);

        turret.updatePIDAlignment(24);

        double distance = aprilTagWebCamSystem.getDistanceFromGoal(24);
        shooter.ChangeAngle(shooter.getServoPositionWithDistance(distance),
                shooter.getServoPositionWithDistance(distance));


        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(x, y),
                rx
        ));

        storage.checkTime();

        if (!storage.waitingForDown) {
            storage.updateColorSensors();
        }
        if(gamepad2.xWasPressed()){
            storage.setOutPutArtifacts(Storage.Artifacts.PURPLE);


        }
        if(gamepad2.bWasPressed()){
            storage.setOutPutArtifacts(Storage.Artifacts.GREEN);

        }
        telemetry.addData("slots1", storage.getArtifactsStorage()[0]);
        telemetry.addData("slots2", storage.getArtifactsStorage()[1]);
        telemetry.addData("slots3", storage.getArtifactsStorage()[2]);


        // 5. THE DRAWING LOGIC
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        // Draw the robot using the pose we just got from the localizer
        field.setStroke("#3F51B5");
        Drawing.drawRobot(field, currentPose);

        // 6. Send everything to Dashboard
        dashboard.sendTelemetryPacket(packet);

        // Optional: Keep your phone telemetry alive
        telemetry.addData("X", currentPose.position.x);
        telemetry.addData("Y", currentPose.position.y);
        telemetry.update();

//        if(gamepad1.dpadUpWasPressed() && !gamepad1.a){
//            if(index < 2){
//                index++;
//            }
//        }
//        if(gamepad1.dpadDownWasPressed() && !gamepad1.a){
//            if(index > 0){
//                index--;
//            }
//        }
//        if(gamepad1.a){
//            if(index == 2){
//                servoPushers[index].setPosition(0);
//            }
//            else{
//                servoPushers[index].setPosition(1);
//            }
//        }
//        else{
//            if(index == 2){
//                servoPushers[index].setPosition(1);
//            }
//            else{
//                servoPushers[index].setPosition(0);
//            }
//        }
        if(gamepad1.a){
            shooter.StartShoot(1);
        }

        if(gamepad1.x){
            if(shooter.GetPower() == 1){
                shooter.StartShoot(1);
            }
            storage.setOutPutArtifacts(Storage.Artifacts.GREEN);
        }
        else if (gamepad1.b) {
            if(shooter.GetPower() == 1){
                shooter.StartShoot(1);
            }
            storage.setOutPutArtifacts(Storage.Artifacts.PURPLE);
        }
        else if(gamepad1.y){
            if(shooter.GetPower() == 1){
                shooter.StartShoot(1);
            }
            storage.setOutPutArtifactsRandom(); // this thing wasn't checked
        }
        else if(gamepad1.dpad_down){
            intake.startIntake(1);
        }
        else if(gamepad1.dpad_up){
            intake.startIntake(-1);
        }
        intake.updateIntake();

        turret.rotateWithJoystick(gamepad2.left_stick_x / 3);

//        shooter.StartShoot(gamepad2.right_stick_y);
        if (gamepad2.dpadUpWasPressed()) {
            shooter.ChangeAngle(shooter.GetPosR() + pos, shooter.GetPosL() + pos);
        }
        if (gamepad2.dpadDownWasPressed()) {
            shooter.ChangeAngle(shooter.GetPosR() - pos, shooter.GetPosL() - pos);
        }
        if(gamepad2.yWasPressed()){
            shooter.StartShoot(shooter.GetPower() + power);
        }
        if(gamepad2.a) {
            storage.pusherUp(0);
        }
        if(gamepad2.x) {
            storage.pusherUp(1);
        }
        if(gamepad2.b) {
            storage.pusherUp(2);
        }


    }

    @Override
    public void stop(){
        shooter.StopShoot();
    }

}
