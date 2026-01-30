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
import org.firstinspires.ftc.teamcode.RobotSystems.IntakeOld;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.ShooterB;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;
import org.firstinspires.ftc.teamcode.Util.PoseStorage;

import java.util.logging.Handler;


@TeleOp(group = "TeleOps",name = "TeleOp Red Auto Shooter")
public class driveOpModeRedAutoShooter extends OpMode {
    Storage storage;
    ShooterB shooter;
    public static Turret turret;
    final double pos = 0.1;
    final double power = 0.1;
    VoltageSensor voltageSensor;
    FtcDashboard dashboard = FtcDashboard.getInstance();
//    Pose2d startPose = new Pose2d(62, 14.80314, Math.toRadians(180));
    Pose2d startPose = PoseStorage.pose;
    boolean startShot;
    double turretOffset = 0;
    double shooterOffset = 0.05;
    public boolean turretOffsetMore = false;

    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    int index = 0;


    MecanumDrive mecanumDrive;
//    Drive drive;

    int indexTuning = 0;

    IntakeOld intake;

    double currentShooterPower = 0;

    double intakePower = 0;





    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        storage = new Storage(hardwareMap);

        shooter = new ShooterB(hardwareMap);
        shooter.ChangeAngle(0,0);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        intake = IntakeOld.getInstance(hardwareMap);

        storage.initServos();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start(){
        turret.startFunction();
        shooter.startCal();
    }

    @Override
    public void loop() {
        mecanumDrive.updatePoseEstimate();
        Pose2d currentPose = mecanumDrive.localizer.getPose();

        turret.aprilTagWebCamSystem.update(currentPose);
        turret.update(currentPose);
        turret.updatePIDAlignment(24, turretOffset);

        double distance = turret.aprilTagWebCamSystem.getDistanceFromGoal(24);
        shooter.ChangeAngle(shooter.getServoPositionWithDistance(distance),
                shooter.getServoPositionWithDistance(distance));


        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.right_stick_x;
        double rx = -gamepad1.left_stick_x;

        if(gamepad1.right_trigger != 0){
            x /= 2;
            y /= 2;
            rx /= 2;
        }

        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(x, y),
                rx
        ));

        storage.checkTime();

        if (!storage.waitingForDown) {
            storage.updateColorSensors();
        }

//        telemetry.addData("slots1", storage.getArtifactsStorage()[0]);
//        telemetry.addData("slots2", storage.getArtifactsStorage()[1]);
//        telemetry.addData("slots3", storage.getArtifactsStorage()[2]);
//

        // 5. THE DRAWING LOGIC
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        // Draw the robot using the pose we just got from the localizer
        field.setStroke("#3F51B5");
        Drawing.drawRobot(field, currentPose);

        // 6. Send everything to Dashboard
        dashboard.sendTelemetryPacket(packet);

        // Optional: Keep your phone telemetry alive
//        telemetry.addData("X", currentPose.position.x);
//        telemetry.addData("Y", currentPose.position.y);
        telemetry.addData("Turret Offset", turretOffset);
        telemetry.addData("Shooter Offset", shooterOffset);
        telemetry.addData("Velocity", shooter.motorVelocity);
        telemetry.update();

//        if(gamepad1.a && shooter.GetPower() == 0){
//            shooter.StartShoot(0.67);
//        }
//        if(gamepad1.a && shooter.GetPower() == 0){
//            shooter.StopShoot();
//        }

        if (gamepad1.aWasPressed()) {
            startShot = !startShot;
        }

        if(gamepad1.dpadUpWasPressed()){
            indexTuning++;
        }
        if(gamepad1.dpadDownWasPressed()){
            indexTuning++;
        }

        if (startShot) {
            if(distance > 100){
                currentShooterPower = shooter.shootWithPID(Kp, Ki, Kd);
//                currentShooterPower = 1;
            }
            else{
                if(gamepad1.dpadLeftWasPressed()){
                    if(indexTuning == 0){
                        Kp += 0.0001;
                    }
                    if (indexTuning == 1) {
                        Ki += 0.0001;
                    }
                    if(indexTuning == 2){
                        Kd += 0.0001;
                    }
                }
                if(gamepad1.dpadRightWasPressed()){
                    if(indexTuning == 0){
                        Kp -= 0.0001;
                    }
                    if (indexTuning == 1) {
                        Ki -= 0.0001;
                    }
                    if(indexTuning == 2){
                        Kd -= 0.0001;
                    }
                }
            }
            if(currentShooterPower > 1){
                currentShooterPower = 1;
            }
            else if(currentShooterPower < -1){
                currentShooterPower = -1;
            }
            shooter.StartShoot(currentShooterPower);
        }
        else{
            shooter.StopShoot();
        }


        if (gamepad1.left_trigger == 1){
            shooter.StopShoot();
        }
        shooter.ChangeAngle(shooter.getServoPositionWithDistance(distance),
                shooter.getServoPositionWithDistance(distance));



        if(gamepad1.x && startShot){
            if(shooter.GetPower() == 1){
                shooter.StartShoot(1);
            }
            storage.setOutPutArtifacts(Storage.Artifacts.GREEN);
        }
        else if (gamepad1.b && startShot) {
            if(shooter.GetPower() == 1){
                shooter.StartShoot(1);
            }
            storage.setOutPutArtifacts(Storage.Artifacts.PURPLE);
        }
        else if(gamepad1.y && startShot){
            if(shooter.GetPower() == 1){
                shooter.StartShoot(1);
            }
            storage.setOutPutArtifactsRandom(); // this thing wasn't checked
        }

        if(!startShot){
            intakePower = 1;
        }
//        if(gamepad1.dpadDownWasPressed() && intakePower < 1){
//            intakePower += 1;
//        }
//        if(gamepad1.dpadUpWasPressed() && intakePower > -1){
//            intakePower -= 1;
//        }
        if(startShot){
            intakePower = 0;
        }
        intake.startIntake(intakePower);
        intake.updateIntake();
//        if(gamepad1.dpad_down && intake.getPower() == 0){
//            intake.startIntake(1);
//        }
//        if(gamepad1.dpad_down && intake.getPower() == 1){
//            intake.stopIntake();
//        }
//        if(gamepad1.dpad_up && intake.getPower() == 0){
//            intake.startIntake(-1);
//        }
//        if(gamepad1.dpad_up && intake.getPower() == -1) {
//            intake.stopIntake();
//        }

//        shooter.StartShoot(gamepad2.right_stick_y);
        if (gamepad2.dpadUpWasPressed()) {
            shooter.ChangeAngle(shooter.GetPosR() + pos, shooter.GetPosL() + pos);
        }
        if (gamepad2.dpadDownWasPressed()) {
            shooter.ChangeAngle(shooter.GetPosR() - pos, shooter.GetPosL() - pos);
        }
//        if(gamepad2.yWasPressed()){
//            shooter.StartShoot(shooter.GetPower() + power);
//        }
        if(gamepad2.a) {
            storage.pusherUp(0);
        }
        if(gamepad2.x) {
            storage.pusherUp(2);
        }
        if(gamepad2.b) {
            storage.pusherUp(1);
        }

        if (gamepad2.yWasPressed()){
            turretOffsetMore = !turretOffsetMore;
        }

        if (turretOffsetMore){
            if(gamepad2.dpadRightWasPressed()){
                turretOffset -= 15;
            }
            if(gamepad2.dpadLeftWasPressed()){
                turretOffset += 15;
            }
        }
        if (!turretOffsetMore){
            if(gamepad2.dpadRightWasPressed()){
                turretOffset -= 3;
            }
            if(gamepad2.dpadLeftWasPressed()){
                turretOffset += 3;
            }
        }



        if(gamepad2.right_trigger == 1){
            shooterOffset += 0.05;
        }
        else if(gamepad2.left_trigger == 1){
            shooterOffset -= 0.05;
        }

    }

    @Override
    public void stop(){
        shooter.StopShoot();
    }

}
