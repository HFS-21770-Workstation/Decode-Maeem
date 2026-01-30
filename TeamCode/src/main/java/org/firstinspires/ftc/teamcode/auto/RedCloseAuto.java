package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.OpModes.driveOpModeRed;
import org.firstinspires.ftc.teamcode.OpModes.driveOpModeRedAutoShooter;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;
import org.firstinspires.ftc.teamcode.Util.PoseStorage;


@Config
@Autonomous(name = "Red Close ", group = "Auto")
public class RedCloseAuto extends LinearOpMode {
    Turret turret;

    Shooter shooter;
    Storage storage;
    VoltageSensor voltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-55.684, 38.344, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);
        driveOpModeRed.turret = turret;
        driveOpModeRedAutoShooter.turret = turret;
        shooter = new Shooter(hardwareMap);
        storage = new Storage(hardwareMap);
        double currentShooterPower;
        shooter.initPos();
        shooter.startCal();
        storage.initServos();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret.startFunction();
        turret.update(startPose);

        Thread alighnTurret = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    Pose2d pose = drive.localizer.getPose();
                    turret.updatePIDAlignment(24, 0);
                    turret.update(pose);
                }
            }
        });

        Action move = drive.actionBuilder(startPose)
                        .strafeTo(new Vector2d(startPose.position.x, startPose.position.y - 24))
                        .build();

        waitForStart();
        alighnTurret.start();

        Actions.runBlocking(
                move
        );
        PoseStorage.pose = drive.localizer.getPose();
        PoseStorage.currentPose = turret.getAngle();
    }
}
