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
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;
import org.firstinspires.ftc.teamcode.Util.PoseStorage;


@Config
@Autonomous(name = "Red Close ", group = "Auto")
public class RedCloseAuto extends LinearOpMode {
    Turret turret;

    Shooter shooter;
    Storage storage;
    VoltageSensor voltageSensor;

    double dis;
    boolean startShoot = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(-69.4145, 38.6391, Math.toRadians(0));
        Vector2d moveVector = new Vector2d(-60,28);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);
//        driveOpModeRed.turret = turret;
//        driveOpModeRedAutoShooter.turret = turret;
//        shooter = Shooter.getInstance(hardwareMap);
        storage = new Storage(hardwareMap);
        double currentShooterPower;
//        shooter.initPos();
//        shooter.startCal();
        storage.initServos();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret.startFunction();
        turret.update(startPose);

        Thread alighnTurret = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    drive.updatePoseEstimate();
                    Pose2d pose = drive.localizer.getPose();
                    turret.updatePIDAlignment(GoalColor.RED, 0);
                    turret.update(pose);
                    dis = turret.aprilTagWebCamSystem.getDistanceFromGoal(GoalColor.RED);
                    telemetry.addData("Dis", dis);
                    telemetry.addData("Shooter Vel", shooter.getVelocity());
                    telemetry.addData("Shooter Target Vel", shooter.getVelocity());
                    shooter.changeAngle(shooter.getServoPositionWithDistance(dis),
                            shooter.getServoPositionWithDistance(dis));

                    if(!startShoot){
                        shooter.stop();
                    }
                    else{
                        shooter.setVelocity(shooter.shootWithAutoPower(dis, 100));
                    }
                }
            }
        });

        Action move = drive.actionBuilder(startPose)
                        .strafeTo(moveVector)
                        .build();

        waitForStart();
//        alighnTurret.start();

        Actions.runBlocking(
                move
        );
        PoseStorage.pose = drive.localizer.getPose();
        PoseStorage.currentPose = turret.getAngle();
    }
}
