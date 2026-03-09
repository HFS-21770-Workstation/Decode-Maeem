package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.IntakeOld;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;
import org.firstinspires.ftc.teamcode.Util.Enums.Artifacts;
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;
import org.firstinspires.ftc.teamcode.Util.PoseStorage;

@Config
@Autonomous(name = "Red Far 6 Artifacts", group = "Auto")
public class RedFarAuto6Artifacts extends LinearOpMode {
    IntakeOld intake;
    Turret turret;
    Shooter shooter;
    Storage storage;
    double dis;
    volatile boolean startShoot = false;
    Artifacts[] currentSort;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new IntakeOld(hardwareMap);
        Pose2d startPose = new Pose2d(61, 15, Math.toRadians(180));
        Pose2d shootPose = new Pose2d(55.5, 10, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        storage = new Storage(hardwareMap);
        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);
        shooter = new Shooter(hardwareMap);

        shooter.initPos();
        shooter.startCal();
        storage.initServos();
        turret.startFunction();

        Action first_score = drive.actionBuilder(startPose)
                .splineToLinearHeading(shootPose, Math.toRadians(180), new TranslationalVelConstraint(30))
                .build();

        Action first_intake = drive.actionBuilder(shootPose)
                .splineToLinearHeading(new Pose2d(34.5, 40, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(50))
                .lineToY(46, (pose, path, v) -> 20)
                .build();

        Action second_score = drive.actionBuilder(new Pose2d(35.4, 46, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(shootPose, Math.toRadians(135), new TranslationalVelConstraint(50))
                .build();

        Action detect_logic = (telemetryPacket) -> {
            currentSort = turret.aprilTagWebCamSystem.getObelisk();
            if (currentSort == null) {
                currentSort = new Artifacts[]{Artifacts.PURPLE, Artifacts.PURPLE, Artifacts.GREEN};
            }
            return false;
        };

        Thread alignThread = new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                Pose2d pose = drive.localizer.getPose();
                turret.update(pose);
                turret.aprilTagWebCamSystem.update(pose);
                turret.updatePIDAlignment(GoalColor.RED, 0);
                dis = turret.aprilTagWebCamSystem.getDistanceFromGoal(GoalColor.RED);
                shooter.changeAngle(shooter.getServoPositionWithDistance(dis), shooter.getServoPositionWithDistance(dis));

                if (!startShoot) shooter.setVelocity(0);
                else shooter.setVelocity(shooter.shootWithAutoPower(dis, 100));

                telemetry.addData("Distance", dis);
                telemetry.update();
            }
        });

        while (!isStarted() && !isStopRequested()) {
            turret.update(startPose);
            storage.start();
            telemetry.update();
        }

        waitForStart();

        Actions.runBlocking(detect_logic);
        alignThread.start();

        Actions.runBlocking(first_score);

        startShoot = true;
        Actions.runBlocking(
                new SequentialAction(
                        storage.outPutBySort(currentSort),
                        new SleepAction(0.3)
                )
        );

        startShoot = false;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake.startIntakeAction(),
                                first_intake
                        ),
                        new SleepAction(0.5),
                        new ParallelAction(
                                intake.stopIntakeAction(),
                                second_score
                        )
                )
        );

        startShoot = true;
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(0.5),
                        storage.outPutBySort(currentSort)
                )
        );

        startShoot = false;
        alignThread.interrupt();

        Action park = drive.actionBuilder(shootPose)
                .strafeTo(new Vector2d(shootPose.position.x - 1, shootPose.position.y + 30))
                .build();
        Actions.runBlocking(park);

        PoseStorage.pose = drive.localizer.getPose();
        PoseStorage.currentPose = turret.getAngle();
    }
}