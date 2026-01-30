package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.OpModes.driveOpModeRed;
import org.firstinspires.ftc.teamcode.OpModes.driveOpModeRedAutoShooter;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.IntakeOld;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;
import org.firstinspires.ftc.teamcode.Util.PoseStorage;


@Config
@Autonomous(name = "Red Far ", group = "Auto")
public class RedFarAuto extends LinearOpMode {
    IntakeOld intake;
    Turret turret;
    Shooter shooter;
    Storage storage;
    Storage.Artifacts[] sort = null;
    VoltageSensor voltageSensor;
    public static double SHOOT_OFFSET = 0.155;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = IntakeOld.getInstance(hardwareMap);
        Pose2d shootPose = new Pose2d(45.5, 10, Math.toRadians(180));

        Pose2d startPose = new Pose2d(61, 15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        storage = new Storage(hardwareMap);
        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);
        driveOpModeRed.turret = turret;
        driveOpModeRedAutoShooter.turret = turret;
        shooter = new Shooter(hardwareMap);
        double currentShooterPower;
        shooter.initPos();
        shooter.startCal();
        storage.initServos();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        turret.startFunction();


        while (!isStarted() && !isStopRequested()) {
            turret.update(startPose);
        }


        Action first_score = drive.actionBuilder(startPose)

                .splineToLinearHeading(
                        (shootPose),
                        Math.toRadians(135),
                        new TranslationalVelConstraint(30)
                )
                .waitSeconds(0.5)
                .build();

        Action first_intake = drive.actionBuilder(new Pose2d(61, 14, Math.toRadians(135)))
                .splineToLinearHeading(
                        new Pose2d(37, 45, Math.toRadians(90)),
                        Math.toRadians(90),
                        new TranslationalVelConstraint(50)
                )
                .lineToY(46, (pose, path, v) -> 20)
                .build();

        Action second_score = drive.actionBuilder(new Pose2d(37, 46, Math.toRadians(90)))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(
                        shootPose,
                        Math.toRadians(135),
                        new TranslationalVelConstraint(50)
                )
                .waitSeconds(3)
                .build();
        Action move_from_park = drive.actionBuilder(new Pose2d(61, 14, Math.toRadians(135)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(
                        new Pose2d(40, 16, Math.toRadians(90)),
                        Math.toRadians(135),
                        new TranslationalVelConstraint(50)
                )
                .build();
        Action second_intake = drive.actionBuilder(new Pose2d(61, 14, Math.toRadians(135)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(
                        new Pose2d(12, 45, Math.toRadians(90)),
                        Math.toRadians(90),
                        new TranslationalVelConstraint(50)
                )
                .lineToY(48, (pose, path, v) -> 20)

                .build();

        Action third_score = drive.actionBuilder(new Pose2d(12, 30, Math.toRadians(90)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(
                        shootPose,
                        Math.toRadians(180),
                        new TranslationalVelConstraint(30)
                )
                .waitSeconds(3)
                .build();

        Action shootGPP = new SequentialAction(
                new SleepAction(2),
                storage.outputArtifactsByIndex(0),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction(),
                new SleepAction(0.5),

                storage.outputArtifactsByIndex(2),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction(),
                new SleepAction(0.5),

                storage.outputArtifactsByIndex(1),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction()
        );

        Action shootPGP = new SequentialAction(
                new SleepAction(2),
                storage.outputArtifactsByIndex(2),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction(),
                new SleepAction(0.5),

                storage.outputArtifactsByIndex(0),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction(),
                new SleepAction(0.5),

                storage.outputArtifactsByIndex(1),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction()

        );

        Action shootPPG = new SequentialAction(
                new SleepAction(2),
                storage.outputArtifactsByIndex(2),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction(),
                new SleepAction(0.5),
                storage.outputArtifactsByIndex(1),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction(),
                new SleepAction(0.5),
                storage.outputArtifactsByIndex(0),
                new SleepAction(2),
                new SleepAction(0.5),
                storage.initServoAction()
        );

        Action shoot_sorted;
        try{
            if (sort[0] == Storage.Artifacts.GREEN) {
                shoot_sorted = shootGPP;
            } else if (sort[1] == Storage.Artifacts.GREEN) {
                shoot_sorted = shootPGP;
            } else {
                shoot_sorted = shootPPG;
            }
        }
        catch (Exception e){
            shoot_sorted = shootPPG;
        }



//        Action final_intake = drive.actionBuilder(new Pose2d(61, 14, Math.toRadians(135)))
//                .setTangent(Math.toRadians(135))
//                .splineToLinearHeading(
//                        new Pose2d(12, 30, Math.toRadians(90)),
//                        Math.toRadians(90),
//                        new TranslationalVelConstraint(25)
//                )
//                .build();

        Action shootAction =
                new ParallelAction(
                        shooter.shootWithAutoPowerAction(turret.aprilTagWebCamSystem.getDistanceFromGoal(24),
                                voltageSensor.getVoltage(), SHOOT_OFFSET),
//                        turret.aimTurretAction(drive.localizer.getPose()),

                        shoot_sorted,
                        storage.updateColorSensorsAction()

                );

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


        waitForStart();

        for(int i = 0; i < 10 && sort == null; i ++){
            sort = turret.aprilTagWebCamSystem.getObelisk();
        }
        telemetry.addData("Sort 1", sort[0]);
        telemetry.addData("Sort 2", sort[1]);
        telemetry.addData("Sort 3", sort[2]);

        alighnTurret.start();

        Actions.runBlocking(
                new ParallelAction(
////                        turret.aimTurretAction(drive.localizer.getPose()),
//                         new SleepAction(3),
                        new SequentialAction(
                                first_score,
                                shootAction
                        )
                )
        );


//        storage.updateColorSensors();

        Actions.runBlocking(
                new SequentialAction(
                        shootAction,
                        new SleepAction(2)
                )
        );

        Actions.runBlocking(move_from_park);
//        Actions.runBlocking(
//            new SequentialAction(
//                    first_score,
//                    new ParallelAction(
//                        shooter.shootWithAutoPowerAction(turret.aprilTagWebCamSystem.getDistanceFromGoal(24),
//                            voltageSensor.getVoltage() + SHOOT_OFFSET),
//                        turret.aimTurretAction(drive.localizer.getPose()),
//                        shoot_sorted
//                            ),
//                    shooter.stopShootAction(),
//                    intake.startIntakeAction(),
//                    first_intake,
//                    intake.stopIntakeAction(),
//                    new ParallelAction(
//                            shooter.shootWithAutoPowerAction(turret.aprilTagWebCamSystem.getDistanceFromGoal(24),
//                                    voltageSensor.getVoltage() + SHOOT_OFFSET),
//                            turret.aimTurretAction(drive.localizer.getPose())
//                    ),
//                    second_score,
//                    intake.startIntakeAction(),
//                    second_intake,
//                    intake.stopIntakeAction(),
//                    third_score
//            )
//
//        );
        PoseStorage.pose = drive.localizer.getPose();
        PoseStorage.currentPose = turret.getAngle();
    }
}
