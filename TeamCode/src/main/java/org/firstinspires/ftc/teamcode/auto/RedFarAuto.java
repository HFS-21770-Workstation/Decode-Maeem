package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;
import org.firstinspires.ftc.teamcode.Util.Enums.Artifacts;



@Config
@Autonomous(name = "Red Far ", group = "Auto")
public class RedFarAuto extends LinearOpMode {
    IntakeOld intake;
    Turret turret;
    Shooter shooter;
    Storage storage;
    Artifacts[] sort = null;
    VoltageSensor voltageSensor;
    public static double SHOOT_OFFSET = 0;
    double dis;
    boolean startShoot = false;
    Artifacts[] currentSort;
    Action finalShoot;
    MecanumDrive drive1;

    public static double kp = 0.0001;
    public static double ki = 0;
    public static double kd = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new IntakeOld(hardwareMap);
        Pose2d shootPose = new Pose2d(55.5, 10, Math.toRadians(180));

        Pose2d startPose = new Pose2d(61, 15, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        storage = new Storage(hardwareMap);
        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);
//        driveOpModeRed.turret = turret;
//        driveOpModeRedAutoShooter.turret = turret;
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
                        Math.toRadians(180),
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
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(0),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction(),
//                new SleepAction(0.5),
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(1),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction(),
//                new SleepAction(0.5),
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(2),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction()
        );

        Action shootPGP = new SequentialAction(
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(2),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction(),
//                new SleepAction(0.5),
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(0),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction(),
//                new SleepAction(0.5),
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(1),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction()

        );

        Action shootPPG = new SequentialAction(
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(2),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction(),
//                new SleepAction(0.5),
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(1),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction(),
//                new SleepAction(0.5),
                new SleepAction(1.5),
                storage.outputArtifactsByIndex(0),
                new SleepAction(1.5),
//                new SleepAction(0.5),
                storage.initServoAction()
        );

        Action shootRandom = new SequentialAction(
                new SleepAction(1),
                storage.outputArtifactsByIndex(0),
                new SleepAction(1),
                storage.initServoAction(),
                new SleepAction(1),
                storage.outputArtifactsByIndex(1),
                new SleepAction(1),
                storage.initServoAction(),
                storage.outputArtifactsByIndex(2),
                new SleepAction(1),
                storage.initServoAction(),
                new SleepAction(1),
                storage.outputArtifactsByIndex(0),
                new SleepAction(1),
                storage.initServoAction(),
                new SleepAction(1),
                storage.outputArtifactsByIndex(1),
                new SleepAction(1),
                storage.initServoAction(),
                storage.outputArtifactsByIndex(2),
                new SleepAction(1),
                storage.initServoAction()

        );

//        Action final_intake = drive.actionBuilder(new Pose2d(61, 14, Math.toRadians(135)))
//                .setTangent(Math.toRadians(135))
//                .splineToLinearHeading(
//                        new Pose2d(12, 30, Math.toRadians(90)),
//                        Math.toRadians(90),
//                        new TranslationalVelConstraint(25)
//                )
//                .build();

        Action shoot_logic = (telemetryPacket) -> {
            currentSort = turret.aprilTagWebCamSystem.getObelisk();
            for (int i = 0;currentSort == null && i < 100; i++){
                currentSort = turret.aprilTagWebCamSystem.getObelisk();
//                turret.update(drive.localizer.getPose());
            }
            try{
                if (currentSort[0] == Artifacts.GREEN) {
                    finalShoot = shootGPP;
                } else if (currentSort[1] == Artifacts.GREEN) {
                    finalShoot = shootPGP;
                } else {
                    finalShoot = shootPPG;
                }
            }
            catch (Exception e){
                finalShoot = shootPPG;
            }

//            finalShoot = shootPPG;

            return false;
        };
        Action shootAction = new ParallelAction(
            new SequentialAction(
//                    new SleepAction(0.5),
                    first_score,
                    new SleepAction(0.5),
                    shoot_logic
            ),
            storage.updateColorSensorsAction()
        );

        Thread alighnTurret = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
//                    drive.updatePoseEstimate();
                    Pose2d pose = drive.localizer.getPose();
                    turret.update(pose);
                    turret.aprilTagWebCamSystem.update(pose);
                    turret.updatePIDAlignment(GoalColor.RED, 0);
                    turret.pidController.updateValues(kp, ki, kd, 30);
                    dis = turret.aprilTagWebCamSystem.getDistanceFromGoal(GoalColor.RED);
                    telemetry.addData("Dis", dis);
                    telemetry.addData("Shooter Vel", shooter.getVelocity());
                    telemetry.addData("Shooter Target Vel", shooter.getVelocity());
                    shooter.changeAngle(shooter.getServoPositionWithDistance(dis),
                        shooter.getServoPositionWithDistance(dis));

                    if(!startShoot){
                        shooter.setVelocity(0);
                    }
                    else{
                        shooter.setVelocity(shooter.shootWithAutoPower(dis, 100));
                    }
                }
            }
        });

        waitForStart();

        alighnTurret.start();
//        alighnShooter.start();

//        Actions.runBlocking(
//                new SequentialAction(
//                    new SleepAction(0.5),
//                    shootAction
////                    first_score
//
//                )
//        );
//        startShoot = true;
//        Actions.runBlocking(
//                new SequentialAction(
//                    finalShoot,
//                    new SleepAction(2),
//                    shootRandom,
//                    new SleepAction(1),
//                    move_from_park,
//                    new SleepAction(10)
//                )
//        );
//
//        startShoot = false;

        startShoot = true;
        Actions.runBlocking(
                new SequentialAction(
                    first_score,
                    new SleepAction(0.5),
                    shoot_logic,
                    new SleepAction(0.5),
                    shootPPG,
                    new SleepAction(2),
                    shootRandom
//                        move_from_park
                )
        );
        startShoot = false;
        Actions.runBlocking(
            new SleepAction(2)
        );

        alighnTurret.interrupt();

        drive1 = new MecanumDrive(hardwareMap, shootPose);

        Action move_from_park = drive1.actionBuilder(shootPose)
                .strafeTo(new Vector2d(shootPose.position.x - 1, shootPose.position.y + 30))
                .build();

        Actions.runBlocking(
                move_from_park
        );


        PoseStorage.pose = drive1.localizer.getPose();
        PoseStorage.currentPose = turret.getAngle();
    }
}

//        for(int i = 0; i < 10 && sort == null; i ++){
//            sort = turret.aprilTagWebCamSystem.getObelisk();
//        }
//        telemetry.addData("Sort 1", sort[0]);
//        telemetry.addData("Sort 2", sort[1]);
//        telemetry.addData("Sort 3", sort[2]);


//        storage.updateColorSensors();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        shootAction,
//                        new SleepAction(2)
//                )
//        );

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
