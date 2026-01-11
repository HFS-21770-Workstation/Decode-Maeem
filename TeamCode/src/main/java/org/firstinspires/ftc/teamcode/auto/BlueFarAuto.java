package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import org.jetbrains.annotations.NotNull;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Arclength;

@Autonomous(name = "Blue Far Auto", group = "Auto")
public class BlueFarAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(61, -15, Math.toRadians(-180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action first_score = drive.actionBuilder(new Pose2d(61, -15, Math.toRadians(-180)))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-135)), Math.toRadians(-135))
                .waitSeconds(5)
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-135)), Math.toRadians(-135))
                .build();

        Action first_intake = drive.actionBuilder(new Pose2d(61, -14, Math.toRadians(-135)))
                .splineToLinearHeading(new Pose2d(37, -35, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToY(-46, (pose, path, v) -> 15)
                .build();

        Action second_score = drive.actionBuilder(new Pose2d(37, -46, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-270))
                .splineToLinearHeading(new Pose2d(61, -12, Math.toRadians(-135)), Math.toRadians(45))
                .waitSeconds(5)
                .splineToLinearHeading(new Pose2d(61, -12, Math.toRadians(-135)), Math.toRadians(-135))
                .build();

        Action second_intake = drive.actionBuilder(new Pose2d(61, -12, Math.toRadians(-135)))
                .splineToLinearHeading(new Pose2d(50, -60, Math.toRadians(-50)), Math.toRadians(-50))
                .setTangent(Math.toRadians(-50))
                .splineToLinearHeading(
                        new Pose2d(67, -60, Math.toRadians(-30)),
                        Math.toRadians(-30),
                        (pose, path, v) -> 15
                )
                .build();

        Action third_score = drive.actionBuilder(new Pose2d(67, -60, Math.toRadians(-30)))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-135)), Math.toRadians(-315))
                .waitSeconds(5)
                .splineToLinearHeading(new Pose2d(61, -14, Math.toRadians(-135)), Math.toRadians(-315))
                .build();

        Action final_intake = drive.actionBuilder(new Pose2d(61, -14, Math.toRadians(-135)))
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        waitForStart();
        if (isStopRequested()) return;

//        Actions.runBlocking(
//                new SequentialAction(
//                        first_score,
//                        first_intake,
//                        second_score,
//                        second_intake,
//                        third_score,
//                        final_intake
//                )
//        );
    }
}
