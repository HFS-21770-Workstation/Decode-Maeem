package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;
import org.firstinspires.ftc.teamcode.Util.Enums.Artifacts;
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;

import java.util.Arrays;

@TeleOp
@Config
public class ShootTest extends OpMode {

    MecanumDrive mecanumDrive;
    Turret turret;
    Shooter shooter;
    Storage storage;

    boolean isAutoShooting = false;
    double turretOffset = 0;
    double measuredVelocity = 0;
    double velocityIncrement = 125;

    public static double kp = 0.03;
    public static double ki = 0;
    public static double kd = 0;

    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    // Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize systems
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);

        turret = Turret.getInstance(hardwareMap, telemetry, dashboard, startPose);

        shooter = new Shooter(hardwareMap);
        storage = Storage.getInstance(hardwareMap);
        storage.initServos();

        // Shooter initial position
        shooter.initPos();
    }

    @Override
    public void start() {
        turret.startFunction();
        shooter.startCal();
        storage.start();
    }

    @Override
    public void loop() {
        // --- Update robot pose ---
        mecanumDrive.updatePoseEstimate();
        Pose2d currentPose = mecanumDrive.localizer.getPose();

        // --- Drive controls ---
        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(x, y), rx));

        // --- Update turret and camera safely ---
        turret.update(currentPose);
        if (turret.aprilTagWebCamSystem != null) {
            turret.aprilTagWebCamSystem.update(currentPose);
        }

        turret.updatePIDAlignment(GoalColor.RED, turretOffset);
        turret.pidController.updateValues(kp, ki, kd, 1);

        // --- Storage color update ---
        storage.checkTime();
        if (!storage.waitingForDown) {
            storage.updateColorSensors();
        }

        // --- Shooter logic ---
        double distance = turret.aprilTagWebCamSystem.getDistanceFromGoal(GoalColor.RED);


        double servoPos = shooter.getServoPositionWithDistance(distance);
        shooter.changeAngle(servoPos, servoPos);

        // Toggle auto shooting
        if (gamepad1.bWasPressed()) {
            isAutoShooting = !isAutoShooting;
        }

        if (isAutoShooting) {
            measuredVelocity = shooter.shootWithAutoPower(distance, 0);
            shooter.setVelocity(measuredVelocity);
        } else {
            shooter.setVelocity(0);
        }

        // Manual storage trigger
        if (gamepad1.aWasPressed()) {
            storage.setOutPutArtifacts(Artifacts.PURPLE);
        }
        if (gamepad1.xWasPressed()) {
            storage.setOutPutArtifacts(Artifacts.GREEN);
        }

        // --- Turret fine tuning ---
        if (gamepad1.dpadRightWasPressed()) turretOffset += 1;
        if (gamepad1.dpadLeftWasPressed()) turretOffset -= 1;

        // --- Dashboard drawing ---
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        field.setStroke("#3F51B5");
        Drawing.drawRobot(field, currentPose);

        double turretAngle = Math.toRadians(turret.getCurrentAngleFromEncoder());
        double lineLength = 10.0;
        field.setStroke("#FF0000");
        field.strokeLine(
                currentPose.position.x,
                currentPose.position.y,
                currentPose.position.x + Math.cos(turretAngle) * lineLength,
                currentPose.position.y + Math.sin(turretAngle) * lineLength
        );

        dashboard.sendTelemetryPacket(packet);

        // --- Telemetry ---
        telemetry.addData("Distance", distance);
        telemetry.addData("Camera visible", turret.isTagVisible());
        telemetry.addData("Turret target angle", turret.getTargetAngleFromEncoder(GoalColor.RED));
        telemetry.addData("PID error", turret.pidController.getError());
        telemetry.addData("Auto shooting", isAutoShooting);
        telemetry.addData("Shooter velocity", shooter.getVelocity());
        telemetry.addData("Polynomial", Arrays.toString(shooter.getPolynomial()));
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        storage.stop();
    }
}