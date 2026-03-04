package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.Util.Enums.Artifacts;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;

@TeleOp
@Config
public class turretTest extends OpMode {

    MecanumDrive mecanumDrive;
    Turret turret;
    Shooter shooter;
    boolean startShot;

    public static double kp = 0.00000001;
    public static double ki = 0.00000001;
    public static double kd = 0.00000001;

    public int index = 0;


    // Dashboard variables
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));


//    VoltageSensor voltageSensor;
    Storage storage;
//    AprilTagWebCamSystem aprilTagWebCamSystem;

    final double pos = 0.1;
    final double power = 0.04;

    double turretOffset = 0;

    @Override
    public void start(){
        turret.startFunction();
    }

    @Override
    public void init() {
        // Initialize MecanumDrive (This replaces your old 'Drive' class)
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);

        // Initialize Turret
        turret = Turret.getInstance(hardwareMap, telemetry, dashboard, startPose);

        shooter = new Shooter(hardwareMap);
//        shooter.initPos();

//        voltageSensor = hardwareMap.voltageSensor.iterator().next();
//
        storage = new Storage(hardwareMap);
        storage.initServos();

//        aprilTagWebCamSystem = new AprilTagWebCamSystem(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);


        // This allows telemetry.addData to show up on both the Phone and the Dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        mecanumDrive.updatePoseEstimate();
        double distance = turret.aprilTagWebCamSystem.getDistanceFromGoal(GoalColor.RED);
        turret.pidController.updateValues(kp, ki, kd, 1);

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

        turret.updatePIDAlignment(GoalColor.RED, turretOffset);

        if(gamepad1.dpadUpWasPressed()){
            if(index == 0){
                kp *= 10;
            }
            else if(index == 1){
                ki *= 10;
            }
            else{
                kd *= 10;
            }
        }
        if(gamepad1.dpadDownWasPressed()){
            if(index == 0){
                kp /= 10;
            }
            else if(index == 1){
                ki /= 10;
            }
            else{
                kd /= 10;
            }
        }

        if(gamepad1.dpadLeftWasPressed()){
            index -= 1;
        }
        if(gamepad1.dpadRightWasPressed()){
            index += 1;
        }

        // 5. THE DRAWING LOGIC
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        // Draw the robot using the pose we just got from the localizer
        field.setStroke("#3F51B5");
        Drawing.drawRobot(field, currentPose);

        double turretAngle = Math.toRadians(turret.getCurrentAngleFromEncoder());
        double lineLength = 10.0;
        field.setStroke("#FF0000"); // RED for turret
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
        telemetry.addData("target angle:", turret.getTargetAngleFromEncoder(GoalColor.RED));
        telemetry.addData("error:", turret.pidController.getError());
//        telemetry.addData("output", turret.pidController.getOutput());
//        telemetry.addData("Power:", shooter.getPower());
//        telemetry.addData("Volt:", voltageSensor.getVoltage());
        telemetry.addData("distance:", distance);
        telemetry.addData("camera", turret.isTagVisible());
        telemetry.update();
    }

//    @Override
//    public void stop(){
//        shooter.stopShoot();
//    }
}
