package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.AprilTagWebCamSystem;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;

@TeleOp
@Config
public class ShootTest extends OpMode {
    Shooter shooter;
    Turret turret;
    AprilTagWebCamSystem aprilTagWebCamSystem;
    final double pos = 0.1;
    final double power = 0.1;
    VoltageSensor voltageSensor;
    Servo servoPusher1;
    boolean startShot = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();


    MecanumDrive mecanumDrive;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        aprilTagWebCamSystem = new AprilTagWebCamSystem(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);
        shooter =  new Shooter(hardwareMap);
        shooter.ChangeAngle(0,0);
        turret = new Turret(hardwareMap, telemetry, dashboard, startPose);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry = FtcDashboard.getInstance().getTelemetry();

        servoPusher1 = hardwareMap.servo.get("servoPusher3");
        servoPusher1.setPosition(1);
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

        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(x, y),
                rx
        ));


        aprilTagWebCamSystem.update(mecanumDrive.localizer.getPose());
        turret.update(currentPose);
        aprilTagWebCamSystem.update(currentPose);

        turret.updatePIDAlignment(24);


        double distance = aprilTagWebCamSystem.getDistanceFromGoal(24);
        if (gamepad1.bWasPressed()) {
            startShot = !startShot;
            if (startShot) {
                shooter.shootWithAutoPower(distance, voltageSensor.getVoltage());
            } else {
                shooter.StartShoot(0);
            }
        }

//        double distance = -1;
//        if(distance == -1){
//            shooter.ChangeAngle(shooter.GetPosR(),shooter.GetPosL());
//        }
        shooter.ChangeAngle(shooter.getServoPositionWithDistance(distance),
                shooter.getServoPositionWithDistance(distance));
//        if (startShot) {
//            if (gamepad1.yWasPressed()) {
//                shooter.StartShoot(shooter.GetPower() + 0.025);
//            }
//            if (gamepad1.aWasPressed()) {
//                shooter.StartShoot(shooter.GetPower() - 0.025);
//            }
//        }
        if(gamepad1.dpad_up){

        }
        if(gamepad1.x){
            servoPusher1.setPosition(0);
        }
        else{
            servoPusher1.setPosition(1);
        }
        telemetry.addData("Power:", shooter.GetPower());
//        telemetry.addData("PosL:", shooter.GetPosL());
//        telemetry.addData("PosR:", shooter.GetPosR());
        telemetry.addData("Volt:", voltageSensor.getVoltage());
//        String distanceStr = "Distance: " + String.format("%.3f", distance);
//        telemetry.addLine(distanceStr);
        telemetry.addData("d",aprilTagWebCamSystem.getDistanceFromGoal(24));

        telemetry.addData("start shoot", startShot);

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
    }
    @Override
    public void stop(){
        aprilTagWebCamSystem.stop();
    }
}