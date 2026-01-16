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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;

@TeleOp
@Config
public class turretTest extends OpMode {

    MecanumDrive mecanumDrive;
    Turret turret;

    // Dashboard variables
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

    @Override
    public void start(){
        turret.startFunction();
    }

    @Override
    public void init() {
        // Initialize MecanumDrive (This replaces your old 'Drive' class)
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);

        // Initialize Turret
        turret = new Turret(hardwareMap, telemetry, dashboard, startPose);

        // This allows telemetry.addData to show up on both the Phone and the Dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
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

        // 4. Update Turret with the current pose
        turret.update(currentPose);

        if(gamepad1.b){
            turret.updatePIDAlignment(24);
        }

        // 5. THE DRAWING LOGIC
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        // Draw the robot using the pose we just got from the localizer
        field.setStroke("#3F51B5");
        Drawing.drawRobot(field, currentPose);

        double turretAngle = Math.toRadians(turret.getCurrentAngleFromEncoder());
        double lineLength = 10.0;
        field.setStroke("#FF0000"); // Red for turret
        field.strokeLine(
                currentPose.position.x,
                currentPose.position.y,
                currentPose.position.x + Math.cos(turretAngle) * lineLength,
                currentPose.position.y + Math.sin(turretAngle) * lineLength
        );

        // 6. Send everything to Dashboard
        dashboard.sendTelemetryPacket(packet);

        // Optional: Keep your phone telemetry alive
        telemetry.addData("X", currentPose.position.x);
        telemetry.addData("Y", currentPose.position.y);
        telemetry.addData("target angle:", turret.getTargetAngleFromEncoder(24));
        telemetry.addData("error:", turret.pidController.getError());
        telemetry.addData("output", turret.pidController.getOutput());
        telemetry.update();
    }
}
