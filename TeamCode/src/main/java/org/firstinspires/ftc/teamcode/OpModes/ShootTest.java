package org.firstinspires.ftc.teamcode.OpModes;

import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.AprilTagWebCamSystem;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;

import java.util.Arrays;

@Config
@TeleOp
public class ShootTest extends OpMode {

    Shooter shooter;
    Turret turret;
//    AprilTagWebCamSystem aprilTagWebCamSystem;
    Servo servoPusher1;
    VoltageSensor voltageSensor;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    MecanumDrive mecanumDrive;
    Storage storage;
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    boolean startShot = false;
    double turretOffset = 0;
    boolean isAutoShooting= false;
    double measuredVelocity = 0;
    double velocityIncrement = 125;

    public static double kp = 0.03;
    public static double ki = 0;
    public static double kd = 0;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);
//        aprilTagWebCamSystem = new AprilTagWebCamSystem(hardwareMap, telemetry, dashboard, startPose);
        turret = new Turret(hardwareMap, telemetry, dashboard, startPose);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        storage = new Storage(hardwareMap);
        servoPusher1 = hardwareMap.servo.get("servoPusher3");
        servoPusher1.setPosition(1);
        shooter = new Shooter(hardwareMap);

        shooter.initPos();
        storage.initServos();
    }

    @Override
    public void start() {
        turret.startFunction();
        shooter.startCal();
    }

    @Override
    public void loop() {
        mecanumDrive.updatePoseEstimate();
        Pose2d currentPose = mecanumDrive.localizer.getPose();
        double distance = turret.aprilTagWebCamSystem.getDistanceFromGoal(GoalColor.RED);

        // --- עדכון מערכות בסיסי ---
        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                -gamepad1.right_stick_x
        ));
        turret.update(currentPose);
        turret.aprilTagWebCamSystem.update(currentPose);
        turret.updatePIDAlignment(GoalColor.RED, turretOffset);
        turret.pidController.updateValues(kp, ki, kp, 1);
        storage.checkTime();

        if (!storage.waitingForDown) {
            storage.updateColorSensors();
        }

        // --- עדכון זווית תמידי ---
        // הסרוואים תמיד יתכווננו לפי המרחק מהמטרה, גם אם המנועים כבויים
        double servoPos = shooter.getServoPositionWithDistance(distance);
        shooter.changeAngle(servoPos, servoPos);

        // --- לוגיקת הפעלת המנועים (B) ---
        if (gamepad1.bWasPressed()) {
            isAutoShooting = !isAutoShooting;
        }

        if (isAutoShooting) {
            // רק המנועים נכנסים לעבודה לפי הפולינום
            measuredVelocity = shooter.shootWithAutoPower(distance, 0);
            shooter.setVelocity(measuredVelocity);

//            if (gamepad1.dpadUpWasPressed()) measuredVelocity += velocityIncrement;
//            if (gamepad1.dpadDownWasPressed()) measuredVelocity -= velocityIncrement;
            // אופציונלי: ירייה אוטומטית של הכדור ברגע שהמהירות מוכנה


        }
        if (isAutoShooting == false){
            // כיבוי מנועי היורה
            shooter.setVelocity(0);
        }

        // כיוונון עדין וטלמטריה
        if(gamepad1.dpadRightWasPressed()) turretOffset += 1;
        if(gamepad1.dpadLeftWasPressed()) turretOffset -= 1;
        if (gamepad1.aWasPressed()) storage.setOutPutArtifactsRandom();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Distance", distance);
        packet.put("Auto Mode", isAutoShooting);
        packet.put("vel", shooter.getVelocity());
        packet.put("Tvel", measuredVelocity);
        packet.put("Polynomial Coefficients", Arrays.toString(shooter.getPolynomial()));
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        shooter.stop();
        storage.stop();

    }
}
