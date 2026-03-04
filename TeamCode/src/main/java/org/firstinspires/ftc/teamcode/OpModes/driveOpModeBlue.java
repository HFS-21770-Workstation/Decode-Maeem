package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSystems.IntakeOld;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;
import org.firstinspires.ftc.teamcode.Util.PoseStorage;
import org.firstinspires.ftc.teamcode.Util.Enums.Artifacts;
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;


@TeleOp(group = "TeleOps",name = "00 TeleOp Blue")
public class driveOpModeBlue extends OpMode {
    Storage storage;
    Shooter shooter;
    public static Turret turret;
    final double pos = 0.1;
    final double power = 0.1;
    VoltageSensor voltageSensor;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    //    Pose2d startPose = new Pose2d(62, 14.80314, Math.toRadians(180));
    Pose2d startPose = PoseStorage.pose;
    boolean startShot;
    double turretOffset = 0;
    double shooterOffset = 50;
    public boolean turretOffsetMore = false;

    double measuredVelocity = 0;


    MecanumDrive mecanumDrive;
//    Drive drive;

    int index = 0;

    IntakeOld intake;

    double currentShooterPower = 0;

    double intakePower = 0;

    public static double kp = 0.03;
    public static double ki = 0;
    public static double kd = 0;

    boolean turretAuto = true;

    @Override
    public void init() {
        if(startPose == null){
            startPose = new Pose2d(40, -16, Math.toRadians(90));
        }
        turretOffset = -PoseStorage.currentPose;
        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance(), startPose);
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        storage = new Storage(hardwareMap);


        shooter = new Shooter(hardwareMap);
        shooter.changeAngle(0,0);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        intake = new IntakeOld(hardwareMap);

        storage.initServos();

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

        turret.update(currentPose);
        turret.aprilTagWebCamSystem.update(currentPose);
        if(turretAuto){
            turret.updatePIDAlignment(GoalColor.BLUE, turretOffset);
        }
        else{
            turret.setPower(0);
        }
        turret.pidController.updateValues(kp, ki, kd, 1);

        double distance = turret.aprilTagWebCamSystem.getDistanceFromGoal(GoalColor.BLUE);
        shooter.changeAngle(shooter.getServoPositionWithDistance(distance),
                shooter.getServoPositionWithDistance(distance));


        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.right_stick_x;
        double rx = -gamepad1.left_stick_x;

        if(gamepad1.right_trigger != 0){
            x /= 2;
            y /= 2;
            rx /= 2;
        }

        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(x, y),
                rx
        ));

        storage.checkTime();

        if (!storage.waitingForDown) {
            storage.updateColorSensors();
        }

        telemetry.addData("slots1", storage.getArtifactsStorage()[0]);
        telemetry.addData("slots2", storage.getArtifactsStorage()[1]);
        telemetry.addData("slots3", storage.getArtifactsStorage()[2]);


        // 5. THE DRAWING LOGIC
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        // Draw the robot using the pose we just got from the localizer
        field.setStroke("#3F51B5");
        Drawing.drawRobot(field, currentPose);

        // 6. Send everything to Dashboard
        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Turret Offset", turretOffset);
        telemetry.addData("Shooter Offset", shooterOffset);
        telemetry.update();

        if (gamepad1.aWasPressed()) {
            startShot = !startShot;
        }

        if (startShot) {
            // רק המנועים נכנסים לעבודה לפי הפולינום
            measuredVelocity = shooter.shootWithAutoPower(distance, shooterOffset);
            shooter.setVelocity(measuredVelocity);

//            if (gamepad1.dpadUpWasPressed()) measuredVelocity += velocityIncrement;
//            if (gamepad1.dpadDownWasPressed()) measuredVelocity -= velocityIncrement;
            // אופציונלי: ירייה אוטומטית של הכדור ברגע שהמהירות מוכנה


        }
        if (startShot == false){
            // כיבוי מנועי היורה
            shooter.setVelocity(0);
        }


        if (gamepad1.left_trigger == 1){
            shooter.stopShoot();
        }
//        shooter.changeAngle(shooter.getServoPositionWithDistance(distance),
//                shooter.getServoPositionWithDistance(distance));



        if(gamepad1.xWasPressed() && startShot){
            if(shooter.getPower() == 1){
//                shooter.startShoot(1);
            }
            storage.setOutPutArtifacts(Artifacts.GREEN);
        }
        else if (gamepad1.bWasPressed() && startShot) {

            storage.setOutPutArtifacts(Artifacts.PURPLE);
        }
        else if(gamepad1.yWasPressed() && startShot){
            if(shooter.getPower() == 1){
//                shooter.startShoot(1);
            }
            storage.setOutPutArtifactsRandom(); // this thing wasn't checked
        }

        if(!startShot){
            intakePower = 1;
        }
        if(gamepad1.dpadDownWasPressed() && intakePower < 1){
            intakePower += 1;
        }
        if(gamepad1.dpadUpWasPressed() && intakePower > -1){
            intakePower -= 1;
        }
        if(startShot){
            intakePower = 0;
        }
        intake.startIntake(intakePower);
        intake.updateIntake();

//        if (gamepad2.dpadUpWasPressed()) {
//            shooter.changeAngle(shooter.getPosR() + pos, shooter.getPosL() + pos);
//        }
//        if (gamepad2.dpadDownWasPressed()) {
//            shooter.changeAngle(shooter.getPosR() - pos, shooter.getPosL() - pos);
//        }
        if(gamepad2.aWasPressed()) {
            storage.pusherUp(0);
        }
        if(gamepad2.xWasPressed()) {
            storage.pusherUp(2);
        }
        if(gamepad2.bWasPressed()) {
            storage.pusherUp(1);
        }

        if (gamepad2.yWasPressed()){
            turretOffsetMore = !turretOffsetMore;
        }

        if (turretOffsetMore){
            if(gamepad2.dpadRightWasPressed()){
                turretOffset -= 15;
            }
            if(gamepad2.dpadLeftWasPressed()){
                turretOffset += 15;
            }
        }
        if (!turretOffsetMore){
            if(gamepad2.dpadRightWasPressed()){
                turretOffset -= 3;
            }
            if(gamepad2.dpadLeftWasPressed()){
                turretOffset += 3;
            }
        }

        if (gamepad2.dpadUpWasPressed()) {
            shooterOffset += 25;
        }
        if (gamepad2.dpadDownWasPressed()) {
            shooterOffset -= 25;
        }
        if(gamepad2.right_trigger == 1){
            shooterOffset = 0;
        }

        if(gamepad2.startWasPressed()){
            turretAuto = !turretAuto;
        }
        if(gamepad2.optionsWasPressed()){
            mecanumDrive.localizer.setPose(new Pose2d(63, 63, Math.toRadians(180)));
        }
    }

    @Override
    public void stop(){
        shooter.stopShoot();
    }

}
