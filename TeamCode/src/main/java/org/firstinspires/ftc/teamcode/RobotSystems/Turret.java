package org.firstinspires.ftc.teamcode.RobotSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.PIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class Turret {
    private DcMotorEx yawMotor;

    private double currentPosition;

    private final double COUNTS_PER_REVOLUTION = 145.1 * 4;

    private double revolutions;

//    private double currentAngle = 0;

//    private double currentAngleNormalizedAngle;

    public static double currentCameraAngle = 0;

    public static double Kp = 0.08;
    public static double Ki = 0.03;
    public static double Kd = 0.013;
//    public double lastError = 0;
//    public double lastValue = 0;
//    public double integralSum = 0;
    public boolean derivativeInitialized = false;

    private double currentAngleFromEncoder;

    boolean foundAprilTag;
    TURRET_STATES curState = TURRET_STATES.AIMING;

    private Telemetry telemetry;

    private FtcDashboard dashboard;

    private Telemetry dashboardTelemetry;

    private AprilTagWebCamSystem aprilTagWebCamSystem;

    public PIDController pidController;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard, Pose2d startPose){
        this.telemetry = telemetry;
        this.dashboard = dashboard;
        this.dashboardTelemetry = this.dashboard.getTelemetry();

        this.pidController = new PIDController(Kp, Ki, Kd);

        yawMotor = hardwareMap.get(DcMotorEx.class, "yawMotor");
        yawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        aprilTagWebCamSystem = new AprilTagWebCamSystem(hardwareMap, telemetry, dashboard, startPose);


        // Turret.java inside constructor
        yawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Calculate the counts equivalent to 180 degrees
        int initialTicks = (int)(180 / 360.0 * COUNTS_PER_REVOLUTION);
        yawMotor.setTargetPosition(initialTicks);
        // This "tricks" the motor into thinking it has already moved to 180
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // Turn the motor back on when we are done

        currentPosition = yawMotor.getCurrentPosition();
    }

    public void update(Pose2d currentPose){

        currentPosition = yawMotor.getCurrentPosition();
        revolutions = currentPosition / COUNTS_PER_REVOLUTION;
        currentAngleFromEncoder = revolutions * 360;

        displayTelemetry();

        aprilTagWebCamSystem.update(currentPose);
        telemetry.update();
        dashboardTelemetry.update();
    }

    public void setPower(double power){
        yawMotor.setPower(power);
    }

    public void InitLoop(){
//        aprilTagWebCamSystem.update();
//        dashboardTelemetry.update();
    }

    public void startFunction(){
        pidController.reset();
    }

    // GET TARGET ANGLE WITH ID
    public double getTargetAngleWithWebcam(int id){
        AprilTagDetection detection = aprilTagWebCamSystem.getDetectionByID(id);
        if(detection == null){
            return getCurrentNormalizedAngle();
        }
        return getCurrentNormalizedAngle() - detection.ftcPose.bearing;
    }

    public double getTargetAngleFromEncoder(int id){
        double goalX = -58.3727;
        double goalY = (id == 24) ? 55.6425 : -55.6425;

        // 1. Get Robot position from Road Runner Pose
        double robotX = aprilTagWebCamSystem.pose.position.x;
        double robotY = aprilTagWebCamSystem.pose.position.y;

        // 2. Convert Robot Heading to Degrees (RR uses Radians)
        double robotHeadingDeg = Math.toDegrees(aprilTagWebCamSystem.pose.heading.toDouble());

        // 3. Math.atan2 gives the angle from robot to goal in Radians
        double angleToGoalRad = Math.atan2(goalY - robotY, goalX - robotX);

        // 4. Convert that to Degrees
        double absoluteFieldAngleDeg = Math.toDegrees(angleToGoalRad);

        // 5. The turret target is the goal angle MINUS the robot's heading
        double targetAngle = absoluteFieldAngleDeg - robotHeadingDeg;

        // 6. Keep it between -180 and 180
        return AngleUnit.normalizeDegrees(targetAngle);
    }

    public void updatePIDAlignment(int id) {
        double target = getTargetAngleFromEncoder(id);
        double current = getCurrentAngleFromEncoder();

        double power = pidController.PIDcontroller(target, current);

        // Limit power for safety during testing
        power = Math.max(-0.5, Math.min(0.5, power));

        yawMotor.setPower(power);
    }

    // GET CURRENT ANGLE FROM ENCODER
    public double getCurrentAngleFromEncoder(){
        return currentAngleFromEncoder;
    }

    // THE CALCULATION IS WRONG
    public double getCurrentAngleWithWebcam(int id){
        AprilTagDetection detection = aprilTagWebCamSystem.getDetectionByID(id);
        if(detection == null){
            return getCurrentAngleFromEncoder();
        }
        double alpha = Math.acos(detection.ftcPose.y / detection.ftcPose.range);
        double beta = detection.ftcPose.bearing;
        return alpha + beta;
    }

    public double getCurrentNormalizedAngle(){
        return getCurrentAngleFromEncoder() % 360;
    }

    public void displayTelemetry(){
        telemetry.addData("current angle", currentAngleFromEncoder);
        telemetry.addData("current normilized angle", getCurrentNormalizedAngle());
        telemetry.addData("currentCameraAngle", currentCameraAngle);

        dashboardTelemetry.addData("current angle", currentAngleFromEncoder);
        dashboardTelemetry.addData("current normilized angle", getCurrentNormalizedAngle());
        dashboardTelemetry.addData("currentCameraAngle", currentCameraAngle);
    }

    public void rotateWithJoystick(double power) {
        yawMotor.setPower(power);
    }

//    public void rotateWithPID(){
//        yawMotor.setPower(pidController.PIDcontroller(getTargetAngleWithWebcam(24), getCurrentAngleFromEncoder()));
//    }
//
//    public void spinControl(int id){
//        AprilTagDetection detection = aprilTagWebCamSystem.getDetectionByID(id);
//        if(detection == null){
//            foundAprilTag = false;
//        }
//        else{
//            foundAprilTag = true;
//        }
//        if(getCurrentAngleFromEncoder() > 180){
//            curState = TURRET_STATES.START_CORRECTING;
//            yawMotor.setPower(DIRECTION.LEFT.getValue());
//        }
//        else if(getCurrentAngleFromEncoder() < -180){
//            curState = TURRET_STATES.START_CORRECTING;
//            yawMotor.setPower(DIRECTION.RIGHT.getValue());
//        }
//
//        if(curState == TURRET_STATES.START_CORRECTING && !foundAprilTag){
//            curState = TURRET_STATES.MIDDLE_CORRECTING;
//        }
//        if(curState == TURRET_STATES.MIDDLE_CORRECTING && foundAprilTag){
//            curState = TURRET_STATES.AIMING;
//        }
//
//        if(curState == TURRET_STATES.AIMING){
//            yawMotor.setPower(pidController.PIDcontroller(getTargetAngleWithWebcam(24), getCurrentAngleFromEncoder()));
//        }
//        else{
//            yawMotor.setPower(yawMotor.getPower());
//        }
//    }
    public enum DIRECTION{
        LEFT(0.4),
        RIGHT(-0.4);
        private double value;

        DIRECTION(double value) {
            this.value = value;
        }

        public double getValue(){
            return this.value;
        }
    }
    public enum TURRET_STATES{
        AIMING,
        START_CORRECTING,
        MIDDLE_CORRECTING;
    }

}