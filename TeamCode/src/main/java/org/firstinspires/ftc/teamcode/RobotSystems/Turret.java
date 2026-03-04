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
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;


@Config
public class Turret {
    private DcMotorEx yawMotor;

    private static Turret INSTANCE;

    public static double currentPosition;

    private final double COUNTS_PER_REVOLUTION = 145.1 * 4;

    private double revolutions;

//    private double currentAngle = 0;

//    private double currentAngleNormalizedAngle;

    public static double currentCameraAngle = 0;

//    public static double Kp = 0.08;
//    public static double Ki = 0.03;
//    public static double Kd = 0.013;
//    public double lastError = 0;
//    public double lastValue = 0;
//    public double integralSum = 0;
    public boolean derivativeInitialized = false;

    private double currentAngleFromEncoder;

    boolean foundAprilTag;

    private Telemetry telemetry;

    private FtcDashboard dashboard;

    private Telemetry dashboardTelemetry;

    public AprilTagWebCamSystem aprilTagWebCamSystem;

    public PIDController pidController;

    public static double goalX = -64.961;       //-69.5; // -59.39127
    public static double goalY = 58.267;   //71; // 64.1121

    private double encoderOffset = 0; // Offset to sync encoder with reality
    private boolean wasTagVisible = false;
    public static double MIN_ANGLE = -160.0;
    public static double MAX_ANGLE = 190.0;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard, Pose2d startPose){
        this.telemetry = telemetry;
        this.dashboard = dashboard;
        this.dashboardTelemetry = this.dashboard.getTelemetry();

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

    // Helper to see if we are currently "calibrated"
    public boolean isTagVisible() {
        return wasTagVisible;
    }

    public double getCurrentAngle() {
        return currentAngleFromEncoder;
    }

    public void setPower(double power){
        yawMotor.setPower(power);
    }

    public double getAngle(){
        return currentPosition;
    }

    public void InitLoop(){
//        aprilTagWebCamSystem.update();
//        dashboardTelemetry.update();
    }

    public void startFunction(){
//        this.pidController = new PIDController( 0.065, 0.00005, 0.001, 1, false);
//        this.pidController = new PIDController( 0.05, 0, 0, 1, false);
//        this.pidController = new PIDController( 0.03, 0, 0, 1, false);
        this.pidController = new PIDController( 0, 0, 0, 1, false);

        pidController.reset();
    }

    // GET TARGET ANGLE WITH ID
    public double getTargetAngleWithWebcam(GoalColor goalColor){
        AprilTagDetection detection = aprilTagWebCamSystem.getDetectionByID(goalColor);
        if(detection == null){
            return getCurrentNormalizedAngle();
        }
        return getCurrentNormalizedAngle() - detection.ftcPose.bearing;
    }

//    public double getTargetAngleFromEncoder(GoalColor goalColor){
////      double goalX = -62.98087;
//
//        if(goalColor.getValue() != 24){
//            goalY = -72;
//        }
//
//        // 1. Get Robot position from Road Runner Pose
//        double robotX = aprilTagWebCamSystem.pose.position.x;
//        double robotY = aprilTagWebCamSystem.pose.position.y;
//
//        // 2. Convert Robot Heading to Degrees (RR uses Radians)
//        double robotHeadingDeg = Math.toDegrees(aprilTagWebCamSystem.pose.heading.toDouble());
//
//        // 3. Math.atan2 gives the angle from robot to goal in Radians
//        double angleToGoalRad = Math.atan2(goalY - robotY, goalX - robotX);
//
//        // 4. Convert that to Degrees
//        double absoluteFieldAngleDeg = Math.toDegrees(angleToGoalRad);
//
//        // 5. The turret target is the goal angle MINUS the robot's heading
//        double targetAngle = absoluteFieldAngleDeg - robotHeadingDeg;
//
//        // 6. Keep it between -180 and 180
//        return AngleUnit.normalizeDegrees(targetAngle);
//    }

    public double getTargetAngleFromEncoder(GoalColor goalColor){
    //     double goalX = -62.98087;

        if(goalColor.getValue() != 24){
            goalY = -72;
        }

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


    public void updatePIDAlignment(GoalColor goalColor, double offset) {
        double target = getTargetAngleFromEncoder(goalColor) + offset;
        double current = getCurrentAngleFromEncoder();

        double power = pidController.updateController(target, current);

        // Limit power for safety during testing
        power = Math.max(-0.8, Math.min(0.8, power));

        yawMotor.setPower(power);
    }


    // GET CURRENT ANGLE FROM ENCODER
    public double getCurrentAngleFromEncoder(){
        return currentAngleFromEncoder;
    }

    // THE CALCULATION IS WRONG
    public double getCurrentAngleWithWebcam(GoalColor goalColor){
        AprilTagDetection detection = aprilTagWebCamSystem.getDetectionByID(goalColor);
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
        telemetry.addData("X", aprilTagWebCamSystem.pose.position.x);
        telemetry.addData("Y", aprilTagWebCamSystem.pose.position.y);
//        telemetry.addData("current normilized angle", getCurrentNormalizedAngle());
//        telemetry.addData("currentCameraAngle", currentCameraAngle);

        dashboardTelemetry.addData("current angle", currentAngleFromEncoder);
//        dashboardTelemetry.addData("X", aprilTagWebCamSystem.pose.position.x);
//        dashboardTelemetry.addData("Y", aprilTagWebCamSystem.pose.position.y);
//        dashboardTelemetry.addData("current normilized angle", getCurrentNormalizedAngle());
//        dashboardTelemetry.addData("currentCameraAngle", currentCameraAngle);
    }

    public void rotateWithJoystick(double power) {
        yawMotor.setPower(power);
    }

    public static Turret getInstance(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard, Pose2d startPose){
        if(INSTANCE == null){
            INSTANCE = new Turret(hardwareMap, telemetry, dashboard, startPose);
        }
        return INSTANCE;
    }
}