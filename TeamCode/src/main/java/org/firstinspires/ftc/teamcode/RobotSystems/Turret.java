package org.firstinspires.ftc.teamcode.RobotSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public static double Kp = 0.015;
    public static double Ki = 0;
    public static double Kd = 0.00018;
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

    private PIDController pidController;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, FtcDashboard dashboard){
        this.telemetry = telemetry;
        this.dashboard = dashboard;
        this.dashboardTelemetry = this.dashboard.getTelemetry();

        this.pidController = new PIDController(Kp, Ki, Kd);

        yawMotor = hardwareMap.get(DcMotorEx.class, "yawMotor");
        yawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        aprilTagWebCamSystem = new AprilTagWebCamSystem(hardwareMap, telemetry, dashboard);

        yawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    // Turn the motor back on when we are done

        currentPosition = yawMotor.getCurrentPosition();
    }

    public void update(){
        currentPosition = yawMotor.getCurrentPosition();
        revolutions = currentPosition / COUNTS_PER_REVOLUTION;
        currentAngleFromEncoder = revolutions * 360;


        aprilTagWebCamSystem.update();
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
//        dashboardTelemetry.update();
    }

    // GET TARGET ANGLE WITH ID
    public double getTargetAngleWithWebcam(int id){
        AprilTagDetection detection = aprilTagWebCamSystem.getDetectionByID(id);
        if(detection == null){
            return getCurrentNormalizedAngle();
        }
        return getCurrentNormalizedAngle() - detection.ftcPose.bearing;
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

    public void rotateWithJoyStick(double power) {
        yawMotor.setPower(power);
    }
    public void spinControl(int id){
        AprilTagDetection detection = aprilTagWebCamSystem.getDetectionByID(id);
        if(detection == null){
            foundAprilTag = false;
        }
        else{
            foundAprilTag = true;
        }
        if(getCurrentAngleFromEncoder() > 180){
            curState = TURRET_STATES.START_CORRECTING;
            yawMotor.setPower(DIRECTION.LEFT.getValue());
        }
        else if(getCurrentAngleFromEncoder() < -180){
            curState = TURRET_STATES.START_CORRECTING;
            yawMotor.setPower(DIRECTION.RIGHT.getValue());
        }

        if(curState == TURRET_STATES.START_CORRECTING && !foundAprilTag){
            curState = TURRET_STATES.MIDDLE_CORRECTING;
        }
        if(curState == TURRET_STATES.MIDDLE_CORRECTING && foundAprilTag){
            curState = TURRET_STATES.AIMING;
        }

        if(curState == TURRET_STATES.AIMING){
            yawMotor.setPower(pidController.PIDcontroller(getTargetAngleWithWebcam(24), getCurrentAngleFromEncoder()));
        }
        else{
            yawMotor.setPower(yawMotor.getPower());
        }
    }
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