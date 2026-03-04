package org.firstinspires.ftc.teamcode.RobotSystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.PIDFController;
import org.firstinspires.ftc.teamcode.Util.calculateShootPower;
import org.firstinspires.ftc.teamcode.Util.Enums.Angle;

public class Shooter {


    DcMotor shooterMotorUp;
    DcMotorEx shooterMotorDown;
    PIDFController pidfController;

    private final double TICKS_PER_REVOLUTION = 28 ;
    private static Shooter INSTANCE;
    Servo hoodRight;
    Servo hoodLeft;
    calculateShootPower powerClose;
    calculateShootPower powerMid;
    calculateShootPower powerFar;

    public double upCurrent = 0;
    public double downCurrent = 0;

    public static double kp = 0.0005;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0.000215;

    public double targetVelocity = 0;


    public Shooter(HardwareMap hardwareMap){
        shooterMotorUp = hardwareMap.get(DcMotor.class,"shooterUp");
        shooterMotorDown = hardwareMap.get(DcMotorEx.class, "shooterDown");


        shooterMotorUp.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorDown.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorDown.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotorUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hoodRight = hardwareMap.servo.get("hoodRight");
        hoodLeft = hardwareMap.servo.get("hoodLeft");
        hoodRight.setDirection(Servo.Direction.FORWARD);
        hoodLeft.setDirection(Servo.Direction.REVERSE);
        pidfController = new PIDFController(kp, ki, kd,kf, 1,0,true);
    }
    public static Shooter getInstance(HardwareMap hardwareMap){
        if(INSTANCE == null){
            INSTANCE = new Shooter(hardwareMap);
        }

        return INSTANCE;
    }
    public void stop(){
        INSTANCE = null;
    }
    public void startCal(){
         powerFar = new calculateShootPower.Builder()
                 .addSample(119.83853896827189 ,2850.0)
                 .addSample(130.00031531871596 ,2875.0)
                 .addSample(143.2954288267012 ,3125.0)
//                 .addSample(138.61051699226041 ,2875.0)
                 .addSample(152.44115343470273 ,3125.0)
//                 .addSample(129.18332644943038 ,2875.0)

                .build();
        powerClose = new calculateShootPower.Builder()
                .addSample(28.19, 2125.0)
                .addSample(48.10, 2250.0)
                .addSample(65.52, 2250.0)
                .addSample(73.13, 2375.0)
                .build();

    }

    public double[] getPolynomial(){
        return powerFar.getPolynomial();
    }
    public void initPos(){
        hoodRight.setPosition(0);
        hoodLeft.setPosition(0);
    }

    public void startShoot(double power) {
        shooterMotorUp.setPower(power);
        shooterMotorDown.setPower(power);
    }

    public void setVelocity(double Velocity){
        if(Velocity == 0){

        }
        startShoot(pidfController.updateController(Velocity, getVelocity()));
    }

    public double shootWithAutoPower(double d, double offSet) {
        double power;

        if (d > 110) return power = (powerFar.calculateVelocity(d)) + offSet;
        return power = (powerClose.calculateVelocity(d)) + offSet;


    }
    public void stopShoot()
    {
        shooterMotorUp.setPower(0);
        shooterMotorDown.setPower(0);
    }

    public void changeAngle(double Rpos, double Lpos){
        hoodLeft.setPosition(Lpos);
        hoodRight.setPosition(Rpos);
    }
    public double getPosR(){
        return hoodRight.getPosition();
    }
    public double getPosL(){
        return hoodLeft.getPosition();
    }
    public double getPower(){
        return shooterMotorUp.getPower();
    }

    public double getVelocity(){
        double velocity = shooterMotorDown.getVelocity();
        return ((velocity * 60) / TICKS_PER_REVOLUTION);
    }

    public double getServoPositionWithDistance(double dis) {
        if (dis < 96 && dis > 0) {
            return Angle.LOW_DIS.getValue();
        }
        if (dis > 96 && dis < 144) {
            return Angle.HIGH_DIS.getValue();
        }
        return hoodRight.getPosition();
    }


//    public class ShootWithAutoPower implements Action{
//        public double distance;
//        public double volt;
//        public double offSet;
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            changeAngle(getServoPositionWithDistance(distance),
//                getServoPositionWithDistance(distance));
//            startShoot(shootWithAutoPower(distance,offSet));
//            return false;
//        }
//
//    }
//
//    public Action shootWithAutoPowerAction(double distance,double offSet){
//        ShootWithAutoPower returnAction = new ShootWithAutoPower();
//        returnAction.distance = distance;
//        returnAction.offSet = offSet;
//        return returnAction;
//    }

    public class ShootWithAutoPower implements Action {
        public double distance;
        public double offSet;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double servoPos = getServoPositionWithDistance(distance);
            changeAngle(servoPos, servoPos);

            double targetVel = shootWithAutoPower(distance, offSet);
            setVelocity(targetVel);

            return true;
        }
    }

    public Action shootWithAutoPowerAction(double distance, double offSet){
        ShootWithAutoPower returnAction = new ShootWithAutoPower();
        returnAction.distance = distance;
        returnAction.offSet = offSet;
        return returnAction;
    }

    public class StopShootAction implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            stopShoot();
            return false;
        }
    }
    public Action stopShootAction(){
        return new StopShootAction();
    }
}

