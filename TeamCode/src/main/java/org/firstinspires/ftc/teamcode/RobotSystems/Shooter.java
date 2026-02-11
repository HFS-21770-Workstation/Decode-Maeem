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
import org.firstinspires.ftc.teamcode.Util.calculateShootPower;
import org.firstinspires.ftc.teamcode.Util.Enums.Angle;

public class Shooter {


    DcMotorEx shooterMotorUp;
    DcMotorEx shooterMotorDown;


    private final double TICKS_PER_REVOLUTION = 28 ;
    private static Shooter INSTANCE;
    Servo hoodRight;
    Servo hoodLeft;
    calculateShootPower powerClose;
    calculateShootPower powerMid;
    calculateShootPower powerFar;

    public double upCurrent = 0;
    public double downCurrent = 0;

    private Shooter(HardwareMap hardwareMap){
        shooterMotorUp = hardwareMap.get(DcMotorEx.class,"shooterUp");
        shooterMotorDown = hardwareMap.get(DcMotorEx.class, "shooterDown");


        shooterMotorUp.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorDown.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotorDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hoodRight = hardwareMap.servo.get("hoodRight");
        hoodLeft = hardwareMap.servo.get("hoodLeft");
        hoodRight.setDirection(Servo.Direction.FORWARD);
        hoodLeft.setDirection(Servo.Direction.FORWARD);

    }

    public static Shooter getInstance(HardwareMap hardwareMap){
        if(INSTANCE == null){
            INSTANCE = new Shooter(hardwareMap);
        }

        return INSTANCE;
    }
    public static void stop(){
        INSTANCE = null;
    }
    public void startCal(){
         powerFar = new calculateShootPower.Builder()
                //.addSample(119.5393881984405, 12.06,0.6498001037629322)
                 .addSample(119.28944643574597,13.708,0.5599230933561204)

                .addSample(132.38881707664072, 11.59, 0.6798913541062654)
                //.addSample(121.48789897090964, 13.13, 0.5598620563371685)
                 .addSample(122.6043298945934,12.69,0.5999328592791529)
                .addSample(113.03211179499166, 12.9, 0.5599536118655964)
                .addSample(117.97265841466928, 12.45, 0.6399121066927091)
                .addSample(114.29950235012214, 12.35, 0.5999328592791529)
                .build();

//         powerMid = new calculateShootPower.Builder()
//                .addSample(71.8, 12.7, 0.49)
//                .addSample(82.34, 12.24, 0.54)
//                .addSample(83.61, 12.41, 0.54)
//                .addSample(85.0, 12.69, 0.549)
//                .addSample(89.1, 12.42, 0.54)
//                .addSample(90.0, 12.7, 0.549)
//                .build();
//
//         powerFar = new calculateShootPower.Builder()
//                .addSample(112.5, 12.5, 0.849)
//                .addSample(119.0, 12.5, 0.79)
//                .addSample(122.0, 13.5, 0.79)
//                .addSample(127.7, 12.6, 0.89)
//                .addSample(127.8, 13.0, 0.74)
//                .addSample(128.01, 14.067, 0.7)
//                .build();

    }
    public void initPos(){
        hoodRight.setPosition(0);
        hoodLeft.setPosition(0);
    }

    public void startShoot(double power) {
        upCurrent = shooterMotorUp.getCurrent(CurrentUnit.AMPS);
        downCurrent = shooterMotorDown.getCurrent(CurrentUnit.AMPS);

        shooterMotorUp.setPower(power);
        shooterMotorDown.setPower(power);
    }
    public double shootWithAutoPower(double d, double v, double offSet) {
        double power;

        if (d > 96 && d < 144) {
            return power = (powerFar.calculate(d, v)) + offSet;
        }
        return 0;
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
        if (dis < 48 && dis > 0) {
            return Angle.LOW_DIS.getValue();
        }
        if (dis >= 48 && dis < 96) {
            return Angle.MID_DIS.getValue();
        }
        if (dis > 96 && dis < 144) {
            return Angle.HIGH_DIS.getValue();
        }
        return hoodRight.getPosition();
    }

    public class ShootWithAutoPower implements Action{
        public double distance;
        public double volt;
        public double offSet;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            changeAngle(getServoPositionWithDistance(distance),
                getServoPositionWithDistance(distance));
            startShoot(shootWithAutoPower(distance, volt, offSet));
            return false;
        }

    }

    public Action shootWithAutoPowerAction(double distance, double volt, double offSet){
        ShootWithAutoPower returnAction = new ShootWithAutoPower();
        returnAction.distance = distance;
        returnAction.volt = volt;
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

