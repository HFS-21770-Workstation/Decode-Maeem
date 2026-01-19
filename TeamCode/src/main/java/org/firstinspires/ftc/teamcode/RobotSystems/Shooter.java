package org.firstinspires.ftc.teamcode.RobotSystems;

import android.sax.StartElementListener;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.calculateShootPower;

public class Shooter {
    DcMotor shooterMotorUp;
    DcMotor shooterMotorDown;
    static Shooter shooter;
    Servo servoRight;
    Servo servoLeft;
    calculateShootPower powerClose;
    calculateShootPower powerMid;
    calculateShootPower powerFar;

    public Shooter(HardwareMap hardwareMap){
        shooterMotorUp = hardwareMap.dcMotor.get("shooterUp");
        shooterMotorDown = hardwareMap.dcMotor.get("shooterDown");

        shooterMotorUp.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorDown.setDirection(DcMotorSimple.Direction.FORWARD);
        servoRight = hardwareMap.servo.get("servoRight");
        servoLeft = hardwareMap.servo.get("servoLeft");

    }

    public void startCal(){
         powerClose = new calculateShootPower.Builder()
                .addSample(37.19, 12.59, 0.449)
                .addSample(45.16, 12.45, 0.449)
                .addSample(58.55, 12.445, 0.449)
                .addSample(39.0, 13.4, 0.39)
                .addSample(46.9, 13.1, 0.4)
                .addSample(47.6, 12.8, 0.44)
                .build();

         powerMid = new calculateShootPower.Builder()
                .addSample(71.8, 12.7, 0.49)
                .addSample(82.34, 12.24, 0.54)
                .addSample(83.61, 12.41, 0.54)
                .addSample(85.0, 12.69, 0.549)
                .addSample(89.1, 12.42, 0.54)
                .addSample(90.0, 12.7, 0.549)
                .build();

         powerFar = new calculateShootPower.Builder()
                .addSample(112.5, 12.5, 0.849)
                .addSample(119.0, 12.5, 0.79)
                .addSample(122.0, 13.5, 0.79)
                .addSample(127.7, 12.6, 0.89)
                .addSample(127.8, 13.0, 0.74)
                .addSample(128.01, 14.067, 0.7)
                .build();


    }
    public void initPos(){
        servoRight.setPosition(0);
        servoLeft.setPosition(0);
    }

    public void StartShoot(double power) {
        shooterMotorUp.setPower(power);
        shooterMotorDown.setPower(power);
    }
    public void shootWithAutoPower(double d, double v){
        if (d < 48 && d > 0) {
            powerClose.calculate(d, v);
        }
        if (d >= 48 && d < 96) {
            powerMid.calculate(d, v);
        }
        if (d > 96 && d < 144) {
            powerFar.calculate(d, v);        }
    }
    public void StopShoot()
    {
        shooterMotorUp.setPower(0);
        shooterMotorDown.setPower(0);
    }

    public void ChangeAngle(double Rpos, double Lpos){
        servoLeft.setPosition(Lpos);
        servoRight.setPosition(Rpos);
    }
    public double GetPosR(){
        return servoLeft.getPosition();

    }
    public double GetPosL(){
        return servoLeft.getPosition();

    }
    public double GetPower(){
        return shooterMotorUp.getPower();
    }
    public static enum Angle{
        LOW_DIS(0),
        MID_DIS(0.225),
        HIGH_DIS(0.4);
        private double value;


        Angle(double value) {
            this.value = value;
        }

        public double getValue(){
            return this.value;
        }

        public Angle next() {
            Angle[] values = Angle.values();
            return values[(this.ordinal() + 1) % values.length];
        }
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
        return servoRight.getPosition();
    }
}

