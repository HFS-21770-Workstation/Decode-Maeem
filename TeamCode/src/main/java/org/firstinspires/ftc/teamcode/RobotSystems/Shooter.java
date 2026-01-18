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
    public calculateShootPower calculatePower =  new calculateShootPower.Builder()
            .addSample(28.48, 13.14, 0.399)
            .addSample(61.94, 12.93, 0.499)
            .addSample(122, 12.9, 0.699)
            .addSample(21.688, 12.84, 0.449)
            .addSample(125.25, 13.66, 0.599)
            .addSample(68.62, 13.33, 0.499)
            .build();
    public Shooter(HardwareMap hardwareMap){
        shooterMotorUp = hardwareMap.dcMotor.get("shooterUp");
        shooterMotorDown = hardwareMap.dcMotor.get("shooterDown");

        shooterMotorUp.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorDown.setDirection(DcMotorSimple.Direction.FORWARD);
        servoRight = hardwareMap.servo.get("servoRight");
        servoLeft = hardwareMap.servo.get("servoLeft");

    }


    public void StartShoot(double power) {
        shooterMotorUp.setPower(power);
        shooterMotorDown.setPower(power);
    }
    public void shootWithAutoPower(double d, double v){
        shooterMotorUp.setPower(calculatePower.calculate(d,v));
        shooterMotorDown.setPower(calculatePower.calculate(d,v));
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
    public enum Angle{
        LOW_DIS(0),
        MID_DIS(0.225),
        HIGH_DIS(0.55);
        private double value;


        Angle(double value) {
            this.value = value;
        }

        public double getValue(){
            return this.value;
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

