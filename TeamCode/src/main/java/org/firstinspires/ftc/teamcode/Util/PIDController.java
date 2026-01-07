package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDController {

    public double Kp;
    public double Ki;
    public double Kd;
    public double lastError = 0;
    public double lastValue = 0;
    public double integralSum = 0;

    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double PIDcontroller(double target, double current){

        double error = target - current;
        double errorRateOfChange = (error - lastError) / timer.seconds();
        lastError = error;
        integralSum += error * timer.seconds();
        timer.reset();
        double output = (error * Kp) + (errorRateOfChange * Kd) + (integralSum * Ki);
        return output;
    }
}
