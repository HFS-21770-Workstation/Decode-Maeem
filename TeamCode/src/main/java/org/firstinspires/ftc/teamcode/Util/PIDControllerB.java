package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PIDControllerB {

    public static double Kp = 0.065;
    public static double Ki = 0.00005;
    public static double Kd = 0.001;
    public static double lastError = 0;
    public static double lastValue = 0;
    public static double integralSum = 0;

    public double error;
    public double output;

    private ElapsedTime timer = new ElapsedTime();

    public PIDControllerB(){

    }

    public double getError(){
        return error;
    }

    public double getOutput(){
        return output;
    }

    public double PIDcontroller(double target, double current, double Kp, double Ki, double Kd){
        double error = target - current;

        // --- ADD THIS SHORTEST PATH LOGIC ---
//        while (error > 180) error -= 360;
//        while (error < -180) error += 360;
        // ------------------------------------

        this.error = error; // Store it for getError()

        double seconds = timer.seconds();
        if (seconds <= 0) seconds = 0.001; // Prevent divide by zero

        double errorRateOfChange = (error - lastError) / seconds;
        lastError = error;
        integralSum += error * seconds;
        timer.reset();

        double output = (error * Kp) + (errorRateOfChange * Kd) + (integralSum * Ki);
        this.output = output;
        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}
