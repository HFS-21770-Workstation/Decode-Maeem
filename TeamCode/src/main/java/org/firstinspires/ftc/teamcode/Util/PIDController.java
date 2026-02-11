//package org.firstinspires.ftc.teamcode.Util;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Config
//public class PIDController {
//
//    public static double Kp;
//    public static double Ki;
//    public static double Kd;
//    public static double lastError = 0;
//    public static double lastValue = 0;
//    public static double integralSum = 0;
//
//
//    public double max;
//    public double error;
//    public double output;
//
//    public boolean useTimer;
//    private ElapsedTime timer = new ElapsedTime();
//
//    public PIDController(double Kp, double Ki, double Kd, double max, boolean useTimer){
//        this.Kp = Kp;
//        this.Ki = Ki;
//        this.Kd = Kd;
//        this.max = max;
//        this.useTimer = useTimer;
//    }
//
//    public void updateValues(double Kp, double Ki, double Kd,double max){
//        this.Kp = Kp;
//        this.Ki = Ki;
//        this.Kd = Kd;
//        this.max = max;
//
//    }
//
//    public double getError(){
//        return error;
//    }
//
//    public double getOutput(){
//        return output;
//    }
//
//    public double updateController(double target, double current){
//        double error = target - current;
//
//        this.error = error; // Store it for getError()
//
//        double seconds = timer.seconds();
//        if (seconds <= 0) seconds = 0.001; // Prevent divide by zero
//
//        double errorRateOfChange = (error - lastError) / seconds;
//        if(this.useTimer){
//
//        }
//        lastError = error;
//        integralSum += error * seconds;
//        timer.reset();
//
//        double output = (error * Kp) + (errorRateOfChange * Kd) + (integralSum * Ki);
//        this.output = output;
////        return output;
//        return Math.max(-max, Math.min(max, output));
//    }
//
//    public void reset() {
//        integralSum = 0;
//        lastError = 0;
//        timer.reset();
//    }
//}

package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;
import java.util.Deque;

@Config
public class PIDController {

    public static double Kp;
    public static double Ki;
    public static double Kd;

    public static double lastError = 0;
    public static double integralSum = 0;

    public double max;
    public double error;
    public double output;

    public boolean useTimer;
    private final ElapsedTime timer = new ElapsedTime();

    // --- Integral window (only used when useTimer == true) ---
    private static final double INTEGRAL_WINDOW_SECONDS = 3.0;

    private static class IntegralSample {
        double value;
        double timestamp;

        IntegralSample(double value, double timestamp) {
            this.value = value;
            this.timestamp = timestamp;
        }
    }

    private final Deque<IntegralSample> integralWindow = new ArrayDeque<>();

    // --- Constructor ---
    public PIDController(double Kp, double Ki, double Kd, double max, boolean useTimer) {
        PIDController.Kp = Kp;
        PIDController.Ki = Ki;
        PIDController.Kd = Kd;
        this.max = max;
        this.useTimer = useTimer;
        timer.reset();
    }

    public void updateValues(double Kp, double Ki, double Kd, double max) {
        PIDController.Kp = Kp;
        PIDController.Ki = Ki;
        PIDController.Kd = Kd;
        this.max = max;
    }

    public double getError() {
        return error;
    }

    public double getOutput() {
        return output;
    }

    public double updateController(double target, double current) {

        error = target - current;

        double dt = 1.0;
        if (useTimer) {
            dt = timer.seconds();
            if (dt <= 0) dt = 0.001;
        }

        // --- Derivative ---
        double derivative;
        if (useTimer) {
            derivative = (error - lastError) / dt;
        } else {
            derivative = error - lastError;
        }

        // --- Integral ---
        if (useTimer) {
            double now = timer.seconds();
            double contribution = error * dt;
            integralWindow.addLast(new IntegralSample(contribution, now));

            while (!integralWindow.isEmpty() &&
                    now - integralWindow.peekFirst().timestamp > INTEGRAL_WINDOW_SECONDS) {
                integralWindow.removeFirst();
            }

            integralSum = 0;
            for (IntegralSample sample : integralWindow) {
                integralSum += sample.value;
            }
        } else {
            // Simple loop-based integral
            integralSum += error;
        }

        // --- PID output ---
        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        // Clamp output
        output = Math.max(-max, Math.min(max, output));

        lastError = error;

        if (useTimer) {
            timer.reset();
        }

        return output;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        integralWindow.clear();
        timer.reset();
    }
}
