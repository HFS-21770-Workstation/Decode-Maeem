package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;
import java.util.Deque;

@Config
public class PIDFController {

    public static double Kp;
    public static double Ki;
    public static double Kd;
    public static double Kf;

    public static double lastError = 0;
    public static double integralSum = 0;

    public double max;
    public double min;
    public double error;
    public double output;

    public boolean useTimer;
    private final ElapsedTime timer = new ElapsedTime();

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

    public PIDFController(double Kp, double Ki, double Kd, double Kf, double max, double min, boolean useTimer) {
        PIDFController.Kp = Kp;
        PIDFController.Ki = Ki;
        PIDFController.Kd = Kd;
        PIDFController.Kf = Kf;
        this.max = max;
        this.min = min;
        this.useTimer = useTimer;
        timer.reset();
    }

    public void updateValues(double Kp, double Ki, double Kd, double Kf, double max, double min) {
        PIDFController.Kp = Kp;
        PIDFController.Ki = Ki;
        PIDFController.Kd = Kd;
        PIDFController.Kf = Kf;
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

        double derivative;
        if (useTimer) {
            derivative = (error - lastError) / dt;
        } else {
            derivative = error - lastError;
        }

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
            integralSum += error;
        }

        double feedforward = Kf * target;

        output = (Kp * error) +
                (Ki * integralSum) +
                (Kd * derivative) +
                feedforward;

        output = Math.max(min, Math.min(max, output));

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
