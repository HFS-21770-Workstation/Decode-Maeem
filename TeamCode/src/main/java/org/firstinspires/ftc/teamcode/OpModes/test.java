package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class test extends OpMode {
    DistanceSensor SensorA;
    DistanceSensor SensorB;
    double SensorADis;
    double SensorBDis;
    double MinDis;
    final int Dis = 10;

    @Override
    public void init() {
        SensorA = hardwareMap.get(DistanceSensor.class, "SensorA");
        SensorB = hardwareMap.get(DistanceSensor.class, "SensorB");

    }

    @Override
    public void loop() {
        SensorADis = SensorA.getDistance(DistanceUnit.INCH);
        SensorBDis = SensorB.getDistance(DistanceUnit.INCH);
        MinDis = (Dis - SensorADis - SensorBDis) + 2.5;
        telemetry.addData("SensorA", SensorADis);
        telemetry.addData("SensorB", SensorBDis);
        telemetry.addData("MinDis", MinDis);

    }
}
