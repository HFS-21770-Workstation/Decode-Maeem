package org.firstinspires.ftc.teamcode.OpModes;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotBetSystems.Intake;

@TeleOp
public class test extends OpMode {
    Intake intake;
    boolean opModeIsActive;
    @Override
    public void init() {

    intake = new Intake(hardwareMap);
    }
    @Override
    public void start(){
        opModeIsActive = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive){
                    sleep(500);
                    telemetry.addData("balls capacity:",  intake.UpdateBallCount());
                }

            }
        }).start();
    }

    @Override
    public void loop() {


//        telemetry.addData("balls capacity:",  intake.UpdateBallCount());
//        SensorADis = SensorA.getDistance(DistanceUnit.INCH);
//        SensorBDis = SensorB.getDistance(DistanceUnit.INCH);
//        MinDis = (Dis - SensorADis - SensorBDis) + 2.5;
//        telemetry.addData("SensorA", SensorADis);
//        telemetry.addData("SensorB", SensorBDis);
//        telemetry.addData("MinDis", MinDis);

//        if (MinDis > 4.5 && MinDis < 5.3)
//        {
//            MaxBalls = 1;
//            telemetry.update();
//        }
//        else if (MinDis > 0 && MinDis < 0.5)
//        {
//            MaxBalls = 1;
//            telemetry.update();
//        }
//        else if (MinDis > 10 && MinDis < 10.6)
//        {
//            MaxBalls = 2;
//            telemetry.update();
//        }
//        else if (MinDis > 5.5 && MinDis < 6)
//        {
//            MaxBalls = 2;
//            telemetry.update();
//        }
//        else if (MinDis > 1.3 && MinDis < 1.6)
//        {
//            MaxBalls = 2;
//            telemetry.update();
//        }
//        else if (MinDis < 0)
//        {
//            MaxBalls = 0;
//            telemetry.update();
//        }
    }

    @Override
    public void stop(){
        opModeIsActive = false;
    }
}


