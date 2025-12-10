package org.firstinspires.ftc.teamcode.RobotBetSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
//    static Intake intake;

//    DcMotor intakeMotor;
    public DistanceSensor SensorA;
    public DistanceSensor SensorB;
    double SensorADis;
    double SensorBDis;
    double MinDis;
    public final int Dis = 10;
    int BallCount = 0;
    public int LastBallDetection = 0;

    public Intake(HardwareMap hardwareMap){
//        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        SensorA = hardwareMap.get(DistanceSensor.class, "SensorA");
        SensorB = hardwareMap.get(DistanceSensor.class, "SensorB");
    }

//     public void startIntake(){
//            intakeMotor.setPower(1);
//     }
//     public void stopIntake(){
//            intakeMotor.setPower(0);
//     }
     public void RestartBallCount(){
        BallCount = 0;
     }
     public int UpdateBallCount(){
         SensorADis = SensorA.getDistance(DistanceUnit.INCH);
         SensorBDis = SensorB.getDistance(DistanceUnit.INCH);
         MinDis = (Dis - SensorADis - SensorBDis) + 2.5;

         if (MinDis < 0)
         {
             LastBallDetection = 0;
         }
         else if (MinDis > 0 && MinDis < 7)
         {
             if(LastBallDetection == 0){
                 BallCount += 1;
                 LastBallDetection = 1;
             }
         }
         else if (MinDis > 7)
         {
             if(LastBallDetection == 0){
                 BallCount += 2;
                 LastBallDetection = 2;
             }
             else if(LastBallDetection == 1){
                 BallCount += 1;
                 LastBallDetection = 2;
             }
         }


//         if (BallCount >= 3) new
//         {
//             BallCount = 0;
//         }
         return BallCount;
     }
//    static public Intake getInstance(HardwareMap hardwareMap){
//        if(intake == null){
//            intake = new Intake(hardwareMap);
//        }
//        return intake;
//    }
}
