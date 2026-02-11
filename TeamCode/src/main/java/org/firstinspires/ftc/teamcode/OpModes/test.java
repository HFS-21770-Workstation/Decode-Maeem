package org.firstinspires.ftc.teamcode.OpModes;


import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotBetSystems.Intake;
import org.firstinspires.ftc.teamcode.RobotSystems.Drive;

@TeleOp(name = "Drive Test")
public class test extends OpMode {
//    Intake intake;
//    boolean opModeIsActive;

    DcMotor shooterMotorUp;
    DcMotor shooterMotorDown;

    Drive drive;
    @Override
    public void init() {
        drive = new Drive(hardwareMap, new Pose2d(0,0,0), FtcDashboard.getInstance());

        shooterMotorUp = hardwareMap.get(DcMotor.class,"shooterUp");
        shooterMotorDown = hardwareMap.get(DcMotor.class, "shooterDown");

//        shooterMotorUp = hardwareMap.dcMotor.get("shooterUp");
//        shooterMotorDown = hardwareMap.dcMotor.get("shooterDown");
//
//        shooterMotorUp.setDirection(DcMotorSimple.Direction.REVERSE);
//        shooterMotorDown.setDirection(DcMotorSimple.Direction.FORWARD);
//        intake = new Intake(hardwareMap);
    }
//    @Override
//    public void start(){
//        opModeIsActive = true;
//        new Thread(new Runnable() {
//            @Override
//            public void run() {
//                while (opModeIsActive){
//                    sleep(500);
//                    telemetry.addData("balls capacity:",  intake.UpdateBallCount());
//                    telemetry.addData("last:",  intake.LastBallDetection);
//                    telemetry.update();
//                }
//
//            }
//        });
//    }

    @Override
    public void loop() {
        drive.wheelControl(0, 0, 0);
        shooterMotorDown.setPower(1);
        shooterMotorUp.setPower(0);
//        telemetry.addData("balls capacity:",  intake.UpdateBallCount());
//        double SensorADis = intake.SensorA.getDistance(DistanceUnit.INCH);
//        double SensorBDis = intake.SensorB.getDistance(DistanceUnit.INCH);
//        double MinDis = (intake.Dis - SensorADis - SensorBDis) + 2.5;
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
//        opModeIsActive = false;

    }
}


