package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.IntakeOld;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Turret;


@TeleOp(name = "Full Robot Test")
public class driveOpMode extends OpMode {
    Storage storage;
    Shooter shooter;
    Turret turret;
    final double pos = 0.1;
    final double power = 0.1;
    VoltageSensor voltageSensor;


    Drive drive;


//
//    Servo servoPusher1;
//    Servo servoPusher2;
//    Servo servoPusher3;
//
//    public Servo[] servoPushers;
    int index = 0;

    IntakeOld intake;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        storage = new Storage(hardwareMap);

        shooter = new Shooter(hardwareMap);
        shooter.ChangeAngle(0,0);
        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance());
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        // telemetry = FtcDashboard.getInstance().getTelemetry();

        intake = IntakeOld.getInstance(hardwareMap);

        storage.initServos();
    }

    @Override
    public void loop() {
        drive.wheelControlRobot(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        storage.checkTime();

        if (!storage.waitingForDown) {
            storage.updateColorSensors();
        }
        if(gamepad2.xWasPressed()){
            storage.setOutPutArtifacts(Storage.Artifacts.PURPLE);


        }
        if(gamepad2.bWasPressed()){
            storage.setOutPutArtifacts(Storage.Artifacts.GREEN);

        }
        telemetry.addData("slots1", storage.getArtifactsStorage()[0]);
        telemetry.addData("slots2", storage.getArtifactsStorage()[1]);
        telemetry.addData("slots3", storage.getArtifactsStorage()[2]);


//        if(gamepad1.dpadUpWasPressed() && !gamepad1.a){
//            if(index < 2){
//                index++;
//            }
//        }
//        if(gamepad1.dpadDownWasPressed() && !gamepad1.a){
//            if(index > 0){
//                index--;
//            }
//        }
//        if(gamepad1.a){
//            if(index == 2){
//                servoPushers[index].setPosition(0);
//            }
//            else{
//                servoPushers[index].setPosition(1);
//            }
//        }
//        else{
//            if(index == 2){
//                servoPushers[index].setPosition(1);
//            }
//            else{
//                servoPushers[index].setPosition(0);
//            }
//        }ssss



        if(gamepad1.back){
            drive.resetIMU();
        }

        if(gamepad1.x){
            intake.startIntake(0.6);
        }
        else if (gamepad1.b) {
            intake.startIntake(-0.6);
        }
        else if(gamepad1.y){
            intake.stopIntake();
        }
        intake.updateIntake();

        turret.rotateWithJoyStick(gamepad2.left_stick_x / 3);

//        shooter.StartShoot(gamepad2.right_stick_y);
        if (gamepad2.dpadUpWasPressed()) {
            shooter.ChangeAngle(shooter.GetPosR() + pos, shooter.GetPosL() + pos);
        }
        if (gamepad2.dpadDownWasPressed()) {
            shooter.ChangeAngle(shooter.GetPosR() - pos, shooter.GetPosL() - pos);
        }
        if(gamepad2.yWasPressed()){
            shooter.StartShoot(shooter.GetPower() + power);
        }
        if(gamepad2.aWasPressed()) {
            shooter.StartShoot(shooter.GetPower() - power);
        }
//        telemetry.addData("Index:", index);
//        telemetry.addData("Power:", shooter.GetPower());
//        telemetry.addData("PosL:", shooter.GetPosL());
//        telemetry.addData("PosR:", shooter.GetPosR());
//        telemetry.addData("Volt:", voltageSensor.getVoltage());
    }

    @Override
    public void stop(){
        shooter.StopShoot();
    }

}
