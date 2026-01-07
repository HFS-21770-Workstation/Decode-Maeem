package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotSystems.Storage;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage1sensor;

@TeleOp
public class colorSensorTeleopTest extends OpMode {
//    Storage1sensor storage1sensor;

    Servo servoPusher1;
    Servo servoPusher2;
    Servo servoPusher3;

    public Servo[] servoPushers = new Servo[] {servoPusher1 , servoPusher2, servoPusher3};
    int index = 0;

    @Override
    public void init() {

//        storage1sensor = Storage1sensor.getInstance(hardwareMap, telemetry);
        servoPusher1 = hardwareMap.servo.get("servoPusher1");
        servoPusher2 = hardwareMap.servo.get("servoPusher2");
        servoPusher3 = hardwareMap.servo.get("servoPusher3");
    }

    @Override
    public void loop() {
//        storage1sensor.updateSlots();

//            if (gamepad1.aWasPressed()){
//                storage1sensor.rotate();
//                storage1sensor.updateSlots();
//
//
//        }
//        if (gamepad1.xWasPressed()){
//            storage1sensor.setOutPutArtifacts(Storage1sensor.Artifacts.GREEN);
//        }
//        if (gamepad1.bWasPressed()){
//            storage1sensor.setOutPutArtifacts((Storage1sensor.Artifacts.PURPLE));
//        }
//
//        telemetry.addData("slots", storage1sensor.getArtifactsStorage()[0]);
//        telemetry.addData("slots", storage1sensor.getArtifactsStorage()[1]);
//        telemetry.addData("slots", storage1sensor.getArtifactsStorage()[2]);
//        telemetry.addData("slots", storage1sensor.servoRevolver.getPosition());

        if(gamepad1.dpadUpWasPressed() && !gamepad1.a){
            if(index < 2){
                index++;
            }
        }
        if(gamepad1.dpadDownWasPressed() && !gamepad1.a){
            if(index > 0){
                index--;
            }
        }
        if(gamepad1.a){
            servoPushers[index].setPosition(1);
        }
        else{
            servoPushers[index].setPosition(0);
        }
    }
}
