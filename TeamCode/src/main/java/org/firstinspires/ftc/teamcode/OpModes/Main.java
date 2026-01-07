package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotSystems.Drive;
import org.firstinspires.ftc.teamcode.RobotSystems.IntakeOld;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;
import org.firstinspires.ftc.teamcode.RobotSystems.Storage;

public class Main extends OpMode {

    Drive drive;
    IntakeOld intakeOld;
    Shooter shooter;
    Storage storage;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        intakeOld = IntakeOld.getInstance(hardwareMap);
        shooter = new Shooter(hardwareMap);
        storage = new Storage(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.back) {
            drive.resetIMU();
        }
        drive.wheelControl(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        if (gamepad1.right_trigger == 1){
            intakeOld.startIntake(0.9);
        }

        else{
            intakeOld.stopIntake();
        }


        shooter.StartShoot(gamepad1.left_trigger);



        if (gamepad1.a){
            storage.setOutPutArtifacts(Storage.Artifacts.PURPLE);
        }

        if (gamepad1.b){
            storage.setOutPutArtifacts(Storage.Artifacts.PURPLE);
        }




    }

}
