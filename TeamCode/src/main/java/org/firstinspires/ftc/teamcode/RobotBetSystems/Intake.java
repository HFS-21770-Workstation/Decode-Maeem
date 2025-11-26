package org.firstinspires.ftc.teamcode.RobotBetSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    DcMotor intake;

    private Intake(HardwareMap hardwareMap){
        intake = hardwareMap.dcMotor.get("intake");
    }

     public void startIntake(){
            intake.setPower(1);
     }
     public void stopIntake(){
            intake.setPower(0);
     }
}
