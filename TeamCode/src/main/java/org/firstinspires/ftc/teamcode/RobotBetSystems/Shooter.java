package org.firstinspires.ftc.teamcode.RobotBetSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter{
    DcMotor shooter;

    private Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.dcMotor.get("shooter");

    }

    public void StartShoot(){
        shooter.setPower(1);
    }

    public void StopShoot(){
        shooter.setPower(0);
    }


}

