package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeOld {
    static IntakeOld intake;

    DcMotor intakeMotor;
    double power = 0;

    private IntakeOld(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
    }
    public void startIntake(double newPower){
        power = newPower;
    }

    public void stopIntake(){
        power = 0;
    }

    public void updateIntake(){
        intakeMotor.setPower(power);
    }

    static public IntakeOld getInstance(HardwareMap hardwareMap){
        if(intake == null){
            intake = new IntakeOld(hardwareMap);
        }
        return intake;
    }


}
