package org.firstinspires.ftc.teamcode.RobotSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeOld{
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
    public double getPower(){
        return intakeMotor.getPower();
    }

    public class StartIntake implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotor.setPower(1);
            return false;
        }
    }
    public class StopIntake implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotor.setPower(0);
            return false;
        }
    }

    public Action startIntakeAction(){
        return new StartIntake();
    }


    public Action stopIntakeAction(){
        return new StopIntake();
    }
}
