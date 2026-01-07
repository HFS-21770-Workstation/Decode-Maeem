package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RobotSystems.AprilTagWebCamSystem;
import org.firstinspires.ftc.teamcode.RobotSystems.Shooter;

@TeleOp
@Config
public class ShootTest extends OpMode {
    Shooter shooter;
//    Turret turret;
    AprilTagWebCamSystem aprilTagWebCamSystem;
    final double pos = 0.1;
    final double power = 0.1;
    VoltageSensor voltageSensor;
    Servo servoPusher1;
    boolean startShot = false;
    @Override
    public void init() {
        aprilTagWebCamSystem = new AprilTagWebCamSystem(hardwareMap, telemetry, FtcDashboard.getInstance());
        shooter =  new Shooter(hardwareMap);
        shooter.ChangeAngle(0,0);
//        turret = new Turret(hardwareMap, telemetry, FtcDashboard.getInstance());
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry = FtcDashboard.getInstance().getTelemetry();

        servoPusher1 = hardwareMap.servo.get("servoPusher3");
        servoPusher1.setPosition(1);

    }

    @Override
    public void loop() {
        aprilTagWebCamSystem.update();
        double distance = aprilTagWebCamSystem.getDistanceFromTag(24);
        if (gamepad1.bWasPressed()) {
            startShot = !startShot;
            if (startShot) {
                shooter.StartShoot(shooter.GetPower());
            } else {
                shooter.StartShoot(-0.3);
            }
        }

//        double distance = -1;
        if(distance == -1){
            shooter.ChangeAngle(shooter.GetPosR(),shooter.GetPosL());
        }
        shooter.ChangeAngle(shooter.getServoPositionWithDistance(distance),
                shooter.getServoPositionWithDistance(distance));
        if (startShot) {
            if (gamepad1.yWasPressed()) {
                shooter.StartShoot(shooter.GetPower() + 0.025);
            }
            if (gamepad1.aWasPressed()) {
                shooter.StartShoot(shooter.GetPower() - 0.025);
            }
        }
        if(gamepad1.dpad_up){

        }
        if(gamepad1.x){
            servoPusher1.setPosition(0);
        }
        else{
            servoPusher1.setPosition(1);
        }
        telemetry.addData("Power:", shooter.GetPower());
//        telemetry.addData("PosL:", shooter.GetPosL());
//        telemetry.addData("PosR:", shooter.GetPosR());
        telemetry.addData("Volt:", voltageSensor.getVoltage());
        String distanceStr = "Distance: " + String.format("%.3f", distance);
        telemetry.addLine(distanceStr);
        telemetry.addData("start shoot", startShot);
    }
    @Override
    public void stop(){
        aprilTagWebCamSystem.stop();
    }
}