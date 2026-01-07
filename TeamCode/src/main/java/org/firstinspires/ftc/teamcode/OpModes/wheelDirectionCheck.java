package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class wheelDirectionCheck extends OpMode {

    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;

    @Override
    public void init() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            frontLeft.setPower(0.5);
        }
        else{
            frontLeft.setPower(0);
        }


        if(gamepad1.b){
            frontRight.setPower(0.5);
        }
        else{
            frontRight.setPower(0);
        }


        if(gamepad1.y){
            backLeft.setPower(0.5);
        }
        else{
            backLeft.setPower(0);
        }


        if(gamepad1.x){
            backRight.setPower(0.5);
        }
        else{
            backRight.setPower(0);
        }
    }

    public void wheelControlRobot(double x, double y, double rx){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }
}
