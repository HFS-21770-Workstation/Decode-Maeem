package org.firstinspires.ftc.teamcode.RobotSystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util.Enums.GoalColor;

public class CameraLocalizer {
    public MecanumDrive drive;
    public AprilTagWebCamSystem aprilTagWebCamSystem;
    private GoalColor goalColor;

    public double[] arrX = new double[5];
    public double[] arrY = new double[5];

    public CameraLocalizer(HardwareMap hardwareMap, Pose2d startPose, Telemetry telemetry, FtcDashboard dashboard, GoalColor goalColor){
        drive = new MecanumDrive(hardwareMap, startPose);
        aprilTagWebCamSystem = new AprilTagWebCamSystem(hardwareMap, telemetry, dashboard, startPose);
        for(int i = 0; i < arrX.length; i++){
            arrX[i] = startPose.position.x;
            arrY[i] = startPose.position.y;
        }
        this.goalColor = goalColor;
    }

    public Pose2d getPose(){
        Pose2d pos = aprilTagWebCamSystem.getRobotPoseByID(goalColor);
        if(pos == null){
            return drive.localizer.getPose();
        }
        else{
//            for(int i = arrX.length - 1; i > 0; i--){
//                arrX[i - 1] = arrX[i];
//                arrY[i - 1] = arrY[i];
//            }
//            arrX[arrX.length - 1] = pos.position.x;
//            arrY[arrY.length - 1] = pos.position.y;

            double x = pos.position.x;
            double y = pos.position.y;
            double heading = drive.localizer.getPose().heading.toDouble();

            Pose2d newPose = new Pose2d(x, y, heading);
            drive.localizer.setPose(newPose);
            return newPose;
        }
    }
    public double arrAvg(double[] arr){
        double avg = 0;
        for(int i = 0; i < arr.length; i++){
            avg += arr[i];
        }
        return avg / arr.length;
    }

    public void update(){
        aprilTagWebCamSystem.update(this.getPose());
        drive.updatePoseEstimate();

        aprilTagWebCamSystem.displayDetectionTelemtry(aprilTagWebCamSystem.getDetectionByID(goalColor));
    }
}
