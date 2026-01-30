package org.firstinspires.ftc.teamcode.RobotSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.RGB;

import java.util.Random;


public class Storage{
    Servo servoPusher1;
    Servo servoPusher2;
    Servo servoPusher3;

    Random rnd = new Random();

    ColorSensor colorSensorA;
    ColorSensor colorSensorB;
    ColorSensor colorSensorC;

    static Storage storage;

    public static boolean sort;
    public static boolean waitingForDown = false;

    final int ALPHA_FOR_DETECTION = 200;
    RGB purpleRGB = new RGB(205.23727,251.49541 ,536.62102);
    RGB greenRGB = new RGB(107.80042,534.36401 ,357.83555);
    public Artifacts[] artifactsStorage = new Artifacts[] {Artifacts.NONE, Artifacts.NONE, Artifacts.NONE};
    public Servo[] servoPushers;
    ElapsedTime time = new ElapsedTime();
    int activePusherIndex = -1;
    final double[] UP_POS   = {1, 1, 0};
    final double[] DOWN_POS = {0, 0, 1};

    public Storage(HardwareMap hardwareMap) {
        colorSensorA = hardwareMap.colorSensor.get("colorSensorA");
        colorSensorB = hardwareMap.colorSensor.get("colorSensorB");
        colorSensorC = hardwareMap.colorSensor.get("colorSensorC");
        servoPusher1 = hardwareMap.servo.get("servoPusher1");
        servoPusher2 = hardwareMap.servo.get("servoPusher2");
        servoPusher3 = hardwareMap.servo.get("servoPusher3");
        servoPushers = new Servo[] {servoPusher2 , servoPusher1, servoPusher3};
    }





//    public void rotate(){
//        if(sort && colorSensorA.alpha() > ALPHA_FOR_DETECTION ){
//            for (int i = 0; i < 3; i++) {
//                if (artifactsStorage[i] == Artifacts.NONE) {
//                    updateSlots();
//                    servoRevolver.setPosition(slotsPos[i]);
//                    updateColorSensors();
//                }
//            }
//        }
//    }

    public void stopStartSorting(){
        sort = !sort;
    }

//    public void updateSlots(){
//        artifactsStorage[0] = getColor(colorSensor0);
//        artifactsStorage[1] = getColor(colorSensor1);
//        artifactsStorage[2] = getColor(colorSensor2);
//    }

    public void updateColorSensors(){
        artifactsStorage[0] = getColor(colorSensorA);
        artifactsStorage[2] = getColor(colorSensorB);
        artifactsStorage[1] = getColor(colorSensorC);
    }

    public static enum Artifacts {
        GREEN,
        PURPLE,
        NONE
    }

    public Artifacts getColor(ColorSensor colorSensor){
        RGB rgb = new RGB(colorSensor.red(), colorSensor.green(), colorSensor.blue());

        if(colorSensor.alpha() >= ALPHA_FOR_DETECTION){
             if(rgb.dis(purpleRGB) > rgb.dis(greenRGB)){
                 return Artifacts.GREEN;
             }else{
                 return Artifacts.PURPLE;
             }
        }else{
            return Artifacts.NONE;
        }
    }
    public void setOutPutArtifacts (Artifacts artifact){
        boolean foundBall = false;
        if (waitingForDown) return;
        for (int i = 0; i < 3; i++) {
            if (artifactsStorage[i] == artifact && !foundBall){
                time.reset();

                servoPushers[i].setPosition(UP_POS[i]);


                artifactsStorage[i] = Artifacts.NONE;
                activePusherIndex = i;
                foundBall = true;


                waitingForDown = true;
                return;

            }
//             return the other servos to position 0
//            else if (artifactsStorage[i] == Artifacts.NONE){
//                if(i == 2){
//                    servoPushers[i].setPosition(1);
//                }
//                else{
//                    servoPushers[i].setPosition(0);
//                }
//            }
        }

    }
    public void pusherUp(int index){
        if (waitingForDown) return;
        time.reset();

        servoPushers[index].setPosition(UP_POS[index]);

        artifactsStorage[index] = Artifacts.NONE;
        activePusherIndex = index;

        waitingForDown = true;
    }
    public void setOutPutArtifactsRandom(){
        if (waitingForDown) return;
        time.reset();

        int i = rnd.nextInt(3);

        servoPushers[i].setPosition(UP_POS[i]);

        artifactsStorage[i] = Artifacts.NONE;
        activePusherIndex = i;

        waitingForDown = true;
    }
    public void checkTime() {
        if (waitingForDown && time.milliseconds() >= 500) {
            downPusher();
            waitingForDown = false;
        }
    }

    public void downPusher() {
        if (activePusherIndex == -1) return;

        servoPushers[activePusherIndex].setPosition(DOWN_POS[activePusherIndex]);

        artifactsStorage[activePusherIndex] = Artifacts.NONE;
        activePusherIndex = -1;
    }
    public void initServos(){
        servoPusher1.setPosition(0);
        servoPusher2.setPosition(0);
        servoPusher3.setPosition(1);
    }

    public Artifacts[] getArtifactsStorage(){
        return artifactsStorage;
    }

    static public Storage getInstance(HardwareMap hardwareMap){
        if(storage == null){
            storage = new Storage(hardwareMap);
        }
       return storage;
    }

    public class OutputArtifactGreen implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            int i;
            boolean foundBall = false;
//            initServos();
            updateColorSensors();
            for(i = 0; i <= 2; i++){
                if(artifactsStorage[i] == Artifacts.GREEN && !foundBall){
                    servoPushers[i].setPosition(UP_POS[i]);
                    artifactsStorage[i] = Artifacts.NONE;
                    foundBall = true;
                    return false;

                }
            }
            return false;
        }
    }
    public boolean hasArtifacts() {
        for (int i = 0; i < 3 ; i++) {
            if (artifactsStorage[i] != Artifacts.NONE) return true;
        }
        return false;
    }
    public Action outPutArtifactGreen(){
        return new OutputArtifactGreen();
    }


    public class OutputArtifactPurple implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            int i;
            boolean foundBall = false;
//            initServos();
            updateColorSensors();
            for(i = 0; i <= 2; i++){
                if(artifactsStorage[i] == Artifacts.PURPLE && !foundBall){
                    servoPushers[i].setPosition(UP_POS[i]);
                    artifactsStorage[i] = Artifacts.NONE;
                    foundBall = true;
                    return false;
                }
            }

            return false;
        }
    }

    public Action outPutArtifactPurple(){
        return new OutputArtifactPurple();
    }

    public class OutputArtifactsByIndex implements Action{
        int i;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servoPushers[i].setPosition(UP_POS[i]);
            artifactsStorage[i] = Artifacts.NONE;
            return false;
        }
    }
    public Action outputArtifactsByIndex(int index){
        OutputArtifactsByIndex retAction = new OutputArtifactsByIndex();
        retAction.i = index;
        return retAction;
    }

    public class UpdateColorSensorsAction implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            updateColorSensors();
            return false;
        }
    }
    public Action updateColorSensorsAction(){
        return new UpdateColorSensorsAction();
    }






    public class InitServoAction implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            initServos();
            return false;
        }
    }
    public Action initServoAction(){
        return new InitServoAction();
    }
}
