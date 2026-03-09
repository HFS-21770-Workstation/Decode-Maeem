package org.firstinspires.ftc.teamcode.RobotSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Enums.Artifacts;
import org.firstinspires.ftc.teamcode.Util.RGB;

import java.util.Random;

@Config

public class Storage{
    Servo servoPusher1;
    Servo servoPusher2;
    Servo servoPusher3;

    Random rnd = new Random();
    ArtifactIDSystem artifactIDSystem;
    ColorSensor colorSensorA;
    ColorSensor colorSensorB;
    ColorSensor colorSensorC;
    public static int[] slot2RGB;
    private static Storage INSTANCE;

    public static boolean sort;
    public static boolean waitingForDown = false;

    final int ALPHA_FOR_DETECTION = 200;
//    RGB purpleRGB = new RGB(205.23727,251.49541 ,536.62102);
//    RGB greenRGB = new RGB(107.80042,534.36401 ,357.83555);
    RGB purpleRGB = new RGB(163,200,241);
    RGB greenRGB = new RGB(87,141,115);
    RGB nullRGb = new RGB(167,170,141);
    public Artifacts[] artifactsStorage = new Artifacts[] {Artifacts.NONE, Artifacts.NONE, Artifacts.NONE};
    public Servo[] servoPushers;
    ElapsedTime time = new ElapsedTime();
    int activePusherIndex = -1;
    final double[] UP_POS   = {1, 1, 0};
    final double[] DOWN_POS = {0, 0, 1};

    final double COLOR_THRESHOLD = 140;
    final int BRIGHTNESS_THRESHOLD = 180;

    public Storage(HardwareMap hardwareMap) {
        artifactIDSystem = new ArtifactIDSystem(hardwareMap);
        colorSensorA = hardwareMap.colorSensor.get("colorSensorA");
        colorSensorB = hardwareMap.colorSensor.get("colorSensorB");
        colorSensorC = hardwareMap.colorSensor.get("colorSensorC");
        servoPusher1 = hardwareMap.servo.get("servoPusher1");
        servoPusher2 = hardwareMap.servo.get("servoPusher2");
        servoPusher3 = hardwareMap.servo.get("servoPusher3");
        servoPushers = new Servo[] {servoPusher2 , servoPusher1, servoPusher3};
    }

    public static Storage getInstance(HardwareMap hardwareMap){
        if(INSTANCE == null)
        {
            INSTANCE = new Storage(hardwareMap);
        }
        return INSTANCE;
    }
    public void stop(){
        INSTANCE = null;
    }

    public void start(){
        artifactIDSystem.start();
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
//        artifactsStorage[0] = getColor(colorSensorA);
//        artifactsStorage[2] = getColor(colorSensorB);
//        artifactsStorage[1] = getColor(colorSensorC);
        artifactsStorage[0] = getColor(artifactIDSystem.getSlot1RGB());
        artifactsStorage[1] = getColor(artifactIDSystem.getSlot2RGB());
        artifactsStorage[2] = getColor(artifactIDSystem.getSlot3RGB());

        slot2RGB = artifactIDSystem.getSlot2RGB();
    }


//    public Artifacts getColor(ColorSensor colorSensor){
//        RGB rgb = new RGB(colorSensor.red(), colorSensor.green(), colorSensor.blue());
//
//        if(colorSensor.alpha() >= ALPHA_FOR_DETECTION){
//             if(rgb.dis(purpleRGB) > rgb.dis(greenRGB)){
//                 return Artifacts.GREEN;
//             }else{
//                 return Artifacts.PURPLE;
//             }
//        }else{
//            return Artifacts.NONE;
//        }
//    }

    public Artifacts getColor(int[] slotRgb){

        int r = slotRgb[0];
        int g = slotRgb[1];
        int b = slotRgb[2];

        RGB rgb = new RGB(r, g, b);

        int brightness = r + g + b;
        if(brightness < BRIGHTNESS_THRESHOLD){
            return Artifacts.NONE;
        }

        if(rgb.dis(nullRGb) < COLOR_THRESHOLD){
            return Artifacts.NONE;
        }


        double distPurple = rgb.dis(purpleRGB);
        double distGreen  = rgb.dis(greenRGB);



        if(distPurple > distGreen){
            return Artifacts.GREEN;
        }else{
            return Artifacts.PURPLE;
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

    public Action outPutBySort(Artifacts[] sort) {
        if (sort == null) return new SleepAction(0);

        return new SequentialAction(
                new SleepAction(0.5),
                shootWithRecovery(sort[0], 0),
                new SleepAction(1.5),
                initServoAction(),

                new SleepAction(0.5),
                shootWithRecovery(sort[1], 1),
                new SleepAction(1.5),
                initServoAction(),

                new SleepAction(0.5),
                shootWithRecovery(sort[2], 2),
                new SleepAction(1.5),
                initServoAction(),

                new SleepAction(0.5),
                clearRemainingArtifacts(),
                new SleepAction(1.5),
                initServoAction()
        );
    }

    private Action shootWithRecovery(Artifacts target, int step) {
        return (telemetryPacket) -> {
            updateColorSensors();
            int slotToLift = -1;

            for (int i = 0; i < 3; i++) {
                if (artifactsStorage[i] == target) {
                    slotToLift = i;
                    break;
                }
            }

            if (slotToLift == -1) {
                for (int i = 0; i < 3; i++) {
                    if (artifactsStorage[i] != Artifacts.NONE) {
                        slotToLift = i;
                        break;
                    }
                }
            }

            if (slotToLift == -1) {
                slotToLift = step;
            }

            servoPushers[slotToLift].setPosition(UP_POS[slotToLift]);
            artifactsStorage[slotToLift] = Artifacts.NONE;

            return false;
        };
    }

    private Action clearRemainingArtifacts() {
        return (telemetryPacket) -> {
            updateColorSensors();
            boolean foundSomething = false;
            for (int i = 0; i < 3; i++) {
                if (artifactsStorage[i] != Artifacts.NONE) {
                    servoPushers[i].setPosition(UP_POS[i]);
                    artifactsStorage[i] = Artifacts.NONE;
                    foundSomething = true;
                }
            }
            return false;
        };
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


