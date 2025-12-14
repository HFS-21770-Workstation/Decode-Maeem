package org.firstinspires.ftc.teamcode.RobotSystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.RGB;


public class Storage{
    Servo servoPusher1;
    Servo servoPusher2;
    Servo servoPusher3;
//    ColorSensor colorSensor0;
//    ColorSensor colorSensor1;
//    ColorSensor colorSensor2;

    ColorSensor colorSensorA;
    ColorSensor colorSensorB;
    ColorSensor colorSensorC;

    static Storage storage;

    public static boolean sort;

    final int ALPHA_FOR_DETECTION = 200;
    RGB purpleRGB = new RGB(205.23727,251.49541 ,536.62102);
    RGB greenRGB = new RGB(107.80042,534.36401 ,357.83555);
    public static Artifacts[] artifactsStorage = new Artifacts[] {Artifacts.NONE, Artifacts.NONE, Artifacts.NONE};
    public Servo[] servoPushers = new Servo[] {servoPusher1 , servoPusher2, servoPusher3};


    private Storage(HardwareMap hardwareMap) {
        colorSensorA = hardwareMap.colorSensor.get("colorSensorA");
        colorSensorB = hardwareMap.colorSensor.get("colorSensorB");
        colorSensorC = hardwareMap.colorSensor.get("colorSensorC");
        servoPusher1 = hardwareMap.servo.get("servoPusher1");
        servoPusher2 = hardwareMap.servo.get("servoPusher2");
        servoPusher3 = hardwareMap.servo.get("servoPusher3");

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
        artifactsStorage[1] = getColor(colorSensorB);
        artifactsStorage[2] = getColor(colorSensorC);
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
        for (int i = 0; i < 3; i++) {
            if (artifactsStorage[i] == artifact){
                servoPushers[i].setPosition(1);
            }

        }

    }

    static public Storage getInstance(HardwareMap hardwareMap){
        if(storage == null){
            storage = new Storage(hardwareMap);
        }
        return storage;
    }

}
