package org.firstinspires.ftc.teamcode.RobotSystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.RGB;

@Config
public class Storage1sensor {
    boolean colorNotFound;
    public Servo servoRevolver;
    ColorSensor colorSensorA;
    public static Storage1sensor storage1sensor;
    boolean sort = true;

    final int ALPHA_FOR_DETECTION = 600;
    RGB purpleRGB = new RGB(205.23727,251.49541 ,536.62102);
    RGB greenRGB = new RGB(107.80042,534.36401 ,357.83555);
    public Artifacts[] artifactsStorage = new Artifacts[] {Artifacts.NONE, Artifacts.NONE, Artifacts.NONE};

    public static double[] slotsPos = new double[] {0,0.21,0.5};
    double currentSlot = 0;


    Telemetry telemetry;

    private Storage1sensor(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensorA = hardwareMap.colorSensor.get("colorSensorA");
        servoRevolver = hardwareMap.servo.get("servoRevolver");

        servoRevolver.setPosition(slotsPos[0]);
        this.telemetry = telemetry;
    }

    public void rotate(){
        if(sort && colorSensorA.alpha() > ALPHA_FOR_DETECTION ){

            for (int i = 0; i < 3; i++) {

                if (artifactsStorage[i] == Artifacts.NONE) {

                    servoRevolver.setPosition(slotsPos[i]);
                    currentSlot = slotsPos[i];

                    // FIXED – זמן לסרבו להגיע !!!
                    try { Thread.sleep(200); } catch (Exception ignored) {}

                    updateSlots();
                    return;
                }
            }
        }

        telemetry.update();

    }
    public void updateSlots(){
        for (int i = 0; i < 3; i++) {
            if (currentSlot  == slotsPos[i]){
                artifactsStorage[i] = getColor(colorSensorA);
            }
        }
    }

    public void stopStartSorting(){
        sort = !sort;
    }


    public enum Artifacts {
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

    public void move(){
        servoRevolver.setPosition(1);
    }
    static public Storage1sensor getInstance(HardwareMap hardwareMap, Telemetry telemetry){
        storage1sensor = new Storage1sensor(hardwareMap, telemetry);

        return storage1sensor;
    }

    public void setOutPutArtifacts(Artifacts artifact) {

        for (int i = 0; i < 3; i++) {

            if (artifactsStorage[i] == artifact) {

                servoRevolver.setPosition(slotsPos[i]);
                currentSlot = slotsPos[i];

                try { Thread.sleep(200); } catch (Exception ignored) {}

                artifactsStorage[i] = Artifacts.NONE;

                colorNotFound = false;
                return;
            }
        }

        // לא נמצא בכלל הצבע
        colorNotFound = true;
    }



    public Artifacts[] getArtifactsStorage(){
        return artifactsStorage;
    }
}
