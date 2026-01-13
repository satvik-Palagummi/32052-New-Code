package org.firstinspires.ftc.teamcode.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Colorsensor {
   ColorSensor colorSensor;

    public Colorsensor(){
    }


    public void initColorSensor(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

    }
    public int getColorVal(){
        if(getBlue()>getGreen()) {
            return 1;
        }
        return 0;
    }
    public int getRed(){
        return colorSensor.red();
    }
    public int getGreen(){
        return colorSensor.green();
    }
    public int getBlue(){
        return colorSensor.blue();
    }

}