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
    public String getColorDataString(){

        return "red = [" + colorSensor.red() + "] green = [ "+ colorSensor.green() + "] blue = [ "+ colorSensor.blue() + "]";
    }


}