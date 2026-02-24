package org.firstinspires.ftc.teamcode.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Colorsensor {
   ColorSensor colorSensor1;
   ColorSensor colorSensor2;
   ColorSensor colorSensor3;
   int[] colorVals;

    public Colorsensor(){
        colorVals = new int[3];
    }


    public void initColorSensor(HardwareMap hardwareMap){
        colorSensor1 = hardwareMap.get(ColorSensor.class, "color_sensor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color_sensor2");
        colorSensor3 = hardwareMap.get(ColorSensor.class, "color_sensor3");
    }
    public int getColorVal1(){
        if(getBlue1()<100 || getGreen2()<100){
            return -1;
        }else if(getBlue1()>getGreen1()) {
            return 1;
        }
        return 0;
    }
    public int getColorVal2(){
        if(getBlue2()<100 || getGreen2()<100){
            return -1;
        }else if(getBlue2()>getGreen2()) {
            return 1;
        }
        return 0;
    }
    public int getColorVal3(){
        if(getBlue3()<100 || getGreen3()<100){
            return -1;
        }else if(getBlue3()>getGreen3()) {
            return 1;
        }
        return 0;
    }
    public int[] getCurrent(){
        colorVals[0] = getColorVal1();
        colorVals[1] = getColorVal2();
        colorVals[2] = getColorVal3();
        return colorVals;
    }
    public int getRed(){return colorSensor1.red();}
    public int getGreen1(){return colorSensor1.green();}
    public int getBlue1(){
        return colorSensor1.blue();
    }
    public int getGreen2(){
        return colorSensor2.green();
    }
    public int getBlue2(){
        return colorSensor2.blue();
    }
    public int getGreen3(){
        return colorSensor3.green();
    }
    public int getBlue3(){
        return colorSensor3.blue();
    }

}