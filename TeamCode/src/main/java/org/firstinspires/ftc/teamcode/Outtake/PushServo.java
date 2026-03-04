package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PushServo {
    private Servo push0 = null;
    private Servo push1 = null;
    private Servo push2 = null;

    public void initPushServos(HardwareMap hweM){
        push0 = hweM.get(Servo.class, "push0");
        push1 = hweM.get(Servo.class, "push1");
        push2 = hweM.get(Servo.class, "push2");
    }

    /**
     * @param servoNum is the position for which the code will either move the servo up to launch an artifact or bring the servo down
     */
    public void propel(int servoNum){
        if(servoNum == 0){
            push0.setPosition(0.767);
        }else if(servoNum == 1){
            push1.setPosition(0);
        }else if(servoNum == 2){
            push2.setPosition(0.4);
        }
    }
    public void propelScan(int servoNum){
        if(servoNum == 0){
            push0.setPosition(0.5);
        }else if(servoNum == 1){
            push1.setPosition(0.55);
        }else if(servoNum == 2){
            push2.setPosition(0.50);
        }
    }
    public void retract(int servoNum) {
        if(servoNum == 0){
            push0.setPosition(0.6);
        }else if(servoNum == 1){
            push1.setPosition(0.95);
        }else if(servoNum == 2){
            push2.setPosition(0.77);
        }
    }
}
