package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PushServo {
    private Servo push0 = null;
    private Servo push1 = null;
    private Servo push2 = null;
    private ElapsedTime time = new ElapsedTime();

    public void initPushServos(HardwareMap hweM){
        push0 = hweM.get(Servo.class, "push0");
        push1 = hweM.get(Servo.class, "push1");
        push2 = hweM.get(Servo.class, "push2");
    }
    public void propel(int servoNum){
        if(servoNum == 0){
            push0.setPosition(0.325);
        }else if(servoNum == 1){
            push1.setPosition(0.6);
        }else if(servoNum == 2){
            push2.setPosition(0.67);
        }
    }
    public void retract(int servoNum) {
        if(servoNum == 0){
            push0.setPosition(0.67);
        }else if(servoNum == 1){
            push1.setPosition(0);
        }else if(servoNum == 2){
            push2.setPosition(0.3);
        }
    }
}
