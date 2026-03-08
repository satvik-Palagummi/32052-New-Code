package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodMovement {
    private Servo hood;
    public void initHoodServo(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, "hood");
    }
    public void adjustHoodUp(){
        hood.setPosition(hood.getPosition()-0.05);
    }
    public void setHood(double pos){
        hood.setPosition(pos);
    }
    public void adjustHoodDown(){
        hood.setPosition(hood.getPosition()+0.055);
    }
    public double hoodPos(){
        return hood.getPosition();
    }
    public void setHoodZero(){
        hood.setPosition(0);
    }
    public void setHoodOne(){
        hood.setPosition(0.9);
    }

}
