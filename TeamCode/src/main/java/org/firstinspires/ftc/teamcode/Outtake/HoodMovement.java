package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodMovement {
    private Servo hood;
    public void initHoodServo(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, "hood");
    }
    public void adjustHoodUp(boolean farShot){
        if(farShot){
            hood.setPosition(hood.getPosition()+10);
        }
    }
    public void adjustHoodDown(boolean farShot){
        if(farShot){
            hood.setPosition(hood.getPosition()-10);
        }
    }
}
