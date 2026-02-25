package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretLocalization {
    private Servo turret = null;
    private AnalogInput servoFeedback;
    private final double posTwo = 0;
    private final double posOne = 0.47;
    private final double posZero = 1.0;
    private final double posTwoV = 1.23;
    private final double posTwoVRange = 0.03;

    private final double posOneV = 1.78;
    private final double posOneVRange = 0.05;
    private final double posZeroV = 2.13;
    private final double posZeroVRange = 0.01;


    private static int position;
    public void initTurretLocalization(HardwareMap hwM){
        turret = hwM.get(Servo.class, "turretLocalization");
        servoFeedback = hwM.get(AnalogInput.class, "servoFeedback");
    }
    public void setTurretPos(double pos){
        turret.setPosition(pos);
    }
    public void moveToLeft(boolean leftDPad){if(leftDPad){setPos(0);}}
    public void moveToMiddle(boolean upDPad){if(upDPad){setPos(1);}}
    public void moveToRight(boolean rightDPad){if(rightDPad){setPos(2);}}
    public int getTurretPos(){
        return position;
    }
    public double getServoFeedback(){
        return servoFeedback.getVoltage();
    }
    public void setPos(int num){
        position = num;
        if(position == 0){
            setTurretPos(posZero);
        }
        if(position == 1){
            setTurretPos(posOne);
        }
        if(position == 2){
            setTurretPos(posTwo);
        }
    }
    public boolean getTurretArrived(int pos){
        double servo = getServoFeedback();
        if(pos == 0){
            if(servo< posZeroV+posZeroVRange && servo > posZeroV-posZeroVRange){
                return true;
            }
        }
        if(pos == 1){
            if(servo< posOneV+posOneVRange && servo > posOneV-posOneVRange){
                return true;
            }
        }
        if(pos == 2){
            if(servo< posTwoV+posTwoVRange && servo > posTwoV-posTwoVRange){
                return true;
            }
        }
        return false;
    }


}
