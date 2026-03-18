package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurretLocalization {
    private Servo turret = null;
    private AnalogInput servoFeedback;
    private final double posTwo = 0;
    private final double posOne = 0.5;
    private final double posZero = 1.0;
    /**
     * The V values for each of the Positions is what the Servo Feedback Matistic Gizmo says the servo is currently at for each of the three shooter positions.
     * SO, because we know what value each position is, and depending on from which other position the shooter is coming from, that is what the range is for.
     * The V range is for the boolean that states whether the shooter has reached the right position.
     * I am using a range because the Servo Feedback gizmo and the servo linkage can sometimes trip heavy and make lets say position 2 anywhere from 1.16 to 1.22 instead of dead-on 1.19 as I've written below.
     * Depending on how wobbly the Shooter servo is, the range is used to correct for that and tell the robot that the servo is in the correct position anyways.
     * This is to avoid the delay of getting exact with the value the shooter is currently at and wasting time.
     * TO EDIT THESE VALUES: Go to the main teleop, either Blue or Red and check what the telemetry prints for the Servo voltage at each of the three positions. If it is off for any of then fix it.
     */
    private final double posTwoV = 1.33;
    private final double posTwoVRange = 0.03;
    private final double posOneV = 1.71;
    private final double posOneVRange = 0.02;
    private final double posZeroV = 2.08;
    private final double posZeroVRange = 0.015;


    private static int position;
    public void initTurretLocalization(HardwareMap hwM){
        turret = hwM.get(Servo.class, "turretLocalization");
        servoFeedback = hwM.get(AnalogInput.class, "servoFeedback");
    }
    public void setTurretPos(double  pos){
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
