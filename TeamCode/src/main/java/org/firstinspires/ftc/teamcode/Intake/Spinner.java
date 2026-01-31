package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner {
    final double speed = 1.0;
    private DcMotor spinnerLeft;
    private DcMotor spinnerRight;
    public void initSpinner(HardwareMap hMap) {
        spinnerLeft = hMap.get(DcMotorEx.class, "spinnerLeft");
        spinnerRight = hMap.get(DcMotorEx.class, "spinnerRight");
        spinnerLeft.setDirection(DcMotor.Direction.REVERSE);
        spinnerRight.setDirection(DcMotor.Direction.FORWARD);
    }
    public void startIntake(){
        spinnerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        spinnerRight.setDirection(DcMotorSimple.Direction.FORWARD);
        spinnerRight.setPower(speed);
        spinnerLeft.setPower(speed);
    }
    public void Intake(boolean square){
        if(square){
            spinnerLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            spinnerRight.setDirection(DcMotorSimple.Direction.FORWARD);
            spinnerLeft.setPower(speed);
            spinnerRight.setPower(speed);
        }else{
            spinnerLeft.setPower(0);
            spinnerRight.setPower(0);
        }
    }
    public double getSpinnerLeft(){
        return spinnerLeft.getPower();

    }
    public void reverse(){
        spinnerLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        spinnerRight.setDirection(DcMotorSimple.Direction.REVERSE);
        spinnerLeft.setPower(speed);
        spinnerRight.setPower(speed);
    }
    public double getSpinnerRight(){
        return spinnerRight.getPower();
    }
    public void stopIntake(){
        spinnerRight.setPower(0);
        spinnerLeft.setPower(0);
    }

}
