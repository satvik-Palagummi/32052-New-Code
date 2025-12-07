package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner {
    final double speed = 0.8;
    private DcMotor spinnerLeft;
    private DcMotor spinnerRight;
    public void initSpinner(HardwareMap hMap) {
        spinnerLeft = hMap.get(DcMotorEx.class, "spinnerLeft");
        spinnerRight = hMap.get(DcMotorEx.class, "spinnerRight");
        spinnerLeft.setDirection(DcMotor.Direction.FORWARD);
        spinnerRight.setDirection(DcMotor.Direction.FORWARD);
    }
    public void startIntake(){
        spinnerRight.setPower(speed);
    }
    public double getSpinnerLeft(){
        return spinnerLeft.getPower();

    }
    public double getSpinnerRight(){
        return spinnerRight.getPower();
    }
    public void stopIntake(){
        spinnerRight.setPower(0);
    }

}
