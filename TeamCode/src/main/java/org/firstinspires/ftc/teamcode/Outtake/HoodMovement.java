package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HoodMovement {
    private DcMotorEx hoodMotor;
    
    // PID coefficients for hood positioning if needed
    // private static final double kP = 5.0;

    public void initHood(HardwareMap hardwareMap){
        hoodMotor = hardwareMap.get(DcMotorEx.class, "hoodMotor");
        hoodMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hoodMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoodMotor.setTargetPosition(0);
        hoodMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodMotor.setPower(1.0);
    }

    public void setHoodPosition(int position){
        hoodMotor.setTargetPosition(position);
        hoodMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodMotor.setPower(1.0);
    }

    public int getHoodPosition() {
        return hoodMotor.getCurrentPosition();
    }
    
    public void stopHood() {
        hoodMotor.setPower(0);
    }
}
