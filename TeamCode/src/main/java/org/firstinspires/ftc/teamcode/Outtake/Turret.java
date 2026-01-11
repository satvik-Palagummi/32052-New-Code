package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    double speed = 0.64;
    private DcMotor spinner;
    public void initTurret(HardwareMap hw) {
        spinner = hw.get(DcMotor.class, "Turret");
        spinner.setDirection(DcMotor.Direction.FORWARD);
    }
    public void startOuttake(){
        spinner.setPower(speed);
    }
    public void setPower(double sped){
        speed = sped;
    }
    public double getTurret(){
        return spinner.getPower();
    }
    public void stopOuttake(){
        spinner.setPower(0);
    }

}
