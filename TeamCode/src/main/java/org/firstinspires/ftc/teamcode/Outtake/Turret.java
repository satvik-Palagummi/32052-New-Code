package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    double speed = 1420;
    private DcMotorEx turret;
    public void initTurret(HardwareMap hw) {
        turret = hw.get(DcMotorEx.class, "Turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorEx.Direction.FORWARD);
    }
    public void startOuttake(){
        turret.setVelocity(speed);
    }
    public void setPower(double sped){
        speed = sped;
    }
    public double getTurretPower(){return turret.getPower();}
    public double getTurretVelocity(){return turret.getVelocity();}
    public void stopOuttake(){
        turret.setPower(0);
    }

}
