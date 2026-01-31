package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Turret {
    double speed = 1420;
    private DcMotorEx turretL;
    private DcMotorEx turretR;

    public void initTurret(HardwareMap hw) {
        turretL = hw.get(DcMotorEx.class, "TurretL");
        turretR = hw.get(DcMotorEx.class, "TurretR");
        turretL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretL.setDirection(DcMotorEx.Direction.FORWARD);
        turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretR.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(170, 0, 0, 12.813);
        turretL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        turretR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }
    public void startOuttake(){
        turretL.setVelocity(speed);
        turretR.setVelocity(speed);
    }

    public void setPower(double sped){
        speed = sped;
    }
    public double getTurretLPower(){return turretL.getPower();}
    public double getTurretRPower(){return turretR.getPower();}
    public double getTurretLVelocity(){return turretL.getVelocity();}
    public double getTurretRVelocity(){return turretR.getVelocity();}
    public void stopOuttake(){
        turretL.setPower(0);
        turretR.setPower(0);
    }

}
