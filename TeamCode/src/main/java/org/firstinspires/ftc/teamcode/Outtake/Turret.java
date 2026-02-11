package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Turret {
    double speed = 1420;//Target Velocity
    ElapsedTime loopTime = new ElapsedTime();
    private double lastVelocity = 0;
    private double lastVelocityError = 0;
    private static double kS = 0.05;
    private double kV = 0.000387;
    private  double kA = 0.0;
    private  double V_kP = 0.00006;
    private  double V_kI = 0.0002;
    private  double V_kD = 0.0000020;
    private static double MAX_ACCELERATION = 15000;
    private double integralSum = 0;
    public DcMotorEx turretL;
    public DcMotorEx turretR;
    double highVelocity = 1640;
    double curTargetVelocity = highVelocity;
    double lowVelocity = 900;
    double Power = 0;
    private boolean powered;
    private double accel;
    private double totalPower;
    private double velError;
    private double lastTotalPower;
/*
    public double calc(){
        double dt = loopTime.seconds();
        loopTime.reset();
        if (dt <= 0 || dt > 0.1) dt = 0.02;
        double currentVelocity = (turretL.getVelocity() + turretR.getVelocity()) / 2.0;
        double measuredAcceleration = (currentVelocity - lastVelocity) / dt;
        lastVelocity = currentVelocity;
        double velocityError = speed - currentVelocity;


        // Calculate desired acceleration (aggressive: close the gap as fast as possible)
        // Clamp to physical limits
        double desiredAcceleration = velocityError / dt;
        desiredAcceleration = clamp(desiredAcceleration,
                -MAX_ACCELERATION,
                MAX_ACCELERATION);
        accel = desiredAcceleration;

        // ==================== FEEDFORWARD ====================
        // Full equation: kS * sign(v) + kV * targetVel + kA * acceleration
        double ffOutput = kS * Math.signum(speed)
                + kV * speed
                + kA * desiredAcceleration;

        // ==================== PID ====================
        // Proportional
        double pOutput = V_kP * velocityError;

        // Integral with anti-windup
        integralSum += velocityError * dt;
        integralSum = clamp(integralSum, -0.2, 0.2);//Integral Max

        // Zero-crossing reset prevents overshoot
        double iOutput = V_kI * integralSum;

        // Derivative (on measurement to avoid setpoint kick)
        double dOutput = V_kD * (velocityError-lastVelocityError)/dt;

        lastVelocityError = velocityError;

        // Combine PID terms
        double pidOutput = pOutput + iOutput + dOutput;
        //lastPidOutput = pidOutput;

        // ==================== TOTAL OUTPUT ====================
        double totalPower = ffOutput + pidOutput;
        double maxDelta = 0.005; // power per loop
        totalPower = clamp(
                totalPower,
                lastTotalPower - maxDelta,
                lastTotalPower + maxDelta
        );

        totalPower = clamp(totalPower, lastTotalPower - maxDelta, lastTotalPower + maxDelta);
        totalPower = clamp(totalPower, 0, 1.0);  // Motor power range (flywheel only spins one direction)
        lastTotalPower = totalPower;

        // Apply power to both motors
        /*robot.shooterMotor1.setPower(totalPower);
        robot.shooterMotor2.setPower(totalPower);


        return totalPower;
    }
    */
    public void initTurret(HardwareMap hw) {
        turretL = hw.get(DcMotorEx.class, "TurretL");
        turretR = hw.get(DcMotorEx.class, "TurretR");
        turretL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretL.setDirection(DcMotorEx.Direction.FORWARD);
        turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretR.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(170, 0, 0, 11.727);
        turretL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        turretR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }
    public void startOuttake(){
        turretL.setVelocity(speed);
        turretR.setVelocity(speed);
    }

    public void startOuttake(boolean outtake){
        if(outtake){
            turretL.setVelocity(speed);
            turretR.setVelocity(speed);
        }else{
            turretL.setVelocity(0);
            turretR.setVelocity(0);
        }
    }

    /*
    public void startOuttake(boolean outtake){
        if(outtake){
            double calculated = calc();
            turretL.setPower(calculated);
            turretR.setPower(calculated);
        }else{
            turretL.setPower(0);
            turretR.setPower(0);
        }
    }

     */



    public void setPower(double sped){
        speed = sped;
    }
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getSpeed() {
        return speed;
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
