package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Outtake.Turret;

@TeleOp
public class FlywheelTunerB extends OpMode {
    double speed = 1420;//Target Velocity
    ElapsedTime loopTime = new ElapsedTime();
    private double lastVelocity = 0;
    private double lastVelocityError = 0;
    private static double kS = 0.05;
    private double kV = 0.000387;
    private  double kA = 0.0;
    private  double V_kP = 0.00010;
    private  double V_kI = 0.00001;
    private  double V_kD = 0.0000020;
    private static double MAX_ACCELERATION = 15000;
    private double integralSum = 0;
    public DcMotorEx turretL;
    public DcMotorEx turretR;
    double highVelocity = 1640;
    double curTargetVelocity = highVelocity;
    double lowVelocity = 900;
    double Power = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001,0.00001,0.000001};
    int stepIndex  = 1;
    private boolean powered;
    private double accel;
    private double totalPower;
    private double velError;
    private double lastTotalPower;

    @Override
    public void init() {
        turretL = hardwareMap.get(DcMotorEx.class, "TurretL");
        turretR = hardwareMap.get(DcMotorEx.class, "TurretR");
        turretL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretL.setDirection(DcMotorEx.Direction.FORWARD);
        turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretR.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry.addLine("Init complete");

    }
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    public double calc(){
        double dt = loopTime.seconds();
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
        if (lastVelocityError != 0 && Math.signum(velocityError) != Math.signum(lastVelocityError)) {
            integralSum = 0;
        }
        double iOutput = V_kI * integralSum;

        // Derivative (on measurement to avoid setpoint kick)
        double dOutput = V_kD * (velocityError-lastVelocityError)/dt;

        lastVelocityError = velocityError;

        // Combine PID terms
        double pidOutput = pOutput + iOutput + dOutput;
        //lastPidOutput = pidOutput;

        // ==================== TOTAL OUTPUT ====================
        double totalPower = ffOutput + pidOutput;
        double maxDelta = 0.01; // power per loop
        totalPower = clamp(
                totalPower,
                lastTotalPower - maxDelta,
                lastTotalPower + maxDelta
        );
        totalPower = clamp(totalPower, 0, 1.0);  // Motor power range (flywheel only spins one direction)
        lastTotalPower = totalPower;

        // Apply power to both motors
        /*robot.shooterMotor1.setPower(totalPower);
        robot.shooterMotor2.setPower(totalPower);

         */
        return totalPower;
    }

    @Override
    public void loop() {
        double dt = loopTime.seconds();
        loopTime.reset();
        if(gamepad1.yWasPressed()){
            powered = !powered;
        }
        if(gamepad1.aWasPressed()){
            speed = 900;
        }
        if(gamepad1.xWasPressed()){
            speed = 1420;
        }
        if(powered) {
            double calculated = calc();
            turretL.setPower(calculated);
            turretR.setPower(calculated);
        }else{
            turretL.setPower(0);
            turretR.setPower(0);
            lastTotalPower = 0;
            integralSum = 0;
        }
        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.dpadUpWasPressed()){
             V_kD+=0.00000005;
        }
        if(gamepad1.dpadDownWasPressed()){
            V_kD-= 0.00000005;
        }
        if(gamepad1.dpadLeftWasPressed()){
            V_kP+= 0.000005;
        }
        if(gamepad1.dpadRightWasPressed()){
            V_kP -= 0.000005;
        }

        telemetry.addData("Current Velocity", (turretL.getVelocity()+turretR.getVelocity())/2);
        telemetry.addData("Target Velocity", speed);
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Total Power", lastTotalPower);
        telemetry.addData("Velocity Error ",  lastVelocityError);
        telemetry.addData("V_kI", V_kI*1000);
        telemetry.addData("V_kP", V_kP*10000);
        telemetry.addData("V_kD", V_kD*100000);
        telemetry.addData("Accel", accel);
        telemetry.addData("total Power", lastTotalPower);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addData("Acceleration", ((turretR.getVelocity()+turretL.getVelocity())/2)/dt);
    }
}
