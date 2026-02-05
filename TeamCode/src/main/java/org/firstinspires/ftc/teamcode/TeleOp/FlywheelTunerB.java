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
    private static double kV = 0;
    private static double kA = 0;
    private static double V_kP = 0.0001;
    private static double V_kI = 0.0002;
    private static double V_kD = 0.0;
    private static double MAX_ACCELERATION;
    private double integralSum = 0;
    public DcMotorEx turretL;
    public DcMotorEx turretR;
    double highVelocity = 1640;
    double curTargetVelocity = highVelocity;
    double lowVelocity = 900;
    double Power = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex  = 1;
    private boolean powered;

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
        loopTime.reset();
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
        integralSum = clamp(integralSum, -0.3, 0.3);//Integral Max

        // Zero-crossing reset prevents overshoot
        if (lastVelocityError != 0 && Math.signum(velocityError) != Math.signum(lastVelocityError)) {
            integralSum = 0;
        }
        double iOutput = V_kI * integralSum;

        // Derivative (on measurement to avoid setpoint kick)
        double dOutput = V_kD * -measuredAcceleration;

        lastVelocityError = velocityError;

        // Combine PID terms
        double pidOutput = pOutput + iOutput + dOutput;
        //lastPidOutput = pidOutput;

        // ==================== TOTAL OUTPUT ====================
        double totalPower = ffOutput + pidOutput;
        totalPower = clamp(totalPower, 0, 1.0);  // Motor power range (flywheel only spins one direction)
        //lastTotalPower = totalPower;

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
        if(powered) turretL.setPower(calc());
        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.dpadUpWasPressed()){
            Power += 1;
        }
        if(gamepad1.dpadDownWasPressed()){
            Power -= 1;
        }

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Tuning Power", Power);
        telemetry.addData("Step Size", stepSizes[stepIndex]);
        telemetry.addData("Acceleration", ((turretR.getVelocity()+turretL.getVelocity())/2)/dt);
    }
}
