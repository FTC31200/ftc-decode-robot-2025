package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// inspired and minified code of FGCLib's PID implementation
public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private double integralSum = 0;

    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    public boolean isAtTargetPoint = false;
    public Telemetry telemetry;

    public PIDController(double kP, double kI, double kD, double kF, Telemetry telemetry) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.telemetry = telemetry;
    }

    public double calculate(double setPoint, double realPosition) {
        double error = setPoint - realPosition;
        double t = timer.seconds();
        integralSum += error * t;
        double derivative = (error - lastError) / t;
        lastError = error;
        timer.reset();
        double output = (error * kP) + (derivative * kD) + (integralSum * kI) + kF;
        telemetry.addData("Error", error);
        isAtTargetPoint = Math.abs(error) < 145.0;
        return output;
    }
    public boolean atTargetPoint() {
        return isAtTargetPoint;
    }
}