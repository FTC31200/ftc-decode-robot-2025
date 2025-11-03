package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// inspired and minified code of FGCLib PID implementation
public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private double integralSum = 0;

    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double calculate(double setPoint, double realPosition) {
        double error = setPoint - realPosition;

        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * kP) + (derivative * kD) + (integralSum * kI) + kF;

        if (Math.abs(error) < 20.0) output = 0;

        return output;
    }
}