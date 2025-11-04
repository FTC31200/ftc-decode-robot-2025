package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Arcade Drive")
public class ArcadeDrive extends OpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotorEx shooterMotor;
    private DcMotor coreHEX;
    private DcMotor intakeMotor;
    private CRServo servo;
    private boolean canSpinShooter = false;
    private boolean intakeToggle = false;
    private boolean prevRightTrigger = false;

    private double shooterSpeed = 1500.0;
    private final double kP = .005;
    private final double kI = 0;
    private final double kD = 0;
    private final double kF = .05;
    PIDController pid = new PIDController(kP, kI, kD, kF);

    @Override
    public void init() {

        // get motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        coreHEX = hardwareMap.get(DcMotor.class, "coreHEX");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        servo = hardwareMap.get(CRServo.class, "servo");

        // set up motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        coreHEX.setDirection(DcMotor.Direction.REVERSE);
        servo.setDirection(CRServo.Direction.REVERSE);
    }

    // spinning the CRServo
    @Override
    public void start() {
        servo.setPower(1.0);
    }

    @Override
    public void loop() {
        // movement
        double drive = -gamepad1.left_stick_y * 0.72;
        double turn = gamepad1.right_stick_x * 0.85;

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        handleButtonInputs();

        if (canSpinShooter) {
            double power = pid.calculate(shooterSpeed, shooterMotor.getVelocity());
            telemetry.addData("PID Power", power);
            shooterMotor.setPower(power);
        } else shooterMotor.setPower(0.0);

        if (gamepad1.right_trigger > 0.05) coreHEX.setPower(1.0);
        else coreHEX.setPower(-1.0);

        if (intakeToggle) intakeMotor.setPower(.9);
        else intakeMotor.setPower(-.9);

        prevRightTrigger = gamepad1.left_trigger > 0.05;

        telemetry.addData("Shooter speed", shooterSpeed);
        telemetry.addData("Shooter State", canSpinShooter ? "On" : "Off");
        telemetry.addData("Intake State", intakeToggle ? "Pushing" : "Pulling");
        telemetry.update();
    }

    private void handleButtonInputs() {
        if (gamepad1.dpadUpWasPressed()) canSpinShooter = !canSpinShooter;

        // just for tweaks during tests
        if (gamepad1.dpadLeftWasPressed()) shooterSpeed -= 50.0;
        if (gamepad1.dpadRightWasPressed()) shooterSpeed += 50.0;
        shooterSpeed = Math.max(600.0, Math.min(10000, shooterSpeed));

        if (gamepad1.left_trigger > 0.05 && !prevRightTrigger) intakeToggle = !intakeToggle;
    }
}