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
    PIDController pid = new PIDController(.005, 0, 0, .05, telemetry);

    private OperatorInputs oi;
    @Override
    public void init() {
        oi = new OperatorInputs(gamepad1, telemetry);
        oi.gamepad = gamepad1;
        oi.telemetry = telemetry;

        // set up motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        servo = hardwareMap.get(CRServo.class, "servo");
        servo.setDirection(CRServo.Direction.REVERSE);

        coreHEX = hardwareMap.get(DcMotor.class, "coreHEX");
        coreHEX.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    // spinning the CRServo
    @Override
    public void start() {
        servo.setPower(1.0);
    }

    @Override
    public void loop() {
        // movement


        double leftPower = oi.drive + oi.turn;
        double rightPower = oi.drive - oi.turn;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if (oi.isShooterOn) {
            double power = pid.calculate(oi.shooterSpeed, shooterMotor.getVelocity());
            telemetry.addData("PID Power", power);
            telemetry.addData("Is at set point", pid.atTargetPoint());
            shooterMotor.setPower(power);
        } else shooterMotor.setPower(0.0);

        // shooting
        if (oi.shootActive && pid.atTargetPoint()) coreHEX.setPower(1.0);
        else coreHEX.setPower(-1.0);

        if (oi.intakeToggle) intakeMotor.setPower(1);
        else intakeMotor.setPower(-1);

        // inputs
        oi.update();

        telemetry.addData("Shooter speed", oi.shooterSpeed);
        telemetry.addData("Shooter State", oi.isShooterOn ? "On" : "Off");
        telemetry.addData("is Shooting", oi.shootActive);
        telemetry.addData("Can shoot", pid.atTargetPoint());
        telemetry.addData("Intake State", oi.intakeToggle ? "Pushing" : "Pulling");
        telemetry.update();

    }
}