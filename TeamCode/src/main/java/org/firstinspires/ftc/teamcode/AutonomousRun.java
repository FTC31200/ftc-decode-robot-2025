package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Autonomous", group = "Linear Opmode")
public class AutonomousRun extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotorEx shooterMotor;
    private DcMotor coreHEX;
    private CRServo servo;
    PIDController pid = new PIDController(.005, 0, 0, .02
            , telemetry);

    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        coreHEX = hardwareMap.get(DcMotor.class, "coreHEX");
        servo = hardwareMap.get(CRServo.class, "servo");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        coreHEX.setDirection(DcMotor.Direction.REVERSE);
        servo.setDirection(CRServo.Direction.REVERSE);

        pid.setTolerance(145.0);

        waitForStart();

        if (opModeIsActive()) {
            coreHEX.setPower(-1.0);
            servo.setPower(1.0);
            sleep(300);

            leftMotor.setPower(-0.5);
            rightMotor.setPower(-0.5);
            sleep(1270);

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(500);

            shooterMotor.setPower(1.0);
            servo.setPower(1.0);
            for (int i = 0; i < 3; i++) {
                sleep(1200);
                pid.isAtTargetPoint = false;
                shoot();
                waitForBall();
            }
        }
        stop();
    }

    void shoot() {
        while (!pid.isAtTargetPoint) {
            telemetry.addData("ShooterSpeed", shooterMotor.getVelocity());
            telemetry.update();
            double power = pid.calculate(1700, shooterMotor.getVelocity());
            shooterMotor.setPower(power);
        }
        sleep(300);
        coreHEX.setPower(1.0);
    }

    void waitForBall() {
        while (true) {
            telemetry.addData("ShooterSpeed", shooterMotor.getVelocity());
            telemetry.update();
            if (shooterMotor.getVelocity() < 1300) break;
        }
        coreHEX.setPower(-1.0);
    }
}