package org.firstinspires.ftc.teamcode.oldproj.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DCMecDriveFlip extends MecanumDrive {

    float coefFL;
    float coefFR;
    float coefBL;
    float coefBR;

    public DCMecDriveFlip(
            DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight,
            float coefFL, float coefFR, float coefBL, float coefBR) {

        super((Motor) frontLeft, (Motor)frontRight, (Motor)backLeft, (Motor)backRight);

        this.coefFL = coefFL;
        this.coefFR = coefFR;
        this.coefBL = coefBL;
        this.coefBR = coefBR;
    }

    @Override
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle) {
        super.driveFieldCentric(forwardSpeed, strafeSpeed, turnSpeed, -gyroAngle);
    }

    @Override
    public void driveWithMotorPowers(
            double frontLeftSpeed, double frontRightSpeed,
            double backLeftSpeed, double backRightSpeed) {

        super.driveWithMotorPowers(
                frontLeftSpeed * coefFL, frontRightSpeed * coefFR,
                backLeftSpeed * coefBL, backRightSpeed * coefBR);
    }
}
