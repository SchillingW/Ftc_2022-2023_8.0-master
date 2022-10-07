package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MecDriveFlip;

import java.util.function.DoubleSupplier;

// mecanum drive bot with odometry for Pure Pursuit
public class PursuitBot {

    // debugging device
    public Telemetry tele;

    // mecanum wheel drive train
    public MecanumDrive drive;
    public Motor motorFL;
    public Motor motorFR;
    public Motor motorBL;
    public Motor motorBR;

    // odometry device
    public OdometrySubsystem odometry;
    public DoubleSupplier encoderL;
    public DoubleSupplier encoderR;
    public DoubleSupplier encoderH;

    // hardware specifications
    public double wheelDiameter = 2.5;
    public double wheelCircumference = wheelDiameter * Math.PI;

    // robot type data
    public double encoderTrackWidth = 5.75;
    public double encoderWheelOffset = 3.5;

    // initialize devices
    public PursuitBot(Telemetry tele, HardwareMap map) {

        // store debugging device
        this.tele = tele;

        // initialize drive train
        motorFL = new Motor(map, "motorFL");
        motorFR = new Motor(map, "motorFR");
        motorBL = new Motor(map, "motorBL");
        motorBR = new Motor(map, "motorBR");

        drive = new MecDriveFlip(
                motorFL, motorFR, motorBL, motorBR,
                -1, 1, 1, -1);

        // initialize odometry
        encoderL = getSupplier(motorFL, 1);
        encoderR = getSupplier(motorFR, -1);
        encoderH = getSupplier(motorBL, -1);
        odometry = new OdometrySubsystem(new HolonomicOdometry(
                encoderL, encoderR, encoderH,
                encoderTrackWidth, encoderWheelOffset));

        // orient to home
        motorFL.resetEncoder();
        motorFR.resetEncoder();
        motorBL.resetEncoder();
        motorBR.resetEncoder();
    }

    // return double supplier representing motor value in inches
    public DoubleSupplier getSupplier(Motor encoder, float coefficient) {

        // convert motor ticks to inches
        double ticksPerInch = 8192 / wheelCircumference;
        return () -> encoder.getCurrentPosition() / ticksPerInch * coefficient;
    }





    public void reachPoint(Pose2d target, Telemetry tele) {

        odometry.update();

        while (!isAtTarget(target)) {

            odometry.update();
            moveTowards(target, tele);
        }
    }

    public void moveTowards(Pose2d target, Telemetry tele) {

        double x = target.getX() - odometry.getPose().getX();
        double y = target.getY() - odometry.getPose().getY();
        double rot = target.getRotation().minus(odometry.getPose().getRotation()).getRadians();
        double magnitude = Math.sqrt(x * x + y * y + rot * rot);
        tele.addData("magnitude", magnitude);
        tele.addData("x", x);
        tele.addData("y", y);
        tele.addData("rot", rot);
        x /= magnitude;
        y /= magnitude;
        rot /= magnitude;
        x *= 0.2;
        y *= 0.2;
        rot *= 0.2;

        drive.driveRobotCentric(x, y, rot);

        DebugFull(tele);
    }

    public boolean isAtTarget(Pose2d target) {

        double x = target.getX() - odometry.getPose().getX();
        double y = target.getY() - odometry.getPose().getY();

        double rot =
                target.getRotation().getDegrees() -
                        odometry.getPose().getRotation().getDegrees();

        return Math.abs(x) <= 0.5 && Math.abs(y) <= 0.5 && Math.abs(rot) <= 3;
    }

    // debug info on bot with telemetry
    public void DebugFull(Telemetry telemetry) {

        telemetry.addData("current pose", odometry.getPose());
        telemetry.addData("encoder vertical left", encoderL.getAsDouble());
        telemetry.addData("encoder vertical right", encoderR.getAsDouble());
        telemetry.addData("encoder horizontal", encoderH.getAsDouble());
        telemetry.update();
    }
}
