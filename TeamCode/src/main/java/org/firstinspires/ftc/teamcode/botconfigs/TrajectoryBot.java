package org.firstinspires.ftc.teamcode.botconfigs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vec2F;
import com.vuforia.Vec2I;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FieldDimensions;
import org.firstinspires.ftc.teamcode.hardware.MecDriveFlip;


import java.util.Optional;
import java.util.function.DoubleSupplier;

// mecanum drive bot with odometry for Pure Pursuit
public class TrajectoryBot {

    // dimensions
    public FieldDimensions xDim = new FieldDimensions();
    public FieldDimensions yDim = new FieldDimensions();

    // debugging devices
    public Telemetry tele;

    public ColorSensor sensor;
    public int dropOffset = 80;

    // mecanum wheel drive train
    public MecanumDrive drive;
    public MotorEx motorFL;
    public MotorEx motorFR;
    public MotorEx motorBL;
    public MotorEx motorBR;

    public LinearSlide linearSlide;

    public double kP, kI, kD, kF;
    public PIDFController pidf;

    // odometry device
    public OdometrySubsystem odometry;
    public DoubleSupplier encoderL;
    public DoubleSupplier encoderR;
    public DoubleSupplier encoderH;

    // hardware specifications
    public double wheelDiameter = 2.3622;
    public double wheelCircumference = wheelDiameter * Math.PI;

    // robot type data
    public double encoderTrackWidth = 8.5;
    public double encoderWheelOffset = 1.5;

    // robot movement datas
    public double adjustSpeed = 0.15;
    public double minSpeed = 0.3;
    public double minGradient = 3;
    public double maxSpeed = 0.6;
    public double maxGradient = 18;
    public double errorMargin = 0.5;
    public double extraTime = 0.5;
    public double degreeToInchEquivFactor = 24.0 / 360.0;

    public double rotErrorMargin = 3;

    // initialize devices
    public TrajectoryBot(Telemetry tele, HardwareMap map) {

        // store debugging device
        this.tele = tele;

        // initialize dimensions hi
        xDim.botanchor2botcenterHARDWARE = 6.5;
        yDim.botanchor2botcenterHARDWARE = 6.5;
        xDim.botanchor2clawcenterHARDWARE = 18.5;
        yDim.botanchor2clawcenterHARDWARE = 6.5;

        // initialize drive train
        motorFL = new MotorEx(map, "motorFL");
        motorFR = new MotorEx(map, "motorFR");
        motorBL = new MotorEx(map, "motorBL");
        motorBR = new MotorEx(map, "motorBR");

        linearSlide = new LinearSlide(tele, map);

        pidf = new PIDFController(kP, kI, kD, kF);

        // set our gains to some value
        pidf.setP(0.37);
        pidf.setI(0.05);
        pidf.setD(1.02);

        // get our gain constants
        kP = pidf.getP();
        kI = pidf.getI();
        kD = pidf.getD();

        // set all gains
        pidf.setPIDF(kP, kI, kD, 0.7);

        // get all gain coefficients
        double[] coeffs = pidf.getCoefficients();
        kP = coeffs[0];
        kI = coeffs[1];
        kD = coeffs[2];
        kF = coeffs[3];

        //sensor = map.colorSensor.get("sensor");

        drive = new MecDriveFlip(
                motorFL, motorFR, motorBL, motorBR,
                1, 1, 1, 1);

        // initialize odometry
        encoderL = getSupplier(motorFL, 1);
        encoderR = getSupplier(motorFR, -1);
        encoderH = getSupplier(motorBL, 1);
        odometry = new OdometrySubsystem(new HolonomicOdometry(
                encoderL, encoderR, encoderH,
                encoderTrackWidth, encoderWheelOffset));

        // orient to home
        motorFL.resetEncoder();
        motorFR.resetEncoder();
        motorBL.resetEncoder();
        motorBR.resetEncoder();
    }

    public int inchToTicks(double inch) {
        double inchToTicks = 8192 / wheelCircumference;
        return (int) Math.round(inch * inchToTicks);
    }

    public int velocityToTicks(double velocity) {
        // Assume the robot has a wheel diameter of 4 inches and 1120 ticks per revolution
        double calcDiam = wheelDiameter * 0.0254; // convert inches to meters
        double ticksPerRev = motorBL.getCPR();
        int ticks = (int)(velocity * ticksPerRev / (Math.PI * calcDiam));
        return ticks;
    }

    // return double supplier representing motor value in inches
    public DoubleSupplier getSupplier(Motor encoder, float coefficient) {

        // convert motor ticks to inches
        double ticksPerInch = 8192 / wheelCircumference;
        return () -> encoder.getCurrentPosition() / ticksPerInch * coefficient;
    }

    // debug info on bot with telemetry
    public void DebugFull(Telemetry telemetry) {

        telemetry.addData("current pose", odometry.getPose());
        telemetry.addData("encoder vertical left", encoderL.getAsDouble());
        telemetry.addData("encoder vertical right", encoderR.getAsDouble());
        telemetry.addData("encoder horizontal", encoderH.getAsDouble());
        telemetry.addData("blue", sensor.blue());
        telemetry.update();
    }
}