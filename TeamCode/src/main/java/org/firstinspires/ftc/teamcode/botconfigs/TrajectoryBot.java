package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FieldDimensions;
import org.firstinspires.ftc.teamcode.hardware.DCMecDriveFlip;


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
    public DcMotorEx motorFL;
    public DcMotorEx motorFR;
    public DcMotorEx motorBL;
    public DcMotorEx motorBR;

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
        motorFL = map.get(DcMotorEx.class, "motorFL");
        motorFR = map.get(DcMotorEx.class, "motorFL");
        motorBL = map.get(DcMotorEx.class, "motorFL");
        motorBR = map.get(DcMotorEx.class, "motorFL");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlide = new LinearSlide(tele, map);

        pidf = new PIDFController(kP, kI, kD, kF);

        // set our gains to some value
        pidf.setP(10.0);
        pidf.setI(3.00);
        pidf.setD(0.00);
        pidf.setF(0.00);

        // get our gain constants
        kP = pidf.getP();
        kI = pidf.getI();
        kD = pidf.getD();

        // set all gains
        pidf.setPIDF(kP, kI, kD, 0.0);

        // get all gain coefficients
        double[] coeffs = pidf.getCoefficients();
        kP = coeffs[0];
        kI = coeffs[1];
        kD = coeffs[2];
        kF = coeffs[3];

        // initialize odometry
        encoderL = getSupplier(motorFL, 1);
        encoderR = getSupplier(motorFR, -1);
        encoderH = getSupplier(motorBL, -1);
        odometry = new OdometrySubsystem(new HolonomicOdometry(
                encoderL, encoderR, encoderH,
                encoderTrackWidth, encoderWheelOffset));

        // orient to home
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int inchToTicks(double inch) {
        double inchToTicks = 8192 / wheelCircumference;
        return (int) Math.round(inch * inchToTicks);
    }

    public int velocityToTicks(double velocity) {
        double calcDiam = wheelDiameter * 0.0254; // convert inches to meters
        double ticksPerRev = 1680;
        int ticks = (int)(velocity * ticksPerRev / (Math.PI * calcDiam));
        return ticks;
    }

    public double inchesToMeters(double inches)
    {
        return inches * 0.0254;
    }

    // return double supplier representing motor value in inches
    public DoubleSupplier getSupplier(DcMotor encoder, float coefficient) {

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