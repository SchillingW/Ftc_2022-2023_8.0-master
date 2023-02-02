package org.firstinspires.ftc.teamcode.NEW.bot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.DriveSlide;
import org.firstinspires.ftc.teamcode.hardware.HolonomicDrive;
import org.firstinspires.ftc.teamcode.hardware.HolonomicNavigation;
import org.firstinspires.ftc.teamcode.hardware.HolonomicOdometry;
import org.firstinspires.ftc.teamcode.hardware.LinearSlide;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;
import org.firstinspires.ftc.teamcode.util.AutonomousTimer;
import org.firstinspires.ftc.teamcode.util.InterpolateClamp;
import org.firstinspires.ftc.teamcode.util.RotateConvert;

// bot configuration for VoltaVinci
public class Volta extends DriveSlide {

    // declare hardware devices
    public Claw claw;
    public VisionDevice vision;
    public AutonomousTimer timer;

    // local position of claw relative to bot center
    public static final double clawX = 0;
    public static final double clawY = 10;

    // frame size
    public static final double frameX = 12.3;
    public static final double frameY = 13.5;

    // width of mecanum wheels extending past frame
    public static final double wheelWidth = 2;

    // slide heights
    public static final double aboveSlide = 4.5;
    public static final double belowSlide = -3;
    public static final double restSlide = 2.5;
    public static final double startSlide = 2.5;

    // delay times
    public static final double grabWait = 0.25;
    public static final double dropBuffer = 0.125;
    public static final double alignTime = 0.5;

    // initialize bot
    public Volta(double startX, double startY, double startRot,
                 HardwareMap map, Telemetry tele) {

        // initialise DriveSlide device
        super(null, null, tele);



        // initialize drive motors
        DcMotor motorFL = map.get(DcMotor.class, "motorFL");
        DcMotor motorFR = map.get(DcMotor.class, "motorFR");
        DcMotor motorBL = map.get(DcMotor.class, "motorBL");
        DcMotor motorBR = map.get(DcMotor.class, "motorBR");

        // encoderL & encoderR record 15.8rot over a 118in linear path
        RotateConvert convertMotor = new RotateConvert(420, 1);
        RotateConvert convertEncoder = new RotateConvert(8192, 7.468);

        // initialize drive train
        HolonomicDrive drive = new HolonomicDrive(
                motorFL, motorFR, motorBL, motorBR,
                convertMotor.instance(1), convertMotor.instance(-1),
                convertMotor.instance(1), convertMotor.instance(-1),
                tele);

        // encoderL records +114in over a 10PIrad rotation
        // encoderR records -108in over a 10PIrad rotation
        // encoderH records -047in over a 10PIrad rotation
        HolonomicOdometry odometry = new HolonomicOdometry(
                motorFL, motorFR, motorBL,
                convertEncoder.instance(1), convertEncoder.instance(-1),
                convertEncoder.instance(-1),
                -3.628, 3.438, -1.496,
                startX, startY, startRot,
                tele);

        // approach speed gradient
        InterpolateClamp approach = new InterpolateClamp(
                0, 18,
                0.15, 0.6);

        // initialize navigation device and add as autonomous subsystem
        nav = new HolonomicNavigation(
                drive, odometry, approach,
                0.5, Math.PI / 96,
                1, 0.8, 48 / Math.PI,
                tele);
        subsystem.set(0, nav);



        // initialize slide motor
        DcMotor motorSlide = map.get(DcMotor.class, "motorSlide");

        // motorSlide records 2.69rot over a 24in displacement
        RotateConvert convertSlide = new RotateConvert(756, 8.922, -1);

        // approach speed gradient from below target
        InterpolateClamp approachBelow = new InterpolateClamp(
                0, 6,
                0.5, 1);

        // approach speed gradient from above target
        InterpolateClamp approachAbove = new InterpolateClamp(
                0, 6,
                0.5, 0.75);

        // initialize linear slide device and add as autonomous subsystem
        slide = new LinearSlide(
                motorSlide, convertSlide,
                approachBelow, approachAbove,
                0.25, 1, 0.125,
                startSlide,
                tele);
        subsystem.set(1, slide);



        // initialize claw servo
        Servo servoClaw = map.get(Servo.class, "servoClaw");

        // initialize claw device
        claw = new Claw(servoClaw, 0, 0.5, tele);

        // add claw as autonomous subsystem
        subsystem.add(claw);



        // initialize vision device
        vision = new VisionDevice(tele, map);



        // initialize timer device
        timer = new AutonomousTimer(tele);

        // add timer as autonomous subsystem
        subsystem.add(timer);
    }
}
