package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="AutoTest", group="PursuitBot")
public class AutoTest extends LinearOpMode {

    public PursuitBot robot;
    public VisionDevice vision;
    public ElapsedTime timer = new ElapsedTime();
    public Motor slide;
    public Servo claw;

    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        slide = new Motor(hardwareMap, "slide");
        claw = hardwareMap.servo.get("claw");

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();
        int result = 0;
        while (!isStarted()) result = vision.perform(1f / 3f);

        waitForStart();

        // START MOVEMENT

        if (opModeIsActive()) slide.set(0);
        if (opModeIsActive()) claw.setPosition(0);
        if (opModeIsActive()) sleep(2000);

        // CONE GRABBED

        robot.reachPoint(new Pose2d(1, 28, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(29, 28, new Rotation2d()), telemetry, this);
        if (opModeIsActive()) Up();
        robot.reachPoint(new Pose2d(29, 42.5, new Rotation2d()), telemetry, this);

        // AT DROP CONE LOCATION

        if (opModeIsActive()) sleep(2000);
        if (opModeIsActive()) slide.set(1);
        if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) claw.setPosition(0.5);
        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        robot.reachPoint(new Pose2d(29, 28, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(1, 28, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(1, 5, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(16, 5, new Rotation2d()), telemetry, this);

        // AT CONE GRAB LOCATION

        if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) claw.setPosition(0);
        if (opModeIsActive()) sleep(2000);

        // CONE GRABBED

        if (opModeIsActive()) Up();
        robot.reachPoint(new Pose2d(29, 5, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(29, 42.5, new Rotation2d()), telemetry, this);

        // AT DROP LOCATION

        if (opModeIsActive()) sleep(2000);
        if (opModeIsActive()) slide.set(1);
        if (opModeIsActive()) sleep(500);
        if (opModeIsActive()) claw.setPosition(0.5);
        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        robot.reachPoint(new Pose2d(29, 4 - 24 + result * 24, new Rotation2d()),
                telemetry, this);
    }
    public void Up()
    {
        timer.reset();

        while(opModeIsActive() && timer.seconds() <= 1)
        {
            slide.set(-1);
        }
    }

    public void Down()
    {
        timer.reset();

        while(opModeIsActive() && timer.seconds() <= 1)
        {
            slide.set(1);
        }
    }
}

