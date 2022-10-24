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
        while (!isStarted()) result = vision.perform();

        waitForStart();
        slide.set(0);
        claw.setPosition(0);
        sleep(2000);
        robot.reachPoint(new Pose2d(0, 28, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(29, 28, new Rotation2d()), telemetry);
        Up();
        robot.reachPoint(new Pose2d(29, 42.5, new Rotation2d()), telemetry);

        sleep(2000);
        slide.set(1);
        sleep(1000);
        claw.setPosition(1);
        sleep(2000);
        robot.reachPoint(new Pose2d(29, 28, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(1, 28, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(1, 5, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(21, 5, new Rotation2d()), telemetry);
        sleep(1000);
        claw.setPosition(0);
        sleep(2000);
        robot.reachPoint(new Pose2d(24, 5, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(24, 28, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(24, 42.5, new Rotation2d()), telemetry);
        Up();
        robot.reachPoint(new Pose2d(29, 42.5, new Rotation2d()), telemetry);
        sleep(2000);
        slide.set(1);
        sleep(500);
        claw.setPosition(1);
        sleep(2000);

        /*
        //turn
        robot.reachPoint(new Pose2d(28, 15, new Rotation2d(-Math.PI/2)), telemetry);
        claw.setPosition(0);
        sleep(2000);
        Up();
        //turn
        robot.reachPoint(new Pose2d(28, 40, new Rotation2d()), telemetry);
        claw.setPosition(0.5);
        Down();
        */


        robot.reachPoint(new Pose2d(29, 4 - 24 + result * 24, new Rotation2d()), telemetry);
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

