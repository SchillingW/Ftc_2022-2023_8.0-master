package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="BluePursuitBotAuto", group="PursuitBot")
public class BlueAutoTest extends LinearOpMode {

    public PursuitBot robot;
    public VisionDevice vision;
    public ElapsedTime timer = new ElapsedTime();
    public Motor slide;
    public Servo claw;

    public int startXOff = -6;

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

        robot.reachPoint(new Pose2d(1, -28 - startXOff, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(29.25, -28 - startXOff, new Rotation2d()), telemetry, this);
        if (opModeIsActive()) Up();
        robot.reachPoint(new Pose2d(29.25, -41.6 - startXOff, new Rotation2d()), telemetry, this);

        // AT DROP CONE LOCATION

        if (opModeIsActive()) sleep(2000);
        if (opModeIsActive()) slide.set(1);
        if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) claw.setPosition(0.5);
        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        robot.reachPoint(new Pose2d(27.5, -41.6 - startXOff, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(27.5, -28 - startXOff, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(1.25, -28 - startXOff, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(1.25, -5 - startXOff, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(17.5, -5 - startXOff, new Rotation2d()), telemetry, this);

        // AT CONE GRAB LOCATION

        if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) claw.setPosition(0);
        if (opModeIsActive()) sleep(2000);

        // CONE GRABBED

        if (opModeIsActive()) Up();
        robot.reachPoint(new Pose2d(27.5, -5 - startXOff, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(27.5, -41.6 - startXOff, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(29.25, -41.6 - startXOff, new Rotation2d()), telemetry, this);

        // AT DROP LOCATION

        if (opModeIsActive()) sleep(2000);
        if (opModeIsActive()) slide.set(1);
        if (opModeIsActive()) sleep(500);
        if (opModeIsActive()) claw.setPosition(0.5);
        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        robot.reachPoint(new Pose2d(27.5, -41.6 - startXOff, new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(27.5, 4 - 24 + result * 24 - startXOff, new Rotation2d()),
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

