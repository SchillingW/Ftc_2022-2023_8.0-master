package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;

@Autonomous(name="Meet1BotAuto", group="PursuitBot")
public class Meet1BotAuto extends LinearOpMode {

    public PursuitBot robot;
    public ElapsedTime timer = new ElapsedTime();
    public Motor slide;

    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        slide = new Motor(hardwareMap, "slide");

        waitForStart();

        robot.reachPoint(new Pose2d(24, 0, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(0, 24, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(0, 12, new Rotation2d()), telemetry);
        Up();
        Down();
        //robot.reachPoint(new Pose2d(-12, -12, Rotation2d.fromDegrees(90)), telemetry);
    }

    public void Up()
    {
        timer.reset();

        while(opModeIsActive() && timer.seconds() <= 5)
        {
            slide.set(-1);
        }
    }

    public void Down()
    {
        timer.reset();

        while(opModeIsActive() && timer.seconds() <= 5)
        {
            slide.set(1);
        }
    }
}
