package org.firstinspires.ftc.teamcode.OLD.autos;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OLD.botconfigs.PursuitBot;

@Autonomous(name="Meet1BotAuto", group="PursuitBot")
public class Meet1BotAuto extends LinearOpMode {

    public ElapsedTime timer = new ElapsedTime();

    public PursuitBot robot;
    public Motor slide;
    public ServoEx claw;

    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        slide = new Motor(hardwareMap, "slide");
        claw = new SimpleServo(hardwareMap, "claw", 0, 180);

        waitForStart();

        /*robot.reachPoint(new Pose2d(24, 0, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(0, 24, new Rotation2d()), telemetry);
        robot.reachPoint(new Pose2d(0, 12, new Rotation2d()), telemetry);*/

        CloseClaw();
        Up();
        OpenClaw();
        Down();
        //robot.reachPoint(new Pose2d(-12, -12, Rotation2d.fromDegrees(90)), telemetry);
    }

    public void OpenClaw()
    {
        claw.setPosition(0.3);
        while(claw.getPosition() <= 0.295)
        {
            if(claw.getPosition() >= 0.295) break;
            continue;
        }
    }

    public void CloseClaw()
    {
        claw.setPosition(0);
        while(claw.getPosition() >= 0.05)
        {
            if(claw.getPosition() <= 0.05) break;
            continue;
        }
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

        while(opModeIsActive() && timer.seconds() <= 1)
        {
            slide.set(1);
        }
    }
}
