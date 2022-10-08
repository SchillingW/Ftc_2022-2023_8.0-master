package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;

@Autonomous(name="DriveByTimeTest", group="PursuitBot")
public class DriveByTimeTest extends LinearOpMode {

    public PursuitBot robot;
    public ElapsedTime timer = new ElapsedTime();
    public Motor slide;

    @Override
    public void runOpMode() {

        slide = new Motor(hardwareMap, "slide");
        robot = new PursuitBot(telemetry, hardwareMap);

        waitForStart();
        Up();
        Down();
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
