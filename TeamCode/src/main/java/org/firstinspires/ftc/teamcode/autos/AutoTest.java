package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;

@Autonomous(name="PursuitBotTestAuto", group="PursuitBot")
public class AutoTest extends LinearOpMode {

    public PursuitBot robot;

    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);

        waitForStart();

        robot.reachPoint(new Pose2d(12, 0, new Rotation2d()));
    }
}
