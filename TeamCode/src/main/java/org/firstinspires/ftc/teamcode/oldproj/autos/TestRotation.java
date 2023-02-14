package org.firstinspires.ftc.teamcode.oldproj.autos;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.oldproj.hardware.VisionDevice;

@Autonomous(name="TestRotation", group="PursuitBot")
public class TestRotation extends LinearOpMode {

    public PursuitBot robot;

    @Override
    public void runOpMode() {
        robot = new PursuitBot(telemetry, hardwareMap);

        waitForStart();
        robot.Rotate(-90);
        robot.Translate(new Translation2d(40, 0), true);
    }
}

