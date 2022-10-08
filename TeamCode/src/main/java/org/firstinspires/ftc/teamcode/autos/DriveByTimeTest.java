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


    @Override
    public void runOpMode() {
        robot = new PursuitBot(telemetry, hardwareMap);
    }


}
