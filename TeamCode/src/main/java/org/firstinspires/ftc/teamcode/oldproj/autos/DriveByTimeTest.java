package org.firstinspires.ftc.teamcode.oldproj.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBot;

@Autonomous(name="DriveByTimeTest", group="PursuitBot")
public class DriveByTimeTest extends LinearOpMode {

    public PursuitBot robot;


    @Override
    public void runOpMode() {
        robot = new PursuitBot(telemetry, hardwareMap);
    }


}
