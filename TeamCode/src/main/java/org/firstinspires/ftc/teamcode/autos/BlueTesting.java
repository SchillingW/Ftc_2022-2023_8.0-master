package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="BlueTesting", group="PursuitBot")
public class BlueTesting extends LinearOpMode {

    public PursuitBotTesting robot;
    public VisionDevice vision;

    public int dropHeight = -2975;

    public boolean moveToNext;
//hi
    @Override
    public void runOpMode() {

        robot = new PursuitBotTesting(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 1.5;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2.5;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 4;

        moveToNext = false;

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();
        waitForStart();
        int result = vision.perform(1f / 3f);
        sleep(2000);
        telemetry.addData("result", result);
        telemetry.update();
        sleep(2000);

        // START MOVEMENT

        if (opModeIsActive()) sleep(2000);


        // CONE GRABBED

        robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(3), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(3), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(2), new Rotation2d()), telemetry, this);

        // AT DROP CONE LOCATION

        //if (opModeIsActive()) sleep(1000);

        //if (opModeIsActive()) slide.set(1);


        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(3), new Rotation2d()), telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);

        // PARK
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(result + 3), new Rotation2d()), telemetry, this);

    }
}

