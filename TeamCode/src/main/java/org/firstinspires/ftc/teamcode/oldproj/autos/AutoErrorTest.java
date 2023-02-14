package org.firstinspires.ftc.teamcode.oldproj.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.oldproj.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.oldproj.hardware.VisionDevice;

@Autonomous(name="AutoErrorTest", group="PursuitBot")
public class AutoErrorTest extends LinearOpMode {

    public PursuitBot robot;

    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 1.5;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2.5;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 1;



        //waitForStart();

        /*robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(0), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(5), robot.yDim.toCell(0), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(5), robot.yDim.toCell(5), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(4), robot.yDim.toCell(5), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(4), robot.yDim.toCell(3), new Rotation2d()), telemetry, this, null);*/
}}
