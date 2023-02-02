package org.firstinspires.ftc.teamcode.OLD.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.OLD.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.OLD.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.OLD.botconfigs.PursuitBotTesting;
import org.firstinspires.ftc.teamcode.OLD.hardware.VisionDevice;

@Autonomous(name="VisionAutoLeft", group="PursuitBot")
public class VisionAutoLeft extends LinearOpMode {

    public PursuitBotTesting robot;
    public VisionDevice vision;

    public boolean moveToNext;

    @Override
    public void runOpMode() {

        robot = new PursuitBotTesting(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 0.125;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 1;


        int result = 0;
        while (!isStarted()) {
            int next = vision.perform(1f / 3f);
            if (next != -1) result = next;
            telemetry.addData("current result", result);
            telemetry.update();
        }
        // START MOVEMENT
        waitForStart();
        telemetry.update();
        sleep(2000);


        robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(1), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this, null);
        sleep(2000);

        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(result), new Rotation2d()), telemetry, this, null);
    }
}

