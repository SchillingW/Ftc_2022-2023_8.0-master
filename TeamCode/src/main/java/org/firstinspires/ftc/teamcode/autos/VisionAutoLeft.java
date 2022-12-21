package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="VisionAutoLeft", group="PursuitBot")
public class VisionAutoLeft extends LinearOpMode {

    public PursuitBot robot;
    public VisionDevice vision;

    public boolean moveToNext;
    //auto
    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 1.5;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2.5;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 1;

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();

        waitForStart();
        int result = vision.perform(1f / 3f);
        sleep(2000);
        telemetry.addData("result", result);
        telemetry.update();
        sleep(2000);
        // START MOVEMENT

        // CONE GRABBED

        //robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(1), new Rotation2d()), telemetry, this, null);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this, null);

        if (opModeIsActive()) sleep(2000);

        // PARK
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(result), new Rotation2d()), telemetry, this, null);
    }
}

