package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="BlueAutoTest", group="PursuitBot")
public class BlueAutoTest extends LinearOpMode {

    public PursuitBot robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    public int highDropHeight = -2975;
    public int medDropHeight = -2075;

    public boolean moveToNext;
//hi
    @Override
    public void runOpMode() {

        robot = new PursuitBot(telemetry, hardwareMap);
        robot.xDim.cellcorner2botanchorPLACEMENT = 1.5;
        robot.yDim.cellcorner2botanchorPLACEMENT = 2.5;
        robot.xDim.cellPLACEMENT = 0;
        robot.yDim.cellPLACEMENT = 4;

        linearSlide = new LinearSlide(telemetry, hardwareMap);
        moveToNext = false;
//hello
        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();
        int result = 0;
        while (!isStarted()) {
            int next = vision.perform(1f / 3f);
            if (next != -1) result = next;
            telemetry.addData("current result", result);
        }
        waitForStart();


        // START MOVEMENT

        /*if (opModeIsActive()) linearSlide.closeClaw();
        if (opModeIsActive()) sleep(2000);
        linearSlide.goToFull(linearSlide.low, telemetry, this);

        // CONE GRABBED

        //robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);
        linearSlide.goToFull(linearSlide.med, telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(3), new Rotation2d()), telemetry, this, null);
        // AT DROP CONE LOCATION
        linearSlide.goToFull(medDropHeight + 200, telemetry, this);

        if (opModeIsActive()) linearSlide.openClaw();
        //if (opModeIsActive()) slide.set(1);

        if (opModeIsActive()) sleep(2000);

        // CONE DROPPED

        //robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.low, telemetry, this);

        // PARK
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(result + 3), new Rotation2d()), telemetry, this, null);
        linearSlide.goToFull(linearSlide.ground, telemetry, this);*/
    }
}

