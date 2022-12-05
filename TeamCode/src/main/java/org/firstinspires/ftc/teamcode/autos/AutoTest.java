package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="AutoTest", group="PursuitBot")
public class AutoTest extends LinearOpMode {

    public PursuitBot robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    public int dropOffset = 50;

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

        linearSlide = new LinearSlide(telemetry, hardwareMap);

        waitForStart();
        int result = vision.perform(1f / 3f);
        sleep(500);
        telemetry.addData("result", result);
        telemetry.update();
        sleep(200);
        // START MOVEMENT

        if (opModeIsActive()) linearSlide.closeClaw();
        if (opModeIsActive()) sleep(2000);
        linearSlide.goToFull(linearSlide.low, telemetry, this);

        // CONE GRABBED
        robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        //linearSlide.goToFull(linearSlide.high, telemetry, this);
        //linearSlide.goToFull(linearSlide.low, telemetry, this);

        // AT DROP CONE LOCATION
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(1), new Rotation2d()), telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(1), new Rotation2d()), telemetry, this);

        linearSlide.goToFull(linearSlide.med + dropOffset, telemetry, this);
        sleep(1000);
        //if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) linearSlide.openClaw();
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        //if (opModeIsActive()) slide.set(1);
        sleep(100);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(0), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0) + 4.5, new Rotation2d()), telemetry, this);
        if (opModeIsActive()) sleep(2000);
        Cycle(0);
        sleep(1000);
        Cycle(1);
        sleep(1000);
        Cycle(2);


        //robot.RotateLeft(-90, 0.3, telemetry, this);

        linearSlide.goToFull(linearSlide.high, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(1), new Rotation2d()), telemetry, this);
        sleep(1000);
        //if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) linearSlide.openClaw();


        // CONE DROPPED


        linearSlide.goToFull(linearSlide.low, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);

        // PARK
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(result), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.ground, telemetry, this);
    }

    public void GrabConeFromCell(int i)
    {
        robot.CellToStack(0.3, telemetry, this, robot.sensor);
        robot.TranslateY(-0.55, 0.2, telemetry, this);
        linearSlide.goToFull(linearSlide.stacks[i], telemetry, this);
        sleep(1000);
        linearSlide.closeClaw();
        sleep(1000);
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        robot.TranslateY(0.55, -0.2, telemetry, this);
        robot.StackToCell(0.3, telemetry, this, robot.sensor);
    }

    public void Cycle(int i)
    {
        GrabConeFromCell(i);
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(0), new Rotation2d()), telemetry, this);
        robot.InvertRot(0.3, telemetry, this, robot.sensor);
        linearSlide.goToFull(linearSlide.low + dropOffset, telemetry, this);
        sleep(1000);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(500);
        robot.InvertRot(0.3, telemetry, this, robot.sensor);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0) + 4.5, new Rotation2d()), telemetry, this);

    }
}

