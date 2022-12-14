package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="BlueTesting", group="PursuitBot")
public class BlueTesting extends LinearOpMode {

    public PursuitBot robot;
    public VisionDevice vision;
    public LinearSlide linearSlide;
    public int dropOffset = 80;

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
        sleep(1000);
        int result = 0;
        while (!isStarted()) result = vision.perform(1f / 3f);
        // START MOVEMENT
        waitForStart();







        if (opModeIsActive()) linearSlide.closeClaw();
        if (opModeIsActive()) sleep(1300);
        linearSlide.goToFull(linearSlide.low, telemetry, this);

        // CONE GRABBED
        /*robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        //linearSlide.goToFull(linearSlide.high, telemetry, this);
        //linearSlide.goToFull(linearSlide.low, telemetry, this);

        linearSlide.goToFull(linearSlide.med + dropOffset, telemetry, this);
        // AT DROP CONE LOCATION
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(1), new Rotation2d()), telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(1), new Rotation2d()), telemetry, this);
*/
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this, null);
        //robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(0), new Rotation2d()), telemetry, this, null);
        /*linearSlide.goToFull(linearSlide.ground, telemetry, this);
        sleep(200);
        //if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) linearSlide.openClaw();
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        //if (opModeIsActive()) slide.set(1);
        /*sleep(100);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);*/
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(0), new Rotation2d()), telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0) + 4.5, new Rotation2d()), telemetry, this, null);
        if (opModeIsActive()) sleep(200);
        Cycle(0);
        Cycle(1);

        // PARK
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(result), new Rotation2d()), telemetry, this, null);
        linearSlide.goToFull(linearSlide.ground, telemetry, this);
    }

    public void GrabConeFromCell(int i)
    {
        robot.CellToStackRight(0.35, telemetry, this, robot.sensor);
        robot.TranslateY(-0.55, 0.2, telemetry, this);
        linearSlide.goToFull(linearSlide.stacks[i], telemetry, this);
        sleep(200);
        linearSlide.closeClaw();
        sleep(300);
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        robot.TranslateY(0.55, -0.2, telemetry, this);
        //robot.StackToCell(0.3, telemetry, this, robot.sensor);
        //robot.reachPoint(new Pose2d(robot.odometry.getPose().getX(), robot.odometry.getPose().getY(), new Rotation2d()), telemetry, this, null);
    }

    public void Cycle(int i)
    {
        GrabConeFromCell(i);
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        linearSlide.goToFull(linearSlide.high + dropOffset, telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(1), new Rotation2d()), telemetry, this, null);
        //robot.InvertRot(0.3, telemetry, this, robot.sensor);
        if (opModeIsActive()) linearSlide.openClaw();
        sleep(500);
        //robot.InvertRot(0.3, telemetry, this, robot.sensor);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0) + 4.5, new Rotation2d()), telemetry, this, null);

    }
}

