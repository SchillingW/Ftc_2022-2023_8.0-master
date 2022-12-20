package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.botconfigs.LinearSlide;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;
import org.firstinspires.ftc.teamcode.hardware.VisionDevice;

@Autonomous(name="TwoConesAutoRight", group="PursuitBot")
public class TwoConesAutoRight extends LinearOpMode {

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
        robot.yDim.cellPLACEMENT = 4;

        vision = new VisionDevice(telemetry, hardwareMap);
        vision.init();

        linearSlide = new LinearSlide(telemetry, hardwareMap);
        sleep(1000);
        int result = 0;
        while (!isStarted()) {
            int next = vision.perform(1f / 3f);
            if (next != -1) result = next;
            telemetry.addData("current result", result);
        }






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
        /*robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.high, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(1), new Rotation2d()), telemetry, this);*/

        robot.reachPoint(new Pose2d(robot.xDim.toCell(0), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);
        linearSlide.goToFull(linearSlide.med, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(3), new Rotation2d()), telemetry, this, null);

        //linearSlide.goToFull(linearSlide.med + dropOffset, telemetry, this);
        sleep(200);
        //if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) linearSlide.openClaw();
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        //if (opModeIsActive()) slide.set(1);
        /*sleep(100);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);*/
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(0), new Rotation2d()), telemetry, this);
        //robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(2), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(5) - 4.5, new Rotation2d()), telemetry, this, null);
        if (opModeIsActive()) sleep(200);
        GrabConeFromCell(0);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(4), new Rotation2d()), telemetry, this, null);

        if(result == 1 || result == 2)
        {
            linearSlide.goToFull(linearSlide.med, telemetry, this);
            robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(3), new Rotation2d()), telemetry, this, null);
            sleep(200);
            //if (opModeIsActive()) sleep(1000);
            if (opModeIsActive()) linearSlide.openClaw();
            linearSlide.goToFull(linearSlide.low, telemetry, this);
        }

        else
        {
            linearSlide.goToFull(linearSlide.low, telemetry, this);
            robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(4), new Rotation2d()), telemetry, this, null);
            sleep(200);
            //if (opModeIsActive()) sleep(1000);
            if (opModeIsActive()) linearSlide.openClaw();
            linearSlide.goToFull(linearSlide.low, telemetry, this);
        }

        // PARK
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(result + 3), new Rotation2d()), telemetry, this, null);
        linearSlide.goToFull(linearSlide.ground, telemetry, this);
    }

    public void GrabConeFromCell(int i)
    {
        robot.CellToStackRight(0.35, telemetry, this, robot.sensor);
        robot.TranslateY(0.55, 0.2, telemetry, this);
        linearSlide.goToFull(linearSlide.stacks[i], telemetry, this);
        sleep(500);
        linearSlide.closeClaw();
        sleep(500);
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        robot.TranslateY(-0.55, -0.2, telemetry, this);
        robot.reachPoint(new Pose2d(robot.odometry.getPose().getX(), robot.odometry.getPose().getY(), new Rotation2d()), telemetry, this, null);
    }

    public void Cycle(int i)
    {
        /*GrabConeFromCell(i);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(1), robot.yDim.toPole(1), new Rotation2d()), telemetry, this);
        linearSlide.goToFull(linearSlide.med, telemetry, this);
        sleep(200);
        //if (opModeIsActive()) sleep(1000);
        if (opModeIsActive()) linearSlide.openClaw();
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(1), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0) + 4.5, new Rotation2d()), telemetry, this);*/

        /*
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(1), new Rotation2d()), telemetry, this);
        if (opModeIsActive()) sleep(2000);
        if (opModeIsActive()) linearSlide.openClaw();
        linearSlide.goToFull(linearSlide.low, telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(1), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0) + 4.5, new Rotation2d()), telemetry, this);
         */
        /*robot.reachPoint(new Pose2d(robot.xDim.toPole(2), robot.yDim.toPole(0), new Rotation2d()), telemetry, this);
        robot.GroundToLow(0.35, telemetry, this, robot.sensor);
        if (opModeIsActive()) linearSlide.openClaw();
        if(opModeIsActive()) sleep(300);
        robot.reachPoint(new Pose2d(robot.odometry.getPose().getX(), robot.odometry.getPose().getY(), new Rotation2d()), telemetry, this);
        robot.reachPoint(new Pose2d(robot.xDim.toCell(2), robot.yDim.toCell(0) + 4.5, new Rotation2d()), telemetry, this);*/
    }
}

