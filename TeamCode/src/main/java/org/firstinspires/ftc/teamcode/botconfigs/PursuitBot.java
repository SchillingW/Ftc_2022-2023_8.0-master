package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MecDriveFlip;

import java.util.function.DoubleSupplier;

// mecanum drive bot with odometry for Pure Pursuit
public class PursuitBot {

    // debugging device
    public Telemetry tele;

    // mecanum wheel drive train
    public MecanumDrive drive;
    public Motor motorFL;
    public Motor motorFR;
    public Motor motorBL;
    public Motor motorBR;

    // odometry device
    public OdometrySubsystem odometry;
    public DoubleSupplier encoderL;
    public DoubleSupplier encoderR;
    public DoubleSupplier encoderH;

    // hardware specifications
    public double wheelDiameter = 2.5;
    public double wheelCircumference = wheelDiameter * Math.PI;

    // robot type data
    public double encoderTrackWidth = 5.75;
    public double encoderWheelOffset = 0.0;

    // initialize devices
    public PursuitBot(Telemetry tele, HardwareMap map) {

        // store debugging device
        this.tele = tele;

        // initialize drive train
        motorFL = new Motor(map, "motorFL");
        motorFR = new Motor(map, "motorFR");
        motorBL = new Motor(map, "motorBL");
        motorBR = new Motor(map, "motorBR");
        drive = new MecDriveFlip(motorFL, motorFR, motorBL, motorBR);

        // initialize odometry
        encoderL = getSupplier(motorFL, 1);
        encoderR = getSupplier(motorFR, -1);
        encoderH = getSupplier(motorBL, -1);
        odometry = new OdometrySubsystem(new HolonomicOdometry(
                encoderL, encoderR, encoderH,
                encoderTrackWidth, encoderWheelOffset));

        // orient to home
        motorFL.resetEncoder();
        motorFR.resetEncoder();
        motorBL.resetEncoder();
        motorBR.resetEncoder();
    }

    // return double supplier representing motor value in inches
    public DoubleSupplier getSupplier(Motor encoder, float coefficient) {

        // convert motor ticks to inches
        double ticksPerInch = 8192 / wheelCircumference;
        return () -> encoder.getCurrentPosition() / ticksPerInch * coefficient;
    }




    public boolean isDoneCorrectingRotation;

    public double movementSpeed = 0.5;
    public double turnSpeed = 0.5;
    public double followRadius = 5;
    public double positionBuffer = 1;
    public double rotationBuffer = Math.toRadians(15);
    public double maxVelocity = 0.5;
    public double maxAcceleration = 0.5;

    public PurePursuitCommand generateCommand(Pose2d... recording) {

        // create start and end waypoints from current pose to last pose in recording
        Waypoint[] points = new Waypoint[recording.length + 2];
        points[0] = new StartWaypoint(odometry.getPose());
        points[points.length - 1] = new EndWaypoint(recording[recording.length - 1],
                movementSpeed, turnSpeed, followRadius, positionBuffer, rotationBuffer);

        // iterate through recorded poses and convert to waypoints hi
        for (int i = 1; i < points.length - 1; i++) {
            points[i] = new GeneralWaypoint(recording[i - 1],
                    movementSpeed, turnSpeed, followRadius);
        }

        // follow path formed by waypoints
        PurePursuitCommand command = new PurePursuitCommand(
                drive, odometry, points);

        return command;
    }

    // run command linearly hi
    public void RunCommand(PurePursuitCommand command, LinearOpMode mode) {

        // follow path hi
        command.schedule();

        // loop while following
        while (mode.opModeIsActive() && !command.isFinished()) {
            command.execute();
            odometry.update();
        }

        command.end(true);
        drive.stop();
        isDoneCorrectingRotation = false;

        //if(state.equals("return home"))
        //{
            //HomeRotationalCorrection(mode);
        //}

        // wait a second
        if (mode.opModeIsActive()) mode.sleep(1000);
    }

    public void HomeRotationalCorrection(LinearOpMode mode)
    {
        drive.stop();

        if(odometry.getPose().getRotation().getDegrees() != 0.0)
        {
            if(!isDoneCorrectingRotation) {

                Rotation2d currentRotation = odometry.getPose().getRotation();
//hello
                if (currentRotation.getDegrees() >= 0.0) {
                    while (currentRotation.getDegrees() >= 0.0 && mode.opModeIsActive()) {
                        drive.driveRobotCentric(0.0, 0.0, -2.0);
                        odometry.update();
                        currentRotation = odometry.getPose().getRotation();

                        if (currentRotation.getDegrees() <= 0) {
                            isDoneCorrectingRotation = true;
                            drive.stop();
                            break;
                        }

                    }
                }

                else if (currentRotation.getDegrees() <= 0.0) {
                    while (currentRotation.getDegrees() <= 0.0 && mode.opModeIsActive()) {
                        drive.driveRobotCentric(0.0, 0.0, 2.0);
                        odometry.update();
                        currentRotation = odometry.getPose().getRotation();

                        if (currentRotation.getDegrees() >= 0) {
                            isDoneCorrectingRotation = true;
                            drive.stop();
                            break;
                        }
                    }
                }

                else
                {
                    return;
                }
            }

        }
    }
}
