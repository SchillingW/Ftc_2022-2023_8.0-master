package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide
{
    private Motor slide;
    private Servo claw;

    private double minMagnitude = 0.1;
    private double maxMagnitude = 0.5;

    private int errorMargin = 20;
    private double armSpeed = 0.75;

    public int ground = 0; public int low = -1250;
    public int med = -2150; public int high = -3050;
    public int[] slidePositions = {ground, low, med, high};

    public LinearSlide(Telemetry tele, HardwareMap map)
    {
        claw = map.servo.get("claw");
        claw.getController().pwmEnable();

        slide = new Motor(map, "slide");
        slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void goTo(int reachHeight, Telemetry telemetry)
    {
        double slideVector = reachHeight - slide.encoder.getPosition();
        double currentMagnitude = Math.abs(slideVector);
        double targetMagnitude = isAtTarget(reachHeight) ? minMagnitude : maxMagnitude;

        slideVector *= targetMagnitude/currentMagnitude;
        slide.set(slideVector);

        telemetry.addData("Current Slide Pos", slide.encoder.getPosition());
        telemetry.addData("Desired Slide Pos", reachHeight);
        telemetry.addData("Current Slide Magnitude", currentMagnitude);
        telemetry.addData("Target Slide Magnitude", targetMagnitude);
    }

    public void moveByJoystick(double mag) {slide.set(mag * armSpeed);}

    public boolean isAtTarget(int reachHeight) {return (Math.abs(slide.encoder.getPosition() - reachHeight) < errorMargin);}

    public void openClaw() {claw.setPosition(0.5);}
    public void closeClaw() {claw.setPosition(0);}
}
