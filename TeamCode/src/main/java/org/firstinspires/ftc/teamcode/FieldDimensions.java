package org.firstinspires.ftc.teamcode;

public class FieldDimensions {

    public double start;
    public double clawOffset;
    public double cellOffset;

    public double toCell(int i) {

        return (i + 0.5) * 24 - start + cellOffset;
    }

    public double toPole(int i) {

        return (i + 1) * 24 - start - clawOffset;
    }
}
