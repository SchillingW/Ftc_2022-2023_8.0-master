package org.firstinspires.ftc.teamcode;

public class FieldDimensions {

    public double botanchor2botcenterHARDWARE;
    public double botanchor2clawcenterHARDWARE;

    public double cellcorner2botanchorPLACEMENT;
    public int cellPLACEMENT;

    public double toCell(int i) {

        return (i + 0.5 - cellPLACEMENT) * 24 - botanchor2botcenterHARDWARE - cellcorner2botanchorPLACEMENT;
    }

    public double toPole(int i) {

        return (i + 1 - cellPLACEMENT) * 24 - botanchor2clawcenterHARDWARE - cellcorner2botanchorPLACEMENT;
    }

    public double toCone(int i) {

        return (i + 0.5 - cellPLACEMENT) * 24 - botanchor2clawcenterHARDWARE - cellcorner2botanchorPLACEMENT;
    }
}
