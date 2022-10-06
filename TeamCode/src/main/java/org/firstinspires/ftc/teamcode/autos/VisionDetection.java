package org.firstinspires.ftc.teamcode.autos;

import android.graphics.Color;
import android.os.Build;

import androidx.annotation.RequiresApi;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.botconfigs.PursuitBot;

@Autonomous(name = "Vision Detection")
public class VisionDetection extends LinearOpMode {
    public PursuitBot robot;
    /**
     * Initilizations of all of hardware
     */

    /**
     * Vuforia
     */


    private static final String VUFORIA_KEY =
            "Ae+gmGj/////AAABmWz20p9iPUvOnbOi93QfB7sXbfkCt0bYRo0ZsF9MfCnyyqSzGT50iAvJq63Zsze7uk3efapcDwvsUKu7VS7cI0PKl2NJjJc3WzUzZw66E7qNLah2J06uP5XNWi262fa0EcXDFRazWernOoMDrdd2Rh6W1l5Wo9m6TWPDXeToJWbxoEAlURg7wosy4dIU5tGFcQNZ8B9ZODO+FxzYKUz7HOQmZ2FVHF7kGtWJsk+7ikLsh80gtIQFs6M9qY8gvTyhUPZJKzzvTGSvbbotaVzpzWd4Brvl1w00NXnGy/rVVr/cvN+6bBIN2/S/Qrxx4OhFF01r5eTNDshoiQV9xTJQ2Zvcl7eVB1C8lqe1RdtM8I1L";


    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public void path1()
    {
        robot.reachPoint(new Pose2d(24, 0, new Rotation2d()));
        robot.reachPoint(new Pose2d(24, 48, new Rotation2d()));
    }

    public void path2() {

    }

    public void path3() {

    }



    public Image getImage(){
        VuforiaLocalizer.CloseableFrame frame = null;
        int r = 0;
        int b = 0;
        int g = 0;
        try{
            frame = vuforia.getFrameQueue().take();
            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
            long numImages = frame.getNumImages();
            Image rgbImage = null;
            int hc;

            for (int i = 0; i < numImages; i++) {
                Image img = frame.getImage(i);

                int fmt = img.getFormat();
                if (fmt == PIXEL_FORMAT.RGB565) {
                    rgbImage = frame.getImage(i);
                    break;
                }
            }
            //telemetry.addData("numImages", numImages);
            //telemetry.addData("rgbImage", rgbImage.getFormat());

            telemetry.update();

            return rgbImage;
        }
        catch(InterruptedException exc){
            return null;
        }
        finally{
            if (frame != null) frame.close();
        }

    }


    @RequiresApi(api = Build.VERSION_CODES.O)
    public void runOpMode() {

        //cameraServo.setPosition(0);
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 20.0 / 20.0);
        }
        int r = 0;
        int b = 0;
        int g = 0;
        int result = 0;
        while (!isStarted()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                String sequenceRGB = getImage().toString();

                telemetry.addData("sequence", sequenceRGB);
                if (sequenceRGB != null) {
                    String hex_value = "#" + sequenceRGB.substring(18, sequenceRGB.length() - 1);
                    telemetry.addData("hex_value", hex_value);
                    try {
                        int rgb_value = Color.parseColor(hex_value);
                        telemetry.addData("rgb", rgb_value);
                        telemetry.addData("rgb", rgb_value);
                    }
                    catch(java.lang.IllegalArgumentException exception)
                    {
                    }

                }


                /*for (int i = 0; i < sequenceRGB; i++) {
                    if (String.valueOf(sequenceRGB).charAt(i) == 'R') {
                        r++;
                    }
                    if (String.valueOf(sequenceRGB).charAt(i) == 'B') {
                        b++;
                    }
                    if (String.valueOf(sequenceRGB).charAt(i) == 'G') {
                        g++;
                    }
                }
                if (r > b && r > g) {
                    result = 1;
                }
                if (b > r && b > g) {
                    result = 2;
                }
                if (g > r && g > b) {
                    result = 3;
                }*/


            }
            telemetry.update();
        }
        waitForStart();

        //switch statements need breaks, otherwise case 1 would run all three cases. Look up fallthrough
        switch (result) {
            case 1:
                path1();
                telemetry.addData("Path 1", 1);
                telemetry.update();
                break;
            case 2:
                path2();
                telemetry.addData("Path 2", 2);
                telemetry.update();
                break;
            case 3:
                path3();
                telemetry.addData("Path 3", 3);
                telemetry.update();
                break;
        }
    }



    //Detect and add if statements for which path to take (path1, path2, path3)


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    }
}





