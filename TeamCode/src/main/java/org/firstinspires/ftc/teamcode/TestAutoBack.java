package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;

@Autonomous(name = "ABack")
//@Disabled
public class TestAutoBack extends LinearOpMode {

    DcMotor motorFrontRight,
    motorFrontLeft,
    motorBackRight,
    motorBackLeft,
    motorWheelRight,
    motorWheelLeft,
    motorLiftRight,
    motorLiftLeft;

    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;


    ColorSensor colorSensor;

    ServoController SC;

    Servo servoCentrRight,
    servoCentrLeft,
    servoJewelRight,
    servoJewelLeft,
    servoGlyth;


    VuforiaLocalizer vuforia;

    ElapsedTime timer = new ElapsedTime();

    double k = 0.001,p,uL=0, uR=0, uo=0, u, u2,u3;
    int dFL = 0, dBL = 0, dBR = 0, dFR = 0;
    double pFL = 0,
            pBL = 0,
            pBR,pBR1,pBR2,pBR3, pBRS,
            pFR,pFR1,pFR2,pFR3, pFRS;



    private GlyphDetector glyphDetector = null;
    int Cryptobox[][], Sipher,JewelPosition, Alliance, a, Column, glyph, OneC = 1000, TwoC = 2000, ToCrypto = 5000;
    double power = 1;

    private JewelDetector jewelDetector = null;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontRight = hardwareMap.dcMotor.get("mFR");
        motorFrontLeft = hardwareMap.dcMotor.get("mFL");
        motorBackRight = hardwareMap.dcMotor.get("mBR");
        motorBackLeft = hardwareMap.dcMotor.get("mBL");


        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorWheelRight = hardwareMap.dcMotor.get("mWR");
        motorWheelLeft = hardwareMap.dcMotor.get("mWL");

//        motorLiftRight = hardwareMap.dcMotor.get("mLR");
//        motorLiftLeft = hardwareMap.dcMotor.get("mLL");

//
//        servoCentrRight = hardwareMap.servo.get("sCR");
//        servoCentrLeft = hardwareMap.servo.get("sCL");
//        servoJewelRight = hardwareMap.servo.get("sJR");
//        servoJewelLeft = hardwareMap.servo.get("sJL");
//        servoGlyth = hardwareMap.servo.get("sG");

//        servoJewelRight.setPosition(0.01);
//        servoJewelLeft.setPosition(0.9);
//        servoCentrRight.setPosition(0.5);
//        servoCentrLeft.setPosition(0.5);
//

        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
        jewelDetector.enable();


//        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
//
//        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;

        telemetry.addData("vu", "Wait");
        telemetry.update();

//        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
//        colorSensor.enableLed(true);

//        modernRoboticsI2cGyro.calibrate();
//        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
//            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
//            telemetry.update();
//            sleep(50);
//        }
//
//        telemetry.log().clear();
//        telemetry.log().add("Gyro Calibrated. Press Start.");
//        telemetry.clear();
//        telemetry.update();

        waitForStart();
        motorWheelRight.setPower(1);
        motorWheelLeft.setPower(-1);
        Yward(0.05, 2000);
        sleep(1000);
        motorWheelRight.setPower(0);
        motorWheelLeft.setPower(0);

//        Yward(0.8, 2000);
//        sleep(2000);
//        Yward(1, 2000);
//
        if (jewelDetector.getLastOrder().toString() == "BLUE_RED") {
            telemetry.addData("JPosition", "BLUE_RED");
            JewelPosition = 1;
        } else {
            telemetry.addData("JPosition", "RED_BLUE");
            JewelPosition = 2;
        }

        jewelDetector.disable();


        if (colorSensor.red() > colorSensor.blue()) {
            telemetry.addData("R", "red");
            telemetry.update();
            Alliance = 1;
        } else {
            telemetry.addData("B", "blue");
            telemetry.update();
            Alliance = 2;
        }


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVGtR4z/////AAAAGR7mv1T9Kk+et5FMVOb0TCEbVxtoc9LEJLzfu6cbgmf8/dTA5Z9FlMw78GFgMsytU5MnyRBKOm+ruNNfz5liVnNaB/5ktZUMGJmEkONfo+0PeZ5GkZyahlZ+QK/BUtVggdorofC9GUHL+Jcnk/67gr+fqWJL/aZgREfCecRXRCfBJ+vVqDT9kVq+Pg5+9kfDThuN4QD8M00w+IJrDwTw8dL2GvLQRtH/ylgVJ5of2mhYVXzPsizjLSojDCr5iGRQJ/0FfKrw3gbGoxZ+rg44OkvtfjFoMIuQLhpP5warc1CToeMQPSAdNQLZbEgjup2OZ8pWtdZLT4BWzg5w9rJGL/ryQWuFt52DHsxD5dk+ax4c";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        sleep(5000);
        relicTrackables.activate();

        while (opModeIsActive() && Column == 0) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    Column = 1;
                    Cryptobox[0][0] = 1;
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    Column = 2;
                    Cryptobox[0][2] = 1;
                }
                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    Column = 3;
                    Cryptobox[0][1] = 1;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
        relicTrackables.deactivate();
        sleep(3000);
        telemetry.addData("123", 123);
        telemetry.update();
        glyphDetector = new GlyphDetector();
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        glyphDetector.minScore = 1;
        glyphDetector.downScaleFactor = 0.3;
        glyphDetector.speed = GlyphDetector.GlyphDetectionSpeed.SLOW;
        glyphDetector.enable();
        sleep(5000);
        glyphDetector.disable();
        if (Alliance == 1) {//red
            if (JewelPosition == 1) {//Blue-Red
//                servoJewelRight.setPosition(0.35);
                sleep(500);
                Yward(0.3, 1000);
//                servoJewelRight.setPosition(0.9);
            } else {//Red-Blue
//                servoJewelRight.setPosition(0.35);
                sleep(500);
                Yward(-0.3, 1000);
//                servoJewelRight.setPosition(0.9);
                sleep(500);
                Yward(0.3, 2000);
            }

            if (Column == 1) {
                Yward(0.2, 2000);
            }
            if (Column == 2) {
                Yward(0.2, 1800);
            }
            if (Column == 3) {
                Yward(0.2, 1600);
            }
            round(0.3, 90);
        } else {//blue
            if (JewelPosition == 2) {//red-blue
//                servoJewelLeft.setPosition(0.35);
                sleep(500);
                Yward(0.3, 1000);
//                servoJewelLeft.setPosition(0.9);
            } else {//Red-Blue
//                servoJewelLeft.setPosition(0.35);
                sleep(500);
                Yward(-0.3, 1000);
//                servoJewelLeft.setPosition(0.9);
                Yward(0.3, 2000);


                if (Column == 1) {
                    Yward(0.2, 1600);
                }
                if (Column == 2) {
                    Yward(0.2, 1800);
                }
                if (Column == 3) {
                    Yward(0.2, 2000);
                }
                round(-0.3, 90);
            }

            sleep(1000);

            Yward(0.5, 800);

//            motorWheelRight.setPower(0.5);
//            motorWheelLeft.setPower(0.5);

            Yward(-0.2, 300);

for (int i=0;i<3;i++) {
    if (c[c][k]== 0){

    }


    if (colorSensor.red() > colorSensor.blue()) {
        switch (Cr[i]) {
            case 1:
                Xward(-1, 1000);
                Yward(-1, 1000);
                //qwe
                Cryptobox[i+1]=2;
                break;
            case 2:
                Xward(1, 1000);
                Yward(-1, 1000);
                //qwe
                Cryptobox[i+1]=1;
                break;
            case 3:
                Sipher = 1;
                telemetry.addData("Sipher", "SNAKE");
                telemetry.update();
                Xward(1, 1000);
                Yward(-1, 1000);
                //qwe
                Cryptobox[i+1]=1;
                break;

        }
    } else {
        switch (Cryptobox[i]) {
            case 1:
                Xward(-1, 500);
                Yward(-1, 1000);
                //qwe
                Cryptobox[i+1]=3;
                break;
            case 2:
                Xward(1, 500);
                Yward(-1, 1000);
                //qwe
                Cryptobox[i+1]=3;
                break;
            case 3:
                Xward(-1, 1000);
                Yward(-1, 1000);
                //qwe
                Cryptobox[i+1]=2;
                break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////
            if (colorSensor.red() > colorSensor.blue()) {
    glyph = 1;}
    else { glyph = 2;
            }
if (Cryptobox[0][0] == 1) {
    if (glyph == 1) {
        Xward(-1, OneC);
        Yward(-1, ToCrypto);
        Cryptobox[0][2] = 1;
    }
    else {//glyph = 2
        Yward(-1, ToCrypto);
        Cryptobox[0][1] = 2;
    }
}
    else {
    if (Cryptobox[0][1] == 1){
        if (glyph == 1){
            Xward(1, OneC);
            Yward(-1, ToCrypto);
            Cryptobox[0][0] = 1;
        }
        else {//glyph = 2
            Xward(-1, OneC);
            Yward(-1, ToCrypto);
            Cryptobox [0][2] = 2;
        }

    }
    if (Cryptobox[0][2] == 2){
        
    }
}


//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////            Alliance = 2;
////            Cryptobox = 1;
//            ///////////////////////////////////////////////////////////////////////////////////////////

        }
    }



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }

    public void round(double p, int digres) {
       sleep(50);
        float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (zAngle > digres) {
            while (opModeIsActive() && zAngle > digres) {
                motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                int rawX = modernRoboticsI2cGyro.rawX();
                int rawY = modernRoboticsI2cGyro.rawY();
                int rawZ = modernRoboticsI2cGyro.rawZ();
                int heading = modernRoboticsI2cGyro.getHeading();
                int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

                AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
                zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
                int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

                telemetry.addLine()
                        .addData("dx", formatRate(rates.xRotationRate))
                        .addData("dy", formatRate(rates.yRotationRate))
                        .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
                telemetry.addData("angle", "%s deg", formatFloat(zAngle));
                telemetry.addData("heading", "%3d deg", heading);
                telemetry.addData("integrated Z", "%3d", integratedZ);
                telemetry.addLine()
                        .addData("rawX", formatRaw(rawX))
                        .addData("rawY", formatRaw(rawY))
                        .addData("rawZ", formatRaw(rawZ));
                telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
                telemetry.update();

                u = k * (dFR - dFL);

                pFL = p + u;
                pFR1 = p - u;



                u2 = k * (dFR - dBL);
                pBL = p + u2;
                pFR2 = p - u2;

                u3 = k * (dFR - dBR);
                pBR = p + u3;
                pFR3 = p - u3;

                pFRS = (pFR1 + pFR2 + pFR3)/3;

                motorFrontRight.setPower(-pFRS);
                motorFrontLeft.setPower(-pFL);
                motorBackRight.setPower(-pBR);
                motorBackLeft.setPower(-pBL);
            }
        } else {
            if (zAngle < digres) {
                while (opModeIsActive() && zAngle < digres) {
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    int rawX = modernRoboticsI2cGyro.rawX();
                    int rawY = modernRoboticsI2cGyro.rawY();
                    int rawZ = modernRoboticsI2cGyro.rawZ();
                    int heading = modernRoboticsI2cGyro.getHeading();
                    int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

                    AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
                    zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                    int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
                    int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

                    telemetry.addLine()
                            .addData("dx", formatRate(rates.xRotationRate))
                            .addData("dy", formatRate(rates.yRotationRate))
                            .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
                    telemetry.addData("angle", "%s deg", formatFloat(zAngle));
                    telemetry.addData("heading", "%3d deg", heading);
                    telemetry.addData("integrated Z", "%3d", integratedZ);
                    telemetry.addLine()
                            .addData("rawX", formatRaw(rawX))
                            .addData("rawY", formatRaw(rawY))
                            .addData("rawZ", formatRaw(rawZ));
                    telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
                    telemetry.update();

                    u = k * (dFR - dFL);

                    pFL = p + u;
                    pFR1 = p - u;



                    u2 = k * (dFR - dBL);
                    pBL = p + u2;
                    pFR2 = p - u2;

                    u3 = k * (dFR - dBR);
                    pBR = p + u3;
                    pFR3 = p - u3;

                    pFRS = (pFR1 + pFR2 + pFR3)/3;

                    motorFrontRight.setPower(pFRS);
                    motorFrontLeft.setPower(pFL);
                    motorBackRight.setPower(pBR);
                    motorBackLeft.setPower(pBL);
                }
            }
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }


    public void Yward (double p, int digres){
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontRight.setTargetPosition(digres);

        while (opModeIsActive() && Math.abs(motorFrontRight.getCurrentPosition()) < Math.abs(motorFrontRight.getTargetPosition())){
            motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

            dFR = motorFrontRight.getCurrentPosition();
            dFL = motorFrontLeft.getCurrentPosition();
            dBR = motorBackRight.getCurrentPosition();
            dBL = motorBackLeft.getCurrentPosition();

            u = k * (dFR - dFL);

            pFL = p + u;
            pFR1 = p - u;



            u2 = k * (dFR - dBL);
            pBL = p + u2;
            pFR2 = p - u2;

            u3 = k * (dFR - dBR);
            pBR = p + u3;
            pFR3 = p - u3;

            pFRS = (pFR1 + pFR2 + pFR3)/3;

            motorFrontRight.setPower(pFRS);
            motorFrontLeft.setPower(pFL);
            motorBackRight.setPower(pBR);
            motorBackLeft.setPower(pBL);

//            telemetry.addData("FR", dFR);
//            telemetry.addData("FL", dFL);
//            telemetry.addData("BL", dBL);

            telemetry.update();
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
    }

//    public void backward (double power, int digres){
//        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motorFrontLeft.setTargetPosition(digres);
//
//        while (opModeIsActive() && Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()){
//            dFR = motorFrontRight.getCurrentPosition();
//            dFL = motorFrontLeft.getCurrentPosition();
//            dBR = motorBackRight.getCurrentPosition();
//            dBL = motorBackLeft.getCurrentPosition();
//
//            u = k * (dBL + dBR);
//
//            pBL = p - u;
//            pBR = p + u;
//
//            u2 = k * (dBL - dFR);
//            pFR = p + u2;
//
//            u3 = k * (dBL + dFL);
//            pFR = p + u3;
//
//            motorFrontRight.setPower(pFR);
//            motorFrontLeft.setPower(-pFL);
//            motorBackRight.setPower(pBR);
//            motorBackLeft.setPower(-pBL);
//        }
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackRight.setPower(0);
//        motorBackLeft.setPower(0);
//
//        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//
    public void Xward (double p, int digres){
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackRight.setTargetPosition(digres);

        while (opModeIsActive() && Math.abs(motorBackRight.getCurrentPosition()) < Math.abs(motorBackRight.getTargetPosition())){
            motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
            motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

            dFR = motorFrontRight.getCurrentPosition();
            dFL = motorFrontLeft.getCurrentPosition();
            dBR = motorBackRight.getCurrentPosition();
            dBL = motorBackLeft.getCurrentPosition();

            u = k * (dBR - dFR);

            pFR = p + u;
            pBR1 = p - u;

            u2 = k * (dBR - dBL);
            pBL = p + u2;
            pBR2 = p - u2;

            u3 = k * (dBR - dFL);
            pFL = p + u3;
            pBR3 = p - u3;

            pBRS = (pBR1 + pBR2 + pBR3)/3;

            motorFrontRight.setPower(pFR);
            motorFrontLeft.setPower(pFL);
            motorBackRight.setPower(pBRS);
            motorBackLeft.setPower(pBL);
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
    }
//    public void leftward (double power, int digres){
//        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motorFrontLeft.setTargetPosition(digres);
//
//        while (opModeIsActive() && Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()){
//            dFR = motorFrontRight.getCurrentPosition();
//            dFL = motorFrontLeft.getCurrentPosition();
//            dBR = motorBackRight.getCurrentPosition();
//            dBL = motorBackLeft.getCurrentPosition();
//
//
//            u = k * (dBL + dFL);
//
//            pBL = p - u;
//            pFL = p + u;
//
//            u2 = k * (dBL - dBR);
//            pBR = p + u2;
//
//            u3 = k * (dBL + dFR);
//            pFR = p + u3;
//
//            motorFrontRight.setPower(-pFR);
//            motorFrontLeft.setPower(-pFL);
//            motorBackRight.setPower(pBR);
//            motorBackLeft.setPower(pBL);
//        }
//        motorFrontRight.setPower(0);
//        motorFrontLeft.setPower(0);
//        motorBackRight.setPower(0);
//        motorBackLeft.setPower(0);
//
//        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }

}

