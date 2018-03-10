package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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

@Autonomous(name = "AFront")
public class TestAutoFront extends LinearOpMode {

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

    int Column, JewelPosition, Alliance;

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

        motorLiftRight = hardwareMap.dcMotor.get("mLR");
        motorLiftLeft = hardwareMap.dcMotor.get("mLL");


        SC = hardwareMap.servoController.get("SS");
        servoCentrRight = hardwareMap.servo.get("sCR");
        servoCentrLeft = hardwareMap.servo.get("sCL");
        servoJewelRight = hardwareMap.servo.get("sJR");
        servoJewelLeft = hardwareMap.servo.get("sJL");
        servoGlyth = hardwareMap.servo.get("sG");

        modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50); }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();


        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
        jewelDetector.enable();





        waitForStart();


        if (jewelDetector.getCurrentOrder().toString() == "BLUE_RED") {
            JewelPosition = 1;
        } else {
            JewelPosition = 2;
        }

        jewelDetector.disable();

        if(colorSensor.red()>colorSensor.blue()){
            telemetry.addData("R","red"); telemetry.update();
            Alliance=1;
        }else{
            telemetry.addData("B","blue"); telemetry.update();
            Alliance=2;
        }

        sleep(1000);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVGtR4z/////AAAAGR7mv1T9Kk+et5FMVOb0TCEbVxtoc9LEJLzfu6cbgmf8/dTA5Z9FlMw78GFgMsytU5MnyRBKOm+ruNNfz5liVnNaB/5ktZUMGJmEkONfo+0PeZ5GkZyahlZ+QK/BUtVggdorofC9GUHL+Jcnk/67gr+fqWJL/aZgREfCecRXRCfBJ+vVqDT9kVq+Pg5+9kfDThuN4QD8M00w+IJrDwTw8dL2GvLQRtH/ylgVJ5of2mhYVXzPsizjLSojDCr5iGRQJ/0FfKrw3gbGoxZ+rg44OkvtfjFoMIuQLhpP5warc1CToeMQPSAdNQLZbEgjup2OZ8pWtdZLT4BWzg5w9rJGL/ryQWuFt52DHsxD5dk+ax4c";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

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
                }
                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    Column = 2;
                }
                if (vuMark == RelicRecoveryVuMark.CENTER) {
                    Column = 3;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }

        sleep(500);


        if (JewelPosition==1){
            if(Alliance==1) {//RED
                servoJewelLeft.setPosition(0.2);
                backward(0.5, 2000);
                servoJewelLeft.setPosition(0.2);
            }else {
                servoJewelRight.setPosition(0.9);
                round(0.2, 15);
                servoJewelRight.setPosition(0.2);
                sleep(500);
                round(0.2, -1);
                backward(0.5, 2000);
            }
        }
        else {
            if (Alliance == 1) {//RED
                servoJewelLeft.setPosition(0.9);
                round(0.2, 15);
                servoJewelLeft.setPosition(0.2);
                sleep(500);
                round(0.2, 3);
                backward(0.5, 2000);
            } else {
                servoJewelRight.setPosition(0.9);
                backward(0.5, 2000);
                servoJewelRight.setPosition(0.2);
            }
        }

        if(Alliance==1){//RED
            if (Column == 1){backward(0.5, 2000);rightward(0.5,2000);}//LEFT
            if (Column == 2){backward(0.5, 2000);rightward(0.5,2000);}//RIGHT
            if (Column == 3){backward(0.5, 2000);rightward(0.5,2000);}
            else{//BLUE
                if (Column == 1){backward(0.5,2000);leftward(0.5,2000);}//LEFT
                if (Column == 2){backward(0.5,2000);leftward(0.5,2000);}//RIGHT
                if (Column == 3){backward(0.5,2000);leftward(0.5,2000);}
            }

            backward(1, 2000);
            servoCentrRight.setPosition(1);
            servoCentrLeft.setPosition(0);
            forward(0.5, 1000);


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

                motorFrontRight.setPower(p);
                motorFrontLeft.setPower(p);
                motorBackLeft.setPower(p);
                motorBackRight.setPower(p);
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

                    motorFrontRight.setPower(-p);
                    motorFrontLeft.setPower(-p);
                    motorBackLeft.setPower(-p);
                    motorBackRight.setPower(-p);
                }
            }
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }

    public void forward (double power, int digres){
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setTargetPosition(digres);

        while (opModeIsActive() && Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()){
            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(-power);
            motorBackRight.setPower(power);
            motorBackLeft.setPower(-power);
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void backward (double power, int digres){
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setTargetPosition(digres);

        while (opModeIsActive() && Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()){
            motorFrontRight.setPower(-power);
            motorFrontLeft.setPower(power);
            motorBackRight.setPower(-power);
            motorBackLeft.setPower(power);
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void rightward (double power, int digres){
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setTargetPosition(digres);

        while (opModeIsActive() && Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()){
            motorFrontRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackRight.setPower(power);
            motorBackLeft.setPower(power);
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void leftward (double power, int digres){
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setTargetPosition(digres);

        while (opModeIsActive() && Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()){
            motorFrontRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackRight.setPower(-power);
            motorBackLeft.setPower(-power);
        }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}


