package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="AB", group = "Au")
@Disabled
public class AutoBack extends LinearOpMode {
    DcMotor motorFrontRight,
            motorFrontLeft,
            motorBackRight,
            motorBackLeft,
            Center,
            Lift;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ServoController SS;
    ColorSensor colorSensor;
    Servo servo;

    int n, V,

    ////////////////////////////////// ////////////////////////////////// ////////////////////////////////// //////////////////////////////////
            nearR = 2500,
            nearB = 2000,

            centerR = 3400,
            centerB = 3600,

            farR = 4600,
            farB = 4600,

            r = -77,
            end = 2000;
    ////////////////////////////////// ////////////////////////////////// ////////////////////////////////// //////////////////////////////////
/**/

    VuforiaLocalizer vuforia;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("mFR");
        motorFrontLeft = hardwareMap.dcMotor.get("mFL");
        motorBackRight = hardwareMap.dcMotor.get("mBR");
        motorBackLeft = hardwareMap.dcMotor.get("mBL");
        SS = hardwareMap.servoController.get("SS");
        servo = hardwareMap.servo.get("servo");

        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Center = hardwareMap.dcMotor.get("C");

        Lift = hardwareMap.dcMotor.get("L");
        Lift.setMode(DcMotor.RunMode.RESET_ENCODERS);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        telemetry.addData("vu", "Wait");
        telemetry.update();

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//        parameters.vuforiaLicenseKey = "AVGtR4z/////AAAAGR7mv1T9Kk+et5FMVOb0TCEbVxtoc9LEJLzfu6cbgmf8/dTA5Z9FlMw78GFgMsytU5MnyRBKOm+ruNNfz5liVnNaB/5ktZUMGJmEkONfo+0PeZ5GkZyahlZ+QK/BUtVggdorofC9GUHL+Jcnk/67gr+fqWJL/aZgREfCecRXRCfBJ+vVqDT9kVq+Pg5+9kfDThuN4QD8M00w+IJrDwTw8dL2GvLQRtH/ylgVJ5of2mhYVXzPsizjLSojDCr5iGRQJ/0FfKrw3gbGoxZ+rg44OkvtfjFoMIuQLhpP5warc1CToeMQPSAdNQLZbEgjup2OZ8pWtdZLT4BWzg5w9rJGL/ryQWuFt52DHsxD5dk+ax4c";
//
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        modernRoboticsI2cGyro.calibrate();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50); }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        waitForStart();

        relicTrackables.activate();
        telemetry.log().clear();
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        sleep(50);

        if(colorSensor.red()>colorSensor.blue()){
            telemetry.addData("R","red"); telemetry.update();

            V=1;
        }else{
            telemetry.addData("B","blue"); telemetry.update();
            V=2;
        }

        Center.setPower(0.3);
        sleep(1000);

//        Lift.setPower(0.5);
//        sleep(1000);
//        Lift.setPower(0);

        servo.setPosition(0.9);
        sleep(2000);

        if (colorSensor.red()>colorSensor.blue()){
            telemetry.addData("color", "red"); telemetry.update();
            sleep(500);
            if(V==1) {//RED
                round(0.2, 15);
                servo.setPosition(0.2);
                sleep(500);
                round(0.2, -1);
            }else {
                round(0.2, -15);
                servo.setPosition(0.2);
                sleep(500);
                round(0.2, 3);
            }
        }
        else{
            telemetry.addData("color", "blue"); telemetry.update();
            sleep(500);
            if(V==1) {//RED
                round(0.2, -15);
                servo.setPosition(0.2);
                sleep(500);
                round(0.2, -3);
            }else {
                round(0.2, 15);
                servo.setPosition(0.2);
                sleep(500);
                round(0.2, 3);
            }
        }

        sleep(2000);
        while (opModeIsActive()&&n==0) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
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
                if (vuMark==RelicRecoveryVuMark.LEFT){n=1;}
                if (vuMark==RelicRecoveryVuMark.RIGHT){n=2;}
                if (vuMark==RelicRecoveryVuMark.CENTER){n=3;}
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

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


        if(V==1){//RED
            if (n == 1){forward(0.2, farR, 0.001);}//LEFT
            if (n == 2){forward(0.2, nearR, 0.001);}//RIGHT
            if (n == 3){forward(0.2, centerR, 0.001);}//Center
            round(0.2, r);
            sleep(500);
        }else{//BLUE
            if (n == 1){backward(0.2, nearB, 0.001);}//LEFT
            if (n == 2){backward(0.2, farB, 0.001);}//RIGHT
            if (n == 3){backward(0.2, centerB, 0.001);}//Center
            round(0.2, r);
            sleep(500);
        }
        sleep(500);

        Lift.setPower(-0.5);
        sleep(1000);
        Lift.setPower(0);

        forward(0.4, end, 0.001);

        Center.setPower(-0.3);
        sleep(1000);
        Center.setPower(0);

        backward(0.2, 500, 0.001);
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
            while (opModeIsActive()&&zAngle > digres) {
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
                motorBackRight.setPower(p); } }
        else {        if (zAngle < digres) {
            while (opModeIsActive()&&zAngle < digres) {
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
                motorBackRight.setPower(-p); } } }
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0); }

    public void forward(double p, int digres, double k) {
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setTargetPosition(digres);

        int gl = 0, gr = 0, gl2 = 0, gr2 = 0;
        double u = 0, pl = 0, pr = 0, u2 = 0, pl2 = 0, pr2 = 0, u3 = 0;

        while (opModeIsActive()&&Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()) {

            telemetry.addLine("pos | ")
                    .addData("FrontLeft", motorFrontLeft.getCurrentPosition())
                    .addData("FrontRight", motorFrontRight.getCurrentPosition())
                    .addData("BackRight", motorBackRight.getCurrentPosition())
                    .addData("BackLeft", motorBackLeft.getCurrentPosition());
            telemetry.update();
            gr = motorFrontRight.getCurrentPosition();
            gl = motorFrontLeft.getCurrentPosition();
            u = k * (gl + gr);

            pl = p - u;
            pr = p + u;
            motorFrontRight.setPower(-pr);
            motorFrontLeft.setPower(pl);

            gl2 = motorBackLeft.getCurrentPosition();
            u2 = k * (gl - gl2);
            pl2 = p + u2;
            motorBackLeft.setPower(pl2);

            gr2 = motorBackRight.getCurrentPosition();
            u3 = k * (gl + gr2);
            pr2 = p + u3;
            motorBackRight.setPower(-pr2);
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

    public void backward(double p, int digres, double k) {
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setTargetPosition(digres);

        int gl = 0, gr = 0, gl2 = 0, gr2 = 0;
        double u = 0, pl = 0, pr = 0, u2 = 0, pl2 = 0, pr2 = 0, u3 = 0;

        while (opModeIsActive()&&Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()) {

            telemetry.addLine("pos | ")
                    .addData("FrontLeft", motorFrontLeft.getCurrentPosition())
                    .addData("FrontRight", motorFrontRight.getCurrentPosition())
                    .addData("BackRight", motorBackRight.getCurrentPosition())
                    .addData("BackLeft", motorBackLeft.getCurrentPosition());
            telemetry.update();
            gr = motorBackLeft.getCurrentPosition();
            gl = motorBackRight.getCurrentPosition();
            u = k * (gl + gr);

            pl = p - u;
            pr = p + u;
            motorBackLeft.setPower(-pr);
            motorBackRight.setPower(pl);

            gl2 = motorFrontRight.getCurrentPosition();
            u2 = k * (gl - gl2);
            pl2 = p + u2;
            motorFrontRight.setPower(pl2);

            gr2 = motorFrontLeft.getCurrentPosition();
            u3 = k * (gl + gr2);
            pr2 = p + u3;
            motorFrontLeft.setPower(-pr2);
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

    public void leftward(double p, int digres, double k) {
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setTargetPosition(digres);

        int gl = 0, gr = 0, gl2 = 0, gr2 = 0;
        double u = 0, pl = 0, pr = 0, u2 = 0, pl2 = 0, pr2 = 0, u3 = 0;

        while (opModeIsActive()&&Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()) {

            telemetry.addLine("pos | ")
                    .addData("FrontLeft", motorFrontLeft.getCurrentPosition())
                    .addData("FrontRight", motorFrontRight.getCurrentPosition())
                    .addData("BackRight", motorBackRight.getCurrentPosition())
                    .addData("BackLeft", motorBackLeft.getCurrentPosition());
            telemetry.update();
            gr = motorFrontLeft.getCurrentPosition();
            gl = motorBackLeft.getCurrentPosition();
            u = k * (gl + gr);

            pl = p - u;
            pr = p + u;
            motorFrontLeft.setPower(-pr);
            motorBackLeft.setPower(pl);

            gl2 = motorBackRight.getCurrentPosition();
            u2 = k * (gl - gl2);
            pl2 = p + u2;
            motorBackRight.setPower(pl2);

            gr2 = motorFrontRight.getCurrentPosition();
            u3 = k * (gl + gr2);
            pr2 = p + u3;
            motorFrontRight.setPower(-pr2);


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

    public void rightward(double p, int digres, double k) {
        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setTargetPosition(digres);

        int gl = 0, gr = 0, gl2 = 0, gr2 = 0;
        double u = 0, pl = 0, pr = 0, u2 = 0, pl2 = 0, pr2 = 0, u3 = 0;

        while (opModeIsActive()&&Math.abs(motorFrontLeft.getCurrentPosition()) < motorFrontLeft.getTargetPosition()) {

            telemetry.addLine("pos | ")
                    .addData("FrontLeft", motorFrontLeft.getCurrentPosition())
                    .addData("FrontRight", motorFrontRight.getCurrentPosition())
                    .addData("BackRight", motorBackRight.getCurrentPosition())
                    .addData("BackLeft", motorBackLeft.getCurrentPosition());
            telemetry.update();
            gr = motorBackRight.getCurrentPosition();
            gl = motorFrontRight.getCurrentPosition();
            u = k * (gl + gr);

            pl = p - u;
            pr = p + u;
            motorBackRight.setPower(-pr);
            motorFrontRight.setPower(pl);

            gl2 = motorFrontLeft.getCurrentPosition();
            u2 = k * (gl - gl2);
            pl2 = p + u2;
            motorFrontLeft.setPower(pl2);

            gr2 = motorBackLeft.getCurrentPosition();
            u3 = k * (gl + gr2);
            pr2 = p + u3;
            motorBackLeft.setPower(-pr2);


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