package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by ROBOT on 23.12.2017.
 */

public class Autonomus extends LinearOpMode {
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

    int n, V;
    VuforiaLocalizer vuforia;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("mFR");
        motorFrontLeft = hardwareMap.dcMotor.get("mFL");
        motorBackRight = hardwareMap.dcMotor.get("mBR");
        motorBackLeft = hardwareMap.dcMotor.get("mBL");
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AVGtR4z/////AAAAGR7mv1T9Kk+et5FMVOb0TCEbVxtoc9LEJLzfu6cbgmf8/dTA5Z9FlMw78GFgMsytU5MnyRBKOm+ruNNfz5liVnNaB/5ktZUMGJmEkONfo+0PeZ5GkZyahlZ+QK/BUtVggdorofC9GUHL+Jcnk/67gr+fqWJL/aZgREfCecRXRCfBJ+vVqDT9kVq+Pg5+9kfDThuN4QD8M00w+IJrDwTw8dL2GvLQRtH/ylgVJ5of2mhYVXzPsizjLSojDCr5iGRQJ/0FfKrw3gbGoxZ+rg44OkvtfjFoMIuQLhpP5warc1CToeMQPSAdNQLZbEgjup2OZ8pWtdZLT4BWzg5w9rJGL/ryQWuFt52DHsxD5dk+ax4c";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

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
    }
}