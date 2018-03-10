package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Color;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
/**
 * Created by ROBOT on 16.02.2018.
 */
@Autonomous(name = "test",group = "to")
public class Testin extends LinearOpMode {
    DcMotor motorFrontRight,
            motorFrontLeft,
            motorBackRight,
            motorBackLeft;
    double k = 0.003, p = 0.1, uL=0, uR=0, uo=0, u, u2,u3;
    int dFL = 0, dBL = 0, dBR = 0, dFR = 0;
    double pFL = 0,
           pBL = 0,
           pBR = 0,
           pFR = 0;
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


        motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive() && 1==1){
            dFR = motorFrontRight.getCurrentPosition();
            dFL = motorFrontLeft.getCurrentPosition();
            dBR = motorBackRight.getCurrentPosition();
            dBL = motorBackLeft.getCurrentPosition();

//            uL = k *(dBL-dFL);
//            uR = k *(dFR-dBR);
//
//            pFR = p - uR;
//            pFL = - p + uL;
//            pBR = p - uR;
//            pBL = p + uL;
//
//            uL = k *(dBL-dFL);
//            uR = k *(dBR-dFR);
//            uo = k *(uL-uR);

//            pFR = p - uo;
//            pFL = - p + uo;
//            pBR = p - uo;
//            pBL = p + uo;

///////////////////////////////FRONT!!!!
            u = k * (dFL + dFR);

            pFL = p - u;
            pFR = p + u;

            u2 = k * (dFL - dBL);
            pBL = p - u2;

            u3 = k * (dFL + dBR);
            pBR = p + u3;

            motorFrontRight.setPower(pFR);
            motorFrontLeft.setPower(pFL);
            motorBackRight.setPower(pBR);
            motorBackLeft.setPower(pBL);

////////////            ///////BACK!!!
            u = k * (dBL + dBR);

            pBL = p - u;
            pBR = p + u;

            u2 = k * (dBL - dFR);
            pFR = p + u2;

            u3 = k * (dBL + dFL);
            pFR = p + u3;

            motorFrontRight.setPower(pFR);
            motorFrontLeft.setPower(-pFL);
            motorBackRight.setPower(pBR);
            motorBackLeft.setPower(-pBL);
////////////            ///////LEFT!!!

            u = k * (dBL + dFL);

            pBL = p - u;
            pFL = p + u;

            u2 = k * (dBL - dBR);
            pBR = p + u2;

            u3 = k * (dBL + dFR);
            pFR = p + u3;

            motorFrontRight.setPower(-pFR);
            motorFrontLeft.setPower(-pFL);
            motorBackRight.setPower(pBR);
            motorBackLeft.setPower(pBL);
////////////            ///////RIGHT!!!

            u = k * (dFR + dBR);

            pFR = p - u;
            pBR = p + u;

            u2 = k * (dFR - dFL);
            pFL = p + u2;

            u3 = k * (dFR + dBL);
            pBL = p + u3;

            motorFrontRight.setPower(-pFR);
            motorFrontLeft.setPower(pFL);
            motorBackRight.setPower(-pBR);
            motorBackLeft.setPower(pBL);

            telemetry.addData("dBL",dBL);
            telemetry.addData("dFL",dFL);
            telemetry.addData("dBR",dBR);
            telemetry.addData("dFR",dFR);
            telemetry.update();
        }
    }
}
