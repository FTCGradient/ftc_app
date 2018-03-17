package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(name = "TOF")
//@Disabled
public class TO extends OpMode {
    int gl = 0, gr = 0, gl2 = 0, gr2 = 0, q;
    double  pl = 0, pr = 0, pl2 = 0, pr2 = 0;
    int a = 0, obnl = 0;
    double sg=0;
    DcMotor motorFrontRight,
            motorFrontLeft,
            motorBackRight,
            motorBackLeft,
            motorWheelRight,
            motorWheelLeft,
            motorLiftLeft,
            motorLiftRight;

    ServoController SC;
    Servo servoCentrRight,
            servoCentrLeft,
            servoJewelRight,
            servoJewelLeft,
            servoGlyth;

    double servoPositionZ, servoPositionP;

    double k = 0.003, p = 0.1, uL=0, uR=0, uo=0, u, u2,u3;
    int dFL = 0, dBL = 0, dBR = 0, dFR = 0;
    double pFL = 0,
            pBL = 0,
            pBR = 0,
            pFR = 0;


    public void init() {

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
//
//        motorLiftLeft = hardwareMap.dcMotor.get("mLL");
//        motorLiftRight = hardwareMap.dcMotor.get("mLR");
//
//        servoCentrRight = hardwareMap.servo.get("sCR");
//        servoCentrLeft = hardwareMap.servo.get("sCL");
//        servoJewelRight = hardwareMap.servo.get("sJR");
//        servoJewelLeft = hardwareMap.servo.get("sJL");
//        servoGlyth = hardwareMap.servo.get("sG");

    }

    public void loop() {
        float Y = gamepad1.left_stick_y;
        float X = gamepad1.left_stick_x;
        float P = gamepad1.right_stick_x;
//        motorBackRight.setPower(X);

//
//        if (a==0)
//        {
////            servoGlyth.setPosition(0.1);
//            servoJewelLeft.setPosition(1);
//            servoJewelRight.setPosition(0);
////            servoCentrRight.setPosition(0.65);
////            servoCentrLeft.setPosition(0.35);
//            a=1;
//        }
////
//        if (gamepad1.a) {
//            motorFrontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//            motorFrontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//            motorBackRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
//            motorBackLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
//            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }

        if (gamepad1.dpad_up == true) {
//            gr = motorBackRight.getCurrentPosition();
//            gl = motorBackLeft.getCurrentPosition();
//            gr2 = motorFrontRight.getCurrentPosition();
//            gl2 = motorFrontLeft.getCurrentPosition();
//
//            u =  k * (gl - gl2);
//            u2 = k * (gl - gr);
//            u3 = k * (gl - gr2);
//
//            pl2 =   p - u;
//            pl =  p - u;
//            pr = p + u2;
//            pr2 = p + u3;
//            motorFrontLeft.setPower(pl2);
//            motorBackLeft.setPower(pl);
//            motorBackRight.setPower(pr);
//            motorFrontRight.setPower(pr2);

            dFR = motorFrontRight.getCurrentPosition();
            dFL = motorFrontLeft.getCurrentPosition();
            dBR = motorBackRight.getCurrentPosition();
            dBL = motorBackLeft.getCurrentPosition();

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

        }
//        else {
//            if (gamepad1.dpad_down == true) {
//                gr = motorBackRight.getCurrentPosition();
//                gl = motorBackLeft.getCurrentPosition();
//                gr2 = motorFrontRight.getCurrentPosition();
//                gl2 = motorFrontLeft.getCurrentPosition();
//
//                u =  k * (gl - gl2);
//                u2 = k * (gl - gr);
//                u3 = k * (gl - gr2);
//
//                pl2 =   p + u;
//                pl =  p + u;
//                pr = p - u2;
//                pr2 = p - u3;
//                motorFrontLeft.setPower(pl2);
//                motorBackLeft.setPower(pl);
//                motorBackRight.setPower(pr);
//                motorFrontRight.setPower(pr2);

//                dFR = motorFrontRight.getCurrentPosition();
//                dFL = motorFrontLeft.getCurrentPosition();
//                dBR = motorBackRight.getCurrentPosition();
//                dBL = motorBackLeft.getCurrentPosition();
//
//                u = k * (dBL + dBR);
//
//                pBL = p - u;
//                pBR = p + u;
//
//                u2 = k * (dBL - dFR);
//                pFR = p + u2;
//
//                u3 = k * (dBL + dFL);
//                pFR = p + u3;
//
//                motorFrontRight.setPower(pFR);
//                motorFrontLeft.setPower(-pFL);
//                motorBackRight.setPower(pBR);
//                motorBackLeft.setPower(-pBL);
//            }
//            else{
//                if(gamepad1.dpad_left == true){
////                    gr = motorBackRight.getCurrentPosition();
////                    gl = motorBackLeft.getCurrentPosition();
////                    gr2 = motorFrontRight.getCurrentPosition();
////                    gl2 = motorFrontLeft.getCurrentPosition();
////
////                    u =  k * (gl - gl2);
////                    u2 = k * (gl - gr);
////                    u3 = k * (gl - gr2);
////
////                    pl2 =   p - u;
////                    pl =  p + u;
////                    pr = p + u2;
////                    pr2 = p - u3;
////                    motorFrontLeft.setPower(pl2);
////                    motorBackLeft.setPower(pl);
////                    motorBackRight.setPower(pr);
////                    motorFrontRight.setPower(pr2);
//
//                    dFR = motorFrontRight.getCurrentPosition();
//                    dFL = motorFrontLeft.getCurrentPosition();
//                    dBR = motorBackRight.getCurrentPosition();
//                    dBL = motorBackLeft.getCurrentPosition();
//
//
//                    u = k * (dBL + dFL);
//
//                    pBL = p - u;
//                    pFL = p + u;
//
//                    u2 = k * (dBL - dBR);
//                    pBR = p + u2;
//
//                    u3 = k * (dBL + dFR);
//                    pFR = p + u3;
//
//                    motorFrontRight.setPower(-pFR);
//                    motorFrontLeft.setPower(-pFL);
//                    motorBackRight.setPower(pBR);
//                    motorBackLeft.setPower(pBL);
//
//                }
//                else{
//                    if(gamepad1.dpad_right == true){
////                        gr = motorBackRight.getCurrentPosition();
////                        gl = motorBackLeft.getCurrentPosition();
////                        gr2 = motorFrontRight.getCurrentPosition();
////                        gl2 = motorFrontLeft.getCurrentPosition();
////
////                        u =  k * (gl - gl2);
////                        u2 = k * (gl - gr);
////                        u3 = k * (gl - gr2);
////
////                        pl2 =   p + u;
////                        pl =  p - u;
////                        pr = p - u2;
////                        pr2 = p + u3;
////                        motorFrontLeft.setPower(pl2);
////                        motorBackLeft.setPower(pl);
////                        motorBackRight.setPower(pr);
////                        motorFrontRight.setPower(pr2);
//
//                        dFR = motorFrontRight.getCurrentPosition();
//                        dFL = motorFrontLeft.getCurrentPosition();
//                        dBR = motorBackRight.getCurrentPosition();
//                        dBL = motorBackLeft.getCurrentPosition();
//
//                        u = k * (dFR + dBR);
//
//                        pFR = p - u;
//                        pBR = p + u;
//
//                        u2 = k * (dFR - dFL);
//                        pFL = p + u2;
//
//                        u3 = k * (dFR + dBL);
//                        pBL = p + u3;
//
//                        motorFrontRight.setPower(-pFR);
//                        motorFrontLeft.setPower(pFL);
//                        motorBackRight.setPower(-pBR);
//                        motorBackLeft.setPower(pBL);
//
//                    }
//                    else{

//                    }
//                }
//            }
//        }
//
//
//            telemetry.addData("gr",gr);
//            telemetry.addData("gl",gl);
//            telemetry.addData("gr2",gr2);
//            telemetry.addData("gl2",gl2);
//            telemetry.update();
//
//
//
        motorWheelRight.setPower(gamepad2.right_stick_y);
        motorWheelLeft.setPower(gamepad2.left_stick_y);

//        if(gamepad1.a){
//            motorWheelRight.setPower(-0.7);
//            motorWheelLeft.setPower(0.7);
//        }
//        else {
//
//            if(gamepad1.b){
//                motorWheelRight.setPower(0.7);
//                motorWheelLeft.setPower(-0.7);
//            }
//            else {
//                motorWheelRight.setPower(0);
//                motorWheelLeft.setPower(0);
//            }
//        }
//
//        if(gamepad2.a) {
//            servoCentrLeft.setPosition(0.5);
//            servoCentrRight.setPosition(0.5);
//        }
//        if(gamepad2.b) {
//            servoCentrRight.setPosition(0.55);
//            servoCentrLeft.setPosition(0.45);
//        }
//
//        if(gamepad2.y) {
//            servoCentrLeft.setPosition(0);
//            servoCentrRight.setPosition(1);
//        }
//        if (gamepad2.x) {
//            servoGlyth.setPosition(0.7);
//        }
//        else{
//            servoGlyth.setPosition(0);
//        }
//        if(gamepad2.right_stick_button){
//            servoJewelLeft.setPosition(1);
//            servoJewelRight.setPosition(0);
//        }
//        else{
//            if(gamepad2.left_stick_button){
//                servoJewelLeft.setPosition(0.5);
//                servoJewelRight.setPosition(0.5);
//            }
//        }
//
//
//        if(gamepad2.right_bumper)
//        {
//            motorLiftRight.setPower(0.5);
//            motorLiftLeft.setPower(0.5);
//       }
//        else
//        {
//            if(gamepad2.left_bumper)
//            {
//                motorLiftLeft.setPower(-0.5);
//                motorLiftRight.setPower(-0.5);}
//            else
//            {
//                motorLiftLeft.setPower(0);
//                motorLiftRight.setPower(0);
//            }
//        }
//
//}
//
//    public void waitS(float sec){
//        long startTime = System.nanoTime();
//        long duration = 0;
//        while(duration<sec*1000000000){
//            duration= System.nanoTime()-startTime;
//        }
//    }

//
//        if (gamepad1.right_bumper) {
//            Center.setPower(-0.1);
//        } else {
//            if (gamepad1.left_bumper) {
//                Center.setPower(1);
//            } else {
//                Center.setPower(0);
//            }
//        }
//   Lift.setPower(gamepad2.left_stick_y);
//    }
//
//
//
//}
//
//        if(gamepad2.left_trigger == 1){
//            motorBalista.setPower(gamepad2.right_stick_x);
//        }else {
//            motorZagim.setPower(gamepad2.right_stick_y/2);
//        }

                //servoZ.setPosition(servoPositionZ);


                //servoP.setPosition(servoPositionP);


//            if(gamepad2.left_bumper) {
//            servoPositionZ = 0.1;
//        }
//        if(gamepad2.right_bumper){
//            servoPositionZ = 1;
//        }
//


        /*if(gamepad2.b){
            servoPositionP = 0.1;
        }
        if(gamepad2.a){
            servoPositionP = 1;
        }


        if (Lift.getCurrentPosition() >= 5000 && gamepad2.left_stick_y <= 0) {
            Lift.setPower(0);}
        if (Lift.getCurrentPosition() <= -5000 && gamepad2.left_stick_y >= 0) {
            Lift.setPower(0);}

        if (Lift.getCurrentPosition() >= 5000 && gamepad2.left_stick_y > 0) {
            Lift.setPower(-gamepad2.left_stick_y/2);}
        if (Lift.getCurrentPosition() <= -5000 && gamepad2.left_stick_y < 0) {
            Lift.setPower(-gamepad2.left_stick_y/2);}

        if (Lift.getCurrentPosition() < 5000 && Lift.getCurrentPosition() > -5000) {
            Lift.setPower(-gamepad2.left_stick_y/2);}

        if (gamepad2.left_stick_button) {
            Lift.setMode(DcMotor.RunMode.RESET_ENCODERS);
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);}

        q = 0;
    }
}
*/
                motorFrontRight.setPower(-Y - X - P);
                motorFrontLeft.setPower(Y - X - P);
                motorBackRight.setPower(-Y + X - P);
                motorBackLeft.setPower(Y + X - P);
            }
        }