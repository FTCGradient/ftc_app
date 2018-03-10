package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by ROBOT on 17.02.2018.
 */
@Autonomous
public class Igor extends OpMode{
    DcMotor motorFrontRight,
            motorFrontLeft,
            motorBackRight,
            motorBackLeft;
    public void init() {
      motorFrontRight = hardwareMap.dcMotor.get("mFR");
        motorFrontLeft = hardwareMap.dcMotor.get("mFL");
        motorBackRight = hardwareMap.dcMotor.get("mBR");
        motorBackLeft= hardwareMap.dcMotor.get("mBL");
    }
    @Override
    public void loop() {
        float Y = gamepad1.left_stick_y;
        float X = gamepad1.left_stick_x;
        float R = gamepad1.right_stick_x;

        motorFrontRight.setPower(Y-X-R);
        motorFrontLeft.setPower(-Y-X-R);
        motorBackRight.setPower(Y+X-R);
        motorBackLeft.setPower(-Y+X-R);
    }
}
