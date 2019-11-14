package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "abdallah",group = "abdallah")

public class Test_Servo extends OpMode {
    Servo test_servo= null;
    DcMotor test_dc_motor = null;

    double pos ;

    @Override
    public void init() {
        test_servo=hardwareMap.get(Servo.class,"test_servo");


        test_dc_motor=hardwareMap.get(DcMotor.class,"test_dc_motor");


    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            test_servo.setPosition(0);
            pos = test_servo.getPosition() ;
            telemetry.addData("servo is", "\t %f",pos);
        }
        else if (gamepad1.dpad_down){
            test_servo.setPosition(1);
            pos = test_servo.getPosition() ;
            telemetry.addData("servo is", "\t %f",pos);
        }
        if (gamepad1.dpad_right){
            test_dc_motor.setTargetPosition( test_dc_motor.getCurrentPosition()+100);
            test_dc_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            test_dc_motor.setPower(.5);
            while( test_dc_motor.isBusy())
            {

            }
            pos = test_dc_motor.getCurrentPosition() ;
            telemetry.addData("motor pos is", "\t %f",pos);
        }
        else if (gamepad1.dpad_left){
            test_dc_motor.setTargetPosition( test_dc_motor.getCurrentPosition()-100);
            test_dc_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            test_dc_motor.setPower(-.5);
            while( test_dc_motor.isBusy())
            {

            }
            pos = test_dc_motor.getCurrentPosition() ;
            telemetry.addData("motor pos is", "\t %f",pos);
        }

        if (gamepad1.x) {
            test_dc_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pos = test_dc_motor.getCurrentPosition() ;
            telemetry.addData("motor pos is", "\t %f",pos);
        }


    }


}