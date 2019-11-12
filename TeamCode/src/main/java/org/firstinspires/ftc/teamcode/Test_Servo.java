package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "abdallah",group = "abdallah")

public class Test_Servo extends OpMode {
    Servo test_servo= null;

    @Override
    public void init() {
        test_servo=hardwareMap.get(Servo.class,"test_servo");


    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            test_servo.setPosition(0);
        }
        else if (gamepad1.dpad_down){
            test_servo.setPosition(1);
        }



    }
}
