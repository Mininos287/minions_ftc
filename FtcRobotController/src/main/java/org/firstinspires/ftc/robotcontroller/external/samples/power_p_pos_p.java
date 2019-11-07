package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "power_p_pos_p",group = "test")

public class power_p_pos_p extends LinearOpMode {


    
    private DcMotor left_motor = null;


    @Override
    public void runOpMode() throws InterruptedException {


        left_motor = hardwareMap.get(DcMotor.class, "left_motor");

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            left_motor.setTargetPosition(1440);

            left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left_motor.setPower(.5);


            while (left_motor.isBusy()) {

            }


            left_motor.setPower(0);

            sleep(500);
        }
    }
}