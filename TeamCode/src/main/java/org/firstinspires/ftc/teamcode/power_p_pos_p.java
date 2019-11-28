package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "power_p_pos_p",group = "test")

public class power_p_pos_p extends LinearOpMode {

    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;
    
    private DcMotor left_motor = null;


    @Override
    public void runOpMode() throws InterruptedException {


        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");


        right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        HelperClass helper_class_object = new HelperClass();

        waitForStart();

        while (opModeIsActive()) {
            right_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



            right_back_motor.setTargetPosition(1440*10);



            right_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            right_back_motor.setPower(.2);



            while (right_back_motor.isBusy()) {

            }


            right_back_motor.setPower(0);

            sleep(500);
        }
    }
}