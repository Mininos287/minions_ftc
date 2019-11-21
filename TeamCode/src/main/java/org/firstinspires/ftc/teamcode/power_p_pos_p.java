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


        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        HelperClass helper_class_object = new HelperClass();

        waitForStart();

        while (opModeIsActive()) {
            left_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            left_back_motor.setTargetPosition(1440);
            left_front_motor.setTargetPosition(1440);
            right_back_motor.setTargetPosition(1440);
            right_front_motor.setTargetPosition(1440);


            left_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            left_back_motor.setPower(30/100);
            left_front_motor.setPower(30/100);
            right_back_motor.setPower(30/100);
            right_front_motor.setPower(30/100);


            while (left_back_motor.isBusy()&& left_front_motor.isBusy()&&right_back_motor.isBusy()&&right_front_motor.isBusy()) {

            }


            left_motor.setPower(0);

            sleep(500);
        }
    }
}