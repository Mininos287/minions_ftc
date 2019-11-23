package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

@TeleOp(name = "AhmedandTaha",group = "FTC")

public class AhmedandTaha extends OpMode {

    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;

    double move_power = 0;
    double diagonal_power = 0;
    double side_power = 0;
    double spin_power = 0;
    double arm_power = 0;
    double stop_power = 0;
    double distanc_acceleration =132;
    double power_acceleration =20;
    HelperClass helper_class_object = new HelperClass();

    @Override
    public void init() {

        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

        right_back_motor.setDirection(DcMotor.Direction.FORWARD);
        right_front_motor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            move_power = 100;
            diagonal_power = 100;
            side_power = 100;
            spin_power = 100;
            arm_power = 50;
        } else if (gamepad1.right_bumper) {

            move_power = 50;
            diagonal_power = 50;
            side_power = 50;
            spin_power = 50;
            arm_power = 25;

        } else {

            move_power = 25;
            diagonal_power = 25;
            side_power = 25;
            spin_power = 25;
            arm_power = 15;


        }







            telemetry.addData("hi","if (1)");

            //helper_class_object.acceleration_deceleration_manual(left_back_motor, left_front_motor,
                    //right_back_motor, right_front_motor, 25, 25, 25, 100, 100, 100);


            right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            right_back_motor.setTargetPosition(4500);
            right_front_motor.setTargetPosition(4500);



            right_back_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            right_back_motor.setPower(0.1);
            right_front_motor.setPower(0.1);



            while (right_back_motor.isBusy() && right_front_motor.isBusy())
            {


            }




        telemetry.addData("hi","if(2)");



    }
}

