package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcontroller.external.samples.HelperClass;

@TeleOp(name = "Ahmed&Taha",group = "FTC")

public class FtcManual extends OpMode {
    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;


    double move_power = 0 ;
    double diagonal_power = 0 ;
    double side_power = 0 ;
    double spin_power = 0 ;
    double stop_power = 0 ;

    HelperClass helper_class_object = new HelperClass();

    @Override
    public void init() {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

        left_back_motor.setDirection(DcMotor.Direction.FORWARD);
        left_front_motor.setDirection(DcMotor.Direction.FORWARD);
        right_back_motor.setDirection(DcMotor.Direction.FORWARD);
        right_front_motor.setDirection(DcMotor.Direction.FORWARD);



    }

    @Override
    public void init_loop() {

        telemetry.addData("Status", "init_loop");

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
    }


    @Override
    public void start() {
        telemetry.addData("Status", "start");


        /*
         * Code to run ONCE when the driver hits PLAY
         */
    }


    @Override
    public void loop() {


        if (gamepad1.right_bumper) {

              move_power = 100 ;
              diagonal_power = 100 ;
              side_power = 100 ;
              spin_power = 100 ;


        } else {

            move_power = 20 ;
            diagonal_power = 20 ;
            side_power = 20 ;
            spin_power = 20 ;


        }

        if (gamepad1.dpad_up) {
            //move forward
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    move_power, 'F');
        }

        else if (gamepad1.dpad_down) {
            //move backward
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    -move_power, 'B');
        }


        else if (gamepad1.dpad_right) {
            //move right
            helper_class_object.move_side_without_encoder(left_back_motor, right_front_motor,
                    left_front_motor, right_back_motor,
                    side_power, 'R');
        }
        else if (gamepad1.dpad_left) {
            //move left
            helper_class_object.move_side_without_encoder(left_back_motor, right_front_motor,
                    left_front_motor, right_back_motor,
                    side_power, 'L');

        }

        else if (gamepad1.b) {
            //spin clockwise
            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, spin_power, 'C');
        }

        else if (gamepad1.x) {
            //spin anti-clockwise
            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, spin_power, 'A');

        }else{
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    stop_power, 'F');
        }




    }
}