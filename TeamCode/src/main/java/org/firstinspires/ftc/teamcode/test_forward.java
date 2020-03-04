package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;
@Disabled

@TeleOp(name = "ahmed tarek",group = "FTC")
public class test_forward extends OpMode {

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
    // private DcMotor arm_motor = null;
    // private TouchSensor touch_sensor ;

    HelperClass helper_class_object = new HelperClass();

    @Override
    public void init() {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

        left_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            helper_class_object.move_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 120, 50, false);
        } else if ( gamepad1.dpad_left) {

            helper_class_object.move_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 120, 75, false);
        } else if (gamepad1.dpad_right ) {

            helper_class_object.move_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 120, 100, false);
        } else if (gamepad1.dpad_down) {

            helper_class_object.move_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 120, 25, false);
        }

        else if(gamepad1.x){

            helper_class_object.move_with_encoderr(left_back_motor,left_front_motor,
                    right_back_motor,right_front_motor,220,100,false);
        }else if(gamepad1.a){

            helper_class_object.move_with_encoderr(left_back_motor,left_front_motor,
                    right_back_motor,right_front_motor,250,100,false);
        }else if(gamepad1.y){

            helper_class_object.move_with_encoderr(left_back_motor,left_front_motor,
                    right_back_motor,right_front_motor,270,100,false);
        }else if(gamepad1.b){

            helper_class_object.move_with_encoderr(left_back_motor,left_front_motor,
                    right_back_motor,right_front_motor,290,100,false);
        }


    }

}