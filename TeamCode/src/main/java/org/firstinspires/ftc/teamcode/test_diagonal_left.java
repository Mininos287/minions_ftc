package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

@TeleOp(name = "test diagonal left",group = "FTC")
public class test_diagonal_left extends OpMode {
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

            helper_class_object.move_diagonale_left_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 70, 50, false);

        } else if (gamepad1.dpad_left) {

            helper_class_object.move_diagonale_left_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 70, 30, false);
        } else if (gamepad1.dpad_right) {

            helper_class_object.move_diagonale_left_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 70, 90, false);
        } else if (gamepad1.dpad_down) {

            helper_class_object.move_diagonale_left_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 70, 20, false);
        } else if (gamepad1.x) {

            helper_class_object.move_diagonale_left_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 70, 40, false);
        } else if (gamepad1.y) {

            helper_class_object.move_diagonale_left_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 100, 55, false);
        } else if (gamepad1.a) {

            helper_class_object.move_diagonale_left_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 70, 40, false);
        } else if (gamepad1.b) {

            helper_class_object.move_diagonale_left_with_encoderr(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, 200, 100, false);
        }
    }

}
