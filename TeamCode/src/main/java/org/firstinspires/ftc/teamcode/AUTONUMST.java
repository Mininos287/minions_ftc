package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Boolean.TRUE;

@Autonomous(name = "ftc auto",group = "FTC")


public class AUTONUMST extends LinearOpMode
{
    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;

    HelperClass helper_class_object = new HelperClass();


    @Override
    public void runOpMode() throws InterruptedException {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
        // arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        // touch_sensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        waitForStart();

        while(opModeIsActive()) {
            helper_class_object.move_with_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 20, 20, 20, 50, 5);
            sleep(2000);
            helper_class_object.move_with_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 20, 20, 20, -50, 5);
            sleep(2000);

            helper_class_object.side__with_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 20, 20, 20, 50, 5, 'R', TRUE);
            sleep(2000);

            helper_class_object.side__with_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 20, 20, 20, 50, 5, 'L', TRUE);
            sleep(2000);

            helper_class_object.spin_with_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 50, 90, 90, 90, 5, 'A');
            sleep(2000);

            helper_class_object.spin_with_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 50, 90, 90, 90, 5, 'C');
            sleep(2000);

        }

    }



}
