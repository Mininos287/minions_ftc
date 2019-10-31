package org.firstinspires.ftc.robotcontroller.external.samples;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.HelperClass ;

@Autonomous(name="Training auto opmode",group="TrainingManual")
//@Disabled

public class TrainingAuto extends LinearOpMode {

    private ElapsedTime run_time = new ElapsedTime();
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    private Servo right_claw;
    private Servo left_claw;


    double servo_position = 0.00;


    @Override
    public void runOpMode() throws InterruptedException {

        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "right_motor");

        right_motor.setDirection(DcMotor.Direction.REVERSE);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        right_claw = hardwareMap.servo.get("right_claw");
        left_claw = hardwareMap.servo.get("left_claw");

        right_claw.setPosition(servo_position);
        left_claw.setPosition(servo_position);

        waitForStart();

        HelperClass helper_class_object = new HelperClass() ;

        run_time.reset();

        while(opModeIsActive()) {


            helper_class_object.lower_to_higher_servo_degrees(right_claw,0,120);
            helper_class_object.higher_to_lower_servo_degrees(left_claw,120,0);

            sleep(1000);

            helper_class_object.higher_to_lower_servo_degrees(right_claw,120,0);
            helper_class_object.lower_to_higher_servo_degrees(left_claw,0,120);

            sleep(3000);


        }
    }

}
