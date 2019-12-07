package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "ARM_MOTOR_TEST",group = "TEST")

public class TestArmMotor extends OpMode {

    private DcMotor arm_motor = null;
    private TouchSensor touch_sensor;

    double move_power = 0;
    double diagonal_power = 0;
    double side_power = 0;
    double spin_power = 0;
    double arm_power = 0;
    double stop_power = 0;
    int target_ticks = 0;

    HelperClass helper_class_object = new HelperClass();


    @Override
    public void init() {
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touch_sensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

    }

    @Override
    public void loop() {

        if (gamepad1.y) {

            helper_class_object.move_arm(arm_motor,50,500,touch_sensor);

        }else if(gamepad1.a){

            helper_class_object.move_arm(arm_motor,-50,500,touch_sensor);

        }

        telemetry.addData("current position is ","%d",arm_motor.getCurrentPosition());

    }



}