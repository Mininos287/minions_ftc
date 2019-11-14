package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.HelperClass;


@Autonomous(name = "test",group = "test")

public class test extends LinearOpMode {
    HelperClass helper_class_object = new HelperClass();

    private Servo antar_servo = null ;
    private  Servo abla_servo = null ;
    private DcMotor arm_motor = null;
    private TouchSensor touch_sensor ;
    private ModernRoboticsI2cColorSensor color_sensor = null ;

    @Override
    public void runOpMode() throws InterruptedException {


        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        touch_sensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        color_sensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"color_sensor");
        antar_servo = hardwareMap.get(Servo.class, "antar");

        abla_servo = hardwareMap.get(Servo.class, "abla");



        arm_motor.setDirection(DcMotor.Direction.FORWARD);


        color_sensor.enableLed(true); //true for objects and false for sensing color

        waitForStart();

        while (opModeIsActive()) {

            while (! touch_sensor.isPressed()){
                helper_class_object.move_arm_without_encoder(arm_motor, 5 ,'U');

            }

            helper_class_object.move_arm_without_encoder(arm_motor, 0 ,'U');



            if((color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 8)){

                antar_servo.setPosition(0);
                abla_servo.setPosition(0);
            }




        }
    }
}