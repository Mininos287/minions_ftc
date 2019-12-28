package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor ;


@TeleOp(name="Training Manual OpMode", group="org.firstinspires.ftc.teamcode.TrainingManual")

//@Disabled
public class TrainingManual extends OpMode {
int YELLOW_COLOR = 8 ;
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    private DcMotor arm_motor = null;
    private Servo right_claw = null ;
    private Servo left_claw = null ;
    private ModernRoboticsI2cColorSensor color_sensor = null ;
    private ModernRoboticsTouchSensor touch_sensor ;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");


        left_motor  = hardwareMap.get(DcMotor.class, "left_motor");
        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");

        right_claw = hardwareMap.get(Servo.class, "right_claw");
        left_claw = hardwareMap.get(Servo.class, "left_claw");

        color_sensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"color_sensor");

        touch_sensor = hardwareMap.get(ModernRoboticsTouchSensor.class , "touch_sensor");

        right_motor.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
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

        color_sensor.enableLed(true); //true for objects and false for sensing color

        /*
         * Code to run ONCE when the driver hits PLAY
         */
    }




    @Override
    public void loop() {

        double left_power;
        double right_power;
        double arm_power;

        left_power  = -gamepad1.left_stick_y ;
        right_power = -gamepad1.right_stick_y ;
        arm_power = -gamepad2.left_stick_y  ;

        left_motor.setPower(left_power/2.0);
        right_motor.setPower(right_power/2.0);

        arm_motor.setPower(arm_power/8.0);

        if(gamepad1.x){
            left_claw.setPosition(.7);
            right_claw.setPosition(0);
        }else{
            left_claw.setPosition(0);
            right_claw.setPosition(.7);
        }


        if((color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == YELLOW_COLOR)){

            left_claw.setPosition(.7);
            right_claw.setPosition(0);
        }


        if(touch_sensor.isPressed()){
            arm_motor.setPower(1/8.0);
        }

        telemetry.addData("Motors", "left (%.2f), right (%.2f) , arm (%.2f)", left_power, right_power,arm_power);
        telemetry.addData("color sensor" ,color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));

    }


    @Override
    public void stop() {
        telemetry.addData("Status", "Stop");





    }
}