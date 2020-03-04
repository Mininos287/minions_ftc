package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "FtcAutoVuforiaBlueAlliance_1_1",group = "Ftc")

public class Weak_blue_arab extends LinearOpMode {

    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;
    private DcMotor arm_motor = null;
    Servo right_foundation_servo = null;
    Servo left_foundation_servo = null;
    Servo arm_servo = null;
    Servo gripper_servo = null;
    private ModernRoboticsI2cColorSensor color_sensor = null;

    private TouchSensor min_end_stop;
    DigitalChannel max_end_stop;

    double move_power = 56;
    double side_power = 56;
    double arm_power = 100;
    double stop_power = 0;
    int flag = 0;
    int BLUE_COLOR = 3;
    int YELLOW_COLOR = 8;
    int RED_COLOR = 10;

    double gyro_start;
    double gyro_angel;

    HelperClass helper_class_object = new HelperClass();
    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)
    //
    // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    //
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYtwReX/////AAABmYoVHtmyZEmSs4DBYiwrM8Q97eHe+rvhAouFjczpsGpqBeGml7GDbt3zXG3w13XiSaa3WwL++gETRyiXsgsAeyTjTdGQy8jPiZrdBk6fAZfyQ367p2cdAur+o9F94baNynVGAKh2TEvfIixpjCAiOE9rniWGY9P8ZcuXyFSoNvSYt54J+ImOFFB/pdUYjWYLYYyx+j13s4dhkvDMZl3i/Z/bouvTA8SvoTjPlhiSayDGrp/N6S0NoAzUB9ciVCsuZeVlKhfqAPO5kkpacHHXrgOCxBhHsTiZgUfa1Y6Tc2KFyXbYiL0i0ZriYO2DvOf+XT6NPdGfc+jqhHhI6324Na23xuuzcZo+NzjVX46eXz0M";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private boolean targetVisible = false;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    private String target_name = "none";

    private double num_of_trials = 0;

    @Override
    public void runOpMode() {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);

//        right_foundation_servo = hardwareMap.get(Servo.class, "right_foundation_servo");
        left_foundation_servo = hardwareMap.get(Servo.class, "left_foundation_servo");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        gripper_servo = hardwareMap.get(Servo.class, "gripper_servo");


        min_end_stop = hardwareMap.get(TouchSensor.class, "min_end_stop");

        max_end_stop = hardwareMap.get(DigitalChannel.class, "max_end_stop");
        max_end_stop.setMode(DigitalChannel.Mode.INPUT);


        color_sensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "color_sensor");


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();


        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameterss = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameterss.vuforiaLicenseKey = VUFORIA_KEY;
        parameterss.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameterss);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
//        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
//        blueRearBridge.setName("Blue Rear Bridge");
//        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
//        redRearBridge.setName("Red Rear Bridge");
//        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
//        redFrontBridge.setName("Red Front Bridge");
//        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
//        blueFrontBridge.setName("Blue Front Bridge");
//        VuforiaTrackable red1 = targetsSkyStone.get(5);
//        red1.setName("Red Perimeter 1");
//        VuforiaTrackable red2 = targetsSkyStone.get(6);
//        red2.setName("Red Perimeter 2");
//        VuforiaTrackable front1 = targetsSkyStone.get(7);
//        front1.setName("Front Perimeter 1");
//        VuforiaTrackable front2 = targetsSkyStone.get(8);
//        front2.setName("Front Perimeter 2");
//        VuforiaTrackable blue1 = targetsSkyStone.get(9);
//        blue1.setName("Blue Perimeter 1");
//        VuforiaTrackable blue2 = targetsSkyStone.get(10);
//        blue2.setName("Blue Perimeter 2");
//        VuforiaTrackable rear1 = targetsSkyStone.get(11);
//        rear1.setName("Rear Perimeter 1");
//        VuforiaTrackable rear2 = targetsSkyStone.get(12);
//        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameterss.cameraDirection);
        }


        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();


        if (left_foundation_servo.getPosition() != 0.7) {
            left_foundation_servo.setPosition(0.7);
            sleep(1000);
        }


        if (gripper_servo.getPosition() != 1) {
            gripper_servo.setPosition(1);
            sleep(1000);

        }
        if (arm_servo.getPosition() != .2) {
            arm_servo.setPosition(0.2);
            sleep(1000);

        }

        if (left_foundation_servo.getPosition() != .25) {
            left_foundation_servo.setPosition(.25);
            sleep(1000);

        }


        waitForStart();

        while (opModeIsActive()) {

            move_with_pid(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 43, move_power, false);
            sleep(1000);
            //   اتحرك لحد البريدج مسافة عشوائيه
            //  اتحرك يمين لغاية ماتركن تحت الكوبري
            // wait 25 second
            // اتحرك يمين
            // ارجع لورا مسافة عشواقية
            //اتحرك شمال لغاية متركن تحت الكوبري


        }
    }

        public void move_with_pid (DcMotor left_back_motor, DcMotor left_front_motor, DcMotor
        right_back_motor,
                DcMotor right_front_motor,double distance, double power, boolean read_gyro_angel){

            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
//


            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            double gyro_start;

            if (read_gyro_angel == true) {
                gyro_start = gyro_angel;

            } else {
                gyro_start = 0;

            }


            double left_power;
            double right_power;
            if (power > 0) {
                left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
                left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
                right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
                right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
                int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                int left_front_motor_target_ticks = left_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                int right_back_motor_target_ticks = right_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                int right_front_motor_target_ticks = right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                double error = 0;
                double last_error = 0;
                double KP = 2.7;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;
                while ((left_back_motor.getCurrentPosition() < left_back_motor_target_ticks) &&
                        (left_front_motor.getCurrentPosition() < left_front_motor_target_ticks)
                        && (right_back_motor.getCurrentPosition() < right_back_motor_target_ticks) &&
                        (right_front_motor.getCurrentPosition() < right_front_motor_target_ticks)) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    left_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    right_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("LEFT_POWER and RIGHT_POWER", "%d %d",
                            left_back_motor.getCurrentPosition(), right_back_motor.getCurrentPosition());
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                sleep(1);
            } else if (power < 0) {
                left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
                left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
                right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
                right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
                int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                int left_front_motor_target_ticks = left_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                int right_back_motor_target_ticks = right_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                int right_front_motor_target_ticks = right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                double error = 0;
                double last_error = 0;
                double KP = 3;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;
                while ((left_back_motor.getCurrentPosition() > left_back_motor_target_ticks) &&
                        (left_front_motor.getCurrentPosition() > left_front_motor_target_ticks)
                        && (right_back_motor.getCurrentPosition() > right_back_motor_target_ticks) &&
                        (right_front_motor.getCurrentPosition() > right_front_motor_target_ticks)) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    left_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    right_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("LEFT_POWER and RIGHT_POWER", "%d %d",
                            left_back_motor.getCurrentPosition(), right_back_motor.getCurrentPosition());
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                sleep(1);
            } else if (power == 0) {
            }
        }


        public void move_side_with_pid (DcMotor left_back_motor, DcMotor left_front_motor,
                DcMotor right_back_motor, DcMotor right_front_motor,double distance, double power,
        boolean read_gyro_angel){
            left_back_motor.setDirection(DcMotor.Direction.REVERSE);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.FORWARD);

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            double gyro_start;

            if (read_gyro_angel == true) {
                gyro_start = gyro_angel;

            } else {
                gyro_start = 0;

            }

            if (power > 0) {


                double increase_power;
                double decrese_power;
                double error = 0;
                double last_error = 0;
                double KP = 2;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;
                int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                int left_front_motor_target_ticks = left_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                int right_back_motor_target_ticks = right_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                int right__front_motor_target_ticks = right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                left_back_motor.setTargetPosition(left_back_motor_target_ticks);
                left_front_motor.setTargetPosition(left_front_motor_target_ticks);
                right_back_motor.setTargetPosition(right_back_motor_target_ticks);
                right_front_motor.setTargetPosition(right__front_motor_target_ticks);
                while ((left_back_motor.getCurrentPosition() > left_back_motor_target_ticks) &&
                        (left_front_motor.getCurrentPosition() < left_front_motor_target_ticks)
                        && (right_back_motor.getCurrentPosition() < right_back_motor_target_ticks) &&
                        (right_front_motor.getCurrentPosition() > right__front_motor_target_ticks)) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    increase_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    decrese_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("decrese_power and increase_power", "%f %f", decrese_power, increase_power);
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                sleep(1);
            } else if (power < 0) {


                // double gyro_start = 0 ;

                double increase_power;
                double decrese_power;
                double error = 0;
                double last_error = 0;
                double KP = 2;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;
                int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                int left_front_motor_target_ticks = left_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                int right_back_motor_target_ticks = right_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                int right__front_motor_target_ticks = right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                left_back_motor.setTargetPosition(left_back_motor_target_ticks);
                left_front_motor.setTargetPosition(left_front_motor_target_ticks);
                right_back_motor.setTargetPosition(right_back_motor_target_ticks);
                right_front_motor.setTargetPosition(right__front_motor_target_ticks);
                while ((left_back_motor.getCurrentPosition() < left_back_motor_target_ticks) &&
                        (left_front_motor.getCurrentPosition() > left_front_motor_target_ticks)
                        && (right_back_motor.getCurrentPosition() > right_back_motor_target_ticks) &&
                        (right_front_motor.getCurrentPosition() < right__front_motor_target_ticks)) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    decrese_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    increase_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("decrese_power and increase_power", "%f %f", decrese_power, increase_power);
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                sleep(1);
            } else if (power == 0) {
            }
        }

        public void move_diagonale_left_with_pid (DcMotor left_front_motor, DcMotor
        left_back_motor, DcMotor right_front_motor,
                DcMotor right_back_motor
            ,double distance, double power, boolean read_gyro_angel)
        {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            double gyro_start;

            if (read_gyro_angel == true) {
                gyro_start = gyro_angel;

            } else {
                gyro_start = 0;

            }
            double left_power;
            double right_power;


            if (power > 0) {
                left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
                right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));

                int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                int right_front_motor_target_ticks = right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);

                double error = 0;
                double last_error = 0;
                double KP = 2.7;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;

                while ((left_back_motor.getCurrentPosition() < left_back_motor_target_ticks) &&
                        (right_front_motor.getCurrentPosition() < right_front_motor_target_ticks)) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    left_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    right_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("LEFT_POWER and RIGHT_POWER", "%d %d",
                            left_back_motor.getCurrentPosition(), right_back_motor.getCurrentPosition());
                    telemetry.update();
                }

                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));

            } else if (power < 0) {
                left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
                right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));

                int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                int right_front_motor_target_ticks = right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);

                double error = 0;
                double last_error = 0;
                double KP = 2.7;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;

                while ((left_back_motor.getCurrentPosition() > left_back_motor_target_ticks) &&
                        (right_front_motor.getCurrentPosition() > right_front_motor_target_ticks)) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    left_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    right_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("LEFT_POWER and RIGHT_POWER", "%d %d",
                            left_back_motor.getCurrentPosition(), right_back_motor.getCurrentPosition());
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            } else if (power == 0) {
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            }
        }

        public void move_diagonale_right_with_pid (DcMotor left_front_motor, DcMotor
        left_back_motor, DcMotor right_front_motor,
                DcMotor right_back_motor
            ,double distance, double power, boolean read_gyro_angel){

            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            double gyro_start;

            if (read_gyro_angel == true) {
                gyro_start = gyro_angel;

            } else {
                gyro_start = 0;

            }
            double left_power;
            double right_power;
            if (power > 0) {


                left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
                right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));

                int left_front_motor_target_ticks = left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
                int right_back_motor_target_ticks = right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);


                double error = 0;
                double last_error = 0;
                double KP = 2.7;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;
                while ((left_front_motor.getCurrentPosition() < left_front_motor_target_ticks) &&
                        (right_back_motor.getCurrentPosition() < right_back_motor_target_ticks)) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    left_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    right_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("LEFT_POWER and RIGHT_POWER", "%d %d",
                            left_back_motor.getCurrentPosition(), right_back_motor.getCurrentPosition());
                    telemetry.update();

                }
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            } else if (power < 0) {
                left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
                right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));

                int left_front_motor_target_ticks = left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
                int right_back_motor_target_ticks = right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);


                double error = 0;
                double last_error = 0;
                double KP = 2.7;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;
                while ((left_front_motor.getCurrentPosition() > left_front_motor_target_ticks) &&
                        (right_back_motor.getCurrentPosition() > right_back_motor_target_ticks)) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    left_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    right_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("LEFT_POWER and RIGHT_POWER", "%d %d",
                            left_back_motor.getCurrentPosition(), right_back_motor.getCurrentPosition());
                    telemetry.update();

                }
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            } else if (power == 0) {
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            }
        }


        public void move_with_pid_with_color (DcMotor left_back_motor, DcMotor
        left_front_motor, DcMotor right_back_motor,
                DcMotor right_front_motor,double color, double power, boolean read_gyro_angel){

            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);



            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            double gyro_start;

            if (read_gyro_angel == true) {
                gyro_start = gyro_angel;

            } else {
                gyro_start = 0;

            }


            double left_power;
            double right_power;
            if (power > 0) {

                double error = 0;
                double last_error = 0;
                double KP = 2.7;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;
                while (color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != color) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    left_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    right_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("LEFT_POWER and RIGHT_POWER", "%d %d",
                            left_back_motor.getCurrentPosition(), right_back_motor.getCurrentPosition());
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                sleep(1);
            } else if (power < 0) {

                double error = 0;
                double last_error = 0;
                double KP = 3;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;
                while (color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != color) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    left_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    right_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("LEFT_POWER and RIGHT_POWER", "%d %d",
                            left_back_motor.getCurrentPosition(), right_back_motor.getCurrentPosition());
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                sleep(1);
            } else if (power == 0) {
            }
        }

        public void move_side_with_pid_with_color (DcMotor left_back_motor, DcMotor
        left_front_motor,
                DcMotor right_back_motor, DcMotor right_front_motor,double color, double power,
        boolean read_gyro_angel){
            left_back_motor.setDirection(DcMotor.Direction.REVERSE);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.FORWARD);

            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
            double gyro_start;

            if (read_gyro_angel == true) {
                gyro_start = gyro_angel;

            } else {
                gyro_start = 0;

            }

            if (power > 0) {


                double increase_power;
                double decrese_power;
                double error = 0;
                double last_error = 0;
                double KP = 2;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;


                while (color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != color) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    increase_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    decrese_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
//                left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power), -100, 100));
//                left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power), -100, 100));
//                right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power), -100, 100));
//                right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power), -100, 100));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("decrese_power and increase_power", "%f %f", decrese_power, increase_power);
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                sleep(1);
            } else if (power < 0) {


                // double gyro_start = 0 ;

                double increase_power;
                double decrese_power;
                double error = 0;
                double last_error = 0;
                double KP = 2;
                double KI = 0;  //.001
                double KD = 0;  //.2
                double probational = 0;
                double derivative = 0;
                double integral = 0;


                while (color_sensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != color) {
                    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                    error = gyro_angel - gyro_start;
                    probational = error;
                    integral = integral + error;
                    derivative = error - last_error;
                    decrese_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                    increase_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));
//                left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power),-100,100));
//                left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power),-100,100));
//                right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power),-100,100));
//                right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power),-100,100));
                    left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power), -100, 100));
                    left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power), -100, 100));
                    right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power), -100, 100));
                    right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power), -100, 100));
                    sleep(1);
                    last_error = error;
                    telemetry.addData("decrese_power and increase_power", "%f %f", decrese_power, increase_power);
                    telemetry.update();
                }
                left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
                sleep(1);
            } else if (power == 0) {
            }
        }


        void composeTelemetry () {

            // At the beginning of each telemetry update, grab a bunch of data
            // from the IMU that we will then display in separate lines.
            telemetry.addAction(new Runnable() {
                @Override
                public void run() {
                    // Acquiring the angles is relatively expensive; we don't want
                    // to do that in each of the three items that need that info, as that's
                    // three times the necessary expense.
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                }
            });

            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });

            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            gyro_angel = Double.parseDouble(helper_class_object.formatAngle(angles.angleUnit, angles.firstAngle));
                            return helper_class_object.formatAngle(angles.angleUnit, angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return helper_class_object.formatAngle(angles.angleUnit, angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return helper_class_object.formatAngle(angles.angleUnit, angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override
                        public String value() {
                            return gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override
                        public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(gravity.xAccel * gravity.xAccel
                                            + gravity.yAccel * gravity.yAccel
                                            + gravity.zAccel * gravity.zAccel));
                        }
                    });
        }
    }



