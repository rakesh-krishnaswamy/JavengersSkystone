package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.libraries.Constants.BRAKE_POINT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LEFT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RIGHT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.NEVEREST_40_REVOLUTION_ENCODER_COUNT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_DOWN_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_GRABBER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_GRABBER_GRAB;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_GRABBER_SCORE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_UP_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_GRAB1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_GRAB2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_REST1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION_REST2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.VUFORIA_KEY;
import static org.firstinspires.ftc.teamcode.libraries.Constants.VUFORIA_READING_TIME;
import static org.firstinspires.ftc.teamcode.libraries.Constants.WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.WHEEL_GEAR_RATIO;

/*
 * Title: AutoLib
 * Date Created: 10/28/2018
 * Date Modified: 1/22/2019
 * Author: Poorvi, Sachin
 * Type: Library
 * Description: This will contain the methods for Autonomous, and other autonomous-related programs.
 */


public class AutoLib {
    private Robot robot;
    private LinearOpMode opMode;
    private List<VuforiaTrackable> allTrackables;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables targetsSkyStone;


    // Declaring TensorFlow detection
    private TFObjectDetector tfod;

    //
    public AutoLib(LinearOpMode opMode) {
        robot = new Robot(opMode);
        this.opMode = opMode;
        OpMode aOpMode;


        initVuforia();
    }

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

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

    String positionSkystone = "";
    boolean startIdentify = true;
    float distanceToDepot = 110;    //115
    float distanceToCenterLine = 5.5f;
    float forwardDistanceSkystone = 28f;
    float turningDegree = -50;
    float foundation = 14;
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;


    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;


    //********** Base Motor Methods **********//

    public void calcMove(float centimeters, float power, Constants.Direction direction) {
        // Calculates target encoder position
        if (direction == Constants.Direction.RIGHT || direction == Constants.Direction.LEFT) {
            centimeters = centimeters + 10;
        } else if (direction == Constants.Direction.FORWARD || direction == Constants.Direction.BACKWARD) {
            centimeters = centimeters - BRAKE_POINT;
        }

        final int targetPosition = (int) ((((centimeters / (Math.PI * WHEEL_DIAMETER)) *
                NEVEREST_40_REVOLUTION_ENCODER_COUNT)) * WHEEL_GEAR_RATIO);

//        switch (direction) {
//            case BACKWARD:
//                prepMotorsForCalcMove(targetPosition, targetPosition, targetPosition, targetPosition);
//                break;
//            case FORWARD:
//                prepMotorsForCalcMove(-targetPosition, -targetPosition, -targetPosition, -targetPosition);
//                break;
//            case LEFT:
//                prepMotorsForCalcMove(-targetPosition, targetPosition, targetPosition, -targetPosition);
//                break;
//            case RIGHT:
//                prepMotorsForCalcMove(targetPosition, -targetPosition, -targetPosition, targetPosition);
//        }

        switch (direction) {
            case BACKWARD:
                prepMotorsForCalcMove(-targetPosition, -targetPosition, -targetPosition, -targetPosition);
                break;
            case FORWARD:
                prepMotorsForCalcMove(targetPosition, targetPosition, targetPosition, targetPosition);
                break;
            case LEFT:
                prepMotorsForCalcMove(-targetPosition, targetPosition, targetPosition, -targetPosition);
                break;
            case RIGHT:
                prepMotorsForCalcMove(targetPosition, -targetPosition, -targetPosition, targetPosition);
        }

        setBaseMotorPowers(power);


        while (areBaseMotorsBusy()) {
            opMode.idle();
        }

        setBaseMotorPowers(0);
    }



    public void rampMove(float distance, float power, Constants.Direction Direction, boolean isRampedPower) throws InterruptedException {
        if (Direction == Constants.Direction.FORWARD) {
            moveRobotToPositionFB(distance * (1f / 1), power, isRampedPower);
        } else if (Direction == Constants.Direction.BACKWARD) {
            moveRobotToPositionFB(-distance * (1f / 1), power, isRampedPower);
        } else if (Direction == Constants.Direction.LEFT) {
            moveRobotToPositionSideways(distance * (1f / 1), power, isRampedPower);
        } else if (Direction == Constants.Direction.RIGHT) {
            moveRobotToPositionSideways(-distance * (1f / 1), power, isRampedPower);
        }

        while (areBaseMotorsBusy()) {
            opMode.idle();
        }

        setBaseMotorPowers(0);

    }

    public void moveRobotToPositionFB(float distance, float power, boolean isRampedPower) throws InterruptedException {
        //we need to store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) (((distance) / (Math.PI * WHEEL_DIAMETER)) * NEVEREST_40_REVOLUTION_ENCODER_COUNT);
        //using the generic method with all powers set to the same value and all positions set to the same position
        robot.runRobotToPosition(power, power, power, power, targetPosition, targetPosition, targetPosition, targetPosition, isRampedPower);
    }

    public void moveRobotToPositionSideways(float distance, float power, boolean isRampedPower)
            throws InterruptedException {
        //we need to
        //store the encoder target position
        int targetPosition;
        //calculate target position from the input distance in cm
        targetPosition = (int) ((distance / (Math.PI * WHEEL_DIAMETER)) * NEVEREST_40_REVOLUTION_ENCODER_COUNT);
        //using the generic method with all powers set to the same value and all positions set to the same position
        robot.runRobotToPosition(power, power, power, power, -targetPosition, targetPosition, targetPosition, -targetPosition, isRampedPower);
    }


    public void calcTurn(int degrees, float power) {
        // Calculates target encoder position
        int targetPosition = (int) (2 * ((TRACK_DISTANCE) * degrees
                * NEVEREST_40_REVOLUTION_ENCODER_COUNT) /
                (WHEEL_DIAMETER * 360));


        prepMotorsForCalcMove(-targetPosition, targetPosition, -targetPosition, targetPosition);

        setBaseMotorPowers(power);

        while (areBaseMotorsBusy()) {
            opMode.idle();
        }

        setBaseMotorPowers(0);
    }

    public void diagonalMove(float centimeters, float power, Constants.Direction direction) {
        // Calculates target encoder position

        final int targetPosition = (int) ((((centimeters / (Math.PI * WHEEL_DIAMETER)) *
                NEVEREST_40_REVOLUTION_ENCODER_COUNT)) * WHEEL_GEAR_RATIO);

        switch (direction) {
            case BACKWARD:
                prepMotorsForDiagonalMove(-targetPosition, -targetPosition);
                break;
            case FORWARD:
                prepMotorsForDiagonalMove(targetPosition, targetPosition);
                break;
        }

        setBaseMotorPowersDiagonal(power);


        while (areBaseMotorsBusyDiagonal()) {
            opMode.idle();
        }

        setBaseMotorPowersDiagonal(0);
    }


    private void setBaseMotorPowers(float power) {
        robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, power);
    }

    private void setBaseMotorPowersDiagonal(float power) {
        robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, power);
    }

    private void prepMotorsForCalcMove(int frontLeftTargetPosition, int frontRightTargetPosition,
                                       int backLeftTargetPosition, int backRightTargetPosition) {

        robot.setDcMotorTargetPosition(MOTOR_FRONT_LEFT_WHEEL, frontLeftTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_FRONT_RIGHT_WHEEL, frontRightTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_BACK_LEFT_WHEEL, backLeftTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_BACK_RIGHT_WHEEL, backRightTargetPosition);

        robot.setDcMotorMode(MOTOR_FRONT_LEFT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_FRONT_RIGHT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_BACK_LEFT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_BACK_RIGHT_WHEEL, STOP_AND_RESET_ENCODER);

        robot.setDcMotorMode(MOTOR_FRONT_LEFT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_FRONT_RIGHT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_BACK_LEFT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_BACK_RIGHT_WHEEL, RUN_TO_POSITION);
    }

    private void prepMotorsForDiagonalMove(int frontRightTargetPosition, int backLeftTargetPosition) {

        robot.setDcMotorTargetPosition(MOTOR_FRONT_RIGHT_WHEEL, frontRightTargetPosition);
        robot.setDcMotorTargetPosition(MOTOR_BACK_LEFT_WHEEL, backLeftTargetPosition);

        robot.setDcMotorMode(MOTOR_FRONT_RIGHT_WHEEL, STOP_AND_RESET_ENCODER);
        robot.setDcMotorMode(MOTOR_BACK_LEFT_WHEEL, STOP_AND_RESET_ENCODER);

        robot.setDcMotorMode(MOTOR_FRONT_RIGHT_WHEEL, RUN_TO_POSITION);
        robot.setDcMotorMode(MOTOR_BACK_LEFT_WHEEL, RUN_TO_POSITION);
    }

    private boolean areBaseMotorsBusy() {
        return robot.isMotorBusy(MOTOR_FRONT_LEFT_WHEEL) || robot.isMotorBusy(MOTOR_FRONT_RIGHT_WHEEL) ||
                robot.isMotorBusy(MOTOR_BACK_LEFT_WHEEL) || robot.isMotorBusy(MOTOR_BACK_RIGHT_WHEEL);
    }

    private boolean areBaseMotorsBusyDiagonal() {
        return robot.isMotorBusy(MOTOR_FRONT_RIGHT_WHEEL) || robot.isMotorBusy(MOTOR_BACK_LEFT_WHEEL);
    }


    //********** Motor Methods **********//

    public void intakeStone() {
        ElapsedTime time = new ElapsedTime();

        robot.setDcMotorPower(MOTOR_RIGHT_INTAKE, -.5f);
        robot.setDcMotorPower(MOTOR_LEFT_INTAKE, .5f);
        while (time.seconds() <= 5) {
            opMode.idle();
        }
    }

    public void armGrab() throws InterruptedException {


        // Thread.sleep(100);
//        robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_GRAB);
    }

//    public void moveArmUp() {
//        robot.setDcMotorPower(MOTOR_ARM, -0.5f);
//
//        while (!robot.isTouchSensorPressed(TOUCH_ARM_TOP)) {
//            opMode.idle();
//            opMode.telemetry.addData("Status", robot.isTouchSensorPressed(TOUCH_ARM_TOP));
//            opMode.telemetry.update();
//        }
//    }

//    public void distanceSensorMove() {
//        while (robot.getDistanceCM() >= 15) {
//            robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, .2f);
//            robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, .2f);
//            robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, .2f);
//            robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, .2f);
//        }
//        opMode.idle();
//    }

    public boolean isFoundationTouchSensorPressed() {
        return robot.isFoundationTouchSensorPressed();
    }

    public void moveUntilSensorTouched(int index, float power) {
        while (!robot.isTouchSensorPressed(index)) {
            moveBackward(power);
        }
        setBaseMotorPowers(0);
    }

    public void moveForward(float power) {
        robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, -power);
        robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, -power);
    }

    public void moveBackward(float power) {
        robot.setDcMotorPower(MOTOR_FRONT_LEFT_WHEEL, -power);
        robot.setDcMotorPower(MOTOR_FRONT_RIGHT_WHEEL, power);
        robot.setDcMotorPower(MOTOR_BACK_LEFT_WHEEL, -power);
        robot.setDcMotorPower(MOTOR_BACK_RIGHT_WHEEL, power);
    }

//    public void moveArmDown() {
//        robot.setDcMotorPower(MOTOR_ARM, 0.7f);
//
//        while (!robot.isTouchSensorPressed(TOUCH_ARM_BOTTOM)) {
//            opMode.idle();
//            opMode.telemetry.addData("Status", robot.isTouchSensorPressed(TOUCH_ARM_BOTTOM));
//            opMode.telemetry.update();
//        }
//    }

//    public void moveArmUpSeconds() {
//        ElapsedTime time = new ElapsedTime();
//
//        robot.setDcMotorPower(MOTOR_ARM_, -1f);
//        while (time.seconds() <= 1) {
//            opMode.idle();
//        }
//        robot.setDcMotorPower(MOTOR_ARM, 0);
//    }
//
//    public void moveArmUpSeconds1() {
//        ElapsedTime time = new ElapsedTime();
//
//        robot.setDcMotorPower(MOTOR_ARM, -.9f);
//        while (time.seconds() <= .5) {
//            opMode.idle();
//        }
//        robot.setDcMotorPower(MOTOR_ARM, 0);
//    }
//
//    public void moveArmDownSeconds() throws InterruptedException {
//        ElapsedTime time = new ElapsedTime();
//
//        robot.setDcMotorPower(MOTOR_ARM, .7f);
//        while (time.seconds() <= 1.25) {
//            opMode.idle();
//        }
//        robot.setDcMotorPower(MOTOR_ARM, 0);
//    }

    //********** Servo Methods **********//

    //    public void recieveServoArm() {
//        robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_RECIEVE);
//    }
//
//    public void scoreServoArm() {
//        robot.setServoPosition(SERVO_ARM, SERVO_ARM_POS_SCORE);
//    }
//
//    public void grabServo() {
//        robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_GRAB);
//    }
//
//    public void scoreServo() {
//        robot.setServoPosition(SERVO_GRABBER, SERVO_GRABBER_REST);
//    }
//
    public void latchServoFoundation() {
        robot.setServoPosition(SERVO_FOUNDATION1, SERVO_FOUNDATION_GRAB1);
        robot.setServoPosition(SERVO_FOUNDATION2, SERVO_FOUNDATION_GRAB2);
    }


    public void restServoFoundation() {
        robot.setServoPosition(SERVO_FOUNDATION1, SERVO_FOUNDATION_REST1);
        robot.setServoPosition(SERVO_FOUNDATION2, SERVO_FOUNDATION_REST2);
    }

    public void autonArmDown() {
        robot.setServoPosition(SERVO_AUTONOMOUS_ARM, SERVO_AUTONOMOUS_DOWN_ARM);
        while (true) {
            if (robot.getServoPosition(SERVO_AUTONOMOUS_ARM) == SERVO_AUTONOMOUS_DOWN_ARM) {
                break;
            }

        }
    }

    public void autonArmUp() {
        robot.setServoPosition(SERVO_AUTONOMOUS_ARM, SERVO_AUTONOMOUS_UP_ARM);
//        while (true){
//            if(robot.getServoPosition(SERVO_AUTONOMOUS_ARM) ==SERVO_AUTONOMOUS_UP_ARM) {
//                break;
//            }
//
//        }

    }

    public void autonGrab() {
        robot.setServoPosition(SERVO_AUTONOMOUS_GRABBER, SERVO_AUTONOMOUS_GRABBER_GRAB);
//        while (true){
//            if(robot.getServoPosition(SERVO_AUTONOMOUS_GRABBER) ==SERVO_AUTONOMOUS_GRABBER_GRAB) {
//                break;
//            }
//
//        }
    }

    public void autonScore() {
        robot.setServoPosition(SERVO_AUTONOMOUS_GRABBER, SERVO_AUTONOMOUS_GRABBER_SCORE);
    }

    //********** Vuforia Methods **********//

    private void initVuforia() {

        webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam1");


        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        Constants.Coordinates coordinates = null;
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
    }

    public Constants.Coordinates readCoordinates() {
        double xPosition = 0;
        double yPosition = 0;

        if (tfod != null) {
            tfod.activate();
        }


        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        if (tfod != null) {
            tfod.shutdown();
        }

        targetsSkyStone.activate();
        if (startIdentify) {
            int count = 0;
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (startIdentify && time.seconds() < VUFORIA_READING_TIME) {
                count++;
                targetVisible = false;
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        targetVisible = true;

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }

                if (targetVisible) {
                    // express position (translation) of robot in inches.
                    VectorF translation = lastLocation.getTranslation();

                    yPosition = translation.get(1);
                    xPosition = translation.get(0);
                    break;
                }
            }
        }
        return new Constants.Coordinates(xPosition, yPosition);
    }

    public double getDistanceCM() {
        return robot.getDistanceCM();
    }

    public double getFoundationDistance() {
        return robot.getFoundationDistance();
    }

}