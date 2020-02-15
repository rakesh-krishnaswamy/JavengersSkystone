package org.firstinspires.ftc.teamcode.mainops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.Constants;


/*
 * Title: AutoBlueCraterBase
 * Date Created: 11/23/2018
 * Date Modified: 2/22/2019
 * Author: Rahul, Poorvi, Varnika
 * Type: Main
 */

@Autonomous(name = "New Blue Side", group = "Concept")
public class NewBlueSide extends LinearOpMode {
    private AutoLib autoLib;
    // Description: Starts on blue crater latcher

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        float fastPower = 0.5f;
        float mediumPower = 0.4f;
        float slowPower = 0.2f;
        float verySlowPower = 0.1f;
        double distance = 0;
        float armDistance = 12f;
        float foundationDistance = 3f;
        float defaultMaxDistance = 15f;

        // Vuforia
        autoLib.autonArmDown();
        autoLib.calcMove(55, mediumPower, Constants.Direction.RIGHT);
        Constants.Coordinates coordinates = autoLib.readCoordinates();
        distance = autoLib.getDistanceCM();
        telemetry.addData("x", coordinates.xPosition);
        telemetry.addData("y", coordinates.yPosition);
        telemetry.addData("Distance at base initial", distance);
        telemetry.update();
        if (coordinates.yPosition < 0) {
            telemetry.addData("pos", "Left");
            telemetry.update();
            autoLib.calcMove((float) (coordinates.yPosition / 10 + 35), slowPower, Constants.Direction.FORWARD); //when decreased- moves to the left
            autoLib.calcMove((float) (-coordinates.xPosition / 10 - 12), slowPower, Constants.Direction.RIGHT);   //when decreased-moves back
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(500);
            autoLib.autonArmUp();
            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
//            autoLib.calcTurn(5, slowPower);    // turn, so that the robot will go straight
            autoLib.calcMove(175, mediumPower, Constants.Direction.FORWARD);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > 9) {
                autoLib.calcMove((float) (distance - 2), slowPower, Constants.Direction.RIGHT);
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();

            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonScore();
//            autoLib.calcTurn(6, slowPower);
            autoLib.autonArmUp();
            /*
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
            autoLib.calcMove(240, fastPower, Constants.Direction.BACKWARD); // move backward towards base
            distance = autoLib.getDistanceCM();
            if(distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at base before reaching to stone 2-a", distance);
            telemetry.update();
            if (distance > armDistance) {
                autoLib.calcMove((float) distance - armDistance, slowPower, Constants.Direction.RIGHT);
            } else if (distance < armDistance) {
                autoLib.calcMove((float) (armDistance - distance), slowPower, Constants.Direction.LEFT);
            }
            telemetry.addData("Distance at base before reaching to stone 2-b", autoLib.getDistanceCM());
            telemetry.update();
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(400);
            autoLib.autonArmUp();
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
            autoLib.calcMove(240, fastPower, Constants.Direction.FORWARD);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if(distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 2-a", distance);
            telemetry.update();
            if (distance > armDistance) {
                autoLib.calcMove((float) distance - armDistance, slowPower, Constants.Direction.RIGHT);
            } else if (distance < armDistance) {
                autoLib.calcMove((float) (armDistance - distance), slowPower, Constants.Direction.LEFT);
            }
            telemetry.addData("Distance at foundation before placing stone 2-b", autoLib.getDistanceCM());
            telemetry.update();

            Thread.sleep(300);
            autoLib.autonArmDown();
            Thread.sleep(300);
            autoLib.autonScore();
            autoLib.autonArmUp();
             */
            autoLib.calcTurn(74, slowPower);    // turn, so that foundation grabbers can be used


            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
            autoLib.calcMove((float) (distance + foundationDistance), verySlowPower, Constants.Direction.BACKWARD);
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();

//            //autoLib.moveUntilSensorTouched(Constants.FOUNDATION_TOUCH_SENSOR, slowPower);
//            boolean isFoundationTouchSensorPressed = autoLib.isFoundationTouchSensorPressed();
//            telemetry.addData("isFoundationTouchSensorPressed 1", isFoundationTouchSensorPressed);
//            telemetry.update();
//            if(!isFoundationTouchSensorPressed) {
//                telemetry.addData("inside move backward", isFoundationTouchSensorPressed);
//                telemetry.update();
//                autoLib.moveBackward(slowPower);
//            }
//            telemetry.addData("isFoundationTouchSensorPressed 2", isFoundationTouchSensorPressed);
//            telemetry.update();


            Thread.sleep(300);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
//            autoLib.calcTurn(25, fastPower);
//            autoLib.calcTurn(30, 1f);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(180, .6f);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(300);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);

        } else if (coordinates.yPosition > 0) {
            telemetry.addData("pos", "Center");
            telemetry.addData("x", coordinates.xPosition);
            telemetry.addData("y", coordinates.yPosition);
            telemetry.addData("Distance at base initial", distance);
            telemetry.update();
//            autoLib.calcMove((float) (coordinates.yPosition / 10 - 3), mediumPower, Constants.Direction.FORWARD); //when decreased- moves to the left
            autoLib.calcMove((float) (-coordinates.xPosition / 10 - 12), slowPower, Constants.Direction.RIGHT);   //when increased-moves back
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(500);
            autoLib.autonArmUp();
            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
//            autoLib.calcTurn(5, slowPower);    // turn, so that the robot will go straight
            autoLib.calcMove(205, mediumPower, Constants.Direction.FORWARD);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > 9) {
                autoLib.calcMove((float) (distance - 6), slowPower, Constants.Direction.RIGHT);     //distance - 9
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();

            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonScore();
//            autoLib.calcTurn(6, slowPower);
            autoLib.autonArmUp();
            autoLib.calcTurn(72, slowPower);    // turn, so that foundation grabbers can be used


            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
//            if (distance > foundationDistance) {
            autoLib.calcMove((float) distance + foundationDistance, .05f, Constants.Direction.BACKWARD);
//            }
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();

            Thread.sleep(300);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
//            autoLib.calcTurn(25, fastPower);
//            autoLib.calcTurn(30, 1f);
            autoLib.calcMove(85, fastPower, Constants.Direction.FORWARD);  // move closer to foundation-70
            autoLib.calcTurn(175, 1f);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(300);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);

        } else {
            telemetry.addData("pos", "Right");
            telemetry.addData("x", coordinates.xPosition);
            telemetry.addData("y", coordinates.yPosition);
            telemetry.addData("Distance at base initial", distance);
            telemetry.update();
            autoLib.calcMove((float) (coordinates.yPosition / 10 - 25), mediumPower, Constants.Direction.FORWARD); //when decreased- moves to the left
            autoLib.calcMove((float) (-coordinates.xPosition / 10 - 12), slowPower, Constants.Direction.RIGHT);   //when increased-moves back
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(500);
            autoLib.autonArmUp();
            Thread.sleep(400);
            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
//            autoLib.calcTurn(5, slowPower);    // turn, so that the robot will go straight
            autoLib.calcMove(240, fastPower, Constants.Direction.FORWARD);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > 9) {
                autoLib.calcMove((float) (distance - 9), slowPower, Constants.Direction.RIGHT);
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();

            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonScore();
//            autoLib.calcTurn(6, slowPower);
            autoLib.autonArmUp();
            autoLib.calcTurn(74, slowPower);    // turn, so that foundation grabbers can be used


            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
//            if (distance > foundationDistance) {
            autoLib.calcMove((float) (distance + foundationDistance), verySlowPower, Constants.Direction.BACKWARD);
//            }
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();

            Thread.sleep(300);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
//            autoLib.calcTurn(25, fastPower);
            autoLib.calcTurn(30, 1f);
            autoLib.calcMove(70, fastPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(130, .6f);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(300);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);
        }
        telemetry.update();
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        autoLib = new AutoLib(this);

        autoLib.restServoFoundation();

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
