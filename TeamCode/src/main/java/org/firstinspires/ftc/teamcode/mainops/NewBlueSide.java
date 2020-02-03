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
 * Description: Starts on blue crater latcher
 */

@Autonomous(name = "New Blue Side", group = "Concept")
public class NewBlueSide extends LinearOpMode {
    private AutoLib autoLib;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        float fastPower = 0.5f;
        float mediumPower = 0.4f;
        float slowPower = 0.2f;
        double distance = 0;
        float armDistance = 12f;
        float defaultMaxDistance = 50f;

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
            autoLib.calcMove((float) (coordinates.yPosition / 10 + 17), mediumPower, Constants.Direction.FORWARD); //when decreased- moves to the left
            autoLib.calcMove((float) (-coordinates.xPosition / 10 - 12), mediumPower, Constants.Direction.RIGHT);   //when increased-moves back
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(500);
            autoLib.autonArmUp();
            Thread.sleep(400);
//            autoLib.calcTurn(5, slowPower);
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
            autoLib.calcTurn(5, slowPower);    // turn, so that the robot will go straight
            autoLib.calcMove(180, fastPower, Constants.Direction.FORWARD);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if(distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > 2) {
                autoLib.calcMove((float) (distance-2), slowPower, Constants.Direction.RIGHT);
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
            autoLib.calcTurn(75, slowPower);    // turn, so that foundation grabbers can be used
            autoLib.calcMove(10, fastPower, Constants.Direction.BACKWARD);
            Thread.sleep(300);
            autoLib.latchServoFoundation();
            Thread.sleep(500);
            autoLib.calcTurn(25,fastPower);
            autoLib.calcMove(90, fastPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(140, fastPower);
            autoLib.calcMove(15,1f, Constants.Direction.BACKWARD);
            Thread.sleep(300);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(50, fastPower, Constants.Direction.FORWARD);

        } else if (coordinates.yPosition > 0) {
            telemetry.addData("pos", "Center");
            telemetry.update();
            autoLib.calcMove((float) (coordinates.yPosition / 10 + 3), mediumPower, Constants.Direction.FORWARD); //when decreased- moves to the left
            autoLib.calcMove((float) (-coordinates.xPosition / 10 - 12), slowPower, Constants.Direction.RIGHT);   //when increased-moves back
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(500);
            autoLib.autonArmUp();
            Thread.sleep(400);
//            autoLib.calcTurn(5, slowPower);
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
            autoLib.calcTurn(3, slowPower);    // turn, so that the robot will go straight
            autoLib.calcMove(194, fastPower, Constants.Direction.FORWARD);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if(distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > armDistance) {
                autoLib.calcMove((float) distance - armDistance, slowPower, Constants.Direction.RIGHT);
            } else if (distance < armDistance) {
                autoLib.calcMove((float) (armDistance - distance), slowPower, Constants.Direction.LEFT);
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();

            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonScore();
//            autoLib.calcTurn(6, slowPower);
            autoLib.autonArmUp();

            autoLib.calcTurn(75, slowPower);    // turn, so that foundation grabbers can be used
            autoLib.calcMove(10, fastPower, Constants.Direction.BACKWARD);
            Thread.sleep(300);
            autoLib.latchServoFoundation();
            Thread.sleep(500);
            autoLib.calcMove(50, fastPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(120, slowPower);
            Thread.sleep(300);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(20, fastPower, Constants.Direction.FORWARD);


        } else {
            telemetry.addData("pos", "Right");
            telemetry.update();
//            autoLib.calcMove((float) (coordinates.yPosition / 10), .9f, Constants.Direction.FORWARD);
//            autoLib.calcMove((float) (-coordinates.xPosition / 10), .9f, Constants.Direction.RIGHT);
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
