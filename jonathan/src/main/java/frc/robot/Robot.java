/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;


/**
 * Challenge 1
 * 
 * Use the front distance sensor and a state machine to stop the robot before crashing
 * into a wall.
 * 
 * State 1: Drive forward. Transition to state 2 when distance sensor senses a close object
 * State 2: Stop.
 */

public class Robot extends TimedRobot {
  private CANSparkMax flMotor;
  private CANSparkMax frMotor;
  private CANSparkMax blMotor;
  private CANSparkMax brMotor;

  private DifferentialDrive drive;

  private AHRS gyro;
  private Ultrasonic backDistanceSensor;
  private Ultrasonic frontDistanceSensor;

  private XboxController controller;

  private String state;

  @Override
  public void robotInit() {

    // create an object to control each motor
    flMotor = new CANSparkMax(1, MotorType.kBrushless);
    frMotor = new CANSparkMax(2, MotorType.kBrushless);
    blMotor = new CANSparkMax(3, MotorType.kBrushless);
    brMotor = new CANSparkMax(4, MotorType.kBrushless);

    // group the motors
    SpeedControllerGroup left = new SpeedControllerGroup(flMotor, blMotor);
    SpeedControllerGroup right = new SpeedControllerGroup(frMotor, brMotor);

    // use the drive object for easily controlling all the motors with arcade or tank drive
    drive = new DifferentialDrive(left, right);

    // use this to get angle readings from the navx's gyro sensor
    gyro = new AHRS(SPI.Port.kMXP);

    controller = new XboxController(0);

    backDistanceSensor = new Ultrasonic(0,1);
    frontDistanceSensor = new Ultrasonic(2, 3);

    backDistanceSensor.setAutomaticMode(true);
    frontDistanceSensor.setAutomaticMode(true);

    state = "moveForward";
  }

  public double getFrontDistance() {
    return frontDistanceSensor.getRangeInches();
  }

  public double getBackDistance() {
    return backDistanceSensor.getRangeInches();
  }

  @Override
  public void autonomousInit() {
    // Resets the gyro angle to 0. Should be called at the beginning of each challenge
    gyro.reset();
  }

  @Override
  public void autonomousPeriodic() {

    double angle = gyro.getAngle();
    double frontDistance = getFrontDistance();
    double backDistance = getBackDistance();

    // set these values to change speed and turn rate of the robot
    double speed = 0.0;
    double turnRate = 0.0;

    // Example state machine which makes the robot rotate left and right
    if (state == "moveForward") {
      // turn right code
      turnRate = 0.0;
      speed = 0.3;

      // transition code
      if (frontDistance < 15.0) {
        state = "stop";
      }
    } 
    else if (state == "stop") {
      // turn left code
      turnRate = 0.0;
      speed = 0.0;

      // transition code
      if (frontDistance < 15.0) {
       state = "moveForward";
     }
      if (backDistance < 15.0) {
        state = "moveBackward";
      }
    }
    else if (state == "moveForward") {
      // turn right code
      turnRate = 0.0;
      speed = -0.3;

      // transition code
      if (backDistance < 15.0) {
        state = "stop";
      }
    } 

    drive.arcadeDrive(speed, turnRate);
  }

  @Override
  public void teleopInit() {
    // Resets the gyro angle to 0. Should be called at the beginning of each challenge
    gyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    double frontDistance = getFrontDistance();
    double backDistance = getBackDistance();

    System.out.println("DISTANCE: " + frontDistance + ", " + backDistance);

    double speed = controller.getY();
    double turnRate = controller.getX();

    drive.arcadeDrive(-speed * .3, turnRate * .5);
  }
}
