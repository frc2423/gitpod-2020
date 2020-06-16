/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Challenge 3
 * 
 * AMORY'S REVENGE! For this challenge we'll be playing a game of kick the robot!
 * You'll need to make the robot cross the room while Amory is kicking it! The robot
 * must stay in bounds and the robot's speed will be capped. This challenge will be
 * timed, and whoever can cross the room in the shortest amount of time wins!
 */

public class Robot extends TimedRobot {
  private CANSparkMax flMotor;
  private CANSparkMax frMotor;
  private CANSparkMax blMotor;
  private CANSparkMax brMotor;

  private DifferentialDrive drive;

  private AHRS gyro;

  private NetworkTableEntry targetAngleEntry;

  private XboxController controller;

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

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("robot");
    targetAngleEntry = table.getEntry("targetAngle");

    controller = new XboxController(0);
  }

  public double getTargetAngle() {
    return 0;
  }

  @Override
  public void autonomousPeriodic() {

    // use this to get the current heading of the robot
    double angle = gyro.getAngle();

    // use this to get the angle the robot needs to be pointed at to face Amory
    double targetAngle = getTargetAngle();

    // print out the angle reading. For Testing purposes.
    System.out.println("current angle: " + angle);
    System.out.println("target angle: " + targetAngle);

    // set these values to change speed and turn rate of the robot
    double speed = 1;
    double turnRate = 0;

    if (angle > targetAngle + 5) {
      turnRate = -.5;
    } else if (angle < targetAngle - 5) {
      turnRate = .5;
    }
// ayo this code kinda cringe 
    drive.arcadeDrive(speed, turnRate);
  }

  @Override
  public void teleopInit() {
    // Resets the gyro angle to 0. Should be called at the beginning of each challenge
    gyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    double speed = controller.getY();
    double turnRate = controller.getX();

    drive.arcadeDrive(-speed, turnRate);
  }
}
