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

/**
 * Challenge 1
 * 
 * Make the robot face Amory and then DESTROY HIM! In this challenge you'll 
 * be given the angle Amory is located relative to the robot through
 * NetworkTables. Using the gyro, you must turn the robot to face Amory and
 * then charge! Your code will need to perform this challenge 3 times in order
 * to pass, so remember to reset the gyro on enable!
 */

public class Robot extends TimedRobot {
  private CANSparkMax flMotor;
  private CANSparkMax frMotor;
  private CANSparkMax blMotor;
  private CANSparkMax brMotor;

  private DifferentialDrive drive;

  private AHRS gyro;

  private NetworkTableEntry targetAngleEntry;

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
  }

  @Override
  public void autonomousInit() {
    // Resets the gyro angle to 0. Should be called at the beginning of each challenge
    gyro.reset();
  }

  public double getTargetAngle() {
    return targetAngleEntry.getDouble(45);
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
    double speed = 0;
    double turnRate = 0;
    // i know something goes here guys i compleely forget kotlin oh no i only know python i have no idea what im doing afurrea
    if (angle != targetAngle){
        turnRate = .1;
    } else{
        speed = .5;
    }


    drive.arcadeDrive(-speed, turnRate);
  }
}

//cringe this code is so cringe 
// doodoo fart poopoo
// poo