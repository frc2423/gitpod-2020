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
 * Challenge 2
 * 
 * Same as the previous challenge, but the angle I provide will be very large.
 * You must complete this challenge in under 15 seconds, 3 times in order to
 * pass! 
 * 
 * HINT: -360, 0 and 360 degrees are the same angle.
 * 
 * BONUS: Make the robot turn in the most optimal direction.
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

    if (angle > targetAngle + 5) {
      turnRate = -.5;
    } else if (angle < targetAngle - 5) {
      turnRate = .5;
    } else {
      speed = .3;
    }

    drive.arcadeDrive(speed, turnRate);
  }
}
