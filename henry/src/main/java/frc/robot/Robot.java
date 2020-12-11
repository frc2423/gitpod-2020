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
import edu.wpi.first.wpilibj.XboxController;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;



/**
 * Challenge 1
 * 
 */

public class Robot extends TimedRobot {

  // create an object to control each motor
  private CANSparkMax flMotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax frMotor = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax blMotor = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax brMotor = new CANSparkMax(4, MotorType.kBrushless);
  
  private DifferentialDrive drive;

  // use this to get angle readings from the navx's gyro sensor
  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private XboxController controller = new XboxController(0);

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("robot");
  private NetworkTableEntry targetAngle = table.getEntry("targetAngle");
  private double accumulatedError = 0;


  @Override
  public void robotInit() {

    // group the motors
    SpeedControllerGroup left = new SpeedControllerGroup(flMotor, blMotor);
    SpeedControllerGroup right = new SpeedControllerGroup(frMotor, brMotor);

    // use the drive object for easily controlling all the motors with arcade or tank drive
    drive = new DifferentialDrive(left, right);

    targetAngle.setDouble(0);
  }

  double bound(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }

  double getLeftVelocity() {
    double LEFT_TICKS_PER_REV = 14.857;
    double FEET_PER_REV = .666667 * Math.PI;
    double LEFT_TICKS_PER_FOOT = LEFT_TICKS_PER_REV / FEET_PER_REV;
    return flMotor.getEncoder().getVelocity() / LEFT_TICKS_PER_FOOT / 60;
  }

  double getRightVelocity() {
    double RIGHT_TICKS_PER_REV = -12.786;
    double FEET_PER_REV = .666667 * Math.PI;
    double RIGHT_TICKS_PER_FOOT = RIGHT_TICKS_PER_REV / FEET_PER_REV;
    return frMotor.getEncoder().getVelocity() / RIGHT_TICKS_PER_FOOT / 60;
  }

  @Override
  public void autonomousInit() {
    gyro.reset();
  }

  @Override
  public void autonomousPeriodic() {

    // gets the current velocity of the robot in ft/s
    double velocity = getLeftVelocity();

    // The robot's current heading
    double angle = gyro.getAngle();

    // The angle we want to face
    double targetAngle = this.targetAngle.getDouble(0);

    double error = targetAngle - angle;

    accumulatedError = accumulatedError + error;

    double speed = 0;
    double turnRate = 0.25 * error + 0.05 * accumulatedError;

    drive.arcadeDrive(bound(speed, -.6, .6), bound(turnRate, -.6, .6));
  }



  @Override
  public void teleopInit() {
    gyro.reset();
  }

  @Override
  public void teleopPeriodic() {

    double speed = controller.getY();
    double turnRate = controller.getX();

    drive.arcadeDrive(-speed * .3, turnRate * .5);

  }
}
