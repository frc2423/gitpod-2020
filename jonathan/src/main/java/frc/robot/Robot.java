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

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


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

  private AHRS gyro;

  private DifferentialDrive drive;

  private XboxController controller;

  private DifferentialDriveOdometry odometry;

  private NetworkTable table;

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

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("target");
  }

  Translation2d getTranslation() {
    return odometry.getPoseMeters().getTranslation();
  }

  Translation2d getTargetTranslation(int targetNumber) {
    double x = table.getEntry("x").getDoubleArray(new double[]{0})[targetNumber];
    double y = table.getEntry("y").getDoubleArray(new double[]{0})[targetNumber];
    return new Translation2d(x, y);
  }

  int getTargetCount() {
    return (int)table.getEntry("count").getDouble(1);
  }

  double getAngle() {
    return normalizeAngle(gyro.getAngle());
  }

  double normalizeAngle(double angle) {
    angle = angle % 360;
    if (angle > 180) {
      return angle - 360;
    } else if (angle < -180) {
      return angle + 360;
    }
    return angle;
  }

  double getRotationBetweenPoints(Translation2d start, Translation2d end) {
    Translation2d delta = end.minus(start);
    Rotation2d rotation = new Rotation2d(delta.getX(), delta.getY());
    return normalizeAngle(rotation.getDegrees());
  }

  double getDistanceBetweenPoints(Translation2d start, Translation2d end) {
    return start.getDistance(end);
  }

  double getDistanceFromTarget(int targetNumber) {
    return getDistanceBetweenPoints(getTranslation(), getTargetTranslation(targetNumber));
  }

  void updateOdometry() {
    // TODO: Get the actual encoder readings and convert to feet
    int TICKS_PER_FOOT = 100;
    double angle = gyro.getAngle();
    double leftDistanceFeet = flMotor.getEncoder().getPosition() / TICKS_PER_FOOT;
    double rightDistanceFeet = frMotor.getEncoder().getPosition() / TICKS_PER_FOOT;
    odometry.update(Rotation2d.fromDegrees(angle), leftDistanceFeet, rightDistanceFeet);
  }

  void resetOdometry() {
    flMotor.getEncoder().setPosition(0);
    frMotor.getEncoder().setPosition(0);

    double angle = gyro.getAngle();
    odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(angle));
  }

  @Override
  public void autonomousInit() {
    gyro.reset();
    resetOdometry();
  }

  @Override
  public void autonomousPeriodic() {

    // set these values to change speed and turn rate of the robot
    double speed = 0.0;
    double turnRate = 0.0;

    drive.arcadeDrive(speed, turnRate);
  }

  @Override
  public void teleopInit() {
    gyro.reset();
    resetOdometry();
  }

  @Override
  public void teleopPeriodic() {

    double speed = controller.getY();
    double turnRate = controller.getX();

    drive.arcadeDrive(-speed * .3, turnRate * .5);
  }
}
