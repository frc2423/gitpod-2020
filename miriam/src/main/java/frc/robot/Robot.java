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
    final SpeedControllerGroup left = new SpeedControllerGroup(flMotor, blMotor);
    final SpeedControllerGroup right = new SpeedControllerGroup(frMotor, brMotor);

    // use the drive object for easily controlling all the motors with arcade or
    // tank drive
    drive = new DifferentialDrive(left, right);

    // use this to get angle readings from the navx's gyro sensor
    gyro = new AHRS(SPI.Port.kMXP);

    controller = new XboxController(0);

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("target");
}

Translation2d getTranslation() {
    return odometry.getPoseMeters().getTranslation();
}

Translation2d getTargetTranslation(final int targetNumber) {
    final double x = table.getEntry("x").getDoubleArray(new double[] { 0 })[targetNumber];
    final double y = table.getEntry("y").getDoubleArray(new double[] { 0 })[targetNumber];
    return new Translation2d(x, y);
}

int getTargetCount() {
    return (int) table.getEntry("count").getDouble(1);
}

double getAngle() {
    return normalizeAngle(gyro.getAngle());
}

double getAngleDelta(double start, double end) {
    start = normalizeAngle(start);
    end = normalizeAngle(end);

    final double delta = end - start;

    if (delta < -180) {
        return delta + 360;
    } else if (delta > 180) {
        return delta - 360;
    }

    return delta;
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

double getRotationBetweenPoints(final Translation2d start, final Translation2d end) {
    final Translation2d delta = end.minus(start);
    final Rotation2d rotation = new Rotation2d(delta.getX(), delta.getY());
    return normalizeAngle(rotation.getDegrees());
}

double getRotationFromTarget(final int targetNumber) {
    return getRotationBetweenPoints(getTranslation(), getTargetTranslation(targetNumber));
}

double getDistanceBetweenPoints(final Translation2d start, final Translation2d end) {
    return start.getDistance(end);
}

double getDistanceFromTarget(final int targetNumber) {
    return getDistanceBetweenPoints(getTranslation(), getTargetTranslation(targetNumber));
}

void updateOdometry() {
    final double LEFT_TICKS_PER_REV = 14.857;
    final double RIGHT_TICKS_PER_REV = -12.786;
    final double FEET_PER_REV = .666667 * Math.PI;
    final double LEFT_TICKS_PER_FOOT = LEFT_TICKS_PER_REV / FEET_PER_REV;
    final double RIGHT_TICKS_PER_FOOT = RIGHT_TICKS_PER_REV / FEET_PER_REV;
    final double angle = gyro.getAngle();
    final double leftDistanceFeet = flMotor.getEncoder().getPosition() / LEFT_TICKS_PER_FOOT;
    final double rightDistanceFeet = frMotor.getEncoder().getPosition() / RIGHT_TICKS_PER_FOOT;
    odometry.update(Rotation2d.fromDegrees(angle), leftDistanceFeet, rightDistanceFeet);
}

void resetOdometry() {
    flMotor.getEncoder().setPosition(0);
    frMotor.getEncoder().setPosition(0);

    final double angle = gyro.getAngle();
    odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(angle));
}

@Override
public void autonomousInit() {
    gyro.reset();
    resetOdometry();
}

@Override
  public void autonomousPeriodic() {

    // Steps:
    // 1. Face the point you want to move to
    // 2. Drive toward that point
    // 3. Stop when you get close

    // gets you the current location of the robot
    final Translation2d currentLocation = getTranslation();

    // get x and y coordinates
    final double currentX = currentLocation.getX();
    final double currentY = currentLocation.getY();

    // gets you the point you want to move to
    final Translation2d targetLocation = getTargetTranslation(0);

    // get angle you want to face to get to target
    final double desiredAngle = getRotationFromTarget(0);

    // The amount you need to turn the robot to face the desired angle
    final double angleDelta = getAngleDelta(getAngle(), desiredAngle);

    // The distance from your target
    final double distance = getDistanceFromTarget(0);
    

    // set these values to change speed and turn rate of the robot
    double speed = 0.0;
    double turnRate = 0.0;

    if (state == "turn") {
        turnRate = 0.3;
        speed = 0.0;
    }

    if (getAngleDelta(getAngle(), desiredAngle) < 5.0 ){
        turnRate = 0.0;
        speed = 0.6;
    }

    if getDistanceFromTarget(0) == 0.0 {
        speed = 0.0;
    }



    drive.arcadeDrive(speed, turnRate);
  }

@Override
public void teleopInit() {
    gyro.reset();
    resetOdometry();
}

@Override
public void teleopPeriodic() {
    updateOdometry();

    final double speed = controller.getY();
    final double turnRate = controller.getX();

    drive.arcadeDrive(-speed * .3, turnRate * .5);
  }
}
