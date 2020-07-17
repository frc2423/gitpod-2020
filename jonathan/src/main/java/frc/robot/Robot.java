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

  private String state;

  private Translation2d targetPos;

  private Translation2d currentPos;

  private double targetAngle;

  private int count = 0;

  private double previousDistance;



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

  double getAngleDelta(double start, double end) {
    start = normalizeAngle(start);
    end = normalizeAngle(end);

    double delta = end - start;

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

  double getRotationBetweenPoints(Translation2d start, Translation2d end) {
    Translation2d delta = end.minus(start);
    Rotation2d rotation = new Rotation2d(delta.getX(), delta.getY());
    return normalizeAngle(rotation.getDegrees());
  }

  double getRotationFromTarget(int targetNumber) {
    return getRotationBetweenPoints(getTranslation(), getTargetTranslation(targetNumber));
  }

  double getDistanceBetweenPoints(Translation2d start, Translation2d end) {
    return start.getDistance(end);
  }

  double getDistanceFromTarget(int targetNumber) {
    return getDistanceBetweenPoints(getTranslation(), getTargetTranslation(targetNumber));
  }

  void updateOdometry() {
    double LEFT_TICKS_PER_REV = 14.857;
    double RIGHT_TICKS_PER_REV = -12.786;
    double FEET_PER_REV = .666667 * Math.PI;
    double LEFT_TICKS_PER_FOOT = LEFT_TICKS_PER_REV / FEET_PER_REV;
    double RIGHT_TICKS_PER_FOOT = RIGHT_TICKS_PER_REV / FEET_PER_REV;
    double angle = gyro.getAngle();
    double leftDistanceFeet = flMotor.getEncoder().getPosition() / LEFT_TICKS_PER_FOOT;
    double rightDistanceFeet = frMotor.getEncoder().getPosition() / RIGHT_TICKS_PER_FOOT;
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

    //  targetPos = getTargetTranslation(0);

    //  currentPos = getTranslation();

    //  targetAngle = getRotationFromTarget(0);

    state = "MOVEUNTIL";
    previousDistance = getDistanceFromTarget(0) + 1;
  }

  @Override
  public void autonomousPeriodic() {

    // set these values to change speed and turn rate of the robot
    updateOdometry();

    double speed = 0;
    double turnRate = 0;
    double startAngle;

    double distance = getDistanceFromTarget(0);

    // int targetCount = getTargetCount();

    // targetPos = getTargetTranslation(count);

    // currentPos = getTranslation();

    // targetAngle = getRotationFromTarget(count);

    if (state == "MOVEUNTIL") {
        speed = 0.3;
        turnRate = 0.0;
        
        if (previousDistance > distance) {
            state = "TURNALITTLE";
            startAngle = getAngle();
        }
        previousDistance = distance;
    }

    else if (state == "TURNALITTLE") {
        speed = 0.0;
        turnRate = 0.3;

        if (startAngle - getAngle() > 18){
            state = "MOVEUNTIL";
        }
    }

    // if (state == "TURN") {

    //     if (getAngleDelta(getAngle(), targetAngle) > 0) {
    //      speed = 0.0;
    //      turnRate = 0.3;

    //      System.out.println("TURN");
    //      System.out.println(getAngleDelta(getAngle(), targetAngle));
	// 		if (getAngleDelta(getAngle(), targetAngle) < 5) {
    //             state = "MOVE";
    //         }
    //     } else {
    //      speed = 0.0;
    //      turnRate = -0.3;

    //      System.out.println("TURN");
    //      System.out.println(getAngleDelta(getAngle(), targetAngle));
	// 		if (getAngleDelta(getAngle(), targetAngle) > -5) {
    //             state = "MOVE";
    //         }
    //     }
    // }

    // else if (state == "MOVE") {
    //      speed = 0.3;
    //      turnRate = 0.0;  
    //         if (getDistanceFromTarget(count) < .1) {
    //             state = "NEXT";
    //         }
    // }

    // else if (state == "STOP") {
    //      speed = 0.0;
    //      turnRate = 0.0;
    // }

    // else if (state == "NEXT") {
    //      speed = 0.0;
    //      turnRate = 0.0;


    //     if (count == targetCount - 1) {
    //         state = "STOP";
    //     } else {
    //         count += 1;
    //         state = "TURN";
    //     }
    // }
 
    System.out.println("STATE: " + state);


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

    double speed = controller.getY();
    double turnRate = controller.getX();

    drive.arcadeDrive(-speed * .3, turnRate * .5);
  }
}
