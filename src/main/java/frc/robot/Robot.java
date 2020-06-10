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

public class Robot extends TimedRobot {
  private CANSparkMax flMotor;
  private CANSparkMax frMotor;
  private CANSparkMax blMotor;
  private CANSparkMax brMotor;

  private DifferentialDrive drive;

  private XboxController controller;


  @Override
  public void robotInit() {

    flMotor = new CANSparkMax(1, MotorType.kBrushless);
    frMotor = new CANSparkMax(2, MotorType.kBrushless);
    blMotor = new CANSparkMax(3, MotorType.kBrushless);
    brMotor = new CANSparkMax(4, MotorType.kBrushless);

    SpeedControllerGroup left = new SpeedControllerGroup(flMotor, blMotor);
    SpeedControllerGroup right = new SpeedControllerGroup(frMotor, brMotor);

    drive = new DifferentialDrive(left, right);

    controller = new XboxController(0);
  }

  @Override
  public void autonomousInit() {
   
  }

  
  @Override
  public void autonomousPeriodic() {
    
  }

  
  @Override
  public void teleopInit() {
  }

  
  @Override
  public void teleopPeriodic() {
    double speed = controller.getY();
    double turnRate = controller.getX();

    drive.arcadeDrive(-speed * .25, turnRate * .25);
  }
}
