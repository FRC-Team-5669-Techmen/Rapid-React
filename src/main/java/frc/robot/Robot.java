// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  Joystick controller = new Joystick(0);

  TalonFX m_leftMotor1 = new TalonFX(1);
  TalonFX m_leftMotor2 = new TalonFX(2);
  TalonFX m_rightMotor1 = new TalonFX(3);
  TalonFX m_rightMotor2 = new TalonFX(4);
  TalonFX m_winch = new TalonFX(6);

  double baseSpeed = 0.25; //base power for motors. AKA fastest a motor can go at any given time
  double mY = 0;          //RAW data from joystick Y axis
  double mR = 0;          //RAW data from joystick Rotational Axis
  double winchSpeed = 0.35; //base power for motors. AKA fastest a motor can go at any given time
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    mY = controller.getRawAxis(1); //assumes that forward is positive and backwards is negative
    mR = controller.getRawAxis(4); //assumes that rotating counter clockwise is negative and clockwise is positive

    boolean b7 = controller.getRawButton(7);  //up
    boolean b9 = controller.getRawButton(9);  //down

    double wM = winchSpeed * ((b7 ? 1 : 0) + (b9 ? -1 : 0));

    double lM = baseSpeed * (mY + mR);
    double rM = baseSpeed * (mY - mR) * -1;

    m_leftMotor1.set(TalonFXControlMode.PercentOutput, lM);
    m_leftMotor2.set(TalonFXControlMode.PercentOutput, lM);
    m_rightMotor1.set(TalonFXControlMode.PercentOutput, rM);
    m_rightMotor2.set(TalonFXControlMode.PercentOutput, rM);
    m_winch.set(TalonFXControlMode.PercentOutput, wM);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
