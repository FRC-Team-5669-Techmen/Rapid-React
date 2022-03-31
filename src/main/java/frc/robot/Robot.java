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
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  String gameData;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  Joystick controller = new Joystick(0);

  Spark blinkin = new Spark(0);

  TalonFX m_leftMotor1 = new TalonFX(1);
  TalonFX m_leftMotor2 = new TalonFX(2);
  TalonFX m_rightMotor1 = new TalonFX(3);
  TalonFX m_rightMotor2 = new TalonFX(4);
  TalonFX m_tail = new TalonFX(5);
  TalonFX m_winch = new TalonFX(6);

  double baseSpeed = 0.25; //base power for motors. AKA fastest a motor can go at any given time
  double mY = 0;          //RAW data from joystick Y axis
  double mR = 0;          //RAW data from joystick Rotational Axis
  double winchSpeed = 0.35; //base power for winch. AKA fastest the winch can go at any given time
  double tailSpeed = 0.35; //base power for tail. AKA fastest the tail can go at any given time

  boolean winchDebounced = false; //if false, let winch input register, if true dont let input register
  double interval = 0; //counts up to set limit, and when reach that set limit allow winchdebounced to become false again
  double limit = 3000; //limit for interval
  double wM = 0;

  boolean b7previous = false;
  boolean b9previous = true;
  
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
  public void autonomousPeriodic() {
    
  }

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
    gameData = DriverStation.getGameSpecificMessage();
    mY = controller.getRawAxis(1); //assumes that forward is positive and backwards is negative
    mR = controller.getRawAxis(4); //assumes that rotating counter clockwise is negative and clockwise is positive

    boolean b7 = controller.getRawButton(7);  //up
    boolean b9 = controller.getRawButton(9);  //down
    boolean b3 = controller.getRawButton(3);  //up
    boolean b5 = controller.getRawButton(5);  //down

    if(b7previous != b7) {  //b7 input changed
      if(winchDebounced == false) {
        winchDebounced = true;
        wM = winchSpeed * 1;
      }else if(b7 == false) {
        wM = 0;
      }
    }
    if(b9previous != b9) {  //b9 input changed
      if(winchDebounced == false) {
        winchDebounced = true;
        wM = winchSpeed * -1;
      }else if(b9 == false) {
        wM = 0;
      }
    }

    b7previous = b7;
    b9previous = b9;

    double tM = tailSpeed * ((b3 ? -1 : 0) + (b5 ? 1 : 0));
    double lM = baseSpeed * (-mY + mR);
    double rM = baseSpeed * (-mY - mR) * -1;



    m_leftMotor1.set(TalonFXControlMode.PercentOutput, lM);
    m_leftMotor2.set(TalonFXControlMode.PercentOutput, lM);
    m_rightMotor1.set(TalonFXControlMode.PercentOutput, rM);
    m_rightMotor2.set(TalonFXControlMode.PercentOutput, rM);
    m_winch.set(TalonFXControlMode.PercentOutput, wM);
    m_tail.set(TalonFXControlMode.PercentOutput, tM);



    interval += 1;
    if (interval > limit) {
      interval = 0;
      winchDebounced = false;
    }

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
