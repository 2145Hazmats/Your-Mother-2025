// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.subsystems.CANdleSubsystem;
//import frc.robot.subsystems.CANdleSubsystem.AnimationTypes;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    //m_robotContainer.getSwerveDrivetrain().configNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.getSwerveDrivetrain().configNeutralMode(NeutralModeValue.Brake);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.SetupCamerasForColor();

    //CANdleSubsystem.changeAnimation(AnimationTypes.ColorFlow);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    //m_robotContainer.getSwerveDrivetrain().configNeutralMode(NeutralModeValue.Coast);
    //CANdleSubsystem.setColors();
  }

  @Override
  public void teleopInit() {
    //m_robotContainer.getShooterBoxx().StopShooterMethod();
    m_robotContainer.getSwerveDrivetrain().configNeutralMode(NeutralModeValue.Brake);
    m_robotContainer.setFirstSpot(); //Experimental stuff
    m_robotContainer.SetupCamerasForColor();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
