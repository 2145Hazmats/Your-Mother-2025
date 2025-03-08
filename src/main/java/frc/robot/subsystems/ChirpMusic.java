// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChirpMusic extends SubsystemBase {
  private Orchestra orchestra;

  /** Creates a new ChirpMusic. */
  public ChirpMusic(CommandSwerveDrivetrain m_drivetrain, ElevatorSubsystem m_ElevatorSubsystem) {
    // orchestra.loadMusic("music path");
    // orchestra.addInstrument(m_drivetrain.getModule(0).getDriveMotor());
    // orchestra.addInstrument(m_drivetrain.getModule(0).getSteerMotor());
    // orchestra.play();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
