// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BoxConstants;

public class ShooterBoxx extends SubsystemBase {
  /** Creates a new ShooterBoxx. */
  private SparkMax Shootermotor = new SparkMax(Constants.shooterBoxxContants.ShooterMotorId, MotorType.kBrushless);
  private SparkMaxConfig shooterConfig = new SparkMaxConfig();
  public ShooterBoxx() {
shooterConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    //Enable voltage compensation
    .voltageCompensation(BoxConstants.kShooterMotorNominalVoltage);
shooterConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
shooterConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
  Shootermotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

public Command RunShooter(double CustomSpeed ) {
return Commands.runOnce(() -> {
Shootermotor.set(CustomSpeed);
} );
}
public Command StopShooterMotor() {
return Commands.runOnce(, null)


}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
