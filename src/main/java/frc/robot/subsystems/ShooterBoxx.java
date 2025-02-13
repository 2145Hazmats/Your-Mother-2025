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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterBoxxContants;

public class ShooterBoxx extends SubsystemBase {
  /** Creates a new ShooterBoxx. */
  private SparkMax shooterMotor = new SparkMax(Constants.shooterBoxxContants.ShooterMotorId, MotorType.kBrushless);
  private SparkMaxConfig shooterConfig = new SparkMaxConfig();

  // Sensor
  private DigitalInput CoralSensor = new DigitalInput(shooterBoxxContants.kCoralSensorChannel);

  public ShooterBoxx() {

    shooterConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    //Enable voltage compensation
    .voltageCompensation(Constants.shooterBoxxContants.kShooterMotorNominalVoltageConstant);
shooterConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
shooterConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
  shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

public Command RunShooter(double CustomSpeed ) {
return Commands.runOnce(() -> {
shooterMotor.set(CustomSpeed);
} );
}
public Command StopShooterMotor() {
return Commands.runOnce(() -> {
  shooterMotor.set(0);
}, this);
}

public Command SuckTillSensor() {
return Commands.runOnce(() -> {
  shooterMotor.set(-.2);
}).until(() -> CoralSensorTriggered());


}
public boolean ShooterBoxSensorTrue() { 
  // Query some boolean state, such as a digital sensor.

  return false;
}


public boolean CoralSensorTriggered() {
  return !CoralSensor.get();
}


public boolean noteSensorUntriggered() {
  return CoralSensor.get();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral Sensor", CoralSensorTriggered());
  }
}
