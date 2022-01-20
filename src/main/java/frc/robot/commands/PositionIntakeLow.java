// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PositionIntakeLow extends CommandBase {
  /** Creates a new PositionIntakeLow. */
  private Intake intake1;
  private final double m_pos;

  public PositionIntakeLow(Intake pIntake) {
    m_pos = 0;
    intake1 = pIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake1.pwmMoveTo(m_pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
