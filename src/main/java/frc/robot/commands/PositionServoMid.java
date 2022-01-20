// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ServoArm;
import java.util.function.Supplier;

public class PositionServoMid extends CommandBase {
  /** Creates a new PositionServo. */
  private ServoArm arm1;
  private final double m_pos;
  
  //Class constructor
  public PositionServoMid(ServoArm pArm) {
    // m_zaxisRotateSupplier = sup;
    m_pos = 0.5;
    arm1 = pArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm1);
  }
  
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    arm1.moveTo(m_pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
