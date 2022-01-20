// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private PWM m_pwm1;
  /** Creates a new Intake. */
  public Intake(int pPort) {
    m_pwm1 = new PWM(pPort);

  }

public void pwmMoveTo(double position) {
  m_pwm1.setPosition(position);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
