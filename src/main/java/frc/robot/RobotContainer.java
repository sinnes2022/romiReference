// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.PositionIntakeHigh;
import frc.robot.commands.PositionIntakeMid;
import frc.robot.commands.PositionIntakeLow;
import frc.robot.commands.PositionServoHigh;
import frc.robot.commands.PositionServoLow;
import frc.robot.commands.PositionServoMid;
import frc.robot.ProportionControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import frc.robot.subsystems.ServoArm;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  private final ServoArm m_servoArm;
  private final Intake m_intake;


  // Assumes a gamepad plugged into channnel 0
  private final Joystick m_controller = new Joystick(0);
  Button m_buttonUp = new JoystickButton(m_controller, 1);
  Button m_buttonDown = new JoystickButton(m_controller, 2);
  Button m_buttonMid = new JoystickButton(m_controller, 3);
  Button m_buttonIntakeHigh = new JoystickButton(m_controller, 4);
  Button m_buttonIntakeLow = new JoystickButton(m_controller, 5);
  Button m_buttonIntakeMid = new JoystickButton(m_controller, 6);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Added for Proportional Control of each axis
  private ProportionControl m_axisProp0 = new ProportionControl();
  private ProportionControl m_axisProp1 = new ProportionControl();


  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_servoArm = new ServoArm(3);

    m_intake = new Intake(2);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // m_servoArm.setDefaultCommand(getServoArmCommand());
    // Example of how to use the onboard IO
    // Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    // onboardButtonA
    //     .whenActive(new PrintCommand("Button A Pressed"))
    //     .whenInactive(new PrintCommand("Button A Released"));
    m_buttonUp.whenPressed(new PositionServoHigh(m_servoArm));
    
    m_buttonDown.whenPressed(new PositionServoLow(m_servoArm));

    m_buttonMid.whenPressed(new PositionServoMid(m_servoArm));

    m_buttonIntakeHigh.whenPressed(new PositionIntakeHigh(m_intake));

    m_buttonIntakeMid.whenPressed(new PositionIntakeMid(m_intake));

    m_buttonIntakeLow.whenPressed(new PositionIntakeLow(m_intake));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  // public Command getServoArmCommand() {
  //   double p;
  //   p = m_controller.getRawAxis(5);

  //   return new PositionServoHigh(m_servoArm, () -> m_controller.getRawAxis(5) );
  // }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> m_axisProp0.prop(-m_controller.getRawAxis(1)), () -> m_axisProp1.prop((m_controller.getRawAxis(4))));
  }
}
