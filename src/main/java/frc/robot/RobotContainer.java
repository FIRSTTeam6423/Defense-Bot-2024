// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Drive.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // private static final VisionUtil visionUtil = new VisionUtil();
  private static final Drive drive = new Drive();

  private static XboxController driver = new XboxController(0);
  private static CommandXboxController driverCommandController = new CommandXboxController(0);

  private SendableChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureDefaultCommands();
    configureBindings();
    configureDefaultCommands();
    drive.configureAutos();
    registerAutoCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  public void registerAutoCommands() {
    NamedCommands.registerCommand("Intake",
        fakeStartIntake().alongWith(new WaitCommand(3)).andThen(fakeRetractIntake()));
    NamedCommands.registerCommand("ShooterRoll", fakeShooterSpinup().withTimeout(1)
        .andThen(fakeIntakeFeed().withTimeout(1)).andThen(fakeShooterStopRollers().alongWith(fakeIntakeStopRollers())));
  }

  public Command fakeStartIntake() {
    return Commands.runOnce(() -> System.out.println("Start Intake!"));
  }

  public Command fakeRetractIntake() {
    return Commands.runOnce(() -> System.out.println("Retract Intake!"));
  }

  public Command fakeShooterSpinup() {
    return Commands.run(() -> System.out.println("Shooter Spinup!"));
  }

  public Command fakeIntakeFeed() {
    return Commands.run(() -> System.out.println("Intake Feed!"));
  }

  public Command fakeShooterStopRollers() {
    return Commands.runOnce(() -> System.out.println("Stop shooter rollers!"));
  }

  public Command fakeIntakeStopRollers() {
    return Commands.runOnce(() -> System.out.println("Stop Intake Rollers!"));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configureDefaultCommands() {
    drive.setDefaultCommand(drive.driveRobot(false));
    //drive.setDefaultCommand(drive.driveForward());
  }

  public static double getDriverLeftXboxY() {
    return driver.getLeftY();
  }

  public static double getDriverLeftXboxX() {
    return driver.getLeftX();
  }

  public static double getDriverRightXboxX() {
    return driver.getRightX();
  }

  public static double getDriverRightXboxY() {
    return driver.getRightY();
  }

  public static double getDriverLeftXboxTrigger() {
    return driver.getLeftTriggerAxis();
  }

  public static double getDriverRightXboxTrigger() {
    return driver.getRightTriggerAxis();
  }

  public static boolean getDriverLeftBumper() {
    return driver.getLeftBumper();
  }
}
