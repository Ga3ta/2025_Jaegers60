// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevadorSubsystem;
import frc.robot.subsystems.ElevadorSubsystem.PosicionBrazo;
import frc.robot.subsystems.ElevadorSubsystem.PosicionElevador;
import frc.robot.subsystems.ElevadorSubsystem.PosicionGiro;
import frc.robot.subsystems.kitbot;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final Joystick joystick2 = new Joystick(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final kitbot m_kitbot = new kitbot();
  public final ElevadorSubsystem m_elevador = new ElevadorSubsystem();

  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // drivetrain.setDefaultCommand(
    //     // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() ->
    //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y
    // (forward)
    //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X
    // (left)
    //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
    // counterclockwise with negative X (left)
    //     )
    // );

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick2.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick2.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick2.getZ()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // m_elevador.setDefaultCommand(m_elevador.Ver_Sensor());
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    m_elevador.setDefaultCommand(
        m_elevador.controlLoopCommand(
            joystick::getLeftY,
            joystick::getRightY,
            joystick::getRightX,
            () -> joystick.getLeftTriggerAxis() > 0.1,
            () -> joystick.getRightTriggerAxis() > 0.1));

    new Trigger(joystick2.button(2, Robot.m_loop)).whileTrue(drivetrain.applyRequest(() -> brake));

    joystick.leftBumper().onTrue(m_elevador.mandarElevadorAPosicion(PosicionElevador.L4));

    new Trigger(joystick2.button(3, Robot.m_loop))
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(-joystick2.getY(), -joystick2.getX()))));

    //  exampleTrigger.onTrue(m_elevador.taker_stop());
    drivetrain.registerTelemetry(logger::telemeterize);

    NamedCommands.registerCommand(
        "L1",
        m_elevador.mandarSistemaAPosiciones(
            PosicionElevador.L1, PosicionBrazo.DejarL1, PosicionGiro.Vertical));
  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}
