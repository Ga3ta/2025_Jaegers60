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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevador;
import frc.robot.subsystems.kitbot;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Joystick joystick2 = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final kitbot m_kitbot = new kitbot();
    public final elevador m_elevador = new elevador();
    

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);


        NamedCommands.registerCommand("PosicionaDejar", m_elevador.Pocisiona_Dejar());
        NamedCommands.registerCommand("PosicionaTomar", m_elevador.Pocisiona_Tomar());
        NamedCommands.registerCommand("DejarL4", m_elevador.Pocisiona_Dejar_L4());
        NamedCommands.registerCommand("Tomar", m_elevador.taker_toma());
        NamedCommands.registerCommand("Tomar_Inicial", m_elevador.taker_inicial());


        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
       // drivetrain.setDefaultCommand(
       //     // Drivetrain will execute this command periodically
       //     drivetrain.applyRequest(() ->
       //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
       //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
       //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
       //     )
       // );

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick2.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick2.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick2.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_elevador.setDefaultCommand(m_elevador.elevadorManual(joystick::getLeftY));
         // m_elevador.setDefaultCommand(m_elevador.Ver_Sensor());
       //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

       new Trigger(joystick2.button(2, Robot.m_loop)).whileTrue(drivetrain.applyRequest(() -> brake));

       new Trigger(joystick2.button(3, Robot.m_loop)).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick2.getY(), -joystick2.getX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.back().and(joystick.y()).onTrue(m_elevador.PruebaGiro());
        joystick.back().and(joystick.x()).onTrue(m_elevador.PruebaGiro_regresa());

        
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.x().onTrue(m_elevador.Pocisiona_Tomar());
        joystick.y().onTrue(m_elevador.Pocisiona_Dejar());
        
        //joystick.y().onTrue(m_elevador.elevaN1());

        joystick.button(6).onTrue(m_elevador.CambiaNivel());
        joystick.button(5).onTrue(m_elevador.CambiaNivelTomar());
        joystick.button(1).onTrue(m_elevador.taker_toma());
        joystick.button(2).onTrue(m_elevador.taker_deja());

        
      //  exampleTrigger.onTrue(m_elevador.taker_stop());
        drivetrain.registerTelemetry(logger::telemeterize);

    }


    
    public Command getAutonomousCommand() {

        return autoChooser.getSelected();

       // String autoSelect = Robot.autoChooser.getSelected();
       // switch (autoSelect) {

       // case "modo1":
       //     return Commands.sequence(// Drivetrain will execute this command periodically
       //         drivetrain.applyRequest2(() ->
       //                 drive.withVelocityX(0.3* MaxSpeed)// Drive forward with negative Y (forward)
       //                     .withVelocityY(0.3* MaxSpeed) // Drive left with negative X (left)
       //                     .withRotationalRate(0) // Drive counterclockwise with negative X (left)  
            
      //          ),
       //         drivetrain.mitiempo(3),
       //         drivetrain.applyRequest2(() ->
       //             drive.withVelocityX(0)// Drive forward with negative Y (forward)
       //                 .withVelocityY(0) // Drive left with negative X (left)
      //                  .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      //          ));
      //          
      //  case "modo2":
      //      return Commands.sequence(// Drivetrain will execute this command periodically
      //          drivetrain.applyRequest2(() ->
      //                  drive.withVelocityX(0.3* MaxSpeed)// Drive forward with negative Y (forward)
      //                      .withVelocityY(0.3* MaxSpeed) // Drive left with negative X (left)
      //                      .withRotationalRate(.3*MaxAngularRate) // Drive counterclockwise with negative X (left)  
      //      
      //          ),
      //          drivetrain.mitiempo(3),
      //          drivetrain.applyRequest2(() ->
      //              drive.withVelocityX(0)// Drive forward with negative Y (forward)
      //                  .withVelocityY(0) // Drive left with negative X (left)
      //                  .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      //          ));

      //  case "modo3":
//
                    //return Commands.print("No autonomous command configured");
  //          return Commands.sequence(// Drivetrain will execute this command periodically
    //                drivetrain.applyRequest2(() ->
    //                        drive.withVelocityX(0.5* MaxSpeed)// Drive forward with negative Y (forward)
    //                            .withVelocityY(0) // Drive left with negative X (left)
    //                            .withRotationalRate(0) // Drive counterclockwise with negative X (left)             
    //                 ),
    //                drivetrain.mitiempo(1),
    //                drivetrain.applyRequest2(() ->
    //                    drive.withVelocityX(0)// Drive forward with negative Y (forward)
    //                        .withVelocityY(0) // Drive left with negative X (left)
    //                        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
    //                ));

       
    //    case "modo_salida":

    //            return Commands.sequence(// Drivetrain will execute this command periodically
    //                drivetrain.applyRequest2(() ->
    //                    drive.withVelocityX(0.3* MaxSpeed)// Drive forward with negative Y (forward)
    //                        .withVelocityY(0) // Drive left with negative X (left)
    //                        .withRotationalRate(.5*MaxAngularRate) // Drive counterclockwise with negative X (left)  
     //                           
    //                ),
    //                drivetrain.mitiempo(5),
    //                drivetrain.applyRequest2(() ->
    //                    drive.withVelocityX(0)// Drive forward with negative Y (forward)
    //                        .withVelocityY(0) // Drive left with negative X (left)
     //                       .withRotationalRate(0) // Drive counterclockwise with negative X (left)
    //                ));
    //    default:
    //            return Commands.sequence(// Drivetrain will execute this command periodically
    //                drivetrain.applyRequest2(() ->
    //                    drive.withVelocityX(0.5* MaxSpeed)// Drive forward with negative Y (forward)
    //                        .withVelocityY(0) // Drive left with negative X (left)
    //                        .withRotationalRate(0) // Drive counterclockwise with negative X (left)                              
    //                ),
    //                drivetrain.mitiempo(2),
    //                drivetrain.applyRequest2(() ->
    //                    drive.withVelocityX(0)// Drive forward with negative Y (forward)
    //                        .withVelocityY(0) // Drive left with negative X (left)
    //                        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
    //                ));

    //    }
    }
}
