// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoRoutines;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorConstants;
import frc.robot.util.OCXboxController;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 1.5 rotations per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle orient = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Manipulator manipulator = new Manipulator();
  public final Elevator elevator = new Elevator();


  private OCXboxController driver = new OCXboxController(0);
  private OCXboxController operator = new OCXboxController(1);

  public final Superstructure superstructure = new Superstructure(drivetrain, logger, manipulator, elevator, driver);
  
  /* Path follower */
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();
  
  public RobotContainer() {
    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory);
    
    autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureDriverBindings(driver);
    configureOperatorBindings(driver);
    configureOperatorBindings(operator);
    setSwerveUpdateFrequency(drivetrain.getModule(0).getDriveMotor());
    setSwerveUpdateFrequency(drivetrain.getModule(0).getSteerMotor());
    setSwerveUpdateFrequency(drivetrain.getModule(1).getDriveMotor());
    setSwerveUpdateFrequency(drivetrain.getModule(1).getSteerMotor());
    setSwerveUpdateFrequency(drivetrain.getModule(2).getDriveMotor());
    setSwerveUpdateFrequency(drivetrain.getModule(2).getSteerMotor());
    setSwerveUpdateFrequency(drivetrain.getModule(3).getDriveMotor());
    setSwerveUpdateFrequency(drivetrain.getModule(3).getSteerMotor());
    ParentDevice.optimizeBusUtilizationForAll(drivetrain.getModule(0).getDriveMotor(), drivetrain.getModule(0).getSteerMotor(), drivetrain.getModule(1).getDriveMotor(), drivetrain.getModule(1).getSteerMotor(), drivetrain.getModule(2).getDriveMotor(), drivetrain.getModule(2).getSteerMotor(), drivetrain.getModule(3).getDriveMotor(), drivetrain.getModule(3).getSteerMotor());
    
    orient.HeadingController = new PhoenixPIDController(7, 0, 0.1);
  }
  
  public void setSwerveUpdateFrequency(CommonTalon motor) {
    motor.getDutyCycle().setUpdateFrequency(100);
    motor.getMotorVoltage().setUpdateFrequency(100);
    motor.getPosition().setUpdateFrequency(100);
    motor.getVelocity().setUpdateFrequency(50);
    motor.getStatorCurrent().setUpdateFrequency(50);
  }
  
  private void configureDriverBindings(OCXboxController controller) {
        //DRIVE COMMAND
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                var speeds = controller.getSpeeds(MaxSpeed, MaxAngularRate);
                return drive.withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond);
            })
        );

        // controller.x().whileTrue(drivetrain.applyRequest(() -> brake));
        // controller.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        // ));

        // reset the robot heading to forward
        controller.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //TODO: ADD GYRO BUTTONS FOR AUTO ALIGN
        controller.povUp().whileTrue(drivetrain.applyRequest(()->orient.withTargetDirection(Rotation2d.kZero)));
        controller.povLeft().whileTrue(drivetrain.applyRequest(()->orient.withTargetDirection(Rotation2d.kCW_90deg)));
        controller.povRight().whileTrue(drivetrain.applyRequest(()->orient.withTargetDirection(Rotation2d.kCCW_90deg)));
        controller.povDown().whileTrue(drivetrain.applyRequest(()->orient.withTargetDirection(Rotation2d.k180deg)));

        controller.leftBumper().whileTrue(superstructure.driveToScorePoint());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureOperatorBindings(OCXboxController controller) {
    //===== ELEVATOR COMMANDS
    controller.a().onTrue(elevator.setL1C());
    controller.x().onTrue(elevator.setL2C());
    controller.y().onTrue(elevator.setL3C());
    controller.b().onTrue(elevator.setL4C());
    //=====

    //===== CORAL MANIPULATOR
    manipulator.setDefaultCommand(manipulator.setVoltageC(0).withName("D:HoldCoral"));
    // Automatically feed coral to a consistent position when detected
    manipulator.isCoralDetected().and(()->manipulator.getCurrentCommand() == manipulator.getDefaultCommand())
      .onTrue(manipulator.feedCoralC());
    controller.rightTrigger().whileTrue(manipulator.setVoltageOutC());
    controller.leftTrigger().whileTrue(manipulator.setVoltageInC());
    //=====

  }

  public Command getAutonomousCommand() {
    /* Run the routine selected from the auto chooser */
    return autoChooser.selectedCommand();
  }

  public void periodic() {
    superstructure.periodic();
  }
}
