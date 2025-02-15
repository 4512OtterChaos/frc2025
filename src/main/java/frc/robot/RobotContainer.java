// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
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

  public final Superstructure superstructure = new Superstructure(drivetrain, manipulator, elevator, driver);


  public RobotContainer() {
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
    // ParentDevice.optimizeBusUtilizationForAll(drivetrain.getModule(0).getDriveMotor(), drivetrain.getModule(0).getSteerMotor(), drivetrain.getModule(1).getDriveMotor(), drivetrain.getModule(1).getSteerMotor(), drivetrain.getModule(2).getDriveMotor(), drivetrain.getModule(2).getSteerMotor(), drivetrain.getModule(3).getDriveMotor(), drivetrain.getModule(3).getSteerMotor());
    
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

        controller.x().whileTrue(drivetrain.applyRequest(() -> brake));
        controller.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
        ));

        // reset the robot heading to forward
        controller.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //TODO: ADD GYRO BUTTONS FOR AUTO ALIGN
        controller.povUp().whileTrue(drivetrain.applyRequest(()->orient.withTargetDirection(Rotation2d.kZero)));
        controller.povLeft().whileTrue(drivetrain.applyRequest(()->orient.withTargetDirection(Rotation2d.kCW_90deg)));
        controller.povRight().whileTrue(drivetrain.applyRequest(()->orient.withTargetDirection(Rotation2d.kCCW_90deg)));
        controller.povDown().whileTrue(drivetrain.applyRequest(()->orient.withTargetDirection(Rotation2d.k180deg)));
  }

  private void configureOperatorBindings(OCXboxController controller) {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {
    superstructure.periodic();
  }
}
