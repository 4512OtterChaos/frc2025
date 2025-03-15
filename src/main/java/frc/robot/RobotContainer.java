// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonTalon;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.robot.subsystems.drivetrain.DriveConstants.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoRoutines;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OCXboxController;

public class RobotContainer {
        
    private final Telemetry logger = new Telemetry(kMaxLinearSpeed);
    
    public final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
    public final Manipulator manipulator = new Manipulator();
    public final Elevator elevator = new Elevator();
    
    private OCXboxController driver = new OCXboxController(0);
    // private OCXboxController operator = new OCXboxController(1);

    public final Superstructure superstructure = new Superstructure(swerve, manipulator, elevator);
    
    private final Vision vision = new Vision();

    private AddressableLED led = new AddressableLED(9);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(15);

    private LEDPattern layer1Pattern = LEDConstants.kPatternBlueScroll;
    private LEDPattern layer2Pattern = LEDPattern.kOff;
    private LEDPattern layer3Pattern = LEDPattern.kOff;
    
    
    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    
    public RobotContainer() {
        autoFactory = swerve.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, swerve, elevator, manipulator);
        
        autoChooser.addCmd("TaxiAuto", autoRoutines::taxiAuto);
        autoChooser.addCmd("TaxiFarAuto", autoRoutines::taxiFar);
        autoChooser.addCmd("Middle1CoralL1", autoRoutines::middle1CoralL1);
        // autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        configureDriverBindings(driver);
        configureOperatorBindings(driver);
        if (Robot.isSimulation()){
            simBindings(driver);
        }
        // configureOperatorBindings(operator);
        TalonFX[] swerveMotors = {
            swerve.getModule(0).getDriveMotor(),
            swerve.getModule(0).getSteerMotor(),
            swerve.getModule(1).getDriveMotor(),
            swerve.getModule(1).getSteerMotor(),
            swerve.getModule(2).getDriveMotor(),
            swerve.getModule(2).getSteerMotor(),
            swerve.getModule(3).getDriveMotor(),
            swerve.getModule(3).getSteerMotor()
        };
        setSwerveUpdateFrequency(swerveMotors);
        ParentDevice.optimizeBusUtilizationForAll(swerveMotors);
        
        bindLEDAnimations();
        led.setLength(15);
        led.start();
    }

    public void periodic() {
        superstructure.periodic();

        vision.getEstimatedGlobalPose().ifPresent(estimate -> {
            var pose = estimate.estimatedPose.toPose2d();
            var stdDevs = vision.getEstimationStdDevs(pose);
            swerve.addVisionMeasurement(pose, estimate.timestampSeconds, stdDevs);
        });

        // LED patterns
        if (!layer3Pattern.equals(LEDPattern.kOff)) {
            layer3Pattern.applyTo(ledBuffer);
        }
        else if (!layer2Pattern.equals(LEDPattern.kOff)) {
            layer2Pattern.applyTo(ledBuffer);
        }
        else {
            layer1Pattern.applyTo(ledBuffer);
        }
        led.setData(ledBuffer);
    }
    
    public void setSwerveUpdateFrequency(CommonTalon... motors) {
        for (var motor : motors) {
            motor.getDutyCycle().setUpdateFrequency(100);
            motor.getMotorVoltage().setUpdateFrequency(100);
            motor.getPosition().setUpdateFrequency(100);
            motor.getVelocity().setUpdateFrequency(50);
            motor.getStatorCurrent().setUpdateFrequency(50);
        }
    }

    private void configureDriverBindings(OCXboxController controller) {
        //DRIVE COMMAND
        swerve.setDefaultCommand(swerve.drive(() -> controller.getSpeeds(
            swerve.driveSpeed.in(MetersPerSecond),
            swerve.turnSpeed.in(RadiansPerSecond))
        ).withName("D:Controller Drive"));
        
        // reset the robot heading to forward
        controller.start().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));

        controller.leftTrigger().whileTrue(superstructure.autoAlignLeft());
        controller.rightTrigger().whileTrue(superstructure.autoAlignRight());
        
        // controller.leftTrigger().whileTrue(swerve.drive(() -> new ChassisSpeeds(0, controller.getLeftTriggerAxis() * 0.3, 0), false, true).withName("Strafe Left"));
        // controller.rightTrigger().whileTrue(swerve.drive(() -> new ChassisSpeeds(0, controller.getRightTriggerAxis() * -0.3, 0), false, true).withName("Strafe Right"));       
        
        swerve.registerTelemetry(logger::telemeterize);
    }
    
    private void configureOperatorBindings(OCXboxController controller) {
        //===== ELEVATOR COMMANDS
        controller.povDown().onTrue(elevator.setMinC());
        controller.a().onTrue(elevator.setL1C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));
        controller.x().onTrue(elevator.setL2C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));
        controller.y().onTrue(elevator.setL3C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));
        controller.b().onTrue(elevator.setL4C()
            .beforeStarting(waitUntil(manipulator.isCoralDetected().negate())));

        swerve.isAligning().and(controller.a()).onTrue(sequence(
            Commands.waitUntil(swerve.isAligned()),
            elevator.setL1C()
                .beforeStarting(waitUntil(manipulator.isCoralDetected().negate()))));
        swerve.isAligning().and(controller.x()).onTrue(sequence(
            Commands.waitUntil(swerve.isAligned()),
            elevator.setL2C()
                .beforeStarting(waitUntil(manipulator.isCoralDetected().negate()))));
        swerve.isAligning().and(controller.y()).onTrue(sequence(
            Commands.waitUntil(swerve.isAligned()),
            elevator.setL3C()
                .beforeStarting(waitUntil(manipulator.isCoralDetected().negate()))));
        swerve.isAligning().and(controller.b()).onTrue(sequence(
            Commands.waitUntil(swerve.isAligned()),
            elevator.setL4C()
                .beforeStarting(waitUntil(manipulator.isCoralDetected().negate()))));
        //=====
        
        //===== CORAL MANIPULATOR
        manipulator.setDefaultCommand(manipulator.holdPositionC());
        // Automatically feed coral to a consistent position when detected
        manipulator.isCoralDetected().and(()->manipulator.getCurrentCommand() == manipulator.getDefaultCommand())
            .onTrue(manipulator.feedCoralSequenceC());

        controller.leftBumper().whileTrue(manipulator.scoreAlgaeC());
        controller.rightBumper().whileTrue(manipulator.scoreCoralC());
        //=====

        //===== COMPOSITION CCOMMANDS
        controller.back().whileTrue(superstructure.algaeShoot());
        //=====
    }

    private void simBindings(OCXboxController controller) {
        controller.start().onTrue(runOnce(()->swerve.disturbeSimPose()));
        
    }

    private void bindLEDAnimations() {
        manipulator.isCoralDetected().whileTrue(sequence(
            run(()->layer2Pattern = LEDConstants.kPatternGreenBlink).withTimeout(0.5),
            run(()->layer2Pattern = LEDConstants.kPatternGreenSolid)
        ).finallyDo(()->layer2Pattern = LEDPattern.kOff));

        swerve.isAligning().whileTrue(startEnd(
            ()->layer3Pattern = LEDConstants.kPatternOrangeBlink,
            ()->layer3Pattern = LEDPattern.kOff
        ));

        swerve.isAligned().onTrue(startEnd(
            ()->layer3Pattern = LEDConstants.kPatternPurpleSolid,
            ()->layer3Pattern = LEDPattern.kOff
        ).withTimeout(1));
    }
    
    // private void configureDriverBindings(OCXboxController controller) {
    //     //DRIVE COMMAND
    //     swerve.setDefaultCommand(swerve.drive(() -> controller.getSpeeds(
    //         swerve.driveSpeed.in(MetersPerSecond),
    //         swerve.turnSpeed.in(RadiansPerSecond))
    //     ).withName("D:Controller Drive"));
        
    //     // controller.x().whileTrue(drivetrain.applyRequest(() -> brake));
    //     // controller.b().whileTrue(drivetrain.applyRequest(() ->
    //     //     point.withModuleDirection(new Rotation2d(-controller.getLeftY(), -controller.getLeftX()))
    //     // ));
        
    //     // reset the robot heading to forward
    //     controller.start().onTrue(swerve.runOnce(() -> swerve.seedFieldCentric()));
        
    //     //TODO: ADD GYRO BUTTONS FOR AUTO ALIGN
    //     // controller.povUp().whileTrue(swerve.applyRequest(()->orient.withTargetDirection(Rotation2d.kZero)).withName("Face Forward"));
    //     // controller.povLeft().whileTrue(swerve.applyRequest(()->orient.withTargetDirection(Rotation2d.fromDegrees(-45))).withName("Align Left Station"));
    //     // controller.povRight().whileTrue(swerve.applyRequest(()->orient.withTargetDirection(Rotation2d.fromDegrees(45))).withName("Align Right Station"));
        
    //     // controller.leftBumper().whileTrue(drivetrain.applyRequest(()->robotCentric.withVelocityY(MetersPerSecond.of(0.3))));
    //     // controller.rightBumper().whileTrue(drivetrain.applyRequest(()->robotCentric.withVelocityY(MetersPerSecond.of(-0.3))));

    //     controller.leftTrigger().whileTrue(swerve.drive(() -> new ChassisSpeeds(0, controller.getLeftTriggerAxis() * 0.3, 0), false, true).withName("Strafe Left"));
    //     controller.rightTrigger().whileTrue(swerve.drive(() -> new ChassisSpeeds(0, controller.getRightTriggerAxis() * -0.3, 0), false, true).withName("Strafe Right"));
        
    //     controller.back().whileTrue(superstructure.autoAlignToReef());
        
    //     // Run SysId routines when holding back/start and X/Y.
    //     // Note that each routine should be run exactly once in a single log.
    //     // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //     // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //     // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //     // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        
        
    //     swerve.registerTelemetry(logger::telemeterize);
    // }
    
    // private void configureOperatorBindings(OCXboxController controller) {
    //     //===== ELEVATOR COMMANDS
    //     controller.povDown().onTrue(elevator.setMinC());
    //     // controller.back().whileTrue(elevator.setVoltageC(2.5)).onFalse(elevator.setVoltageC(0));
    //     controller.a().onTrue(elevator.setL1C());
    //     controller.x().onTrue(
    //       elevator.setL2C().deadlineFor(
    //       sequence(
    //         manipulator.setVoltageC(-0.5).withTimeout(0.5),
    //         waitSeconds(0.5)
    //       )
    //     )
    //     );
    //     controller.y().onTrue(
    //       elevator.setL3C()
    //       );
    //     controller.b().onTrue(
    //       elevator.setL4C()
    //       );
    //     //=====
        
    //     //===== CORAL MANIPULATOR
    //     manipulator.setDefaultCommand(manipulator.holdCoralC());
    //     // Automatically feed coral to a consistent position when detected
    //     manipulator.isCoralDetected().and(()->manipulator.getCurrentCommand() == manipulator.getDefaultCommand())
    //     .onTrue(manipulator.feedCoralC());
    //     controller.rightBumper().whileTrue(manipulator.setVoltageOutC());
    //     controller.leftBumper().whileTrue(manipulator.setVoltageInC());
    //     //=====
        
    // }
    
    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        return autoChooser.selectedCommand();
    }

    public void simulationPeriodic() {
        vision.simulationPeriodic(swerve.getState().Pose);
    }
}
