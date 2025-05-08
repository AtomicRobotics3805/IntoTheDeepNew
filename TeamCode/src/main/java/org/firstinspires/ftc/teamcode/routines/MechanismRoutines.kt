package org.firstinspires.ftc.teamcode.routines

import com.pedropathing.follower.Follower
import com.pedropathing.pathgen.Path
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.ForcedParallelCommand
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil
import com.rowanmcalpin.nextftc.ftc.hardware.Drivetrain
import com.rowanmcalpin.nextftc.pedro.FollowPath
import org.firstinspires.ftc.teamcode.auto.TrajectoryFactory
import org.firstinspires.ftc.teamcode.subsystems.motors.Extension
import org.firstinspires.ftc.teamcode.subsystems.motors.Lift
import org.firstinspires.ftc.teamcode.subsystems.servos.Arm
import org.firstinspires.ftc.teamcode.subsystems.servos.Claw
import org.firstinspires.ftc.teamcode.subsystems.servos.Intake
import org.firstinspires.ftc.teamcode.subsystems.servos.Pivot

object MechanismRoutines {
    val outToIntake: Command
        get() = SequentialGroup(
            Lift.toSlightlyHigh,
            ParallelGroup(
                Extension.toOut,
                Pivot.toIntake,
                Intake.intake
            ),
            Lift.toIntake
        )

    val nearIntake: Command
        get() = SequentialGroup(
            Lift.toSlightlyHigh,
            ParallelGroup(
                Extension.toMiddlePos,
                Pivot.toIntake,
                Intake.intake
            ),
            Lift.toIntake
        )

    val transfer: Command
        get() = SequentialGroup(
            ParallelGroup(
                Pivot.toTransfer,
                Intake.slowIntake,
                Lift.toSlightlyHigh,
                Extension.toTransfer,
                Claw.open,
                Arm.toDown
            ),
            Lift.toIntake
        )

    val scoreSample: Command
        get() = SequentialGroup(
            Claw.close,
            Pivot.toTransfer,
            ParallelGroup(
                Intake.stop,
                Extension.toSlightlyOut,
                Lift.toHigh,
                SequentialGroup(
                    WaitUntil { Lift.motorGroup.currentPosition > 1000 },
                    Extension.toTransfer,
                    Arm.toUp
                )
            )
        )

    val scoreAndReset: Command
        get() = SequentialGroup(
            Claw.open,
            ParallelGroup(
                Arm.toDown,
                Lift.toIntake,
                Extension.toSlightlyOut
            ),
            Extension.toTransfer
        )

    val fullReset: Command
        get() = SequentialGroup(
            Claw.open,
            ParallelGroup(
                Pivot.toTransfer,
                Extension.toFullIn,
                Arm.toDown,
                Claw.close
            ),
            Lift.zero
        )

    val toHang: Command
        get() = ParallelGroup(
            Claw.close,
            Pivot.toTransfer,
            Extension.toFullIn,
            Lift.toHang,
            Arm.toUp
        )

    val specimenPickup: Command
        get() = SequentialGroup(
            Arm.toSpecimenPickup,
            ParallelGroup(
                Claw.specimenOpen,
                Pivot.toTransfer,
                Extension.toSlightlyOut,
            ),
            ForcedParallelCommand(Lift.toSpecimenPickup),
            WaitUntil { Lift.motorGroup.currentPosition >= 40 },
            Extension.toTransfer
        )

    val specimenScore: Command
        get() = SequentialGroup(
            ParallelGroup(
                Claw.close,
                Pivot.toTransfer,
                Extension.toSlightlyOut,
            ),
            Delay(0.3),
            ForcedParallelCommand(Lift.toSpecimenScore),
            WaitUntil { Lift.motorGroup.currentPosition >= 100 },
            Arm.toUp,
            Extension.toTransfer
        )

    val autoPark: Command
        get() = SequentialGroup(
            fullReset,
            Arm.toUp
        )

    val hang: Command
        get() = ParallelGroup(
            Lift.hang,
            SequentialGroup(
                WaitUntil { Lift.motorGroup.currentPosition < 500 },
                Arm.toSpecimenPickup
            )
        )
}