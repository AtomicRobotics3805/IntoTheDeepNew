package org.firstinspires.ftc.teamcode.routines

import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import com.rowanmcalpin.nextftc.pedro.FollowPath
import org.firstinspires.ftc.teamcode.auto.TrajectoryFactory
import org.firstinspires.ftc.teamcode.subsystems.misc.IntakeSensor

object SampleRoutines {
    val sampleAuto: Command
        get() = SequentialGroup(
            ParallelGroup(
                MechanismRoutines.scoreSample,
                FollowPath(TrajectoryFactory.scorePreloadSample)
            ),
            ParallelGroup(
                SequentialGroup(
                    MechanismRoutines.scoreAndReset,
                    MechanismRoutines.outToIntake
                ),
                SequentialGroup(
                    Delay(0.5),
                    FollowPath(TrajectoryFactory.pickupFirstSample)
                )
            ),
            IntakeSensor.WaitUntilSample(),
            ParallelGroup(
                SequentialGroup(
                    MechanismRoutines.transfer,
                    MechanismRoutines.scoreSample
                ),
                SequentialGroup(
                    Delay(0.5),
                    FollowPath(TrajectoryFactory.scoreFirstSample)
                )
            ),
            ParallelGroup(
                SequentialGroup(
                    MechanismRoutines.scoreAndReset,
                    MechanismRoutines.outToIntake
                ),
                SequentialGroup(
                    Delay(0.5),
                    FollowPath(TrajectoryFactory.pickupSecondSample)
                )
            ),
            IntakeSensor.WaitUntilSample(),
            ParallelGroup(
                SequentialGroup(
                    MechanismRoutines.transfer,
                    MechanismRoutines.scoreSample
                ),
                SequentialGroup(
                    Delay(0.5),
                    FollowPath(TrajectoryFactory.scoreSecondSample)
                )
            ),
            ParallelGroup(
                SequentialGroup(
                    MechanismRoutines.scoreAndReset,
                    MechanismRoutines.outToIntake
                ),
                SequentialGroup(
                    Delay(0.5),
                    FollowPath(TrajectoryFactory.pickupThirdSample)
                )
            ),
            IntakeSensor.WaitUntilSample(),
            ParallelGroup(
                SequentialGroup(
                    MechanismRoutines.transfer,
                    MechanismRoutines.scoreSample
                ),
                SequentialGroup(
                    Delay(0.5),
                    FollowPath(TrajectoryFactory.scoreThirdSample)
                )
            ),
            ParallelGroup(
                MechanismRoutines.autoPark,
                SequentialGroup(
                    Delay(0.5),
                    FollowPath(TrajectoryFactory.samplePark)
                )
            )
        )
}