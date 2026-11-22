# Open numbers

Every value that is not yet confirmed carries a `// PLACEHOLDER(<reason>)` comment on its line.
This file lists each reason and what resolves it. Find the affected values with
`grep -rn "PLACEHOLDER(" src`.

| Reason | Meaning | What resolves it |
|---|---|---|
| `funnel-redesign` | Value depends on the funnel/intake geometry that is being redesigned. | The released CAD for the redesigned funnel. Replace the value with the measured dimension or setpoint. |
| `drawing` | Value should come from a mechanical drawing or CAD that does not exist yet. | The released drawing for that mechanism. Replace with the dimension read from it. |
| `tuning` | Value is a starting guess for a gain, limit or setpoint that must be found on the robot or in simulation. | A tuning session on the built robot. Replace with the tuned value and remove the comment. |
| `field-check` | Value depends on the real field's geometry or tolerances. | Measuring on a real field (or the official field drawings) at the first event or practice session. |

Rule: when a value is confirmed, change it and delete its `PLACEHOLDER` comment in the same commit.

## Known simulation limits

- **Elevator cruise speed.** `ElevatorK.CRUISE_IN_PER_S` is 65 in/s (about 1.3 s for full travel). The design value was 110 in/s. At 110 in/s in the Phoenix simulator the Motion Magic reference itself stops slowing down properly (its velocity falls from 110 to about 100 in/s, then at only about 12 in/s² instead of 231.65) and the carriage reaches the top hard stop. Below about 70 in/s the profile is normal. Gains, FOC, soft limits and the physics step rate make no difference. See `docs/HANDOFF.md`, Open problems. Do not raise the value on a real robot near the top of travel until this is understood.
- **CANrange dropout.** Phoenix simulates a CANrange's distance (`CANrangeSimState.setDistance`) but cannot take one off the bus. `CoralRollersIOSim.setRangeConnected` and `AlgaeClawIOSim.setRangeConnected` fake the dropout at the IO layer by reporting `rangeConnected = false`; the distance signal is still the simulated one.
- **Claw hold at stall.** Stalled on an algae, the 1.5 V claw hold voltage would draw about 45 A per motor, so the 30 A stator limit holds it to about 0.8 V. Intake (6 V) is limited the same way. Check on the robot that 30 A grips an algae firmly enough; if it does, the hold voltage could drop to about 0.8 V with no loss of grip.
