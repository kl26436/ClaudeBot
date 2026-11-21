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
