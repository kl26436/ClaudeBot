// Calculators for the playbook's Code it chapter, in the shape src/calcs.ts uses.
// To add them: paste the entries into `calcs`, the helpers above it, and 'Code' into CalcDef's group type.
// Default values are the robot's own numbers; the working is in code-decisions-and-math.md.

import type { CalcDef, Values } from '../calcs';

const profileRamp = (v: Values) => (v.cruise * v.cruise) / (2 * v.accel); // distance to reach cruise
const profileTime = (v: Values) =>
  v.dist >= 2 * profileRamp(v)
    ? (2 * v.cruise) / v.accel + (v.dist - 2 * profileRamp(v)) / v.cruise
    : 2 * Math.sqrt(v.dist / v.accel);
const motorKv = (v: Values) => 12 / (v.rpm / 60); // volts per motor turn per second

export const codeCalcs: Record<string, CalcDef> = {
  'motion-profile': {
    title: 'How long does a move take?',
    group: 'Code',
    page: 'code-it',
    inputs: [
      { key: 'dist', label: 'Distance to move', unit: 'in', value: 65.25, step: 0.25 },
      { key: 'cruise', label: 'Cruise speed', unit: 'in/s', value: 110, step: 5 },
      { key: 'accel', label: 'Acceleration', unit: 'in/s²', value: 212, step: 5 },
    ],
    outputs: [
      { label: 'Distance to reach cruise', unit: 'in', compute: profileRamp },
      { label: 'Fastest it gets', unit: 'in/s', digits: 0, compute: (v) => Math.min(v.cruise, Math.sqrt(v.dist * v.accel)) },
      { label: 'Time for the move', unit: 's', digits: 2, compute: profileTime },
    ],
    steps: [
      'Distance to reach cruise = cruise speed² ÷ (2 × acceleration). Slowing down takes the same distance.',
      'If the move is longer than both together: time = 2 × cruise ÷ acceleration + the rest ÷ cruise.',
      'If it is shorter, it never reaches cruise: time = 2 × √(distance ÷ acceleration).',
    ],
    note: 'Calculated. The acceleration is 0.55 g, under the 0.6 g cap that keeps the elevator ropes tight on the way down.',
  },

  'motor-voltage': {
    title: 'Voltage to hold and move a mechanism',
    group: 'Code',
    page: 'code-it',
    inputs: [
      { key: 'rpm', label: 'Motor free speed', unit: 'rpm', value: 5800, step: 100 },
      { key: 'stallAmps', label: 'Motor stall current', unit: 'A', value: 483, step: 1 },
      { key: 'holdAmps', label: 'Current to hold still, each motor', unit: 'A', value: 15.8, step: 0.1 },
      { key: 'perTurn', label: 'Travel per motor turn', unit: 'in', value: 1.375, step: 0.005 },
      { key: 'speed', label: 'Speed', unit: 'in/s', value: 110, step: 5 },
    ],
    outputs: [
      { label: 'Voltage to hold (kG)', unit: 'V', digits: 2, compute: (v) => (v.holdAmps * 12) / v.stallAmps },
      { label: 'Voltage per motor turn per second (kV)', unit: 'V', digits: 3, compute: motorKv },
      { label: 'Voltage to hold and cruise', unit: 'V', digits: 1, compute: (v) => (v.holdAmps * 12) / v.stallAmps + (v.speed / v.perTurn) * motorKv(v) },
    ],
    steps: [
      'Winding resistance = 12 V ÷ stall current.',
      'kG = holding current × resistance.',
      'kV = 12 V ÷ free speed in turns per second.',
      'Motor speed = mechanism speed ÷ travel per turn. Cruise voltage = kG + motor speed × kV.',
    ],
    note: 'Calculated, for a Kraken X60 with FOC. If the answer is close to 12 V there is nothing left to speed up with.',
  },

  'stopping-distance': {
    title: 'How far does it go before it stops?',
    group: 'Code',
    page: 'code-it',
    inputs: [
      { key: 'speed', label: 'Speed', unit: 'in/s', value: 110, step: 5 },
      { key: 'accel', label: 'Deceleration', unit: 'in/s²', value: 212, step: 5 },
      { key: 'loop', label: 'Time before the new command arrives', unit: 's', value: 0.02, step: 0.005 },
    ],
    outputs: [
      { label: 'Before it starts to slow', unit: 'in', compute: (v) => v.speed * v.loop },
      { label: 'While slowing', unit: 'in', compute: (v) => (v.speed * v.speed) / (2 * v.accel) },
      { label: 'In all', unit: 'in', compute: (v) => v.speed * v.loop + (v.speed * v.speed) / (2 * v.accel) },
    ],
    steps: [
      'Distance before slowing = speed × loop time.',
      'Distance while slowing = speed² ÷ (2 × deceleration).',
    ],
    note: 'Calculated. For the wrist use degrees: 400 °/s and 1,600 °/s² is 58°.',
  },

  'steady-error': {
    title: 'How far off does it settle?',
    group: 'Code',
    page: 'code-it',
    inputs: [
      { key: 'volts', label: 'Voltage error (friction, or a wrong kG)', unit: 'V', value: 0.13, step: 0.01 },
      { key: 'kp', label: 'kP', unit: 'V per motor turn', value: 0.7, step: 0.1 },
      { key: 'perTurn', label: 'Travel per motor turn', unit: 'in', value: 1.375, step: 0.005 },
    ],
    outputs: [
      { label: 'Settles off by', unit: 'in', digits: 2, compute: (v) => (v.volts / v.kp) * v.perTurn },
    ],
    steps: [
      'With no integral term, the loop stops pushing when kP × error equals the voltage it is short.',
      'Error in motor turns = voltage error ÷ kP. Multiply by travel per turn.',
    ],
    note: 'Calculated. Any interlock that waits for this mechanism needs a window wider than this.',
  },
};
