import { createContext, useContext } from 'react';

import type { RobotConnection } from './ros/connection';
import type { RobotData } from './ros/robotData';

export type Robot = { connection: RobotConnection; data: RobotData };

export const RobotContext = createContext<Robot | null>(null);

export function useRobot(): Robot {
  const robot = useContext(RobotContext);
  if (!robot) throw new Error('useRobot outside RobotContext');
  return robot;
}
