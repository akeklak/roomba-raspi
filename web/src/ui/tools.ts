import { Store } from '../store';

/**
 * Clicking on the floor in the 3D view does something only while a tool is picked:
 *   goal  "Go to": the robot drives there by itself
 *   pose  "Set position": tells localization where the robot really is
 * Click a spot, or press and drag to also choose which way the robot should face. Picking a spot
 * puts the tool down again; Esc cancels. Later phases add drawing no-go zones the same way.
 */
export type Tool = 'none' | 'goal' | 'pose';

export const tool = new Store<Tool>('none');

/** The spot being picked, in the frame the scene is drawn in. Read every frame by the 3D view. */
export const toolDrag = { active: false, x: 0, y: 0, yaw: 0, hasHeading: false };

/** Below this drag distance (meters) a press counts as a click, with no direction chosen. */
export const MIN_DRAG = 0.2;
