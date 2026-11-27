import { useEffect, useMemo } from 'react';

import { RobotContext } from './context';
import { RobotConnection, defaultBridgeUrl } from './ros/connection';
import { RobotData } from './ros/robotData';
import { Scene } from './scene/Scene';
import { settings } from './settings';
import { useStore } from './store';
import { CameraWindow } from './ui/CameraWindow';
import { DrivePad } from './ui/DrivePad';
import { LayersPanel } from './ui/LayersPanel';
import { MapPanel, ToolHint } from './ui/MapPanel';
import { ConnectionHelp, StatusPanel } from './ui/StatusPanel';

export function App() {
  const robot = useMemo(() => {
    const connection = new RobotConnection(defaultBridgeUrl());
    return { connection, data: new RobotData(connection) };
  }, []);
  const s = useStore(settings);

  useEffect(() => {
    robot.data.start();
    robot.connection.connect();
    return () => {
      robot.data.stop();
      robot.connection.close();
    };
  }, [robot]);

  useEffect(() => {
    robot.data.setCameraEnabled(s.cameraPip || s.camera3d);
  }, [robot, s.cameraPip, s.camera3d]);

  useEffect(() => {
    robot.data.setCostmapEnabled(s.costmap);
  }, [robot, s.costmap]);

  return (
    <RobotContext.Provider value={robot}>
      <div className="app">
        <Scene />
        <div className="overlay">
          <div className="left-stack">
            <StatusPanel />
            <MapPanel />
          </div>
          <LayersPanel />
          <ConnectionHelp />
          <ToolHint />
          {s.cameraPip ? <CameraWindow /> : null}
          <DrivePad />
        </div>
      </div>
    </RobotContext.Provider>
  );
}
