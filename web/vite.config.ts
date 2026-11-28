import react from '@vitejs/plugin-react';
import { defineConfig } from 'vitest/config';

export default defineConfig({
  plugins: [react()],
  // The 3D view loads the robot's meshes straight from ros2_ws (see scene/RobotModel.tsx).
  server: { host: true, port: 8080, fs: { allow: ['..'] } },
  preview: { host: true, port: 8080 },
  test: { environment: 'node' },
});
