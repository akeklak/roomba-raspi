import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { Object3D } from 'three';

import { App } from './App';
import './styles.css';

// ROS is Z-up. Making three.js Z-up too means positions go straight from ROS into the scene.
Object3D.DEFAULT_UP.set(0, 0, 1);

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
