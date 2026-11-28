# The web UI, built once and served by nginx on port 80 (mapped to 8080 by docker/compose.yaml).
# Built from the repo root: the 3D view shows the Roomba 675 meshes from
# ros2_ws/src/roomba_bringup/models/roomba_675, and Dockerfile.dockerignore leaves out the rest.
FROM node:22-alpine AS build
WORKDIR /repo/web
COPY web/package.json web/package-lock.json ./
RUN npm ci --no-audit --no-fund
COPY web/ ./
COPY ros2_ws/src/roomba_bringup/models/roomba_675/meshes /repo/ros2_ws/src/roomba_bringup/models/roomba_675/meshes
RUN npm run build

FROM nginx:1.27-alpine
COPY web/nginx.conf /etc/nginx/conf.d/default.conf
COPY --from=build /repo/web/dist /usr/share/nginx/html
