# Physical AI & Humanoid Robotics Documentation

This website is built using [Docusaurus](https://docusaurus.io/), a modern static website generator, to provide comprehensive documentation for the Physical AI & Humanoid Robotics course.

## About

This documentation site covers Module 1: The Robotic Nervous System (ROS 2), Module 2: The Digital Twin (Gazebo & Unity), and Module 3: The AI-Robot Brain (NVIDIA Isaac™), providing in-depth information on:

- Core ROS 2 components: Nodes, Topics, and Services
- Bridging Python agents to ROS controllers using rclpy
- Robot modeling with URDF (Unified Robot Description Format) for humanoid robots
- Gazebo physics simulation
- Unity rendering for human-robot interaction
- Sensor simulation for perception systems
- NVIDIA Isaac Sim for photorealistic simulation and synthetic data generation
- Isaac ROS for hardware-accelerated perception pipelines
- Nav2 for humanoid navigation in complex environments

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

The site is configured for deployment to GitHub Pages using GitHub Actions. When you push changes to the main branch, the workflow in `.github/workflows/gh-pages-deploy.yml` will automatically build and deploy the site.

For manual deployment to GitHub Pages:

Using SSH:
```bash
USE_SSH=true npm run deploy
```

Not using SSH:
```bash
GIT_USER=<Your GitHub username> npm run deploy
```
