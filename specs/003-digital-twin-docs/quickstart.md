# Quickstart Guide: Digital Twin Module (Gazebo & Unity)

## Prerequisites
- Access to the existing Docusaurus site (Module 1: ROS 2)
- Understanding of basic robotics concepts
- Familiarity with simulation environments (Gazebo, Unity)
- Basic knowledge of humanoid robot systems

## Setup Instructions

### 1. Navigate to the Digital Twin Module
The Digital Twin module is located at `/docs/module-2/` in the Docusaurus project structure.

### 2. Understanding the Module Structure
The module consists of 3 chapters:
- `intro.md`: Module overview
- `chapter-1-gazebo-physics.md`: Gazebo physics simulation
- `chapter-2-unity-hri.md`: Unity rendering and human-robot interaction
- `chapter-3-sensor-simulation.md`: Sensor simulation

### 3. Content Creation Guidelines
When creating or updating content for this module:

1. Maintain consistency with Module 1's style and format
2. Include practical examples with code/configurations
3. Add diagrams and visual aids where appropriate
4. Ensure all examples are tested and functional
5. Link to official Gazebo, Unity Robotics Hub, and ROS documentation

## Project Structure
```
docusaurus/
├── docs/
│   ├── module-1/           # Existing module (ROS 2)
│   │   ├── intro.md
│   │   ├── chapter-1-nodes-topics-services.md
│   │   ├── chapter-2-rclpy-bridge.md
│   │   └── chapter-3-urdf-humanoids.md
│   ├── module-2/           # New module (Digital Twin)
│   │   ├── intro.md        # Module overview
│   │   ├── chapter-1-gazebo-physics.md      # Gazebo physics simulation
│   │   ├── chapter-2-unity-hri.md           # Unity rendering and HRI
│   │   └── chapter-3-sensor-simulation.md   # Sensor simulation
│   └── ...
├── src/
├── static/
├── docusaurus.config.js
├── sidebars.js             # Updated to include Module 2
├── package.json
└── README.md
```

## Adding New Content

### To add a new documentation page:
1. Create a new `.md` file in the `docs/module-2/` directory
2. Add the required frontmatter:
   ```markdown
   ---
   title: My New Page
   description: A brief description of this page
   sidebar_position: [position number]
   ---
   ```
3. Add your content using Markdown syntax following the established pattern
4. Update `sidebars.js` to include the new page in the navigation if needed

### To update the sidebar navigation:
Edit the `sidebars.js` file to organize your documentation pages in the desired order, ensuring Module 2 appears correctly after Module 1.

## Configuration

### Site Configuration
The `docusaurus.config.js` file contains all site configuration including:
- Site metadata (title, tagline, URL)
- Theme configuration
- Plugin configuration
- Deployment settings

### Navigation
The `sidebars.js` file defines the navigation structure for your documentation.

## Building for Production

### To build the site for deployment:
```bash
npm run build
```
This command generates static content in the `build` directory, which can be served using any static hosting service.

### Deployment to GitHub Pages
The site is configured for deployment to GitHub Pages using GitHub Actions. When you push changes to the main branch, the workflow will automatically build and deploy the site.

## Best Practices for Digital Twin Documentation

### Gazebo Physics Content
- Focus on practical examples with world files and model configurations
- Explain physics parameters and their effects on humanoid robots
- Include downloadable example files
- Address collision detection and performance optimization

### Unity Rendering Content
- Emphasize creating immersive visualization environments
- Document Human-Robot Interaction interface design principles
- Address performance considerations for real-time rendering
- Provide examples of robot model import and setup

### Sensor Simulation Content
- Explain the differences between real and simulated sensors
- Include sample output data for each sensor type
- Document configuration parameters and their effects
- Provide validation techniques for sensor models