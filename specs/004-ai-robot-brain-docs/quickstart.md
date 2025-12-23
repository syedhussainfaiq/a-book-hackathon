# Quickstart Guide: AI-Robot Brain Module (NVIDIA Isaac™)

## Prerequisites
- Access to the existing Docusaurus site (Modules 1 & 2)
- Understanding of basic robotics concepts
- Familiarity with simulation environments (Gazebo, Unity from Module 2)
- Basic knowledge of humanoid robot systems
- Access to NVIDIA GPU hardware for Isaac tools (recommended)

## Setup Instructions

### 1. Navigate to the AI-Robot Brain Module
The AI-Robot Brain module is located at `/docs/module-3/` in the Docusaurus project structure.

### 2. Understanding the Module Structure
The module consists of 3 chapters:
- `intro.md`: Module overview
- `chapter-1-isaac-sim-synthetic-data.md`: Isaac Sim and synthetic data generation
- `chapter-2-isaac-ros-vslam.md`: Isaac ROS and Visual SLAM
- `chapter-3-nav2-humanoid.md`: Nav2 for humanoid navigation

### 3. Content Creation Guidelines
When creating or updating content for this module:

1. Maintain consistency with Modules 1-2's style and format
2. Include practical examples with code/configurations
3. Add diagrams and visual aids where appropriate
4. Ensure all examples are tested and functional
5. Link to official NVIDIA Isaac Sim/ROS and Nav2 documentation
6. Highlight GPU requirements where applicable
7. Prepare students for Vision-Language-Action in Module 4

## Project Structure
```
docusaurus/
├── docs/
│   ├── module-1/           # Existing module (ROS 2)
│   │   ├── intro.md
│   │   ├── chapter-1-nodes-topics-services.md
│   │   ├── chapter-2-rclpy-bridge.md
│   │   └── chapter-3-urdf-humanoids.md
│   ├── module-2/           # Existing module (Digital Twin)
│   │   ├── intro.md
│   │   ├── chapter-1-gazebo-physics.md
│   │   ├── chapter-2-unity-hri.md
│   │   └── chapter-3-sensor-simulation.md
│   ├── module-3/           # New module (AI-Robot Brain)
│   │   ├── intro.md        # Module overview
│   │   ├── chapter-1-isaac-sim-synthetic-data.md      # Isaac Sim and synthetic data
│   │   ├── chapter-2-isaac-ros-vslam.md               # Isaac ROS and VSLAM
│   │   └── chapter-3-nav2-humanoid.md                 # Nav2 for humanoid navigation
│   └── ...
├── src/
├── static/
├── docusaurus.config.js
├── sidebars.js             # Updated to include Module 3
├── package.json
└── README.md
```

## Adding New Content

### To add a new documentation page:
1. Create a new `.md` file in the `docs/module-3/` directory
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
Edit the `sidebars.js` file to organize your documentation pages in the desired order, ensuring Module 3 appears correctly after Module 2.

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

## Best Practices for AI-Robot Brain Documentation

### Isaac Sim Content
- Focus on practical examples with scene creation and lighting configuration
- Explain synthetic data generation for perception model training
- Include downloadable example scene files
- Address performance optimization for synthetic data generation

### Isaac ROS and VSLAM Content
- Emphasize leveraging hardware acceleration capabilities
- Document perception pipeline configurations
- Address common issues with VSLAM performance
- Provide benchmarks for performance expectations

### Nav2 for Humanoid Content
- Explain differences between wheeled and bipedal navigation
- Include sample configuration files for humanoid robots
- Document gait-specific navigation parameters
- Provide validation techniques for navigation stability