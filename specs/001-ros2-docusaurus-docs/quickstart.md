# Quickstart Guide: ROS 2 Documentation Site

## Prerequisites
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git
- A GitHub account (for deployment)

## Setup Instructions

### 1. Clone or Initialize the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to the Docusaurus Directory
```bash
cd docusaurus
```

### 3. Install Dependencies
```bash
npm install
```

### 4. Start the Development Server
```bash
npm start
```
This command starts a local development server and opens the documentation site in your browser at `http://localhost:3000`. Most changes are reflected live without restarting the server.

## Project Structure
```
docusaurus/
├── docs/
│   ├── module-1/
│   │   ├── intro.md
│   │   ├── chapter-1-nodes-topics-services.md
│   │   ├── chapter-2-rclpy-bridge.md
│   │   └── chapter-3-urdf-humanoids.md
│   └── ...
├── src/
│   ├── components/
│   ├── pages/
│   └── css/
├── static/
├── docusaurus.config.js
├── sidebars.js
├── package.json
└── README.md
```

## Adding New Content

### To add a new documentation page:
1. Create a new `.md` file in the appropriate directory under `docs/`
2. Add the required frontmatter:
   ```markdown
   ---
   title: My New Page
   description: A brief description of this page
   sidebar_position: [position number]
   ---
   ```
3. Add your content using Markdown syntax
4. Update `sidebars.js` to include the new page in the navigation

### To update the sidebar navigation:
Edit the `sidebars.js` file to organize your documentation pages in the desired order.

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
The site is configured for deployment to GitHub Pages using GitHub Actions. When you push changes to the main branch, the workflow in `.github/workflows/gh-pages-deploy.yml` will automatically build and deploy the site.

## Customization

### Styling
- Custom CSS can be added to `src/css/custom.css`
- Docusaurus themes can be customized in `docusaurus.config.js`
- Components can be themed using Docusaurus' swizzling feature

### Components
- Custom React components can be added to `src/components/`
- These can be used throughout your documentation pages

## Development Workflow

1. Make changes to documentation files
2. Preview changes on the local development server
3. Commit changes to a feature branch
4. Create a pull request for review
5. Once approved, merge to main to trigger deployment