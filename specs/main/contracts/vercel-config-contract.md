# Vercel Configuration Contract

## Purpose
This contract defines the expected behavior and functionality of the vercel.json configuration for the Physical AI & Humanoid Robotics Docusaurus site.

## Routing Contract

### Client-Side Routing
- **Request**: Any route path (e.g., `/docs/module1/chapter1`, `/blog/2023/ai-advances`)
- **Expected Response**: The request should be rewritten to `/index.html`
- **Implementation**: Using Vercel's rewrites feature with pattern `"/(.*)"` → `"/index.html"`
- **Rationale**: Allows React Router in Docusaurus to handle client-side navigation

### Clean URLs
- **Request**: A page request (e.g., `/docs/intro`)
- **Expected Response**: Should return the content without requiring a file extension
- **Implementation**: Using `"cleanUrls": true` in vercel.json
- **Rationale**: Provides SEO-friendly URLs without `.html` extensions

## Performance Contract

### Asset Caching
- **Request**: Static assets (JS, CSS, images, fonts)
- **Expected Response**: Assets served with long-term caching headers
- **Implementation**: 
  - Pattern: `"/assets/(.*)"` and `"/(.*)\\.(js|css|woff2|woff|ttf|eot|png|jpg|jpeg|gif|svg|webp|avif)"`
  - Header: `"Cache-Control": "public, max-age=31536000, immutable"`
- **Rationale**: Improves performance by reducing repeated downloads

### Caching Duration
- **Requirement**: Static assets cached for 1 year (31,536,000 seconds)
- **Implementation**: `max-age=31536000` in Cache-Control header
- **Rationale**: Balances performance with ability to update assets when needed

## Build Contract

### Build Process
- **Command**: `"cd docusaurus && npm run build"`
- **Expected Output**: Docusaurus site built to `docusaurus/build` directory
- **Verification**: Output directory contains complete static site

### Output Directory
- **Location**: `"docusaurus/build"`
- **Contents**: Complete static site ready for deployment
- **Access**: Vercel should serve files from this directory

## Compatibility Contract

### Docusaurus Version
- **Requirement**: Compatible with Docusaurus 3.x
- **Verification**: Configuration tested with Docusaurus 3.9.2
- **Constraints**: Uses standard Docusaurus build process and file structure

### Base URL Handling
- **Requirement**: Works with existing `baseUrl` configuration in Docusaurus
- **Verification**: Routing works correctly with `baseUrl: '/humanoid-robotics-book/'`
- **Note**: May require Vercel project settings adjustment for subpath deployments