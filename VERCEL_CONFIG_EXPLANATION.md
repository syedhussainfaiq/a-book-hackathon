# Vercel Configuration for Docusaurus Deployment

## Overview

This document explains the `vercel.json` configuration file created for the **Physical AI & Humanoid Robotics** Docusaurus site. The configuration optimizes deployment on Vercel while ensuring proper client-side routing and performance.

## Configuration Sections

### 1. Clean URLs
```json
"cleanUrls": true
```
- Enables clean URLs by removing file extensions (e.g., `/docs/intro.html` becomes `/docs/intro`)
- Provides cleaner, more SEO-friendly URLs
- Works well with Docusaurus's default routing

### 2. Trailing Slash Handling
```json
"trailingSlash": false
```
- Removes trailing slashes from URLs (e.g., `/docs/intro/` becomes `/docs/intro`)
- Ensures consistency in URL structure across the site
- Prevents duplicate content issues for SEO

### 3. Asset Caching Headers
```json
"headers": [
  {
    "source": "/assets/(.*)",
    "headers": [
      {
        "key": "Cache-Control",
        "value": "public, max-age=31536000, immutable"
      }
    ]
  },
  {
    "source": "/(.*)\\.(js|css|woff2|woff|ttf|eot|png|jpg|jpeg|gif|svg|webp|avif)",
    "headers": [
      {
        "key": "Cache-Control",
        "value": "public, max-age=31536000, immutable"
      }
    ]
  }
]
```
- Sets long-term caching (1 year) for static assets
- Improves site performance by reducing repeated downloads
- Uses `immutable` directive to indicate assets won't change during cache period
- Covers common asset types: images, fonts, JavaScript, and CSS files

### 4. Client-Side Routing (SPA Fallback)
```json
"rewrites": [
  {
    "source": "/(.*)",
    "destination": "/index.html"
  }
]
```
- Ensures all routes fall back to `index.html` for client-side routing
- Critical for Docusaurus SPAs to handle deep links correctly
- Prevents 404 errors when users navigate directly to nested routes
- Allows React Router to handle routing on the client side

### 5. Build Configuration
```json
"buildCommand": "cd docusaurus && npm run build",
"outputDirectory": "docusaurus/build",
"devCommand": "cd docusaurus && npm run start"
```
- Specifies the build command for Vercel to execute
- Points to the correct output directory where Docusaurus builds the site
- Defines the development command for Vercel Preview deployments
- Accounts for the Docusaurus project being in a subdirectory

## Special Considerations for Your Project

### Base URL Handling
Your Docusaurus configuration uses `baseUrl: '/humanoid-robotics-book/'`, which means the site is designed to be deployed at a subpath. However, when deploying to Vercel with this configuration, you'll need to consider:

1. If deploying to the root of a custom domain, you might want to update your Docusaurus config to use `baseUrl: '/'`
2. If deploying to a subpath on Vercel, you can map the subpath in Vercel's dashboard settings
3. For GitHub Pages deployment, the current `baseUrl` is appropriate

## How to Test Locally

1. **Build the Docusaurus site:**
   ```bash
   cd docusaurus
   npm run build
   ```

2. **Serve the build locally:**
   ```bash
   npx serve docusaurus/build
   ```

3. **Navigate to various routes:**
   - Visit the homepage
   - Navigate to deep links like `/docs/module-1/intro`
   - Test different sections to ensure routing works correctly

4. **Verify asset caching:**
   - Open browser DevTools
   - Check Network tab to confirm assets are cached appropriately
   - Look for `Cache-Control` headers on asset requests

## Verification Checklist

- [ ] Site deploys successfully on Vercel with no 404 errors on client-side routes
- [ ] Static assets (images, JS, CSS) served with long-term caching
- [ ] Rewrites correctly fallback to index.html for SPA routing
- [ ] Clean URLs enabled and working properly
- [ ] Deployment preview and production behave identically in routing
- [ ] Build process completes successfully with specified commands

## References

- [Vercel Configuration Documentation](https://vercel.com/docs/project-configuration)
- [Docusaurus Deployment Guide](https://docusaurus.io/docs/deployment)
- [Docusaurus with Vercel Guide](https://v2.docusaurus.io/docs/deployment/#vercel)