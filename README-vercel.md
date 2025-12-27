# Vercel Configuration for Physical AI & Humanoid Robotics Docusaurus Site

This repository contains the configuration for deploying the **Physical AI & Humanoid Robotics** Docusaurus site to Vercel.

## Files Included

- `vercel.json` - Main configuration file for Vercel deployment
- `VERCEL_CONFIG_EXPLANATION.md` - Detailed explanation of the configuration
- `TESTING_INSTRUCTIONS.md` - Instructions for testing the configuration locally

## Purpose

The `vercel.json` file optimizes the Docusaurus site for deployment on Vercel by:

- Enabling clean URLs without file extensions
- Setting up proper client-side routing for SPA functionality
- Configuring long-term caching for static assets
- Specifying the correct build and output directories

## Key Features

- **SPA Routing**: All routes fall back to index.html to support client-side navigation
- **Asset Optimization**: Static assets are served with long-term caching headers
- **Clean URLs**: File extensions are removed from URLs for better SEO
- **Optimized Build**: Uses the correct build command and output directory for the Docusaurus setup

## Deployment

When deployed to Vercel, this configuration ensures:

- No 404 errors on client-side routes (e.g., /docs/module1/chapter1)
- Fast loading times due to asset caching
- Consistent behavior between preview and production deployments
- Proper handling of the Docusaurus site structure

## Testing

To test the configuration locally, follow the instructions in `TESTING_INSTRUCTIONS.md`.