# Quickstart Guide: Vercel Configuration for Docusaurus

## Overview
This guide provides a quick setup for deploying your Docusaurus site to Vercel with optimal configuration.

## Prerequisites
- Docusaurus site already created and functional
- Git repository set up
- Vercel account

## Step 1: Add vercel.json to Your Project
Create a `vercel.json` file in your project root with the following content:

```json
{
  "cleanUrls": true,
  "trailingSlash": false,
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
  ],
  "rewrites": [
    {
      "source": "/(.*)",
      "destination": "/index.html"
    }
  ],
  "buildCommand": "cd docusaurus && npm run build",
  "outputDirectory": "docusaurus/build",
  "devCommand": "cd docusaurus && npm run start"
}
```

## Step 2: Verify Your Docusaurus Configuration
Make sure your `docusaurus.config.js` (or `.ts`) has the correct `baseUrl` setting:
- For root domain deployment: `baseUrl: '/'`
- For subpath deployment: `baseUrl: '/your-subpath/'`

## Step 3: Deploy to Vercel
1. Push your code to a Git repository
2. Import your project into Vercel
3. Vercel will automatically detect the configuration and apply it

## Step 4: Test Your Deployment
1. Visit your deployed site
2. Test deep links directly (e.g., /docs/some-deep-page)
3. Verify assets are loading with proper caching headers
4. Confirm clean URLs are working without file extensions

## Troubleshooting
- If deep links return 404, check the rewrite rules in vercel.json
- If assets aren't caching, verify the header patterns match your asset paths
- If build fails, ensure the build command matches your project structure