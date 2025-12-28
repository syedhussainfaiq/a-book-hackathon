# Testing the Vercel Configuration Locally

## Prerequisites

Before testing the configuration locally, ensure you have:

1. Node.js (version 18 or higher)
2. npm or yarn package manager
3. The project dependencies installed

## Step-by-Step Testing Instructions

### 1. Install Dependencies

First, navigate to the docusaurus directory and install dependencies:

```bash
cd docusaurus
npm install
```

### 2. Build the Site

Build the Docusaurus site using the same command specified in your `vercel.json`:

```bash
npm run build
```

This command will create a `build` directory in the docusaurus folder with the static site.

### 3. Install a Local Server (if not already installed)

Install `serve` globally to serve the static files:

```bash
npm install -g serve
```

### 4. Serve the Built Site

Serve the built site from the output directory specified in `vercel.json`:

```bash
serve -s docusaurus/build
```

The site will be available at `http://localhost:3000` (or another available port).

### 5. Test Client-Side Routing

Navigate to various routes to ensure SPA routing works correctly:

1. Visit the homepage: `http://localhost:3000`
2. Navigate to a documentation page directly: `http://localhost:3000/docs/module-1/intro`
3. Test other nested routes like: `http://localhost:3000/docs/module-1/installation`
4. Verify that navigation between pages works correctly without 404 errors

### 6. Verify Asset Caching

1. Open browser DevTools (F12)
2. Go to the Network tab
3. Refresh the page
4. Check that static assets (CSS, JS, images) have appropriate `Cache-Control` headers
5. Look for headers like: `Cache-Control: public, max-age=31536000, immutable`

### 7. Test Clean URLs

Verify that clean URLs work without file extensions:

1. Navigate to documentation pages
2. Check that URLs don't have `.html` extensions
3. Verify that trailing slashes are handled correctly based on your `trailingSlash` setting

### 8. Test Build Command

Confirm that the build command specified in `vercel.json` works correctly:

```bash
cd docusaurus && npm run build
```

This should generate the site without errors.

## Alternative Testing with Vercel CLI

If you want to test more closely with Vercel's actual deployment process:

### 1. Install Vercel CLI

```bash
npm i -g vercel
```

### 2. Link Your Project

```bash
vercel link
```

### 3. Deploy for Preview

```bash
vercel --prod
```

This will deploy your site to a preview URL where you can test all the functionality.

## Troubleshooting Common Issues

### Issue: 404 errors on direct navigation
**Solution:** Check that the rewrite rules in `vercel.json` are correctly redirecting all routes to `index.html`.

### Issue: Assets not caching properly
**Solution:** Verify that the header rules in `vercel.json` match the actual paths of your assets.

### Issue: Build fails
**Solution:** Ensure that the build command in `vercel.json` matches what works locally.

## Verification Checklist

After testing, confirm:

- [ ] The site builds successfully with `npm run build`
- [ ] All routes work when accessed directly (no 404 errors)
- [ ] Assets are served with appropriate caching headers
- [ ] Clean URLs work without file extensions
- [ ] Navigation between pages works correctly
- [ ] The site functions as expected in a local production-like environment