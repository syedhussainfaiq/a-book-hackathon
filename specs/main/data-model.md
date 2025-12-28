# Data Model for Vercel Configuration

## Configuration Structure

The vercel.json file contains the following main sections:

### 1. Routing Configuration
- `cleanUrls`: Boolean to enable clean URLs without extensions
- `trailingSlash`: Boolean to control trailing slash behavior
- `rewrites`: Array of rewrite rules for client-side routing

### 2. Performance Configuration
- `headers`: Array of header rules for asset caching
- `source`: Pattern to match files
- `key`/`value`: Cache-Control headers for assets

### 3. Build Configuration
- `buildCommand`: Command to build the Docusaurus site
- `outputDirectory`: Directory where built site is located
- `devCommand`: Command for development server

## Asset Types for Caching

### Long-term Cacheable Assets
- JavaScript files (.js)
- CSS files (.css)
- Font files (.woff2, .woff, .ttf, .eot)
- Image files (.png, .jpg, .jpeg, .gif, .svg, .webp, .avif)

### Cache Headers
- `Cache-Control`: public, max-age=31536000, immutable
- Ensures assets are cached for 1 year with immutable flag