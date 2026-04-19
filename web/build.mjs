import * as esbuild from "esbuild";
import { createHash } from "crypto";
import {
  readFileSync,
  writeFileSync,
  copyFileSync,
  mkdirSync,
  rmSync,
  existsSync,
} from "fs";
import { join, basename } from "path";

const dev = process.argv.includes("--dev");
const watch = process.argv.includes("--watch");

function contentHash(buf) {
  return createHash("sha256").update(buf).digest("hex").slice(0, 10);
}

// --- Dev mode: unhashed output to dist/, optional watch ---
if (dev) {
  const config = {
    entryPoints: ["src/app.ts"],
    bundle: true,
    outfile: "dist/app.js",
    format: "esm",
    sourcemap: true,
    target: "es2020",
    external: ["../pkg/propwash_web.js"],
    define: {
      __WASM_URL__: JSON.stringify("pkg/propwash_web_bg.wasm"),
    },
  };

  if (watch) {
    const ctx = await esbuild.context(config);
    await ctx.watch();
    console.log("Watching for changes...");
  } else {
    await esbuild.build(config);
    console.log("Dev build complete: dist/app.js");
  }
  process.exit(0);
}

// --- Production build: content-hashed output to out/ ---
rmSync("out", { recursive: true, force: true });
mkdirSync("out", { recursive: true });

function hashAndCopy(srcPath, prefix, ext) {
  const buf = readFileSync(srcPath);
  const hash = contentHash(buf);
  const name = `${prefix}-${hash}.${ext}`;
  copyFileSync(srcPath, join("out", name));
  return name;
}

// 1. Hash and copy WASM binary
const wasmName = hashAndCopy("pkg/propwash_web_bg.wasm", "propwash_web_bg", "wasm");

// 2. Hash and copy CSS
const cssName = hashAndCopy("style.css", "style", "css");

// 3. Hash and copy vendor scripts (self-hosted CDN libs for offline support)
const uplotCssName = hashAndCopy("vendor/uplot.min.css", "uplot", "css");
const uplotJsName = hashAndCopy("vendor/uplot.min.js", "uplot", "js");
const echartsJsName = hashAndCopy("vendor/echarts.min.js", "echarts", "js");

// 4. Bundle app + wasm bridge with content hash
const result = await esbuild.build({
  entryPoints: ["src/app.ts"],
  bundle: true,
  outdir: "out",
  entryNames: "[name]-[hash]",
  format: "esm",
  sourcemap: true,
  target: "es2020",
  metafile: true,
  define: {
    __WASM_URL__: JSON.stringify(wasmName),
  },
});

// 5. Find output JS filename from metafile
const jsOutput = Object.keys(result.metafile.outputs).find(
  (f) => f.endsWith(".js") && !f.endsWith(".js.map")
);
const jsName = basename(jsOutput);

// 6. Generate index.html with hashed asset paths
let html = readFileSync("index.html", "utf-8");
html = html
  .replace('href="style.css"', `href="${cssName}"`)
  .replace('src="dist/app.js"', `src="${jsName}"`)
  .replace(
    /https:\/\/cdn\.jsdelivr\.net\/npm\/uplot@[^"]+uPlot\.min\.css/,
    uplotCssName
  )
  .replace(
    /https:\/\/cdn\.jsdelivr\.net\/npm\/uplot@[^"]+uPlot\.iife\.min\.js/,
    uplotJsName
  )
  .replace(
    /https:\/\/cdn\.jsdelivr\.net\/npm\/echarts@[^"]+echarts\.min\.js/,
    echartsJsName
  );
writeFileSync(join("out", "index.html"), html);

// 7. Generate service worker with precache manifest
const swTemplate = readFileSync("sw.template.js", "utf-8");
const precache = [
  "./",
  "./index.html",
  jsName,
  cssName,
  wasmName,
  uplotCssName,
  uplotJsName,
  echartsJsName,
];
const precacheJson = JSON.stringify(precache);
const swVersion = contentHash(Buffer.from(precacheJson));
const sw = swTemplate
  .replace("__PRECACHE__", precacheJson)
  .replace("__VERSION__", swVersion);
writeFileSync(join("out", "sw.js"), sw);

// 8. Copy kill-switch (deployed manually as sw.js to recover from a broken SW)
if (existsSync("sw-killswitch.js")) {
  copyFileSync("sw-killswitch.js", join("out", "sw-killswitch.js"));
}

// 9. Copy CNAME for GitHub Pages custom domain
if (existsSync("CNAME")) {
  copyFileSync("CNAME", join("out", "CNAME"));
}

console.log(
  `Built: ${jsName}, ${cssName}, ${wasmName}, sw.js (v${swVersion})`
);
