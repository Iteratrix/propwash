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

// 1. Hash and copy WASM binary
const wasmBuf = readFileSync("pkg/propwash_web_bg.wasm");
const wasmHash = contentHash(wasmBuf);
const wasmName = `propwash_web_bg-${wasmHash}.wasm`;
copyFileSync("pkg/propwash_web_bg.wasm", join("out", wasmName));

// 2. Hash and copy CSS
const cssBuf = readFileSync("style.css");
const cssHash = contentHash(cssBuf);
const cssName = `style-${cssHash}.css`;
copyFileSync("style.css", join("out", cssName));

// 3. Bundle app + wasm bridge with content hash
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

// 4. Find output JS filename from metafile
const jsOutput = Object.keys(result.metafile.outputs).find(
  (f) => f.endsWith(".js") && !f.endsWith(".js.map")
);
const jsName = basename(jsOutput);

// 5. Generate index.html with hashed asset paths
let html = readFileSync("index.html", "utf-8");
html = html.replace('href="style.css"', `href="${cssName}"`);
html = html.replace('src="dist/app.js"', `src="${jsName}"`);
writeFileSync(join("out", "index.html"), html);

// 6. Copy CNAME for GitHub Pages custom domain
if (existsSync("CNAME")) {
  copyFileSync("CNAME", join("out", "CNAME"));
}

console.log(`Built: ${jsName}, ${cssName}, ${wasmName}`);
