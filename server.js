const http = require("http");
const fs = require("fs");
const path = require("path");

const repoRoot = __dirname;
const port = Number(process.env.PORT || 8080);

const contentTypes = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".wasm": "application/wasm",
  ".map": "application/json; charset=utf-8",
};

function resolveRequestPath(urlPath) {
  let relativePath;
  if (urlPath === "/") {
    relativePath = "examples/threejs-demo/index.html";
  } else {
    relativePath = decodeURIComponent(urlPath.slice(1));
    if (relativePath.endsWith("/")) {
      relativePath += "index.html";
    }
  }
  const absolutePath = path.normalize(path.join(repoRoot, relativePath));
  if (!absolutePath.startsWith(repoRoot)) {
    return null;
  }
  return absolutePath;
}

const server = http.createServer((req, res) => {
  const absolutePath = resolveRequestPath(req.url || "/");
  if (!absolutePath) {
    res.writeHead(400, { "Content-Type": "text/plain; charset=utf-8" });
    res.end("Bad request");
    return;
  }

  fs.readFile(absolutePath, (err, data) => {
    if (err) {
      res.writeHead(404, { "Content-Type": "text/plain; charset=utf-8" });
      res.end(`Not found: ${path.relative(repoRoot, absolutePath)}`);
      return;
    }

    const ext = path.extname(absolutePath);
    res.writeHead(200, {
      "Access-Control-Allow-Origin": "*",
      "Access-Control-Allow-Methods": "*",
      "Cache-Control": "no-cache",
      "Content-Type": contentTypes[ext] || "application/octet-stream",
    });
    res.end(data);
  });
});

function startServer(currentPort) {
  server
    .once("error", (err) => {
      if (err.code === "EADDRINUSE") {
        const nextPort = currentPort + 1;
        startServer(nextPort);
        return;
      }
      throw err;
    })
    .listen(currentPort, () => {
      console.log(`server start at http://localhost:${currentPort}`);
      console.log(
        "default demo:",
        `http://localhost:${currentPort}/examples/threejs-demo/`,
      );
    });
}

startServer(port);
