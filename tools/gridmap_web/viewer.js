const byId = (id) => document.getElementById(id);
const canvas = byId("scene");
const layerSelect = byId("layer");
const status = byId("status");
const gl = canvas.getContext("webgl2", { antialias: true });

function makeRenderer() {
  const compile = (type, source) => {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      throw new Error(gl.getShaderInfoLog(shader));
    }
    return shader;
  };
  const program = gl.createProgram();
  gl.attachShader(program, compile(gl.VERTEX_SHADER, `#version 300 es
    layout(location=0) in vec3 position;
    uniform vec3 right, up, depth;
    uniform vec2 scale, target;
    uniform float depthScale, pointSize;
    uniform int kind;
    uniform float low, span;
    out vec3 color;
    void main() {
      vec3 local = position - vec3(target, 0.0);
      gl_Position = vec4(dot(right, local) * scale.x,
                         dot(up, local) * scale.y,
                         dot(depth, position) * depthScale, 1.0);
      gl_PointSize = pointSize;
      float t = clamp((position.z - low) / span, 0.0, 1.0);
      color = mix(vec3(.13,.36,.62), vec3(.27,.82,.65), min(t * 2.0, 1.0));
      color = mix(color, vec3(.96,.67,.31), max(t * 2.0 - 1.0, 0.0));
      if (kind == 1) color = vec3(.54,.96,1.0);
    }`));
  gl.attachShader(program, compile(gl.FRAGMENT_SHADER, `#version 300 es
    precision mediump float;
    uniform bool roundPoint;
    in vec3 color;
    out vec4 pixel;
    void main() {
      if (roundPoint) {
        float radius = distance(gl_PointCoord, vec2(.5));
        if (radius > .48) discard;
        if (radius > .34) {
          pixel = vec4(.07, .09, .13, 1.0);
          return;
        }
      }
      pixel = vec4(color, 1.0);
    }`));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error(gl.getProgramInfoLog(program));
  }
  gl.useProgram(program);
  const uniforms = Object.fromEntries(
    ["right", "up", "depth", "scale", "target", "depthScale", "pointSize", "kind", "low", "span", "roundPoint"]
      .map((name) => [name, gl.getUniformLocation(program, name)])
  );
  const terrain = gl.createBuffer();
  const indices = gl.createBuffer();
  const cells = gl.createBuffer();
  const overlays = gl.createBuffer();
  gl.enableVertexAttribArray(0);
  gl.clearColor(.047, .075, .11, 1);
  gl.enable(gl.DEPTH_TEST);

  let dimensions = 1;
  let depthRange = 1;
  let indexCount = 0;
  let vertexCount = 0;
  const scanCanvas = byId("scan-overlay");
  const scanContext = scanCanvas.getContext("2d");
  let scanPoints = [];
  let robotCount = 0;
  let azimuth = -0.65;
  let elevation = 0.85;
  let zoom = 1;
  let scanBounds = null;
  let focusScan = false;
  const focusButton = byId("focus-scan");
  let frameRange = null;
  let lockedRange = null;
  let reference = 0;
  const colorLock = byId("color-lock");

  function updateColorScale() {
    const range = lockedRange ?? frameRange;
    colorLock.disabled = range === null;
    colorLock.setAttribute("aria-pressed", String(lockedRange !== null));
    byId("gradient").hidden = range === null;
    if (range === null) {
      byId("range").textContent = "No finite cells";
      return;
    }
    const [low, high] = range;
    gl.uniform1f(uniforms.low, low - reference);
    gl.uniform1f(uniforms.span, high > low ? high - low : 0.001);
    byId("range").textContent = `${low.toFixed(2)} to ${high.toFixed(2)} m`;
  }

  colorLock.addEventListener("click", () => {
    lockedRange = lockedRange === null ? frameRange : null;
    updateColorScale();
    draw();
  });

  function draw() {
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(canvas.clientWidth * ratio));
    const h = Math.max(1, Math.round(canvas.clientHeight * ratio));
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    if (scanCanvas.width !== w || scanCanvas.height !== h) {
      scanCanvas.width = w;
      scanCanvas.height = h;
    }
    gl.viewport(0, 0, w, h);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    const x = Math.cos(azimuth), y = Math.sin(azimuth);
    const sin = Math.sin(elevation), cos = Math.cos(elevation);
    gl.uniform3f(uniforms.right, x, y, 0);
    gl.uniform3f(uniforms.up, -y * sin, x * sin, cos);
    gl.uniform3f(uniforms.depth, -y * cos, x * cos, -sin);
    let viewWidth = dimensions;
    let targetX = 0, targetY = 0;
    if (focusScan && scanBounds) {
      const [minX, maxX, minY, maxY] = scanBounds;
      targetX = (minX + maxX) / 2;
      targetY = (minY + maxY) / 2;
      viewWidth = Math.max(Math.hypot(maxX - minX, maxY - minY), .3);
    }
    const scale = zoom * 1.45 / viewWidth;
    gl.uniform2f(uniforms.target, targetX, targetY);
    const scaleX = scale * Math.min(1, h / w);
    const scaleY = scale * Math.min(1, w / h);
    gl.uniform2f(uniforms.scale, scaleX, scaleY);
    gl.uniform1f(uniforms.depthScale, 1 / depthRange);

    gl.uniform1i(uniforms.kind, 0);
    gl.uniform1i(uniforms.roundPoint, 0);
    gl.uniform1f(uniforms.pointSize, 2 * ratio);
    if (indexCount) {
      gl.bindBuffer(gl.ARRAY_BUFFER, terrain);
      gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indices);
      gl.drawElements(gl.TRIANGLES, indexCount, gl.UNSIGNED_INT, 0);
    }
    if (vertexCount) {
      gl.bindBuffer(gl.ARRAY_BUFFER, cells);
      gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
      gl.drawArrays(gl.POINTS, 0, vertexCount);
    }

    if (robotCount) {
      gl.bindBuffer(gl.ARRAY_BUFFER, overlays);
      gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
      gl.uniform1f(uniforms.pointSize, 12 * ratio);
      gl.disable(gl.DEPTH_TEST);
      gl.uniform1i(uniforms.roundPoint, 1);
      gl.uniform1i(uniforms.kind, 1);
      gl.drawArrays(gl.POINTS, 0, 1);
      gl.uniform1i(uniforms.roundPoint, 0);
      if (robotCount > 1) gl.drawArrays(gl.TRIANGLES, 1, robotCount - 1);
      gl.enable(gl.DEPTH_TEST);
    }

    // Use the same view projection as the mesh on a transparent canvas above it.
    scanContext.clearRect(0, 0, w, h);
    scanContext.strokeStyle = "#151922";
    scanContext.lineWidth = 1.6 * ratio;
    for (const [px, py, pz, valid] of scanPoints) {
      const dx = px - targetX, dy = py - targetY;
      const u = (1 + (x * dx + y * dy) * scaleX) * w / 2;
      const v = (1 - ((-y * dx + x * dy) * sin + pz * cos) * scaleY) * h / 2;
      scanContext.beginPath();
      scanContext.arc(u, v, 4 * ratio, 0, 2 * Math.PI);
      scanContext.fillStyle = valid ? "#ffe066" : "#ff5570";
      scanContext.fill();
      scanContext.stroke();
    }
  }

  function show(frame, heights) {
    const { nx, ny, resolution, center, scan, robot } = frame;
    let lo = Infinity, hi = -Infinity;
    for (const z of heights) {
      if (Number.isFinite(z)) {
        lo = Math.min(lo, z);
        hi = Math.max(hi, z);
      }
    }
    const hasHeight = Number.isFinite(lo);
    reference = hasHeight ? (lo + hi) / 2 : (robot?.[0][2] ?? 0);
    frameRange = hasHeight ? [lo, hi] : null;
    const vertices = new Float32Array(3 * heights.length);
    const triangles = new Uint32Array(6 * (nx - 1) * (ny - 1));
    indexCount = 0;
    for (let y = 0; y < ny; y++) {
      for (let x = 0; x < nx; x++) {
        const i = y * nx + x;
        vertices[3 * i] = (nx / 2 - x - 0.5) * resolution;
        vertices[3 * i + 1] = (ny / 2 - y - 0.5) * resolution;
        vertices[3 * i + 2] = Number.isFinite(heights[i]) ? heights[i] - reference : NaN;
        if (x + 1 === nx || y + 1 === ny) continue;
        // A triangle must have three observed vertices.
        const a = heights[i], b = heights[i + 1], c = heights[i + nx], d = heights[i + nx + 1];
        if (Number.isFinite(a) && Number.isFinite(b) && Number.isFinite(c)) {
          triangles[indexCount++] = i;
          triangles[indexCount++] = i + 1;
          triangles[indexCount++] = i + nx;
        }
        if (Number.isFinite(b) && Number.isFinite(c) && Number.isFinite(d)) {
          triangles[indexCount++] = i + 1;
          triangles[indexCount++] = i + nx + 1;
          triangles[indexCount++] = i + nx;
        }
      }
    }
    scanPoints = [];
    const heading = [];
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const [x, y, z] of scan) {
      const localX = x - center[0], localY = y - center[1];
      minX = Math.min(minX, localX);
      maxX = Math.max(maxX, localX);
      minY = Math.min(minY, localY);
      maxY = Math.max(maxY, localY);
      scanPoints.push([localX, localY, z === null ? 0 : z - reference, z !== null]);
    }
    scanBounds = scan.length ? [minX, maxX, minY, maxY] : null;
    focusButton.disabled = !scanBounds;
    if (robot) {
      const [[x, y, z], [frontX, frontY]] = robot;
      const dx = frontX - x, dy = frontY - y;
      const length = Math.hypot(dx, dy);
      const ox = x - center[0], oy = y - center[1], oz = z - reference;
      heading.push(ox, oy, oz);
      if (length > 1e-6) {
        // A horizontal arrow follows the projection of the robot's +X axis.
        const sideX = -dy / length, sideY = dx / length;
        const point = (along, width) => [
          ox + dx * along + sideX * width,
          oy + dy * along + sideY * width,
          oz,
        ];
        const tailLeft = point(0, .018), tailRight = point(0, -.018);
        const neckLeft = point(.65, .018), neckRight = point(.65, -.018);
        heading.push(...tailLeft, ...tailRight, ...neckLeft,
                     ...neckLeft, ...tailRight, ...neckRight,
                     ...point(.60, .08), ...point(.60, -.08), ...point(1, 0));
      }
    }
    robotCount = heading.length / 3;
    dimensions = Math.max(Math.hypot(nx, ny) * resolution, .1);
    depthRange = dimensions + (hasHeight ? hi - lo : 0);
    const observed = new Float32Array(vertices.length);
    vertexCount = 0;
    for (let i = 0; i < heights.length; i++) {
      if (Number.isFinite(heights[i])) {
        observed.set(vertices.subarray(3 * i, 3 * i + 3), 3 * vertexCount++);
      }
    }
    gl.bindBuffer(gl.ARRAY_BUFFER, cells);
    gl.bufferData(gl.ARRAY_BUFFER, observed.subarray(0, 3 * vertexCount), gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, terrain);
    gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indices);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, triangles.subarray(0, indexCount), gl.DYNAMIC_DRAW);
    gl.bindBuffer(gl.ARRAY_BUFFER, overlays);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(heading), gl.DYNAMIC_DRAW);
    updateColorScale();
    draw();
  }

  canvas.addEventListener("pointerdown", (event) => canvas.setPointerCapture(event.pointerId));
  canvas.addEventListener("pointermove", (event) => {
    if (!canvas.hasPointerCapture(event.pointerId)) return;
    azimuth += event.movementX * .007;
    elevation = Math.max(.12, Math.min(Math.PI / 2, elevation + event.movementY * .007));
    draw();
  });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    zoom = Math.max(.25, Math.min(12, zoom * Math.exp(-event.deltaY * .001)));
    draw();
  }, { passive: false });
  byId("top").onclick = () => { azimuth = 0; elevation = Math.PI / 2; draw(); };
  focusButton.onclick = () => {
    focusScan = true;
    azimuth = 0;
    elevation = Math.PI / 2;
    zoom = 1;
    draw();
  };
  byId("orbit").onclick = () => {
    focusScan = false;
    azimuth = -.65;
    elevation = .85;
    zoom = 1;
    draw();
  };
  new ResizeObserver(draw).observe(canvas);
  return show;
}

async function run() {
  if (!gl) throw new Error("WebGL 2 is unavailable in this browser.");
  const show = makeRenderer();
  let frame, lastFrame = 0, received = [];
  const display = (value, unit) => value == null ? "—" : `${value.toFixed(1)} ${unit}`;
  while (true) {
    try {
      const start = performance.now();
      const requestedLayer = layerSelect.value;
      const query = new URLSearchParams({ layer: requestedLayer, since: frame?.seq ?? "" });
      const response = await fetch(`/frame?${query}`, { signal: AbortSignal.timeout(4000) });
      if (response.status === 200) {
        const payload = await response.arrayBuffer();
        if (layerSelect.value !== requestedLayer) continue;
        const view = new DataView(payload);
        const headerSize = view.getUint32(0, true);
        const next = JSON.parse(new TextDecoder().decode(new Uint8Array(payload, 4, headerSize)));
        const heights = new Float32Array(next.nx * next.ny);
        if (payload.byteLength !== 4 + headerSize + 4 * heights.length) {
          throw new Error("Malformed terrain frame");
        }
        for (let i = 0; i < heights.length; i++) {
          heights[i] = view.getFloat32(4 + headerSize + 4 * i, true);
        }
        const now = performance.now();
        if (frame?.seq !== next.seq) {
          received.push(now);
          lastFrame = now - next.held_ms;
        }
        if (next.layers.join() !== frame?.layers.join()) {
          layerSelect.replaceChildren(...next.layers.map((name) => new Option(name, name)));
          layerSelect.disabled = false;
        }
        layerSelect.value = next.layer;
        frame = next;
        show(frame, heights);
        byId("orin").textContent = display(frame.hz, "Hz");
        byId("age").textContent = display(frame.age_ms, "ms");
        byId("fetch").textContent = display(now - start, "ms");
        const valid = frame.scan.filter((point) => point[2] !== null).length;
        const scanCount = frame.scan.length ? ` · ${valid}/${frame.scan.length} valid` : "";
        byId("details").textContent = `${frame.frame} · ${frame.nx} × ${frame.ny} · ${frame.resolution} m/cell · stamp ${frame.stamp} · ${frame.scan_status}${scanCount}${frame.robot ? "" : " · robot TF unavailable"}`;
      } else if (response.status !== 204) {
        throw new Error(await response.text());
      }
      const now = performance.now();
      received = received.filter((time) => now - time < 5000);
      const stale = frame && now - lastFrame > 1500;
      const rate = received.length > 1 ? 1000 * (received.length - 1) / (received.at(-1) - received[0]) : null;
      byId("browser").textContent = display(stale ? 0 : rate, "Hz");
      if (stale) byId("orin").textContent = "0.0 Hz";
      status.textContent = stale ? `No new GridMap for ${((now - lastFrame) / 1000).toFixed(1)} s` : "";
    } catch (error) {
      status.textContent = error.message;
      byId("browser").textContent = "—";
      byId("orin").textContent = "—";
    }
    await new Promise((resolve) => setTimeout(resolve, 65));
  }
}

run().catch((error) => { status.textContent = error.message; });
