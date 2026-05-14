#!/usr/bin/env python3
"""
3D Web Visualizer — dual-panel Three.js visualization.
Left: Robot's view (occupancy grid as 3D voxels, path, lidar rays)
Right: Reality (true room geometry, robot, lidar rays, path)
HTTP server on port 8080.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
import threading
import json
import math
from http.server import HTTPServer, BaseHTTPRequestHandler

STATE = {
    'map': {'width': 200, 'height': 200, 'resolution': 0.05,
            'origin_x': -5.0, 'origin_y': -5.0, 'data': [], 'revision': 0},
    'robot': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    'path': [],
    'goal': None,
    'trail': [],
    'scan': {'ranges': [], 'angle_min': 0.0, 'angle_max': 6.283,
             'angle_increment': 0.01745, 'range_max': 3.5},
    'walls': [
        [-4, -4,  4, -4],
        [ 4, -4,  4,  4],
        [ 4,  4, -4,  4],
        [-4,  4, -4, -4],
        [-1.5, -1.5, -1.5,  0.5],
        [ 1.0, -2.5,  1.0, -0.5],
        [ 0.5,  2.0,  2.5,  2.0],
    ],
}

HTML_PAGE = open('/home/client/projects/ros_robot/ros2_ws/src/global_planner/src/viz.html').read() if False else ""


class WebVizNode(Node):
    def __init__(self):
        super().__init__('web_viz_node')
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/global_plan', self.path_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.trail_counter = 0
        self.get_logger().info('Web Viz 3D started on http://0.0.0.0:8080')

    def map_cb(self, msg):
        STATE['map']['width'] = msg.info.width
        STATE['map']['height'] = msg.info.height
        STATE['map']['resolution'] = msg.info.resolution
        STATE['map']['origin_x'] = msg.info.origin.position.x
        STATE['map']['origin_y'] = msg.info.origin.position.y
        STATE['map']['data'] = list(msg.data)
        STATE['map']['revision'] += 1

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        STATE['robot'] = {'x': x, 'y': y, 'yaw': yaw}
        self.trail_counter += 1
        if self.trail_counter % 5 == 0:
            STATE['trail'].append([x, y])
            if len(STATE['trail']) > 2000:
                STATE['trail'] = STATE['trail'][-1000:]

    def path_cb(self, msg):
        STATE['path'] = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

    def scan_cb(self, msg):
        STATE['scan'] = {
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_max': msg.range_max,
        }

    def publish_goal(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = x
        msg.pose.position.y = y
        self.goal_pub.publish(msg)
        STATE['goal'] = [x, y]
        STATE['trail'] = []
        self.get_logger().info(f'Goal published: ({x:.2f}, {y:.2f})')


viz_node = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(get_html().encode())
        elif self.path == '/state':
            # Лёгкий эндпоинт без map.data — клиент опрашивает часто (50 мс)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            s = {
                'robot': STATE['robot'],
                'path': STATE['path'],
                'goal': STATE['goal'],
                'trail': STATE['trail'][-500:],
                'walls': STATE['walls'],
                'scan': STATE['scan'],
                'map_revision': STATE['map']['revision'],
                'map_meta': {
                    'width': STATE['map']['width'],
                    'height': STATE['map']['height'],
                    'resolution': STATE['map']['resolution'],
                    'origin_x': STATE['map']['origin_x'],
                    'origin_y': STATE['map']['origin_y'],
                },
            }
            self.wfile.write(json.dumps(s).encode())
        elif self.path == '/map':
            # Тяжёлый эндпоинт с полной картой — клиент тянет только при
            # изменении revision
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({
                'revision': STATE['map']['revision'],
                'width': STATE['map']['width'],
                'height': STATE['map']['height'],
                'resolution': STATE['map']['resolution'],
                'origin_x': STATE['map']['origin_x'],
                'origin_y': STATE['map']['origin_y'],
                'data': STATE['map']['data'],
            }).encode())
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/goal':
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            if viz_node:
                viz_node.publish_goal(body['x'], body['y'])
            self.send_response(200)
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(b'ok')
        else:
            self.send_response(404)
            self.end_headers()


def get_html():
    return HTML_CONTENT


def main(args=None):
    global viz_node
    rclpy.init(args=args)
    viz_node = WebVizNode()
    server = HTTPServer(('0.0.0.0', 8080), Handler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    rclpy.spin(viz_node)
    server.shutdown()
    viz_node.destroy_node()
    rclpy.shutdown()


HTML_CONTENT = r"""<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>ROS2 A* Navigator — 3D</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0a1a;color:#ccc;font-family:'Segoe UI',sans-serif;overflow:hidden}
#header{height:36px;display:flex;align-items:center;justify-content:center;gap:30px;
  background:#111;border-bottom:1px solid #333;font-size:13px}
#header .label{color:#0ff;font-weight:bold}
#status{color:#888;font-size:12px}
#container{display:flex;height:calc(100vh - 36px)}
.panel{flex:1;position:relative;border-right:1px solid #222}
.panel:last-child{border-right:none}
.panel-title{position:absolute;top:8px;left:50%;transform:translateX(-50%);
  z-index:10;background:rgba(0,0,0,0.7);padding:3px 12px;border-radius:4px;
  font-size:12px;color:#0ff;pointer-events:none}
canvas{display:block;width:100%!important;height:100%!important}
</style>
</head><body>
<div id="header">
  <span class="label">ROS2 A* Navigator</span>
  <span id="status">Connecting...</span>
</div>
<div id="container">
  <div class="panel" id="left-panel">
    <div class="panel-title">ROBOT VIEW — Occupancy Grid</div>
  </div>
  <div class="panel" id="right-panel">
    <div class="panel-title">REALITY — 3D Environment (click to set goal)</div>
  </div>
</div>

<script type="importmap">
{
  "imports": {
    "three": "https://unpkg.com/three@0.160.0/build/three.module.js",
    "three/addons/": "https://unpkg.com/three@0.160.0/examples/jsm/"
  }
}
</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

let state = null;
let mapData = null;       // { width, height, resolution, origin_x, origin_y, data }
let lastMapRev = -1;

// ===================== HELPERS =====================
function createScene(container, cameraPos, lookAt) {
  const scene = new THREE.Scene();
  const w = container.clientWidth, h = container.clientHeight;
  const camera = new THREE.PerspectiveCamera(50, w / h, 0.1, 100);
  camera.position.set(...cameraPos);
  camera.lookAt(...lookAt);

  const renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
  renderer.setSize(w, h);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x0a0a1a);
  container.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.1;
  controls.target.set(...lookAt);

  // Lights
  scene.add(new THREE.AmbientLight(0x404060, 1.5));
  const dl = new THREE.DirectionalLight(0xffffff, 1.0);
  dl.position.set(5, 10, 5);
  scene.add(dl);

  return {scene, camera, renderer, controls};
}

function makeRobot(color = 0x00ff88) {
  const g = new THREE.Group();
  // Body cylinder
  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(0.15, 0.15, 0.2, 16),
    new THREE.MeshPhongMaterial({color, transparent: true, opacity: 0.9})
  );
  body.rotation.x = 0; // upright
  body.position.y = 0.1;
  g.add(body);
  // Direction arrow
  const arrow = new THREE.Mesh(
    new THREE.ConeGeometry(0.06, 0.2, 8),
    new THREE.MeshPhongMaterial({color: 0xffffff})
  );
  arrow.rotation.z = -Math.PI / 2;
  arrow.position.set(0.2, 0.1, 0);
  g.add(arrow);
  return g;
}

function makeGrid() {
  const grid = new THREE.GridHelper(10, 40, 0x222244, 0x111133);
  grid.position.y = -0.01;
  return grid;
}

// ===================== LEFT PANEL: Robot View =====================
const leftPanel = document.getElementById('left-panel');
const L = createScene(leftPanel, [0, 8, 0.1], [0, 0, 0]);

// Grid
L.scene.add(makeGrid());

// Robot
const lRobot = makeRobot(0x00ff88);
L.scene.add(lRobot);

// Map mesh — flat plane показывает только free/unknown,
// стены отрисовываются 3D-кубиками (см. wallMesh ниже)
const mapCanvas = document.createElement('canvas');
mapCanvas.width = 200; mapCanvas.height = 200;
const mapCtx = mapCanvas.getContext('2d');
const mapTexture = new THREE.CanvasTexture(mapCanvas);
mapTexture.magFilter = THREE.NearestFilter;
mapTexture.minFilter = THREE.NearestFilter;
const mapMesh = new THREE.Mesh(
  new THREE.PlaneGeometry(10, 10),
  new THREE.MeshBasicMaterial({map: mapTexture, transparent: true, opacity: 0.7})
);
mapMesh.rotation.x = -Math.PI / 2;
mapMesh.position.y = 0.001;
L.scene.add(mapMesh);
let mapDirty = false;

// Wall voxels — один долгоживущий InstancedMesh, переиспользуется
const WALL_CAPACITY = 8000;
const wallGeo = new THREE.BoxGeometry(0.05, 0.3, 0.05);
const wallMat = new THREE.MeshPhongMaterial({color: 0xff3355});
const wallMesh = new THREE.InstancedMesh(wallGeo, wallMat, WALL_CAPACITY);
wallMesh.count = 0;
wallMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
L.scene.add(wallMesh);
const wallDummy = new THREE.Object3D();

// Path line
let lPathLine = null;
// Lidar lines
let lLidarLines = null;
// Trail line
let lTrailLine = null;
// Goal marker
let lGoalMarker = null;

function updateLeftPanel() {
  if (!state) return;
  const r = state.robot;

  // Robot
  lRobot.position.set(r.x, 0.1, -r.y);
  lRobot.rotation.y = r.yaw;

  // Карта обновляется только когда пришли новые данные (mapDirty),
  // а не каждый кадр — иначе createImageData(200,200) жрёт CPU зря
  const m = mapData;
  if (mapDirty && m && m.data && m.data.length > 0) {
    mapDirty = false;
    const img = mapCtx.createImageData(m.width, m.height);
    let wallCount = 0;
    for (let y = 0; y < m.height; y++) {
      for (let x = 0; x < m.width; x++) {
        const si = (m.height - 1 - y) * m.width + x;
        const di = (y * m.width + x) * 4;
        const v = m.data[si];
        // Стены НЕ рисуем на текстуре — они идут 3D-кубиками
        if (v === -1) { img.data[di]=20; img.data[di+1]=20; img.data[di+2]=40; img.data[di+3]=180; }
        else if (v === 0) { img.data[di]=180; img.data[di+1]=200; img.data[di+2]=180; img.data[di+3]=180; }
        else { img.data[di]=180; img.data[di+1]=200; img.data[di+2]=180; img.data[di+3]=180; wallCount++; }
      }
    }
    mapCtx.putImageData(img, 0, 0);
    mapTexture.needsUpdate = true;

    // Кубики стен — переиспользуем тот же InstancedMesh
    let wi = 0;
    for (let y = 0; y < m.height && wi < WALL_CAPACITY; y++) {
      for (let x = 0; x < m.width && wi < WALL_CAPACITY; x++) {
        if (m.data[y * m.width + x] === 100) {
          const wx = m.origin_x + x * m.resolution + m.resolution / 2;
          const wy = m.origin_y + y * m.resolution + m.resolution / 2;
          wallDummy.position.set(wx, 0.15, -wy);
          wallDummy.updateMatrix();
          wallMesh.setMatrixAt(wi++, wallDummy.matrix);
        }
      }
    }
    wallMesh.count = wi;
    wallMesh.instanceMatrix.needsUpdate = true;
  }

  // Path
  if (lPathLine) L.scene.remove(lPathLine);
  if (state.path.length > 1) {
    const pts = state.path.map(p => new THREE.Vector3(p[0], 0.05, -p[1]));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    lPathLine = new THREE.Line(geo, new THREE.LineBasicMaterial({color: 0x00aaff, linewidth: 2}));
    L.scene.add(lPathLine);
  }

  // Trail
  if (lTrailLine) L.scene.remove(lTrailLine);
  if (state.trail.length > 1) {
    const pts = state.trail.map(p => new THREE.Vector3(p[0], 0.02, -p[1]));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    lTrailLine = new THREE.Line(geo, new THREE.LineBasicMaterial({color: 0xffff00, transparent: true, opacity: 0.5}));
    L.scene.add(lTrailLine);
  }

  // Lidar
  if (lLidarLines) L.scene.remove(lLidarLines);
  const sc = state.scan;
  if (sc.ranges && sc.ranges.length > 0) {
    const pts = [];
    for (let i = 0; i < sc.ranges.length; i += 2) {
      const range = sc.ranges[i];
      if (range >= sc.range_max) continue;
      const a = r.yaw + sc.angle_min + i * sc.angle_increment;
      pts.push(new THREE.Vector3(r.x, 0.08, -r.y));
      pts.push(new THREE.Vector3(r.x + range * Math.cos(a), 0.08, -(r.y + range * Math.sin(a))));
    }
    if (pts.length > 0) {
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      lLidarLines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({color: 0xff6600, transparent: true, opacity: 0.3}));
      L.scene.add(lLidarLines);
    }
  }

  // Goal
  if (lGoalMarker) L.scene.remove(lGoalMarker);
  if (state.goal) {
    const g = new THREE.Group();
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.15, 0.02, 8, 24),
      new THREE.MeshPhongMaterial({color: 0xff00ff, emissive: 0x880088})
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(state.goal[0], 0.05, -state.goal[1]);
    g.add(ring);
    lGoalMarker = g;
    L.scene.add(lGoalMarker);
  }
}

// ===================== RIGHT PANEL: Reality =====================
const rightPanel = document.getElementById('right-panel');
const R = createScene(rightPanel, [6, 6, 6], [0, 0, 0]);

// Grid + floor
R.scene.add(makeGrid());
const floor = new THREE.Mesh(
  new THREE.PlaneGeometry(10, 10),
  new THREE.MeshPhongMaterial({color: 0x1a1a2e, side: THREE.DoubleSide})
);
floor.rotation.x = -Math.PI / 2;
floor.position.y = -0.02;
R.scene.add(floor);

// Build real walls
function buildWalls(walls) {
  const wallMat = new THREE.MeshPhongMaterial({color: 0x4466aa, transparent: true, opacity: 0.8});
  walls.forEach(([x1, y1, x2, y2]) => {
    const dx = x2 - x1, dy = y2 - y1;
    const len = Math.sqrt(dx * dx + dy * dy);
    const angle = Math.atan2(dy, dx);
    const geo = new THREE.BoxGeometry(len, 0.5, 0.08);
    const mesh = new THREE.Mesh(geo, wallMat);
    mesh.position.set((x1 + x2) / 2, 0.25, -(y1 + y2) / 2);
    mesh.rotation.y = -angle;
    R.scene.add(mesh);
    // Top edge glow
    const edgeGeo = new THREE.BoxGeometry(len, 0.02, 0.1);
    const edgeMat = new THREE.MeshPhongMaterial({color: 0x88aaff, emissive: 0x334466});
    const edge = new THREE.Mesh(edgeGeo, edgeMat);
    edge.position.set((x1 + x2) / 2, 0.5, -(y1 + y2) / 2);
    edge.rotation.y = -angle;
    R.scene.add(edge);
  });
}

// Robot
const rRobot = makeRobot(0x00ff88);
R.scene.add(rRobot);

// Path, trail, lidar, goal for right panel
let rPathLine = null, rTrailLine = null, rLidarLines = null, rGoalMarker = null;
let wallsBuilt = false;

// Raycaster for click-to-goal
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

rightPanel.addEventListener('click', (e) => {
  const rect = rightPanel.getBoundingClientRect();
  mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(mouse, R.camera);
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const pt = new THREE.Vector3();
  raycaster.ray.intersectPlane(plane, pt);
  if (pt) {
    const wx = pt.x, wy = -pt.z;
    fetch('/goal', {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({x: wx, y: wy})});
    document.getElementById('status').textContent = `Goal: (${wx.toFixed(2)}, ${wy.toFixed(2)})`;
  }
});

function updateRightPanel() {
  if (!state) return;

  // Build walls once
  if (!wallsBuilt && state.walls) {
    buildWalls(state.walls);
    wallsBuilt = true;
  }

  const r = state.robot;
  rRobot.position.set(r.x, 0.1, -r.y);
  rRobot.rotation.y = r.yaw;

  // Path
  if (rPathLine) R.scene.remove(rPathLine);
  if (state.path.length > 1) {
    const pts = state.path.map(p => new THREE.Vector3(p[0], 0.03, -p[1]));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    rPathLine = new THREE.Line(geo, new THREE.LineBasicMaterial({color: 0x00aaff}));
    R.scene.add(rPathLine);
  }

  // Trail
  if (rTrailLine) R.scene.remove(rTrailLine);
  if (state.trail.length > 1) {
    const pts = state.trail.map(p => new THREE.Vector3(p[0], 0.02, -p[1]));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    rTrailLine = new THREE.Line(geo, new THREE.LineBasicMaterial({color: 0xffff00, transparent: true, opacity: 0.4}));
    R.scene.add(rTrailLine);
  }

  // Lidar rays
  if (rLidarLines) R.scene.remove(rLidarLines);
  const sc = state.scan;
  if (sc.ranges && sc.ranges.length > 0) {
    const pts = [];
    for (let i = 0; i < sc.ranges.length; i += 2) {
      const range = sc.ranges[i];
      if (range >= sc.range_max) continue;
      const a = r.yaw + sc.angle_min + i * sc.angle_increment;
      pts.push(new THREE.Vector3(r.x, 0.1, -r.y));
      pts.push(new THREE.Vector3(r.x + range * Math.cos(a), 0.1, -(r.y + range * Math.sin(a))));
    }
    if (pts.length > 0) {
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      rLidarLines = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({color: 0xff4400, transparent: true, opacity: 0.25}));
      R.scene.add(rLidarLines);
    }
  }

  // Goal
  if (rGoalMarker) R.scene.remove(rGoalMarker);
  if (state.goal) {
    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(0.2, 0.03, 8, 24),
      new THREE.MeshPhongMaterial({color: 0xff00ff, emissive: 0xaa00aa})
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(state.goal[0], 0.05, -state.goal[1]);
    rGoalMarker = ring;
    R.scene.add(rGoalMarker);
  }
}

// ===================== MAIN LOOP =====================
async function poll() {
  try {
    const res = await fetch('/state');
    state = await res.json();
    if (state.map_revision !== lastMapRev) {
      lastMapRev = state.map_revision;
      const mres = await fetch('/map');
      mapData = await mres.json();
      mapDirty = true;
    }
    const r = state.robot;
    document.getElementById('status').textContent =
      `Robot: (${r.x.toFixed(2)}, ${r.y.toFixed(2)}) | Yaw: ${(r.yaw*180/Math.PI).toFixed(0)} | Path: ${state.path.length} pts`;
  } catch(e) {
    document.getElementById('status').textContent = 'Disconnected...';
  }
}
setInterval(poll, 60);

function animate() {
  requestAnimationFrame(animate);
  if (state) {
    updateLeftPanel();
    updateRightPanel();
  }
  L.controls.update();
  R.controls.update();
  L.renderer.render(L.scene, L.camera);
  R.renderer.render(R.scene, R.camera);
}
animate();

// Resize
window.addEventListener('resize', () => {
  [L, R].forEach((v, i) => {
    const c = i === 0 ? leftPanel : rightPanel;
    const w = c.clientWidth, h = c.clientHeight;
    v.camera.aspect = w / h;
    v.camera.updateProjectionMatrix();
    v.renderer.setSize(w, h);
  });
});
// Trigger initial resize
setTimeout(() => window.dispatchEvent(new Event('resize')), 100);

</script>
</body></html>"""


if __name__ == '__main__':
    main()
