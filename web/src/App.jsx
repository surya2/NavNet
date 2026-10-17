import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { parsePacket, FrameAssembler, MAX_POINTS, POINT_CLOUD, OBSTACLES, PATH, STATS } from './protocol.js';

const WS_PORT = 8080;

function makePoints(scene, size, color, vertexColors) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
  if (vertexColors) geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
  geometry.setDrawRange(0, 0);
  const material = new THREE.PointsMaterial({ size, color, vertexColors, sizeAttenuation: false });
  const points = new THREE.Points(geometry, material);
  points.frustumCulled = false;
  scene.add(points);
  return points;
}

const tmpColor = new THREE.Color();

function applyFrame(points, frame) {
  const position = points.geometry.attributes.position;
  const color = points.geometry.attributes.color;
  const hRange = frame.max[0] - frame.min[0] || 1;
  for (let i = 0; i < frame.count; i++) {
    const h = frame.values[i * 3];
    position.array[i * 3] = frame.values[i * 3 + 1];
    position.array[i * 3 + 1] = h;
    position.array[i * 3 + 2] = -frame.values[i * 3 + 2];
    if (color) {
      tmpColor.setHSL(0.66 - 0.5 * ((h - frame.min[0]) / hRange), 1, 0.5);
      tmpColor.toArray(color.array, i * 3);
    }
  }
  position.needsUpdate = true;
  if (color) color.needsUpdate = true;
  points.geometry.setDrawRange(0, frame.count);
}

export default function App() {
  const mountRef = useRef(null);
  const statusRef = useRef(null);

  useEffect(() => {
    const mount = mountRef.current;
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(mount.clientWidth, mount.clientHeight);
    mount.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xeeeeee);
    scene.add(new THREE.GridHelper(10, 20, 0x999999, 0xcccccc));
    scene.add(new THREE.AxesHelper(0.5));

    const camera = new THREE.PerspectiveCamera(60, mount.clientWidth / mount.clientHeight, 0.01, 100);
    camera.position.set(0, 2, 2);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, -1.5);
    controls.update();

    const layers = [];
    layers[POINT_CLOUD] = makePoints(scene, 2, 0xffffff, true);
    layers[OBSTACLES] = makePoints(scene, 5, 0xff0000, false);
    layers[PATH] = makePoints(scene, 7, 0x000000, false);
    layers[PATH].material.depthTest = false;
    layers[PATH].renderOrder = 1;

    const latest = [null, null, null];
    const info = new Map();
    const entry = (id) => {
      if (!info.has(id)) {
        info.delete((id + 128) & 255);
        info.set(id, { counts: [null, null, null], stats: null });
      }
      return info.get(id);
    };
    let shownId = null;
    let lastStats = null;
    let dirty = false;
    const assembler = new FrameAssembler((type, frame) => {
      latest[type] = frame;
      entry(frame.frameId).counts[type] = frame.count;
      dirty = true;
    });

    let ws;
    let reconnect;
    const connect = () => {
      ws = new WebSocket(`ws://${window.location.hostname}:${WS_PORT}`);
      ws.binaryType = 'arraybuffer';
      ws.onmessage = (event) => {
        const packet = parsePacket(event.data);
        if (!packet) return;
        if (packet.type === STATS) {
          entry(packet.frameId).stats = packet.stats;
          lastStats = packet.stats;
          dirty = true;
        } else {
          assembler.push(packet);
        }
      };
      ws.onclose = () => {
        reconnect = setTimeout(connect, 1000);
      };
    };
    connect();

    let animation;
    const render = () => {
      animation = requestAnimationFrame(render);
      for (let type = 0; type < 3; type++) {
        if (!latest[type]) continue;
        applyFrame(layers[type], latest[type]);
        if (type === POINT_CLOUD) shownId = latest[type].frameId;
        latest[type] = null;
      }
      if (dirty && shownId !== null) {
        const { counts } = info.get(shownId);
        const stats = info.get(shownId).stats ?? lastStats;
        const pts = (i) => (counts[i] === null ? '-' : counts[i]);
        const ms = (i) => (stats ? stats[i].toFixed(1) : '-');
        statusRef.current.textContent = [
          `frame      ${shownId}`,
          `cloud      ${pts(POINT_CLOUD)} pts`,
          `obstacles  ${pts(OBSTACLES)} pts`,
          `path       ${pts(PATH)} pts`,
          '',
          `total      ${ms(0)} ms`,
          `capture    ${ms(1)} ms`,
          `obstacles  ${ms(2)} ms`,
          `path       ${ms(3)} ms`,
        ].join('\n');
        dirty = false;
      }
      renderer.render(scene, camera);
    };
    render();

    const onResize = () => {
      camera.aspect = mount.clientWidth / mount.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(mount.clientWidth, mount.clientHeight);
    };
    window.addEventListener('resize', onResize);

    return () => {
      window.removeEventListener('resize', onResize);
      cancelAnimationFrame(animation);
      clearTimeout(reconnect);
      ws.onclose = null;
      ws.close();
      controls.dispose();
      renderer.dispose();
      mount.removeChild(renderer.domElement);
    };
  }, []);

  return (
    <div ref={mountRef} style={{ width: '100%', height: '100%', position: 'relative' }}>
      <pre
        ref={statusRef}
        style={{ position: 'absolute', top: 8, left: 8, margin: 0, padding: 8, font: '13px monospace', background: 'rgba(255,255,255,0.8)' }}
      />
    </div>
  );
}
