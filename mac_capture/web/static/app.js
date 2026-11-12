(() => {
  'use strict';

  // ---------- three.js live point cloud viewer ----------
  const canvas = document.getElementById('three-canvas');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x050607);

  const camera = new THREE.PerspectiveCamera(60, 1, 0.05, 2000);
  camera.up.set(0, 0, 1);
  camera.position.set(6, -6, 4);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;

  scene.add(new THREE.AxesHelper(1));

  const liveGeometry = new THREE.BufferGeometry();
  const liveMaterial = new THREE.PointsMaterial({ size: 0.03, vertexColors: true });
  const liveCloud = new THREE.Points(liveGeometry, liveMaterial);
  scene.add(liveCloud);

  let fitted = false;

  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / Math.max(h, 1);
    camera.updateProjectionMatrix();
  }
  window.addEventListener('resize', resize);
  resize();

  function fitCameraTo(positions, n) {
    let minX = Infinity, minY = Infinity, minZ = Infinity;
    let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (let i = 0; i < n; i++) {
      const x = positions[i * 3], y = positions[i * 3 + 1], z = positions[i * 3 + 2];
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
      if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
    }
    const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, cz = (minZ + maxZ) / 2;
    const radius = Math.max(
      Math.hypot(maxX - minX, maxY - minY, maxZ - minZ) / 2, 1.0
    );
    controls.target.set(cx, cy, cz);
    camera.position.set(cx + radius * 1.4, cy - radius * 1.4, cz + radius * 1.0);
    camera.updateProjectionMatrix();
  }

  const viewerBadge = document.getElementById('viewer-badge');

  function setBadge(connected) {
    viewerBadge.textContent = connected ? 'LIVE' : 'NO SIGNAL';
    viewerBadge.classList.toggle('badge-live', connected);
    viewerBadge.classList.toggle('badge-down', !connected);
  }

  async function pollLive() {
    try {
      const resp = await fetch('/api/live', { cache: 'no-store' });
      const buf = await resp.arrayBuffer();
      const dv = new DataView(buf);
      const n = dv.getUint32(0, true);
      const connected = dv.getUint32(4, true) === 1;
      setBadge(connected);
      // header is 8 bytes (point count + connected flag) so the position array below starts on
      // a 4-byte boundary, which Float32Array requires.
      if (n > 0) {
        const positions = new Float32Array(buf, 8, n * 3);
        const colorsU8 = new Uint8Array(buf, 8 + n * 3 * 4, n * 3);
        const colors = new Float32Array(n * 3);
        for (let i = 0; i < colors.length; i++) colors[i] = colorsU8[i] / 255;
        liveGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        liveGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        liveGeometry.computeBoundingSphere();
        if (!fitted) { fitCameraTo(positions, n); fitted = true; }
      } else {
        liveGeometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(0), 3));
        liveGeometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(0), 3));
      }
    } catch (e) {
      setBadge(false);
    } finally {
      setTimeout(pollLive, 150);
    }
  }
  pollLive();

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();

  // ---------- settings form ----------
  const fields = {
    lidar_ip: document.getElementById('f-lidar-ip'),
    host_ip: document.getElementById('f-host-ip'),
    delay: document.getElementById('f-delay'),
    accum: document.getElementById('f-accum'),
    output_dir: document.getElementById('f-output-dir'),
    prefix: document.getElementById('f-prefix'),
  };

  function readForm() {
    return {
      lidar_ip: fields.lidar_ip.value.trim(),
      host_ip: fields.host_ip.value.trim(),
      delay: parseFloat(fields.delay.value),
      accum: parseFloat(fields.accum.value),
      output_dir: fields.output_dir.value.trim(),
      prefix: fields.prefix.value.trim(),
    };
  }

  function fillForm(s) {
    fields.lidar_ip.value = s.lidar_ip;
    fields.host_ip.value = s.host_ip;
    fields.delay.value = s.delay;
    fields.accum.value = s.accum;
    fields.output_dir.value = s.output_dir;
    fields.prefix.value = s.prefix;
  }

  async function loadSettings() {
    const r = await fetch('/api/settings');
    const s = await r.json();
    fillForm(s);
    loadGallery();
  }

  // ---------- output folder picker ----------
  const browseModal = document.getElementById('browse-modal');
  const browsePathEl = document.getElementById('browse-path');
  const browseListEl = document.getElementById('browse-list');
  const browseUpBtn = document.getElementById('browse-up');
  let browseCurrentPath = null;
  let browseParentPath = null;

  function joinPath(dir, name) {
    return dir.endsWith('/') ? dir + name : dir + '/' + name;
  }

  async function navigateBrowse(path) {
    const r = await fetch('/api/browse?path=' + encodeURIComponent(path));
    const data = await r.json();
    browseCurrentPath = data.path;
    browseParentPath = data.parent;
    browsePathEl.textContent = data.path;
    browseUpBtn.disabled = !data.parent;
    browseListEl.innerHTML = '';
    if (!data.dirs.length) {
      const empty = document.createElement('div');
      empty.className = 'browse-empty';
      empty.textContent = '(하위 폴더 없음)';
      browseListEl.appendChild(empty);
      return;
    }
    for (const name of data.dirs) {
      const item = document.createElement('div');
      item.className = 'browse-item';
      item.textContent = '\u{1F4C1} ' + name;
      item.addEventListener('click', () => navigateBrowse(joinPath(data.path, name)));
      browseListEl.appendChild(item);
    }
  }

  document.getElementById('btn-browse').addEventListener('click', () => {
    browseModal.classList.remove('hidden');
    navigateBrowse(fields.output_dir.value.trim() || '/');
  });
  browseUpBtn.addEventListener('click', () => {
    if (browseParentPath) navigateBrowse(browseParentPath);
  });
  document.getElementById('browse-cancel').addEventListener('click', () => {
    browseModal.classList.add('hidden');
  });
  document.getElementById('browse-select').addEventListener('click', () => {
    fields.output_dir.value = browseCurrentPath;
    browseModal.classList.add('hidden');
    loadGallery();
  });
  browseModal.addEventListener('click', (e) => {
    if (e.target === browseModal) browseModal.classList.add('hidden');
  });

  // ---------- capture ----------
  const btnStart = document.getElementById('btn-start');
  const statusLine = document.getElementById('status-line');
  const statusBarFill = document.getElementById('status-bar-fill');
  let statusTimer = null;

  const STATE_LABEL = {
    idle: 'idle', waiting: 'starting in…', recording: 'recording…',
    done: 'done', error: 'error',
  };

  function renderStatus(s) {
    if (s.state === 'waiting' || s.state === 'recording') {
      const total = (parseFloat(fields.delay.value) || 0) + (parseFloat(fields.accum.value) || 0);
      const pct = total > 0 ? Math.min(100, (s.elapsed / total) * 100) : 0;
      statusBarFill.style.width = pct + '%';
      const label = s.state === 'waiting' ? 'starting in' : 'recording, ends in';
      statusLine.textContent = `${label} ${s.remaining.toFixed(1)}s` +
        (s.state === 'recording' ? ` — ${s.pkts} pkts / ${s.pts} pts` : '');
    } else if (s.state === 'done') {
      statusBarFill.style.width = '100%';
      statusLine.textContent = `done — ${s.pts} pts saved`;
    } else if (s.state === 'error') {
      statusBarFill.style.width = '0%';
      statusLine.textContent = `error: ${s.error}`;
    } else {
      statusBarFill.style.width = '0%';
      statusLine.textContent = STATE_LABEL.idle;
    }
  }

  async function pollStatus() {
    const r = await fetch('/api/capture/status');
    const s = await r.json();
    renderStatus(s);
    if (s.state === 'waiting' || s.state === 'recording') {
      statusTimer = setTimeout(pollStatus, 300);
    } else {
      btnStart.disabled = false;
      if (s.state === 'done') loadGallery();
    }
  }

  btnStart.addEventListener('click', async () => {
    btnStart.disabled = true;
    const params = readForm();
    const r = await fetch('/api/capture/start', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(params),
    });
    if (r.status === 409) {
      btnStart.disabled = false;
      alert('이미 캡처가 진행 중입니다.');
      return;
    }
    clearTimeout(statusTimer);
    pollStatus();
  });

  // ---------- gallery ----------
  const galleryGrid = document.getElementById('gallery-grid');
  const lightbox = document.getElementById('lightbox');
  const lightboxImg = document.getElementById('lightbox-img');
  const lightboxMeta = document.getElementById('lightbox-meta');

  function openLightbox(item) {
    lightboxImg.src = item.thumb_url || '';
    const parts = [item.name, item.timestamp, `${item.point_count} pts`];
    if (item.delay != null) parts.push(`delay ${item.delay}s`);
    if (item.accum != null) parts.push(`accum ${item.accum}s`);
    lightboxMeta.textContent = parts.join('  ·  ');
    lightbox.classList.remove('hidden');
  }

  document.getElementById('lightbox-close').addEventListener('click', () => {
    lightbox.classList.add('hidden');
  });
  lightbox.addEventListener('click', (e) => {
    if (e.target === lightbox) lightbox.classList.add('hidden');
  });

  async function loadGallery() {
    const dir = fields.output_dir.value.trim();
    if (!dir) return;
    galleryGrid.innerHTML = '<div id="gallery-empty">loading…</div>';
    try {
      const r = await fetch('/api/captures?dir=' + encodeURIComponent(dir));
      const items = await r.json();
      galleryGrid.innerHTML = '';
      if (!items.length) {
        galleryGrid.innerHTML = '<div id="gallery-empty">no captures yet</div>';
        return;
      }
      for (const item of items) {
        const el = document.createElement('div');
        el.className = 'thumb';
        const img = document.createElement('img');
        img.src = item.thumb_url || '';
        img.loading = 'lazy';
        const cap = document.createElement('div');
        cap.className = 'cap';
        cap.textContent = item.name;
        el.appendChild(img);
        el.appendChild(cap);
        el.addEventListener('click', () => openLightbox(item));
        galleryGrid.appendChild(el);
      }
    } catch (e) {
      galleryGrid.innerHTML = '<div id="gallery-empty">failed to load</div>';
    }
  }

  document.getElementById('btn-refresh').addEventListener('click', loadGallery);
  fields.output_dir.addEventListener('change', loadGallery);

  loadSettings();
  pollStatus();
})();
