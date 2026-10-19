/* cam_panel.js - injected into index.html by server.py. Adds: STOP button, Camera tab (video, click-to-move,
   XYZ entry + jog, detection + pick, marker calibration, roll calibration, pick settings) and a log that shows on every tab. */
(function () {
  const $ = id => document.getElementById(id);
  let ws, V = null, S = null;
  const send = o => ws && ws.readyState == 1 && ws.send(JSON.stringify(o));

  const css = document.createElement('style');
  css.textContent = `
  #cam h4{margin:0 0 8px;font-size:14px} #cam .col{flex:1 1 340px;display:flex;flex-direction:column;gap:12px;min-width:300px}
  #cam .row label{width:auto} #cam .wrap{flex-wrap:wrap} #cam .help{font-size:13px;margin:4px 0 8px}
  #cvid{width:100%;display:block;border-radius:6px;background:#101215;cursor:crosshair;min-height:220px}
  .chip.err{background:#4a1616;color:var(--bad)} .chips{display:flex;flex-direction:column;gap:6px;align-items:flex-start;margin-bottom:8px}
  #logbox{height:190px;overflow:auto;background:#101215;border-radius:6px;padding:6px 8px;font:12px/1.5 ui-monospace,Consolas,monospace}
  #logbox div{white-space:pre-wrap;border-bottom:1px solid #1a1d22} .l-warn{color:var(--warn)} .l-err{color:var(--bad)} .l-ok{color:var(--ok)} .l-info{color:var(--fg)}
  #stopbtn{background:#7a1f1f;border-color:var(--bad);font-weight:700;color:#fff}
  #cam .mk{display:grid;grid-template-columns:70px 1fr;gap:4px 8px;align-items:center;margin:4px 0}
  #cam details summary{cursor:pointer;color:var(--acc);margin:4px 0} #cam input[type=text]{background:#101215;color:var(--fg);border:1px solid var(--line);border-radius:4px;padding:4px;flex:1}
  #cam .jog button{padding:5px 9px}`;
  document.head.appendChild(css);

  // ---- header: STOP + Camera tab ----
  const stop = document.createElement('button'); stop.id = 'stopbtn'; stop.textContent = 'STOP (Esc)';
  stop.onclick = () => send({ type: 'stop' });
  $('live').after(stop);
  document.addEventListener('keydown', e => { if (e.key == 'Escape') send({ type: 'stop' }); });
  const tab = document.createElement('button'); tab.className = 'tab'; tab.dataset.tab = 'cam'; tab.textContent = 'Camera';
  document.querySelector('.tab[data-tab=cal]').after(tab);

  const SET = [['x_trim', 'X trim (mm)'], ['y_trim', 'Y trim (mm)'], ['object_z', 'Aim height above table (mm)'], ['hover', 'Hover height (mm)'],
    ['z_grasp_offset', 'Grasp z = table z + this (mm)'], ['z_floor', 'Lowest allowed tip z (mm)'],
    ['pick_pitch', 'Pick tool pitch (blank = keep current, -90 = straight down)'], ['grip_open', 'Gripper open value'], ['grip_close', 'Gripper close value'],
    ['drop_x', 'Drop X'], ['drop_y', 'Drop Y'], ['drop_z', 'Drop Z']];

  const cam = document.createElement('div'); cam.id = 'cam'; cam.className = 'views'; cam.style.display = 'none';
  cam.innerHTML = `
  <div class="card" style="flex:2 1 620px">
    <div id="cwarn" class="chips"></div>
    <img id="cvid" alt="camera feed (click a point on the table)">
    <div class="mono" id="cstat" style="margin:6px 0;white-space:pre-wrap;line-height:1.5"></div>
    <div class="row wrap">
      <label><input type="checkbox" data-s="show_grid"> table grid</label>
      <label><input type="checkbox" data-s="show_tip"> arm tip</label>
      <label><input type="checkbox" data-s="show_markers"> marker check</label>
      <button id="clock">Lock pose</button>
    </div>
    <p class="help">Grid lines are table coordinates in mm (X forward, Y left). <b style="color:#ffff00">Cyan cross</b> = where the arm believes its tip is, <b style="color:#ff8c00">orange line</b> = jaw closing direction, <b style="color:#ff00ff">magenta</b> = your click.
    <b>Yellow circles</b> should sit on the green marker centres; a long yellow line means the marker coordinates are wrong.</p>
    <div class="row wrap"><label><input type="checkbox" id="cdet"> Detect objects</label>
      <input type="text" id="cprompt" placeholder="screwdriver, pen, ..."><button id="cpickd" class="on">Pick detected</button></div>
    <div class="mono" id="cdets" style="white-space:pre"></div>
  </div>
  <div class="col">
    <div class="card"><h4>Click target</h4><div class="mono" id="cclick">Click the video to choose a point on the table.</div>
      <div class="row wrap"><button id="chov">Hover above</button><button id="clow">Go to grasp height</button><button id="cpick" class="on">Pick here</button></div></div>
    <div class="card"><h4>Move the tip (mm)</h4><div class="mono" id="ctip"></div><div id="cflag" class="chip warn" style="display:none;margin:6px 0"></div>
      <div class="row wrap"><label>X</label><input type="number" id="cx"><label>Y</label><input type="number" id="cy"><label>Z</label><input type="number" id="cz"><button id="cgo" class="on">Go</button></div>
      <div class="row wrap"><label>Pitch</label><input type="number" id="cpitch"><button id="cpset">Set pitch</button><label><input type="checkbox" id="ckeep" checked> keep pitch on Go</label></div>
      <div class="jog" id="cjog"></div><p class="help">+X forward, +Y left, +Z up. Jog uses the current pitch.</p></div>
    <div class="card"><h4>Marker calibration (measure with the arm)</h4>
      <p class="help">Do this once after the base is fixed. Live ON, then jog the tip onto the <b>centre</b> of a marker and press Record. 4 markers is best; 2 far-apart markers also works (the rest are deduced). Record at the same tool pitch you pick with.</p>
      <div id="cmk"></div>
      <div class="row wrap"><button id="cmapply" class="on">Apply measured markers</button><button id="cmtape">Rebuild from tape</button><button id="cmreset">Reset to old values</button></div>
      <div class="mono" id="cmtapechk" style="white-space:pre-wrap"></div>
      <div class="mono" id="cmfit" style="white-space:pre-wrap"></div></div>
    <div class="card"><h4>Wrist roll calibration</h4>
      <p class="help">1. Turn the roll slider until the jaws close along the arm, press <b>Set zero</b>. 2. Test: type a table angle (0 = along +X) and press Rotate jaws. The orange line on the video should match the real jaws; if the wrist turns the wrong way press Flip.</p>
      <div class="row"><label>Roll</label><input type="range" id="croll" min="0" max="180" step="1" style="flex:1"><span id="crollv"></span></div>
      <div class="row wrap"><button id="crz">Set zero here</button><button id="crf">Flip direction</button>
        <input type="number" id="cja" value="0"><button id="crj">Rotate jaws to table angle</button></div>
      <div class="mono" id="crinfo"></div></div>
    <div class="card"><details><summary>Pick settings</summary><div id="cset"></div></details></div>
  </div>`;
  document.querySelector('main').appendChild(cam);

  const logCard = document.createElement('div'); logCard.className = 'card'; logCard.style.flex = '1 1 100%';
  logCard.innerHTML = `<div class="row"><b>Log</b><button id="logclr">Clear</button><label style="width:auto"><input type="checkbox" id="logw"> warnings only</label></div><div id="logbox"></div>`;
  document.querySelector('main').appendChild(logCard);

  // ---- tabs ----
  const vid = $('cvid');
  function showCam(on) {
    cam.style.display = on ? '' : 'none';
    if (on) { $('move').style.display = 'none'; $('cal').style.display = 'none'; if (!vid.src.endsWith('/video')) vid.src = '/video'; send({ type: 'unlock', on: false }); }
    else if (vid.getAttribute('src')) vid.removeAttribute('src');
  }
  tab.addEventListener('click', () => { document.querySelectorAll('.tab').forEach(x => x.classList.toggle('sel', x == tab)); showCam(true); });
  document.querySelectorAll('.tab:not([data-tab=cam])').forEach(b => b.addEventListener('click', () => showCam(false)));

  // ---- controls ----
  vid.onclick = e => { const r = vid.getBoundingClientRect(); send({ type: 'cam_click', u: (e.clientX - r.left) / r.width, v: (e.clientY - r.top) / r.height }); };
  cam.querySelectorAll('input[data-s]').forEach(c => c.onchange = () => send({ type: 'vset', key: c.dataset.s, value: c.checked }));
  $('clock').onclick = () => send({ type: 'pose_lock', on: !(V && V.locked) });
  const det = () => send({ type: 'detect', on: $('cdet').checked, prompt: $('cprompt').value });
  $('cdet').onchange = det; $('cprompt').onchange = det;
  $('cpickd').onclick = () => send({ type: 'pick_detected' });
  $('chov').onclick = () => send({ type: 'goto_click', mode: 'hover' });
  $('clow').onclick = () => send({ type: 'goto_click', mode: 'low' });
  $('cpick').onclick = () => send({ type: 'pick_click' });
  $('cgo').onclick = () => send({ type: 'tip', x: +$('cx').value, y: +$('cy').value, z: +$('cz').value, lock: $('ckeep').checked, log: true });
  $('cpset').onclick = () => send({ type: 'pitch', pitch: +$('cpitch').value });
  $('cjog').innerHTML = [['X', 'dx'], ['Y', 'dy'], ['Z', 'dz']].map(([n, k]) => `<div class="row"><label style="width:20px">${n}</label>` +
    [-10, -1, 1, 10].map(s => `<button data-k="${k}" data-s="${s}">${s > 0 ? '+' : ''}${s}</button>`).join('') + '</div>').join('');
  $('cjog').onclick = e => { const b = e.target.closest('button'); if (b) send({ type: 'jog', [b.dataset.k]: +b.dataset.s }); };
  $('cmk').innerHTML = [0, 1, 2, 3].map(i => `<div class="mk"><button data-m="${i}">Record ${i}</button><span class="mono" id="cm${i}"></span></div>`).join('');
  $('cmk').onclick = e => { const b = e.target.closest('button'); if (b) send({ type: 'marker_record', id: +b.dataset.m }); };
  $('cmapply').onclick = () => send({ type: 'marker_apply' });
  $('cmreset').onclick = () => { if (confirm('Go back to the old hand-measured marker values?')) send({ type: 'marker_reset' }); };
  $('cmtape').onclick = () => { if (confirm('Rebuild the marker model from the tape measurements?\n\nThe shape comes from the tape; the placement is fitted onto the arm-measured points.')) send({ type: 'marker_tape' }); };
  $('croll').oninput = e => { $('crollv').textContent = e.target.value; send({ type: 'roll', angle: +e.target.value }); };
  $('crz').onclick = () => send({ type: 'roll_zero' });
  $('crf').onclick = () => send({ type: 'roll_flip' });
  $('crj').onclick = () => send({ type: 'jaws', angle: +$('cja').value });
  $('cset').innerHTML = SET.map(([k, l]) => `<div class="row"><label style="width:240px">${l}</label><input type="number" step="any" data-k="${k}"></div>`).join('');
  $('cset').querySelectorAll('input').forEach(i => i.onchange = () => send({ type: 'vset', key: i.dataset.k, value: i.value }));

  // ---- log ----
  const box = $('logbox');
  function addLog(m) {
    const d = document.createElement('div'); d.className = 'l-' + m.lvl; d.dataset.l = m.lvl; d.textContent = m.t + '  ' + m.msg;
    if ($('logw').checked && (m.lvl == 'info')) d.style.display = 'none';
    const stick = box.scrollTop + box.clientHeight >= box.scrollHeight - 30;
    box.appendChild(d); while (box.childNodes.length > 300) box.removeChild(box.firstChild);
    if (stick) box.scrollTop = box.scrollHeight;
  }
  $('logclr').onclick = () => box.innerHTML = '';
  $('logw').onchange = () => box.querySelectorAll('div').forEach(d => d.style.display = ($('logw').checked && d.dataset.l == 'info') ? 'none' : '');

  // ---- rendering ----
  const idle = id => document.activeElement !== $(id);
  function renderState() {
    if (!S) return; const t = S.tool;
    $('ctip').textContent = `target tip  X ${t.x.toFixed(1)}  Y ${t.y.toFixed(1)}  Z ${t.z.toFixed(1)}  |  pitch ${t.pitch.toFixed(0)}  roll ${t.roll.toFixed(0)}  |  ${S.live ? 'Live ON' : 'Live OFF - arm will not move'}`;
    [['cx', t.x], ['cy', t.y], ['cz', t.z], ['cpitch', t.pitch]].forEach(([id, v]) => { if (idle(id)) $(id).value = v.toFixed(1); });
    const f = []; if (S.reach) f.push('IK changed the target to make it reachable'); if (S.limited) f.push('a joint limit was hit');
    $('cflag').style.display = f.length ? '' : 'none'; $('cflag').textContent = f.join(' + ');
    if (idle('croll')) { $('croll').value = t.roll; $('crollv').textContent = Math.round(t.roll); }
  }
  function chip(cls, txt) { const c = document.createElement('div'); c.className = 'chip ' + cls; c.textContent = txt; return c; }
  function renderVision() {
    if (!V) return;
    const w = $('cwarn'); w.innerHTML = '';
    if (!V.enabled) w.appendChild(chip('err', 'Camera off: ' + V.problem));
    else {
      if (V.problem) w.appendChild(chip('err', V.problem));
      V.warn.forEach(x => w.appendChild(chip('warn', x)));
      if (!V.pose) w.appendChild(chip('warn', 'Camera pose not found: all 4 ArUco markers must be visible.'));
      if (!V.measured) w.appendChild(chip('warn', 'Marker positions are still the old hand-measured values. Use "Marker calibration" after moving the base.'));
      if (V.tape_check && !V.tape_check.ok) w.appendChild(chip('err', `Marker model fails the tape check by ${V.tape_check.worst.toFixed(0)} mm. This is a SHAPE error - solvePnP will absorb part of it as camera tilt and every table coordinate will be wrong. Rebuild from tape.`));
      // Pose health decides whether picking is allowed at all, so it leads.
      if (V.pose) w.appendChild(chip(V.healthy ? 'ok' : 'err', (V.healthy ? 'Pose OK - ' : 'POSE UNRELIABLE - ') + V.health));
      if (V.rejects) w.appendChild(chip('warn', `${V.rejects} pose solves rejected in a row - the pose on screen is stale. Check the log for why.`));
      if (V.calib && V.calib.rms != null && V.calib.rms > 1.0) w.appendChild(chip('warn', `Camera calibration RMS is ${V.calib.rms.toFixed(2)} px. Above ~1 px the intrinsics are too coarse for mm work - rerun calibrate_camera.py with a larger board.`));
      if (V.calib && V.calib.rms == null) w.appendChild(chip('warn', 'No camera_calib.json: the stream resolution cannot be checked against the calibration. Rerun calibrate_camera.py.'));
      if (V.busy) w.appendChild(chip('ok', 'Pick running. STOP cancels.'));
    }
    // Two reprojection figures, deliberately both shown: "solve" is the raw per-frame fit,
    // "in use" is the averaged pose actually driving the arm. solve small + in-use large
    // means the averaging is corrupting a good solve, which is a different bug from both
    // being large (wrong marker coordinates or wrong intrinsics).
    const cal = V.calib && V.calib.res ? `calib ${V.calib.res.join('x')}${V.calib.rms != null ? ` rms ${V.calib.rms.toFixed(2)}px` : ''}` : 'calib unknown';
    $('cstat').textContent =
      `${V.res.join('x')}  ${V.fps} fps  ${cal}  markers seen: ${V.seen.join(', ') || 'none'}\n` +
      `table plane z = ${V.plane[0]}x + ${V.plane[1]}y + ${V.plane[2]}` +
      (V.cam_pos ? `   camera at X${V.cam_pos[0]} Y${V.cam_pos[1]} Z${V.cam_pos[2]}` : '') +
      (V.diag ? `\n${V.diag}` : '');
    cam.querySelectorAll('input[data-s]').forEach(c => c.checked = !!V.settings[c.dataset.s]);
    $('clock').textContent = V.locked ? 'Unlock pose' : 'Lock pose';
    if (idle('cdet')) $('cdet').checked = V.detect;
    if (idle('cprompt')) $('cprompt').value = V.prompts.join(', ');
    $('cdets').textContent = V.dets.length ? V.dets.map(d => `${d.name} ${d.conf}  X${d.x} Y${d.y}` + (d.closing != null ? `  jaws ${d.closing}°` : '')).join('\n') : (V.detect ? 'no detections' : '');
    $('cclick').textContent = V.click ? `Target  X ${V.click.x}  Y ${V.click.y}  (table z ${V.click.z})` : 'Click the video to choose a point on the table.';
    [0, 1, 2, 3].forEach(i => {
      const m = V.markers[i], t = V.touched[i], e = V.err[i];
      // touched entries are [x, y, z, tool_pitch, roll]; pitch/roll are absent on older records
      const pose = t && t.length > 4 && t[3] != null ? `  @pitch ${t[3]}° roll ${t[4]}` : '';
      $('cm' + i).textContent = (t ? `measured ${t.slice(0, 3).join(', ')}${pose}` : `stored ${m.join(', ')}`)
        + (e != null ? `   ${e.toFixed(1)} px` : '');
    });
    const tc = V.tape_check;
    if (!tc) {
      $('cmtapechk').textContent = 'No tape measurements in vision.json. Measuring the six distances between marker centres by hand is the only check on the arm that does not use the arm.';
    } else {
      $('cmtapechk').textContent = 'tape vs model (shape only - independent of where the model sits)\n'
        + tc.rows.map(r => `  ${r.pair}   tape ${r.tape.toFixed(0).padStart(5)} mm   model ${r.model.toFixed(1).padStart(6)} mm   ${r.diff > 0 ? '+' : ''}${r.diff.toFixed(1)}`).join('\n')
        + `\n  worst ${tc.worst.toFixed(1)} mm  ${tc.ok ? '- OK' : '- SHAPE ERROR, no trim can fix this'}`;
    }
    $('cmfit').textContent = V.fit_info;
    $('crinfo').textContent = `roll zero = servo ${V.roll.zero}   servo up turns jaws ${V.roll.sign > 0 ? 'CCW' : 'CW'} (from above)` + (V.jaw != null ? `   jaws now close along ${V.jaw.toFixed(0)}° (table)` : '');
    $('cset').querySelectorAll('input').forEach(i => { if (document.activeElement !== i) { const v = V.settings[i.dataset.k]; i.value = v == null ? '' : v; } });
  }

  // ---- own websocket (the server broadcasts to every client) ----
  function connect() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onclose = () => setTimeout(connect, 1000);
    ws.onmessage = e => {
      const m = JSON.parse(e.data);
      if (m.type == 'log') addLog(m); else if (m.type == 'vision') { V = m; renderVision(); } else if (m.type == 'state') { S = m; renderState(); }
    };
  }
  connect();
})();