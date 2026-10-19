/* cam_panel.js - injected into index.html by server.py. Adds: STOP button, Camera tab (video, click-to-move,
   XYZ entry + jog, detection + pick, marker calibration, roll calibration, pick settings) and a log that shows on every tab.
   Plain ES5 like index.html (Android 4.2 WebView), and it shares index.html's connection through window.ARM. */
(function () {
  function $(id) { return document.getElementById(id); }
  var A = window.ARM, send = A.send, each = A.each, txt = A.txt, V = null, S = null, sDirty = false, vDirty = false;
  function pad(s, n) { s = String(s); while (s.length < n) s = ' ' + s; return s; }
  function after(ref, el) { ref.parentNode.insertBefore(el, ref.nextSibling); }

  var css = document.createElement('style');
  css.textContent = [
    '#cam h4{margin:0 0 8px;font-size:14px} #cam .col{flex:1 1 340px;display:flex;flex-direction:column;gap:12px;min-width:300px}',
    '#cam .row label{width:auto} #cam .wrap{flex-wrap:wrap} #cam .help{font-size:13px;margin:4px 0 8px}',
    '#cvid{width:100%;display:block;border-radius:6px;background:#101215;cursor:crosshair;min-height:220px}',
    '.chip.err{background:#4a1616;color:#ef5350} .chips{display:flex;flex-direction:column;gap:6px;align-items:flex-start;margin-bottom:8px}',
    '#logbox{height:190px;overflow:auto;background:#101215;border-radius:6px;padding:6px 8px;font:12px/1.5 ui-monospace,Consolas,monospace}',
    '#logbox div{white-space:pre-wrap;border-bottom:1px solid #1a1d22} .l-warn{color:#f0a030} .l-err{color:#ef5350} .l-ok{color:#38c172} .l-info{color:#e6e8eb}',
    '#stopbtn{background:#7a1f1f;border-color:#ef5350;font-weight:700;color:#fff}',
    '#cam .mk{display:grid;grid-template-columns:70px 1fr;gap:4px 8px;align-items:center;margin:4px 0}',
    '#cam details summary{cursor:pointer;color:#4ea1ff;margin:4px 0} #cam input[type=text]{background:#101215;color:#e6e8eb;border:1px solid #2c3138;border-radius:4px;padding:4px;flex:1}',
    '#cam .jog button{padding:5px 9px}',
    // no flex-wrap (see index.html): video left, controls right, as inline-blocks
    '.noflex #cam .camv{display:inline-block;vertical-align:top;width:58%;margin-right:1%}',
    '.noflex #cam .col{display:inline-block;vertical-align:top;width:40%;min-width:0}',
    '.noflex #cam .col>.card{margin-bottom:12px} .noflex .chips .chip{display:table;margin-bottom:6px}',
    '.noflex #cam .mk{display:block} .noflex #cam .mk>*{display:inline-block;vertical-align:middle;margin-right:8px}',
    '.noflex #cam input[type=text]{width:50%}'].join('\n');
  document.head.appendChild(css);

  // ---- header: STOP + Camera tab ----
  var stop = document.createElement('button'); stop.id = 'stopbtn'; stop.textContent = 'STOP (Esc)';
  stop.onclick = function () { send({ type: 'stop' }); };
  after($('live'), stop);
  document.addEventListener('keydown', function (e) { if (e.key == 'Escape' || e.keyCode == 27) send({ type: 'stop' }); });
  var tab = document.createElement('button'); tab.className = 'tab'; tab.setAttribute('data-tab', 'cam'); tab.textContent = 'Camera';
  after(document.querySelector('.tab[data-tab=cal]'), tab);

  var SET = [['x_trim', 'X trim (mm)'], ['y_trim', 'Y trim (mm)'], ['object_z', 'Aim height above table (mm)'], ['hover', 'Hover height (mm)'],
    ['z_grasp_offset', 'Grasp z = table z + this (mm)'], ['z_floor', 'Lowest allowed tip z (mm)'],
    ['pick_pitch', 'Pick tool pitch (blank = keep current, -90 = straight down)'], ['grip_open', 'Gripper open value'], ['grip_close', 'Gripper close value'],
    ['drop_x', 'Drop X'], ['drop_y', 'Drop Y'], ['drop_z', 'Drop Z']];

  var cam = document.createElement('div'); cam.id = 'cam'; cam.className = 'views'; cam.style.display = 'none';
  cam.innerHTML = [
  '<div class="card camv" style="flex:2 1 620px">',
  '  <div id="cwarn" class="chips"></div>',
  '  <img id="cvid" alt="camera feed (click a point on the table)">',
  '  <div class="mono" id="cstat" style="margin:6px 0;white-space:pre-wrap;line-height:1.5"></div>',
  '  <div class="row wrap">',
  '    <label><input type="checkbox" data-s="show_grid"> table grid</label>',
  '    <label><input type="checkbox" data-s="show_tip"> arm tip</label>',
  '    <label><input type="checkbox" data-s="show_markers"> marker check</label>',
  '    <button id="clock">Lock pose</button>',
  '  </div>',
  '  <p class="help">Grid lines are table coordinates in mm (X forward, Y left). <b style="color:#ffff00">Cyan cross</b> = where the arm believes its tip is, <b style="color:#ff8c00">orange line</b> = jaw closing direction, <b style="color:#ff00ff">magenta</b> = your click.',
  '  <b>Yellow circles</b> should sit on the green marker centres; a long yellow line means the marker coordinates are wrong.</p>',
  '  <div class="row wrap"><label><input type="checkbox" id="cdet"> Detect objects</label>',
  '    <input type="text" id="cprompt" placeholder="screwdriver, pen, ..."><button id="cpickd" class="on">Pick detected</button></div>',
  '  <div class="mono" id="cdets" style="white-space:pre"></div>',
  '</div>',
  '<div class="col">',
  '  <div class="card"><h4>Click target</h4><div class="mono" id="cclick">Click the video to choose a point on the table.</div>',
  '    <div class="row wrap"><button id="chov">Hover above</button><button id="clow">Go to grasp height</button><button id="cpick" class="on">Pick here</button></div></div>',
  '  <div class="card"><h4>Move the tip (mm)</h4><div class="mono" id="ctip"></div><div id="cflag" class="chip warn" style="display:none;margin:6px 0"></div>',
  '    <div class="row wrap"><label>X</label><input type="number" id="cx"><label>Y</label><input type="number" id="cy"><label>Z</label><input type="number" id="cz"><button id="cgo" class="on">Go</button></div>',
  '    <div class="row wrap"><label>Pitch</label><input type="number" id="cpitch"><button id="cpset">Set pitch</button><label><input type="checkbox" id="ckeep" checked> keep pitch on Go</label></div>',
  '    <div class="jog" id="cjog"></div><p class="help">+X forward, +Y left, +Z up. Jog uses the current pitch.</p></div>',
  '  <div class="card"><h4>Marker calibration (measure with the arm)</h4>',
  '    <p class="help">Do this once after the base is fixed. Live ON, then jog the tip onto the <b>centre</b> of a marker and press Record. 4 markers is best; 2 far-apart markers also works (the rest are deduced). Record at the same tool pitch you pick with.</p>',
  '    <div id="cmk"></div>',
  '    <div class="row wrap"><button id="cmapply" class="on">Apply measured markers</button><button id="cmtape">Rebuild from tape</button><button id="cmreset">Reset to old values</button></div>',
  '    <div class="mono" id="cmtapechk" style="white-space:pre-wrap"></div>',
  '    <div class="mono" id="cmfit" style="white-space:pre-wrap"></div></div>',
  '  <div class="card"><h4>Wrist roll calibration</h4>',
  '    <p class="help">1. Turn the roll slider until the jaws close along the arm, press <b>Set zero</b>. 2. Test: type a table angle (0 = along +X) and press Rotate jaws. The orange line on the video should match the real jaws; if the wrist turns the wrong way press Flip.</p>',
  '    <div class="row"><label>Roll</label><input type="range" id="croll" min="0" max="180" step="1" style="flex:1"><span id="crollv"></span></div>',
  '    <div class="row wrap"><button id="crz">Set zero here</button><button id="crf">Flip direction</button>',
  '      <input type="number" id="cja" value="0"><button id="crj">Rotate jaws to table angle</button></div>',
  '    <div class="mono" id="crinfo"></div></div>',
  '  <div class="card"><details><summary>Pick settings</summary><div id="cset"></div></details></div>',
  '</div>'].join('\n');
  document.querySelector('main').appendChild(cam);

  var logCard = document.createElement('div'); logCard.id = 'logcard'; logCard.className = 'card'; logCard.style.flex = '1 1 100%';
  logCard.innerHTML = '<div class="row"><b>Log</b><button id="logclr">Clear</button><label style="width:auto"><input type="checkbox" id="logw"> warnings only</label></div><div id="logbox"></div>';
  document.querySelector('main').appendChild(logCard);

  // ---- tabs ----
  // Old browsers cannot show the multipart MJPEG stream in an <img>, so there the image
  // reloads /frame.jpg as soon as the previous frame has arrived.
  var vid = $('cvid'), vidOn = false, snapT = null;
  function snap() { snapT = null; if (vidOn) vid.src = '/frame.jpg?t=' + Date.now(); }
  function nextSnap(ms) { if (A.legacy && vidOn && !snapT) snapT = setTimeout(snap, ms); }
  vid.onload = function () { nextSnap(150); };   // leave the tablet time to decode
  vid.onerror = function () { nextSnap(500); };
  function showCam(on) {
    cam.style.display = on ? '' : 'none';
    if (on) {
      $('move').style.display = 'none'; $('cal').style.display = 'none';
      if (A.legacy) { if (!vidOn) { vidOn = true; snap(); } }
      else if ((vid.getAttribute('src') || '').indexOf('/video') < 0) vid.src = '/video';
      send({ type: 'unlock', on: false }); sDirty = vDirty = true; A.schedule();
    } else {
      vidOn = false; if (snapT) { clearTimeout(snapT); snapT = null; }
      if (vid.getAttribute('src')) vid.removeAttribute('src');
    }
  }
  tab.addEventListener('click', function () { each(document.querySelectorAll('.tab'), function (x) { A.tog(x, 'sel', x == tab); }); showCam(true); });
  each(document.querySelectorAll('.tab:not([data-tab=cam])'), function (b) { b.addEventListener('click', function () { showCam(false); }); });

  // ---- controls ----
  vid.onclick = function (e) {
    var r = vid.getBoundingClientRect(), x = e.pageX - (r.left + window.pageXOffset), y = e.pageY - (r.top + window.pageYOffset);
    send({ type: 'cam_click', u: x / r.width, v: y / r.height });
  };
  each(cam.querySelectorAll('input[data-s]'), function (c) { c.onchange = function () { send({ type: 'vset', key: c.getAttribute('data-s'), value: c.checked }); }; });
  $('clock').onclick = function () { send({ type: 'pose_lock', on: !(V && V.locked) }); };
  function det() { send({ type: 'detect', on: $('cdet').checked, prompt: $('cprompt').value }); }
  $('cdet').onchange = det; $('cprompt').onchange = det;
  $('cpickd').onclick = function () { send({ type: 'pick_detected' }); };
  $('chov').onclick = function () { send({ type: 'goto_click', mode: 'hover' }); };
  $('clow').onclick = function () { send({ type: 'goto_click', mode: 'low' }); };
  $('cpick').onclick = function () { send({ type: 'pick_click' }); };
  $('cgo').onclick = function () { send({ type: 'tip', x: +$('cx').value, y: +$('cy').value, z: +$('cz').value, lock: $('ckeep').checked, log: true }); };
  $('cpset').onclick = function () { send({ type: 'pitch', pitch: +$('cpitch').value }); };
  $('cjog').innerHTML = [['X', 'dx'], ['Y', 'dy'], ['Z', 'dz']].map(function (a) {
    return '<div class="row"><label style="width:20px">' + a[0] + '</label>' +
      [-10, -1, 1, 10].map(function (s) { return '<button data-k="' + a[1] + '" data-s="' + s + '">' + (s > 0 ? '+' : '') + s + '</button>'; }).join('') + '</div>';
  }).join('');
  $('cjog').onclick = function (e) { var b = A.closest(e.target, 'button'); if (b) { var m = { type: 'jog' }; m[b.getAttribute('data-k')] = +b.getAttribute('data-s'); send(m); } };
  $('cmk').innerHTML = [0, 1, 2, 3].map(function (i) { return '<div class="mk"><button data-m="' + i + '">Record ' + i + '</button><span class="mono" id="cm' + i + '"></span></div>'; }).join('');
  $('cmk').onclick = function (e) { var b = A.closest(e.target, 'button'); if (b) send({ type: 'marker_record', id: +b.getAttribute('data-m') }); };
  $('cmapply').onclick = function () { send({ type: 'marker_apply' }); };
  $('cmreset').onclick = function () { if (confirm('Go back to the old hand-measured marker values?')) send({ type: 'marker_reset' }); };
  $('cmtape').onclick = function () { if (confirm('Rebuild the marker model from the tape measurements?\n\nThe shape comes from the tape; the placement is fitted onto the arm-measured points.')) send({ type: 'marker_tape' }); };
  A.onRange($('croll'), function (e) { $('crollv').textContent = e.target.value; send({ type: 'roll', angle: +e.target.value }); });
  $('crz').onclick = function () { send({ type: 'roll_zero' }); };
  $('crf').onclick = function () { send({ type: 'roll_flip' }); };
  $('crj').onclick = function () { send({ type: 'jaws', angle: +$('cja').value }); };
  $('cset').innerHTML = SET.map(function (a) { return '<div class="row"><label style="width:240px">' + a[1] + '</label><input type="number" step="any" data-k="' + a[0] + '"></div>'; }).join('');
  each($('cset').querySelectorAll('input'), function (i) { i.onchange = function () { send({ type: 'vset', key: i.getAttribute('data-k'), value: i.value }); }; });

  // ---- log ----
  var box = $('logbox');
  function addLog(m) {
    var d = document.createElement('div'); d.className = 'l-' + m.lvl; d.setAttribute('data-l', m.lvl); d.textContent = m.t + '  ' + m.msg;
    if ($('logw').checked && (m.lvl == 'info')) d.style.display = 'none';
    var stick = box.scrollTop + box.clientHeight >= box.scrollHeight - 30;
    box.appendChild(d); while (box.childNodes.length > 300) box.removeChild(box.firstChild);
    if (stick) box.scrollTop = box.scrollHeight;
  }
  $('logclr').onclick = function () { box.innerHTML = ''; };
  $('logw').onchange = function () { each(box.querySelectorAll('div'), function (d) { d.style.display = ($('logw').checked && d.getAttribute('data-l') == 'info') ? 'none' : ''; }); };

  // ---- rendering ----
  function idle(id) { return document.activeElement !== $(id); }
  function renderState() {
    if (!S) return; var t = S.tool;
    txt($('ctip'), 'target tip  X ' + t.x.toFixed(1) + '  Y ' + t.y.toFixed(1) + '  Z ' + t.z.toFixed(1) + '  |  pitch ' + t.pitch.toFixed(0) + '  roll ' + t.roll.toFixed(0) + '  |  ' + (S.live ? 'Live ON' : 'Live OFF - arm will not move'));
    each([['cx', t.x], ['cy', t.y], ['cz', t.z], ['cpitch', t.pitch]], function (a) { var v = a[1].toFixed(1); if (idle(a[0]) && $(a[0]).value !== v) $(a[0]).value = v; });
    var f = []; if (S.reach) f.push('IK changed the target to make it reachable'); if (S.limited) f.push('a joint limit was hit');
    A.show($('cflag'), f.length); txt($('cflag'), f.join(' + '));
    if (idle('croll')) { if (Math.abs($('croll').value - t.roll) > 0.5) $('croll').value = t.roll; txt($('crollv'), Math.round(t.roll)); }
  }
  var chipKey = null;
  function renderVision() {
    if (!V) return;
    var chips = [], w = { add: function (cls, t) { chips.push([cls, t]); } };
    if (!V.enabled) w.add('err', 'Camera off: ' + V.problem);
    else {
      if (V.problem) w.add('err', V.problem);
      V.warn.forEach(function (x) { w.add('warn', x); });
      if (!V.pose) w.add('warn', 'Camera pose not found: all 4 ArUco markers must be visible.');
      if (!V.measured) w.add('warn', 'Marker positions are still the old hand-measured values. Use "Marker calibration" after moving the base.');
      if (V.tape_check && !V.tape_check.ok) w.add('err', 'Marker model fails the tape check by ' + V.tape_check.worst.toFixed(0) + ' mm. This is a SHAPE error - solvePnP will absorb part of it as camera tilt and every table coordinate will be wrong. Rebuild from tape.');
      // Pose health decides whether picking is allowed at all, so it leads.
      if (V.pose) w.add(V.healthy ? 'ok' : 'err', (V.healthy ? 'Pose OK - ' : 'POSE UNRELIABLE - ') + V.health);
      if (V.rejects) w.add('warn', V.rejects + ' pose solves rejected in a row - the pose on screen is stale. Check the log for why.');
      if (V.calib && V.calib.rms != null && V.calib.rms > 1.0) w.add('warn', 'Camera calibration RMS is ' + V.calib.rms.toFixed(2) + ' px. Above ~1 px the intrinsics are too coarse for mm work - rerun calibrate_camera.py with a larger board.');
      // 'no provenance file' and 'file present but RMS unrecorded' are different things:
      // only the first stops the stream resolution being checked.
      if (V.calib && V.calib.res == null) w.add('warn', 'No camera_calib.json: the stream resolution cannot be checked against the calibration. Rerun calibrate_camera.py.');
      if (V.busy) w.add('ok', 'Pick running. STOP cancels.');
    }
    var key = chips.join('\n');
    if (key !== chipKey) {
      chipKey = key; var box = $('cwarn'); box.innerHTML = '';
      each(chips, function (c) { var d = document.createElement('div'); d.className = 'chip ' + c[0]; d.textContent = c[1]; box.appendChild(d); });
    }
    // Two reprojection figures, deliberately both shown: "solve" is the raw per-frame fit,
    // "in use" is the averaged pose actually driving the arm. solve small + in-use large
    // means the averaging is corrupting a good solve, which is a different bug from both
    // being large (wrong marker coordinates or wrong intrinsics).
    var cal = V.calib && V.calib.res ? 'calib ' + V.calib.res.join('x') + (V.calib.rms != null ? ' rms ' + V.calib.rms.toFixed(2) + 'px' : '') : 'calib unknown';
    txt($('cstat'),
      V.res.join('x') + '  ' + V.fps + ' fps  ' + cal + '  markers seen: ' + (V.seen.join(', ') || 'none') + '\n' +
      'table plane z = ' + V.plane[0] + 'x + ' + V.plane[1] + 'y + ' + V.plane[2] +
      (V.cam_pos ? '   camera at X' + V.cam_pos[0] + ' Y' + V.cam_pos[1] + ' Z' + V.cam_pos[2] : '') +
      (V.diag ? '\n' + V.diag : ''));
    each(cam.querySelectorAll('input[data-s]'), function (c) { c.checked = !!V.settings[c.getAttribute('data-s')]; });
    $('clock').textContent = V.locked ? 'Unlock pose' : 'Lock pose';
    if (idle('cdet')) $('cdet').checked = V.detect;
    if (idle('cprompt')) $('cprompt').value = V.prompts.join(', ');
    txt($('cdets'), V.dets.length ? V.dets.map(function (d) { return d.name + ' ' + d.conf + '  X' + d.x + ' Y' + d.y + (d.closing != null ? '  jaws ' + d.closing + '°' : ''); }).join('\n') : (V.detect ? 'no detections' : ''));
    txt($('cclick'), V.click ? 'Target  X ' + V.click.x + '  Y ' + V.click.y + '  (table z ' + V.click.z + ')' : 'Click the video to choose a point on the table.');
    each([0, 1, 2, 3], function (i) {
      var m = V.markers[i], t = V.touched[i], e = V.err[i];
      // touched entries are [x, y, z, tool_pitch, roll]; pitch/roll are absent on older records
      var pose = t && t.length > 4 && t[3] != null ? '  @pitch ' + t[3] + '° roll ' + t[4] : '';
      txt($('cm' + i), (t ? 'measured ' + t.slice(0, 3).join(', ') + pose : 'stored ' + m.join(', '))
        + (e != null ? '   ' + e.toFixed(1) + ' px' : ''));
    });
    var tc = V.tape_check;
    if (!tc) {
      $('cmtapechk').textContent = 'No tape measurements in vision.json. Measuring the six distances between marker centres by hand is the only check on the arm that does not use the arm.';
    } else {
      $('cmtapechk').textContent = 'tape vs model (shape only - independent of where the model sits)\n'
        + tc.rows.map(function (r) { return '  ' + r.pair + '   tape ' + pad(r.tape.toFixed(0), 5) + ' mm   model ' + pad(r.model.toFixed(1), 6) + ' mm   ' + (r.diff > 0 ? '+' : '') + r.diff.toFixed(1); }).join('\n')
        + '\n  worst ' + tc.worst.toFixed(1) + ' mm  ' + (tc.ok ? '- OK' : '- SHAPE ERROR, no trim can fix this');
    }
    txt($('cmfit'), V.fit_info);
    txt($('crinfo'), 'roll zero = servo ' + V.roll.zero + '   servo up turns jaws ' + (V.roll.sign > 0 ? 'CCW' : 'CW') + ' (from above)' + (V.jaw != null ? '   jaws now close along ' + V.jaw.toFixed(0) + '° (table)' : ''));
    each($('cset').querySelectorAll('input'), function (i) { if (document.activeElement !== i) { var v = V.settings[i.getAttribute('data-k')]; v = v == null ? '' : String(v); if (i.value !== v) i.value = v; } });
  }

  // ---- messages (index.html owns the connection; the server broadcasts to every client) ----
  A.listen(function (m) {
    if (m.type == 'hello') box.innerHTML = '';   // (re)connected: the server replays its log next
    else if (m.type == 'log') addLog(m);
    else if (m.type == 'vision') { V = m; vDirty = true; A.schedule(); }
    else if (m.type == 'state') { S = m; sDirty = true; A.schedule(); }
  });
  // nothing on the Camera tab is drawn while it is hidden; showCam() marks both dirty again
  A.onRender(function () {
    if (cam.style.display == 'none') return;
    if (sDirty) { sDirty = false; renderState(); }
    if (vDirty) { vDirty = false; renderVision(); }
  });
})();
