/* cam_panel.js - injected into index.html by server.py. Adds the Camera tab (video, tap-to-target,
   detection + pick, finger follow, jog), its Setup sub-page (marker calibration, wrist roll calibration, pick
   and follow settings, XYZ entry), the in-app keyboard for the detect prompt, and the log / warnings sheet.
   Plain ES5 like index.html (Android 4.2 WebView), sharing index.html's connection, press handling,
   keypad and dirty-write helpers through window.ARM. Same fixed 1024x600 absolute layout. */
(function () {
  var A = window.ARM, $ = A.$, send = A.send, each = A.each, txt = A.txt, html = A.html, show = A.show, tog = A.tog, act = A.act;
  var f0 = A.f0, f1 = A.f1, MINUS = A.MINUS, V = null, S = null, sDirty = false, vDirty = false;
  function pad(s, n) { s = String(s); while (s.length < n) s = ' ' + s; return s; }
  function esc(s) { return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/"/g, '&quot;'); }
  function trim(s) { return s.replace(/^\s+|\s+$/g, ''); }

  var css = document.createElement('style');
  css.textContent = [
    '.pchip{left:0;top:0;height:26px;line-height:24px;padding:0 10px;border:1px solid #30d158;border-radius:3px;color:#30d158;font-weight:600;font-size:11px;letter-spacing:.1em;white-space:nowrap}',
    '.pchip .pd{display:inline-block;width:6px;height:6px;border-radius:3px;background:#30d158;margin-right:7px;vertical-align:middle;position:relative;top:-1px}',
    '#hrow.bad .pchip{border-color:#d92d20;color:#d92d20}#hrow.bad .pchip .pd{background:#d92d20}',
    '#hstat{left:100px;top:0;width:138px;line-height:26px;font-size:12px;color:#8a8a8a;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}#hrow.bad #hstat{left:164px;width:74px}',
    '#wbtn{right:92px;top:0;width:118px;height:26px;line-height:24px;border:1px solid #3a3a3a;border-radius:3px;text-align:center;color:#8a8a8a;font-weight:600;font-size:11px;letter-spacing:.1em;text-transform:uppercase;cursor:pointer;white-space:nowrap}',
    '#wbtn.w{border-color:#ffb020;color:#ffb020}#wbtn.prs{background:#262626}',
    '#vwrap{left:16px;top:46px;width:642px;height:362px;border:1px solid #262626;border-radius:3px;background:#0c0c0c;cursor:crosshair}',
    '#cvid{position:absolute;left:0;top:0;width:640px;height:360px;display:block}',
    '.ax{color:#5c5c5c;font-size:13px}.tv{display:inline-block;width:100px}',
    '.tok{display:inline-block;vertical-align:middle;height:36px;line-height:34px;padding:0 6px 0 12px;margin-right:8px;border:1px solid #fff;border-radius:18px;font-weight:500;font-size:15px;cursor:pointer}',
    '.tok span{display:inline-block;width:24px;text-align:center;color:#8a8a8a;font-size:14px}.tok.prs{background:#fff;color:#000}',
    '.sg{display:inline-block;vertical-align:top;height:42px;line-height:40px;padding:0 18px;margin-right:8px;border:1px solid #3a3a3a;border-radius:21px;font-weight:500;font-size:15px;cursor:pointer}.sg.prs{background:#fff;color:#000;border-color:#fff}',
    '#kbt{left:90px;top:12px;width:820px;height:36px;line-height:36px;white-space:nowrap;overflow:hidden}',
    '#kbcur{font-size:20px;line-height:36px;white-space:pre;vertical-align:middle}#kbcar{display:inline-block;width:2px;height:26px;background:#fff;margin-left:1px;vertical-align:middle}',
    '#kbph{font-size:17px;color:#5c5c5c;margin-left:4px;vertical-align:middle}',
    '.hp{font-size:15px;line-height:21px}.pre{font-size:11px;line-height:13px;white-space:pre-wrap;overflow:hidden;color:#8a8a8a}',
    '.sl2{font-size:13px;line-height:16px;color:#8a8a8a}',
    '.wi2{border:1px solid #ffb020;border-radius:3px;padding:8px 10px;margin-bottom:8px;font-size:13px;line-height:18px;color:#ffb020}.wi2.err{border-color:#d92d20;color:#ff6a5c}.wi2.ok{border-color:#30d158;color:#30d158}',
    '#logbox{font-size:12px;line-height:18px}#logbox div{white-space:pre-wrap;border-bottom:1px solid #141414}',
    '.l-warn{color:#ffb020}.l-err{color:#ff6a5c}.l-ok{color:#30d158}.l-info{color:#fff}#logbox.wo .l-info{display:none}',
    '#cflag{left:28px;top:68px;height:28px;line-height:26px;padding:0 10px;border:1px solid #ffb020;border-radius:3px;color:#ffb020;font-size:12px;white-space:nowrap}',
    // follow toggle while on: green = tracking the finger, amber = arming / holding (plain .on = waiting for a pointing hand)
    '.b.on.fg{background:#30d158;border-color:#30d158;color:#000}.b.on.fa{background:#ffb020;border-color:#ffb020;color:#000}',
    '.b.on.fg.prs,.b.on.fa.prs{background:#b8b8b8;border-color:#b8b8b8}',
    // follow height buttons: .on = the height is exactly that preset
    '.b.fz{height:38px;line-height:36px}.b.jt{height:40px;line-height:38px}'
  ].join('\n');
  document.head.appendChild(css);
  function el(h) { var d = document.createElement('div'); d.innerHTML = h; return d.firstChild; }
  function B(cls, a, l, t, w, label, extra) {
    return '<div class="x b ' + cls + '" data-a="' + a + '" style="left:' + l + 'px;top:' + t + 'px' + (w ? ';width:' + w + 'px;padding:0' : '') + '"' + (extra || '') + '>' + label + '</div>';
  }
  // rows every rh px (default 52) of bh px tall buttons (default 44, the .jg height)
  function jogRows(l, t, w, gap, rh, bh) {
    var h = '', sz = bh ? ';height:' + bh + 'px;line-height:' + (bh - 2) + 'px' : '';
    rh = rh || 52; bh = bh || 44;
    each(['X', 'Y', 'Z'], function (ax, r) {
      h += '<div class="x m" style="left:' + l + 'px;top:' + (t + r * rh) + 'px;width:24px;line-height:' + bh + 'px;font-size:13px;font-weight:600">' + ax + '</div>';
      each([-10, -1, 1, 10], function (s, c) {
        h += '<div class="x b jg" data-a="jog" data-k="d' + ax.toLowerCase() + '" data-s="' + s + '" style="left:' + (l + 30 + c * (w + gap)) + 'px;top:' + (t + r * rh) + 'px;width:' + w + 'px' + sz + '">' + (s > 0 ? '+' + s : MINUS + (-s)) + '</div>';
      });
    });
    return h;
  }

  // ---------------- Camera tab, main screen (body coordinates: y 0 = just under the top bar) ----------------
  var cam = el('<div id="cam" class="v" style="display:none"></div>');
  cam.innerHTML = [
    '<div id="cmain" class="x" style="left:0;top:0;width:1024px;height:472px">',
    ' <div id="hrow" class="x" style="left:16px;top:12px;width:642px;height:26px">',
    '  <div class="x pchip"><span class="pd"></span><span id="pchipt">POSE OK</span></div>',
    '  <div id="hstat" class="x m"></div>',
    '  <div id="wbtn" class="x" data-a="logs">LOG</div>',
    '  <div id="clock" class="x tl" data-a="plock" style="right:0;top:-9px;height:44px;line-height:44px">Lock pose</div>',
    ' </div>',
    ' <div id="vwrap" class="x"><img id="cvid" alt=""></div>',
    ' <div class="x" style="left:16px;top:418px;width:642px;height:44px">',
    '  <div class="x pl" data-a="vtog" data-s="show_grid" style="left:0;top:0;width:60px">Grid</div>',
    '  <div class="x pl" data-a="vtog" data-s="show_tip" style="left:68px;top:0;width:84px">Arm tip</div>',
    '  <div class="x pl" data-a="vtog" data-s="show_markers" style="left:160px;top:0;width:124px">Marker check</div>',
    '  <div class="x f" style="right:0;top:14px;text-align:right;font-size:12px;line-height:16px;white-space:nowrap">Tap the table to set a target</div>',
    ' </div>',
    // right column: target + detect, or (behind the Jog tip button) the jog pad; the video keeps its size either way
    ' <div class="x bl" style="left:674px;top:0;width:350px;height:472px">',
    '  <div id="ctgt" class="x" style="left:0;top:0;width:349px;height:472px">',
    '   <div class="x bb" style="left:0;top:0;width:349px;height:148px">',
    '    <div class="x lb" style="left:16px;top:14px">Target</div>',
    '    <div class="x tl" data-a="setup" style="right:0;top:0;height:40px;line-height:40px;padding:0 16px" id="setupb">Setup</div>',
    '    <div id="tvals" class="x m" style="left:16px;top:38px;font-size:18px;line-height:18px;white-space:nowrap;display:none"><span class="tv"><span class="ax">X </span><span id="tx"></span></span><span class="tv"><span class="ax">Y </span><span id="ty"></span></span><span class="tv"><span class="ax">Z </span><span id="tz"></span></span></div>',
    '    <div id="tnone" class="x f" style="left:16px;top:36px;font-size:15px;line-height:22px;white-space:nowrap">Tap the video to choose a table point.</div>',
    '    ' + B('', 'chov', 16, 69, 96, 'Hover') + B('', 'clow', 120, 69, 96, 'Lower') + B('pr', 'cpick', 224, 69, 110, 'Pick here'),
    '    <div id="cbusy" class="x lb" style="left:16px;top:127px;color:#30d158;display:none">Pick running · STOP cancels</div>',
    '   </div>',
    '   <div class="x bb" style="left:0;top:148px;width:349px;height:141px">',
    '    <div class="x lb" style="left:16px;top:14px">Detect</div>',
    '    <div id="cdet" class="x" data-a="det" style="right:0;top:0;width:110px;height:48px;cursor:pointer"><span class="sw s2" style="right:16px;top:8px"><i></i></span></div>',
    '    <div id="cprompt" class="x fld" data-a="kbopen" style="left:16px;top:52px;width:240px;font-size:14px"></div>',
    '    ' + B('pr', 'pickd', 264, 52, 70, 'Pick'),
    '    <div id="cdets" class="x m d" style="left:16px;top:112px;width:318px;font-size:12px;line-height:14px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis"></div>',
    '   </div>',
    '   <div class="x bb" style="left:0;top:289px;width:349px;height:131px">',
    '    ' + B('', 'follow', 16, 12, 318, 'Follow finger', ' id="cfol"'),
    '    ' + B('fz', 'folz', 16, 66, 102, 'Low', ' id="cfzlow" data-p="low" data-d="-1"'),
    '    ' + B('fz', 'folz', 124, 66, 102, 'Hover', ' id="cfzhover" data-p="hover" data-d="0"'),
    '    ' + B('fz', 'folz', 232, 66, 102, 'High', ' id="cfzhigh" data-p="high" data-d="1"'),
    '    <div id="cfols" class="x m d" style="left:16px;top:110px;width:318px;font-size:12px;line-height:14px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis"></div>',
    '   </div>',
    '   ' + B('jt', 'jogo', 16, 426, 318, 'Jog tip ' + A.RARR),
    '  </div>',
    '  <div id="cjog" class="x" style="left:0;top:0;width:349px;height:472px;display:none">',
    '   <div class="x lb" style="left:16px;top:14px">Jog tip · mm</div>',
    '   <div id="cjtip" class="x m" style="left:16px;top:38px;font-size:14px;line-height:16px;white-space:nowrap"></div>',
    '   <div class="x f" style="left:16px;top:62px;font-size:11px;line-height:11px">+X fwd · +Y left · +Z up</div>',
    '   ' + jogRows(16, 88, 67, 6, 76, 64),
    '   ' + B('', 'jogx', 16, 414, 318, A.LARR + ' Target & detect'),
    '  </div>',
    ' </div>',
    '</div>',
    '<div id="setup" class="x" style="left:0;top:0;width:1024px;height:472px;display:none"></div>'
  ].join('');
  $('app').insertBefore(cam, $('abar'));
  A.addView('cam', cam);
  $('setupb').textContent = 'Setup ' + A.RARR;

  // ---------------- Setup sub-page ----------------
  var SET = [['x_trim', 'X trim', 'mm'], ['y_trim', 'Y trim', 'mm'], ['object_z', 'Aim height above table', 'mm'], ['hover', 'Hover height', 'mm'],
    ['z_grasp_offset', 'Grasp z = table z + this', 'mm'], ['z_floor', 'Lowest allowed tip z', 'mm'],
    ['pick_pitch', 'Pick tool pitch', 'blank = keep current, ' + MINUS + '90 = straight down'], ['grip_open', 'Gripper open value', ''], ['grip_close', 'Gripper close value', ''],
    ['handover_x', 'Hand-over X', 'mm, + toward you'], ['handover_y', 'Hand-over Y', 'mm, + your right'], ['handover_z', 'Hand-over Z', 'mm'],
    ['handover_pitch', 'Hand-over tool pitch', MINUS + '90 = straight down']];
  var FOL = [['follow_hover', 'Hover height above table', 'mm, 20-400 (the arm may reach less)'], ['follow_finger_z', 'Finger height above table', 'mm, corrects the aim'],
    ['follow_lead', 'Aim ahead of the fingertip', 'mm'], ['follow_deadband', 'Deadband', 'mm, ignores hand tremor'],
    ['follow_pitch', 'Follow tool pitch', 'preferred; blank = pitch at engage, ' + MINUS + '90 = down']];
  var BLANK = { pick_pitch: 1, follow_pitch: 1 };   // settings where blank means "keep the current pitch"
  var SUBS = [['back', A.LARR + ' Camera'], ['mk', 'Markers'], ['roll', 'Wrist roll'], ['set', 'Pick settings'], ['fol', 'Finger follow'], ['xyz', 'Go to XYZ'], ['log', 'Log & warnings']];
  var sh = '<div class="x br" style="left:0;top:0;width:260px;height:472px"><div class="x lb" style="left:20px;top:16px">Camera setup</div>';
  each(SUBS, function (s, i) { sh += '<div id="su' + s[0] + '" class="cr nodot" data-a="sub" data-s="' + s[0] + '" style="top:' + (39 + i * 52) + 'px"><span class="nm">' + esc(s[1]) + '</span></div>'; });
  sh += '</div>';
  // markers
  sh += '<div id="sxmk" class="x" style="left:260px;top:0;width:764px;height:472px;display:none">' +
    '<div class="x hp" style="left:28px;top:20px;width:708px">Do this once after the base is fixed. LIVE on, then jog the tip onto the <b>centre</b> of a marker and press RECORD. 4 markers is best; 2 far-apart markers also works (the rest are deduced). Record at the same tool pitch you pick with.</div>';
  each([0, 1, 2, 3], function (i) { sh += B('h44', 'mrec', 28, 96 + i * 52, 112, 'Record ' + i, ' data-m="' + i + '"') + '<div id="cm' + i + '" class="x m pre" style="left:152px;top:' + (97 + i * 52) + 'px;width:236px;height:42px"></div>'; });
  sh += '<div class="x lb" style="left:400px;top:92px">Jog tip · mm</div>' + jogRows(400, 110, 64, 6) +
    '<div id="cmtapechk" class="x m pre" style="left:400px;top:272px;width:336px;height:130px"></div>' +
    '<div id="cmfit" class="x m pre" style="left:28px;top:304px;width:360px;height:96px"></div>' +
    B('pr h44', 'mapply', 28, 412, 230, 'Apply measured markers') + B('h44', 'mtape', 266, 412, 190, 'Rebuild from tape') + B('am h44', 'mreset', 464, 412, 200, 'Reset to old values') + '</div>';
  // wrist roll
  sh += '<div id="sxroll" class="x" style="left:260px;top:0;width:764px;height:472px;display:none">' +
    '<div class="x hp" style="left:28px;top:20px;width:708px">1. Move the roll slider until the jaws close along the arm, press <b>Set zero here</b>. 2. Test: enter a table angle (0 = along +X) and press Rotate jaws. The orange line on the video should match the real jaws; if the wrist turns the wrong way press Flip.</div>' +
    '<div class="x lb" style="left:28px;top:100px">Roll servo</div>' + A.hSliderHTML('crollS', 28, 112, 556) +
    '<div id="crollv" class="x m" style="left:604px;top:128px;font-size:20px;line-height:24px">–</div>' +
    B('h44', 'crz', 28, 188, 170, 'Set zero here') + B('h44', 'crf', 206, 188, 170, 'Flip direction') +
    '<div class="x lb" style="left:28px;top:256px">Jaws to table angle · °</div>' + A.fieldHTML('cja', 28, 274, 140) + B('', 'crj', 176, 274, 170, 'Rotate jaws') +
    '<div id="crinfo" class="x m d" style="left:28px;top:342px;width:708px;font-size:12px;line-height:18px"></div></div>';
  // pick settings, follow settings: two columns of label + vision.json field
  function setGrid(list) {
    var h = '';
    each(list, function (a, i) {
      var l = 28 + (i % 2) * 360, t = 20 + Math.floor(i / 2) * 60;
      h += '<div class="x sl2" style="left:' + l + 'px;top:' + (t + 8) + 'px;width:220px">' + esc(a[1]) + (a[2] ? '<br><span class="f">' + esc(a[2]) + '</span>' : '') + '</div>' + A.fieldHTML('set_' + a[0], l + 228, t + 2, 118);
    });
    return h;
  }
  var SAVED = '<div class="x f" style="left:28px;top:446px;font-size:13px;line-height:16px">Values are saved to vision.json as soon as you press SET.</div>';
  sh += '<div id="sxset" class="x" style="left:260px;top:0;width:764px;height:472px;display:none">' + setGrid(SET) + SAVED + '</div>';
  sh += '<div id="sxfol" class="x" style="left:260px;top:0;width:764px;height:472px;display:none">' + setGrid(FOL) +
    '<div class="x sl2" style="left:28px;top:212px;width:708px">With <b>Follow finger</b> on, point with your index finger (other fingers curled) and the tool hovers at the hover height over the spot just ahead of your fingertip. Curl the finger or pull the hand away and it holds. ' +
    'The camera cannot see how high your finger is, so it assumes the finger height above: if the aim is always off away from or toward the camera, change it. On a keyboard, F toggles follow.</div>' + SAVED + '</div>';
  // xyz
  sh += '<div id="sxxyz" class="x" style="left:260px;top:0;width:764px;height:472px;display:none">' +
    '<div id="ctip" class="x m" style="left:28px;top:20px;width:708px;font-size:14px;line-height:18px;white-space:nowrap"></div>' +
    '<div id="ctipl" class="x d" style="left:28px;top:42px;font-size:13px;line-height:16px"></div>' +
    '<div id="cflag" class="x" style="display:none"></div>' +
    '<div class="x lb" style="left:28px;top:112px">X</div><div class="x lb" style="left:176px;top:112px">Y</div><div class="x lb" style="left:324px;top:112px">Z · mm</div>' +
    A.fieldHTML('cx', 28, 128, 140) + A.fieldHTML('cy', 176, 128, 140) + A.fieldHTML('cz', 324, 128, 140) +
    B('pr', 'cgo', 472, 128, 120, 'Go') + '<div id="ckeep" class="x pl on" data-a="ckeep" style="left:600px;top:130px;width:136px">Keep pitch</div>' +
    '<div class="x lb" style="left:28px;top:200px">Tool pitch · °</div>' + A.fieldHTML('cpitch', 28, 216, 140) + B('', 'cpset', 176, 216, 140, 'Set pitch') +
    '<div class="x sl2" style="left:28px;top:292px;width:708px">+X forward, +Y left, +Z up. Edited values stay amber until GO; the others follow the arm. KEEP PITCH holds the tool pitch on GO.</div></div>';
  $('setup').innerHTML = sh;

  // ---------------- log / warnings sheet ----------------
  var logs = el('<div id="logs" class="x" style="left:0;top:56px;width:1024px;height:544px;background:#000;display:none"></div>');
  logs.innerHTML = '<div class="x lb" style="left:16px;top:25px">Warnings</div><div class="x lb" style="left:416px;top:25px">Log</div>' +
    '<div id="logw" class="x pl" data-a="logw" style="right:252px;top:8px;width:150px">Warnings only</div>' +
    '<div class="x b h44" data-a="logclr" style="right:134px;top:8px;width:110px;padding:0">Clear log</div>' +
    '<div class="x b h44 pr" data-a="logx" style="right:16px;top:8px;width:110px;padding:0">Close</div>' +
    '<div id="wlist" class="x" style="left:16px;top:64px;width:384px;height:464px;overflow:hidden"></div>' +
    '<div id="logbox" class="x m scroll" style="left:416px;top:64px;width:592px;height:464px"></div>';
  $('app').insertBefore(logs, $('pad'));

  // ---------------- detect keyboard (built once; open/close toggles display) ----------------
  var kb = el('<div id="kb" style="display:none"></div>');
  (function () {
    var h = '<div class="x scrim" data-a="kbhide" style="left:0;top:56px;width:1024px;height:172px"></div>' +
      '<div id="kbs" class="x sheet" style="left:0;top:228px;width:1024px;height:372px">' +
      '<div class="x lb" style="left:16px;top:25px">Detect</div>' +
      '<div id="kbt" class="x"><span id="kbch"></span><span id="kbcur"></span><span id="kbcar"></span><span id="kbph">type an object to look for…</span></div>' +
      '<div class="x tl" data-a="kk" data-k="clr" style="right:16px;top:10px;height:40px;line-height:40px;padding:0 14px">Clear</div>' +
      '<div class="x" style="left:16px;top:59px;width:992px;height:1px;background:#262626"></div>' +
      '<div id="kbsg" class="x" style="left:16px;top:68px;width:992px;height:44px;white-space:nowrap;overflow:hidden"></div>';
    function key(l, t, w, k, lab, c, st) { return '<div class="ky' + (c || '') + '" data-a="kk" data-k="' + k + '" style="left:' + l + 'px;top:' + t + 'px' + (w != 84 ? ';width:' + w + 'px' : '') + (st ? ';' + st : '') + '">' + lab + '</div>'; }
    each('qwertyuiop'.split(''), function (c, i) { h += key(16 + i * 92, 120, 84, c, c); });
    h += key(936, 120, 72, 'bs', A.G('⌫', 'DEL'), A.G('⌫', '') ? '' : ' sp');
    each('asdfghjkl'.split(''), function (c, i) { h += key(62 + i * 92, 182, 84, c, c); });
    h += key(890, 182, 118, 'go', 'Detect' + (A.G('↵', '') ? ' ↵' : ''), ' sp go');
    each('zxcvbnm-'.split(''), function (c, i) { h += key(154 + i * 92, 244, 84, c, c); });
    h += key(16, 306, 132, 'hide', 'Hide' + (A.G('⌄', '') ? ' ⌄' : ''), ' sp') + key(156, 306, 652, 'sp', 'Space', ' sp', 'color:#5c5c5c') +
      key(816, 306, 192, 'nx', ', Next object', ' sp', 'color:#fff;border-color:#8a8a8a');
    kb.innerHTML = h + '</div>';
  })();
  $('app').insertBefore(kb, $('pad'));
  var X_ = A.G('✕', 'x');

  // ---------------- video ----------------
  // Old browsers cannot show the multipart MJPEG stream in an <img>, so there the image reloads
  // /frame.jpg (downscaled by the server to exactly the size the image is shown at) once the previous frame has arrived.
  var vid = $('cvid'), vidOn = false, snapT = null, sub = null, KB = { open: false, tokens: [], cur: '', dirty: false }, logOpen = false;
  // the slot is 640x360; the image keeps the camera's aspect and is centred in it (16:9 fills it, 4:3 is 480x360)
  var vidW = 640, vidH = 360;
  function fitVid(res) {
    var w = 640, h = 360;
    if (res && res[0] && res[1]) { if (res[1] * 640 > res[0] * 360) w = Math.round(360 * res[0] / res[1]); else h = Math.round(640 * res[1] / res[0]); }
    if (w === vidW && h === vidH) return; vidW = w; vidH = h;
    vid.style.width = w + 'px'; vid.style.height = h + 'px'; vid.style.left = ((640 - w) >> 1) + 'px'; vid.style.top = ((360 - h) >> 1) + 'px';
  }
  function snap() { snapT = null; if (vidOn) vid.src = '/frame.jpg?w=' + vidW + '&h=' + vidH + '&t=' + Date.now(); }
  function nextSnap(ms) { if (A.legacy && vidOn && !snapT) snapT = setTimeout(snap, ms); }
  vid.onload = function () { nextSnap(150); };   // leave the tablet time to decode
  vid.onerror = function () { nextSnap(500); };
  // the feed runs only while it is actually visible
  function vidSet() {
    var want = A.tab() == 'cam' && !sub && !KB.open && !logOpen;
    if (want == vidOn) return; vidOn = want;
    if (want) { if (A.legacy) snap(); else vid.src = '/video'; }
    else { if (snapT) { clearTimeout(snapT); snapT = null; } if (vid.getAttribute('src')) vid.removeAttribute('src'); }
  }
  A.onTab(function (t) {
    if (t == 'cam') { send({ type: 'unlock', on: false }); sDirty = vDirty = true; }
    else if (KB.open) kbClose();
    vidSet();
  });
  A.track($('vwrap'), {
    down: function (id, e) {
      var r = vid.getBoundingClientRect(), x = e.pageX - (r.left + window.pageXOffset), y = e.pageY - (r.top + window.pageYOffset);
      if (x < 0 || y < 0 || x > r.width || y > r.height) return false;
      send({ type: 'cam_click', u: x / r.width, v: y / r.height }); return true;
    }
  });

  // ---------------- sub-page / sheets ----------------
  function setSub(s) {
    if (s == 'log') { openLogs(); return; }
    if (s == 'back') s = null;
    sub = s; show($('cmain'), !s); show($('setup'), !!s);
    each(SUBS, function (q) { if (q[0] != 'back' && q[0] != 'log') { show($('sx' + q[0]), q[0] == s); tog($('su' + q[0]), 'cur', q[0] == s); } });
    sDirty = vDirty = true; A.schedule(); vidSet();
  }
  act('setup', { tap: function () { setSub('mk'); } });
  act('sub', { tap: function (el) { setSub(el.getAttribute('data-s')); } });
  A.openSetup = function () { A.setTab('cam'); setSub('mk'); };
  function openLogs() { logOpen = true; show(logs, true); vDirty = true; A.schedule(); vidSet(); box.scrollTop = box.scrollHeight; }
  act('logs', { tap: openLogs });
  act('logx', { tap: function () { logOpen = false; show(logs, false); vDirty = sDirty = true; A.schedule(); vidSet(); } });

  // ---------------- controls ----------------
  act('vtog', { tap: function (el) { var k = el.getAttribute('data-s'); send({ type: 'vset', key: k, value: !(V && V.settings[k]) }); } });
  act('plock', { tap: function () { send({ type: 'pose_lock', on: !(V && V.locked) }); } });
  act('chov', { tap: function () { send({ type: 'goto_click', mode: 'hover' }); } });
  // while a pick is holding something, Lower / Pick here become Put down / Bring here (Hover carries it)
  function held() { return !!(S && S.held); }
  act('clow', { tap: function () { send(held() ? { type: 'put_down' } : { type: 'goto_click', mode: 'low' }); } });
  act('cpick', { tap: function () { send(held() ? { type: 'handover' } : { type: 'pick_click' }); } });
  act('pickd', { tap: function () { send({ type: 'pick_detected' }); } });
  act('det', { tap: function () { send({ type: 'detect', on: !(V && V.detect), prompt: promptText() }); } });
  // follow: the server owns the state (and switches it off on any manual move); this only asks to flip it
  function followOn() { return !!(S && S.follow && S.follow.on); }
  function toggleFollow() { send({ type: 'follow', on: !followOn() }); }
  act('follow', { tap: toggleFollow });
  // Follow height: a tap loads the preset; holding Low / High moves the height from where it is.
  // The hold resends every 100 ms like creep, so a lost release only overruns by the server's 0.4 s deadline.
  var fzHold = null, fzRep = null, fzHeld = false;
  function fzStop() { clearTimeout(fzHold); fzHold = null; if (fzRep) { clearInterval(fzRep); fzRep = null; send({ type: 'follow_z', dir: 0 }); } }
  act('folz', {
    down: function (el) {
      var d = +el.getAttribute('data-d'); fzStop(); fzHeld = false;
      if (d) fzHold = setTimeout(function () {
        fzHeld = true; send({ type: 'follow_z', dir: d });
        fzRep = setInterval(function () { send({ type: 'follow_z', dir: d }); }, 100);
      }, 350);
    },
    up: fzStop,
    tap: function (el) { if (!fzHeld) send({ type: 'follow_z', preset: el.getAttribute('data-p') }); }
  });
  // F toggles follow on a keyboard, unless something is being typed (the detect keyboard or the number pad has the keys).
  // It can switch follow OFF from any tab, but ON only from the Camera tab, where the button and status show what it is doing.
  document.addEventListener('keydown', function (e) {
    if (e.keyCode != 70 || e.ctrlKey || e.altKey || e.metaKey || e.repeat) return;
    if (KB.open || A.pad.id) return;
    var tn = e.target && e.target.tagName; if (tn == 'INPUT' || tn == 'TEXTAREA' || tn == 'SELECT') return;
    if (!followOn() && (A.tab() != 'cam' || sub)) return;
    toggleFollow(); e.preventDefault();
  });
  var jogOpen = false;
  function setJog(on) { jogOpen = on; show($('ctgt'), !on); show($('cjog'), on); sDirty = vDirty = true; A.schedule(); }
  act('jogo', { tap: function () { setJog(true); } });
  act('jogx', { tap: function () { setJog(false); } });
  act('jog', { tap: function (el) { var m = { type: 'jog' }; m[el.getAttribute('data-k')] = +el.getAttribute('data-s'); send(m); } });
  act('mrec', { tap: function (el) { send({ type: 'marker_record', id: +el.getAttribute('data-m') }); } });
  act('mapply', { tap: function () { send({ type: 'marker_apply' }); } });
  act('mtape', { tap: function (el) { A.twice(el, function () { send({ type: 'marker_tape' }); }); } });
  act('mreset', { tap: function (el) { A.twice(el, function () { send({ type: 'marker_reset' }); }); } });
  act('crz', { tap: function () { send({ type: 'roll_zero' }); } });
  act('crf', { tap: function () { send({ type: 'roll_flip' }); } });
  var jawAngle = 0;
  A.field('cja', { label: 'Jaws to table angle · °', hint: '0 = jaws close along +X', get: function () { return jawAngle; }, set: function (v) { jawAngle = v; A.fieldShow('cja', f1(v)); } });
  A.fieldShow('cja', '0.0');
  act('crj', { tap: function () { send({ type: 'jaws', angle: jawAngle }); } });
  var croll = A.Slider($('crollS'), { size: 556, thumb: 32, min: 0, max: 180, step: 1,
    onInput: function (v) { txt($('crollv'), f0(v)); A.sendDrag('roll', { type: 'roll', angle: v }); }, onEnd: function () { A.flush(); } });
  each(SET.concat(FOL), function (a) {
    A.field('set_' + a[0], { label: a[1], hint: a[2], blank: !!BLANK[a[0]],
      get: function () { return V ? V.settings[a[0]] : null; },
      set: function (v) { send({ type: 'vset', key: a[0], value: v }); } });
  });
  // XYZ entry: an edited value is pinned (amber) until GO; the others follow the arm
  var pin = {}, keep = true;
  each([['cx', 'x', 'X'], ['cy', 'y', 'Y'], ['cz', 'z', 'Z'], ['cpitch', 'pitch', 'Tool pitch']], function (a) {
    A.field(a[0], { label: a[2] + (a[0] == 'cpitch' ? ' · °' : ' · mm'), hint: a[0] == 'cpitch' ? MINUS + '90 = straight down' : '+X forward, +Y left, +Z up',
      get: function () { return pin[a[0]] != null ? pin[a[0]] : S ? +S.tool[a[1]].toFixed(1) : null; },
      set: function (v) { pin[a[0]] = v; sDirty = true; A.schedule(); } });
  });
  function cur(id, k) { return pin[id] != null ? pin[id] : S ? S.tool[k] : 0; }
  act('cgo', { tap: function () { send({ type: 'tip', x: cur('cx', 'x'), y: cur('cy', 'y'), z: cur('cz', 'z'), lock: keep, log: true }); pin.cx = pin.cy = pin.cz = null; sDirty = true; A.schedule(); } });
  act('cpset', { tap: function () { send({ type: 'pitch', pitch: cur('cpitch', 'pitch') }); pin.cpitch = null; sDirty = true; A.schedule(); } });
  act('ckeep', { tap: function (el) { keep = !keep; tog(el, 'on', keep); } });

  // ---------------- detect keyboard behaviour ----------------
  var VOCAB = ['pen', 'pencil', 'pliers', 'phone', 'paper clip', 'plate', 'cup', 'cube', 'cutter', 'coin', 'cap', 'cable', 'block', 'bottle', 'bolt', 'battery', 'ball', 'box', 'tape', 'tape measure', 'scissors', 'screw', 'sponge', 'spoon', 'marker', 'mug', 'nut', 'key', 'knife', 'eraser', 'glue stick', 'wrench', 'remote', 'lego brick', 'usb stick', 'washer', 'die', 'dice', 'fork', 'hex key', 'lid', 'toy car'];
  var DEFAULTS = ['pen', 'cup', 'tape', 'block', 'bottle', 'scissors', 'marker'], recent = [];
  try { recent = JSON.parse(localStorage.getItem('arm.recent') || '[]') || []; } catch (e) { recent = []; }
  (function () {   // vision.json "vocab" (served as /vocab.json) replaces the built-in list
    var x = new XMLHttpRequest(); x.open('GET', '/vocab.json', true);
    x.onreadystatechange = function () { if (x.readyState == 4 && x.status == 200) try { var v = JSON.parse(x.responseText).vocab; if (v && v.length) VOCAB = v; } catch (e) { } };
    x.send(null);
  })();
  var bsT = null, bsI = null, chKey = null, sgKey = null, sugg = [];
  function promptText() { return KB.dirty ? KB.tokens.concat(trim(KB.cur) ? [trim(KB.cur)] : []).join(', ') : (V ? V.prompts.join(', ') : ''); }
  function kbCommit() { var w = trim(KB.cur); if (w && KB.tokens.indexOf(w) < 0) KB.tokens.push(w); KB.cur = ''; }
  function kbBack() { if (KB.cur) KB.cur = KB.cur.slice(0, -1); else if (KB.tokens.length) KB.cur = KB.tokens.pop(); KB.dirty = true; kbRender(); }
  function bsStop() { if (bsT) { clearTimeout(bsT); bsT = null; } if (bsI) { clearInterval(bsI); bsI = null; } }
  function kbOpen() {
    if (!KB.dirty) { KB.tokens = V ? V.prompts.slice() : []; KB.cur = ''; }
    KB.open = true; show(kb, true); A.setKeys(kbKeyDown); kbRender(); vidSet();
  }
  function kbClose() { bsStop(); KB.open = false; show(kb, false); A.setKeys(null); vDirty = true; A.schedule(); vidSet(); }
  function kbApply() {
    kbCommit(); var p = KB.tokens.join(', ');
    send({ type: 'detect', on: true, prompt: p });
    each(KB.tokens, function (w) { var i = recent.indexOf(w); if (i >= 0) recent.splice(i, 1); recent.unshift(w); });
    recent = recent.slice(0, 12); try { localStorage.setItem('arm.recent', JSON.stringify(recent)); } catch (e) { }
    KB.dirty = false; kbClose();
  }
  function kbKey(k, el) {
    if (k == 'bs') { bsStop(); kbBack(); bsT = setTimeout(function () { bsI = setInterval(kbBack, 70); }, 380); return; }
    if (k == 'go') { kbApply(); return; }
    if (k == 'hide') { kbClose(); return; }
    if (k == 'sp') { if (KB.cur && KB.cur.charAt(KB.cur.length - 1) != ' ') KB.cur += ' '; }
    else if (k == 'nx') kbCommit();
    else if (k == 'clr') { KB.tokens = []; KB.cur = ''; }
    else if (k == 'sug') { KB.cur = el.getAttribute('data-w'); kbCommit(); }   // a suggestion chip (never a letter: 'g' was both)
    else if (k.charAt(0) == 'x' && k.length > 1) KB.tokens.splice(+k.slice(1), 1);
    else KB.cur += k;
    KB.dirty = true; kbRender();
  }
  act('kk', { down: function (el) { kbKey(el.getAttribute('data-k'), el); }, up: function (el) { if (el.getAttribute('data-k') == 'bs') bsStop(); } });
  act('kbopen', { tap: kbOpen });
  act('kbhide', { tap: kbClose });
  function kbKeyDown(e) {
    var c = e.keyCode, k = e.key;
    if (c == 8) kbBack(); else if (c == 13) kbApply(); else if (c == 32) kbKey('sp');
    else if (k == ',' || c == 188) kbKey('nx');
    else if (k && k.length == 1 && /[a-z\-]/i.test(k)) kbKey(k.toLowerCase());
    else if (c >= 65 && c <= 90) kbKey(String.fromCharCode(c + 32));
    else return;
    e.preventDefault();
  }
  // only the header and the suggestion row re-render per keystroke
  function kbRender() {
    var ck = KB.tokens.join('|');
    if (ck !== chKey) {
      chKey = ck; var h = '';
      each(KB.tokens, function (w, i) { h += '<span class="tok" data-a="kk" data-k="x' + i + '">' + esc(w) + '<span>' + X_ + '</span></span>'; });
      $('kbch').innerHTML = h;
    }
    txt($('kbcur'), KB.cur);
    show($('kbph'), !KB.cur && !KB.tokens.length);
    var c = trim(KB.cur), src = c ? recent.concat(VOCAB) : recent.concat(DEFAULTS), out = [];
    for (var i = 0; i < src.length && out.length < 7; i++) {
      var w = src[i];
      if (c && (w.indexOf(c) !== 0 || w == c)) continue;
      if (KB.tokens.indexOf(w) >= 0 || out.indexOf(w) >= 0) continue;
      out.push(w);
    }
    var sk = out.join('|');
    if (sk !== sgKey) { sgKey = sk; var s = ''; each(out, function (w) { s += '<span class="sg" data-a="kk" data-k="sug" data-w="' + esc(w) + '">' + esc(w) + '</span>'; }); $('kbsg').innerHTML = s; }
    // keep the caret in view once the chips overflow the header (rough width estimate avoids a layout read per key)
    if (ck.length * 9 + KB.tokens.length * 60 + KB.cur.length * 11 > 760) $('kbt').scrollLeft = 99999; else if ($('kbt').scrollLeft) $('kbt').scrollLeft = 0;
  }

  // ---------------- log ----------------
  var box = $('logbox'), wOnly = false;
  function addLog(m) {
    var d = document.createElement('div'); d.className = 'l-' + m.lvl; d.textContent = m.t + '  ' + m.msg;
    var stick = logOpen && box.scrollTop + box.clientHeight >= box.scrollHeight - 30;
    box.appendChild(d); while (box.childNodes.length > 300) box.removeChild(box.firstChild);
    if (stick) box.scrollTop = box.scrollHeight;
  }
  act('logclr', { tap: function () { box.innerHTML = ''; } });
  act('logw', { tap: function (el) { wOnly = !wOnly; tog(el, 'on', wOnly); tog(box, 'wo', wOnly); } });

  // ---------------- rendering ----------------
  function renderState() {
    if (!S) return; var t = S.tool;
    if (sub == 'xyz') {
      txt($('ctip'), 'X ' + f1(t.x) + '   Y ' + f1(t.y) + '   Z ' + f1(t.z) + '   ·   pitch ' + f0(t.pitch) + '°   roll ' + f0(t.roll) + '°');
      txt($('ctipl'), S.live ? 'LIVE on: GO moves the arm.' : 'LIVE off: the arm will not move.');
      var f = []; if (S.reach) f.push('IK changed the target to make it reachable'); if (S.limited) f.push('a joint limit was hit');
      show($('cflag'), f.length); txt($('cflag'), f.join(' + '));
      each([['cx', t.x], ['cy', t.y], ['cz', t.z], ['cpitch', t.pitch]], function (a) {
        var p = pin[a[0]] != null; A.fieldShow(a[0], f1(p ? pin[a[0]] : a[1])); tog($('F' + a[0]), 'pin', p);
      });
    }
    if (!sub && jogOpen) txt($('cjtip'), 'X ' + f1(t.x) + '   Y ' + f1(t.y) + '   Z ' + f1(t.z));
    if (!sub && !jogOpen) {   // servers without follow send no S.follow: shown as off
      var fo = S.follow, on = !!(fo && fo.on), fs = on ? fo.state : '', fb = $('cfol');
      tog(fb, 'on', on); tog(fb, 'fg', fs == 'tracking'); tog(fb, 'fa', fs == 'arming' || fs == 'holding');
      var hv = fo && fo.hover != null ? 'height ' + f0(fo.hover) + ' mm' + (fo.hlimit ? ' (' + fo.hlimit + ' here)' : '') : '';
      txt($('cfols'), on && fo.status ? fo.status + (hv ? ' · ' + hv : '') : hv);
      each(['low', 'hover', 'high'], function (p) { tog($('cfz' + p), 'on', !!fo && fo.preset == p); });
    }
    if (sub == 'roll' && croll.drag == null) { croll.set(Math.round(t.roll)); txt($('crollv'), f0(t.roll)); }
    if (!sub) renderHold();
  }
  function warnings() {
    var w = [], add = function (c, s) { w.push([c, s]); };
    if (!V.enabled) { add('err', 'Camera off: ' + V.problem); return w; }
    if (V.problem) add('err', V.problem);
    each(V.warn, function (x) { add('warn', x); });
    if (!V.pose) add('warn', 'Camera pose not found: all 4 ArUco markers must be visible.');
    if (!V.measured) add('warn', 'Marker positions are still the old hand-measured values. Use Setup ' + A.RARR + ' Markers after moving the base.');
    if (V.tape_check && !V.tape_check.ok) add('err', 'Marker model fails the tape check by ' + V.tape_check.worst.toFixed(0) + ' mm. This is a SHAPE error - solvePnP will absorb part of it as camera tilt and every table coordinate will be wrong. Rebuild from tape.');
    if (V.pose && !V.healthy) add('err', 'POSE UNRELIABLE - ' + V.health);
    if (V.rejects) add('warn', V.rejects + ' pose solves rejected in a row - the pose on screen is stale. Check the log for why.');
    if (V.calib && V.calib.rms != null && V.calib.rms > 1.0) add('warn', 'Camera calibration RMS is ' + V.calib.rms.toFixed(2) + ' px. Above ~1 px the intrinsics are too coarse for mm work - rerun calibrate_camera.py with a larger board.');
    // 'no provenance file' and 'file present but RMS unrecorded' are different things: only the first stops the stream resolution being checked.
    if (V.calib && V.calib.res == null) add('warn', 'No camera_calib.json: the stream resolution cannot be checked against the calibration. Rerun calibrate_camera.py.');
    return w;
  }
  var wKey = null;
  function renderVision() {
    if (!V) return;
    fitVid(V.res);
    var w = warnings(), n = w.length;
    if (logOpen) {
      var all = V.enabled && V.pose && V.healthy ? [['ok', 'Pose OK - ' + V.health]].concat(w) : w, k = all.join('\n');
      if (k !== wKey) { wKey = k; var h = ''; each(all, function (c) { h += '<div class="wi2 ' + c[0] + '">' + esc(c[1]) + '</div>'; }); $('wlist').innerHTML = h || '<div class="f" style="font-size:13px">No warnings.</div>'; }
    }
    if (sub) { renderSetup(); return; }
    var ok = V.enabled && V.pose && V.healthy;
    tog($('hrow'), 'bad', !ok);
    txt($('pchipt'), !V.enabled ? 'CAMERA OFF' : !V.pose ? 'NO POSE' : V.healthy ? 'POSE OK' : 'POSE UNRELIABLE');
    var worst = null; for (var e in V.err) if (worst == null || V.err[e] > worst) worst = V.err[e];
    txt($('hstat'), (worst == null ? MINUS : worst.toFixed(1)) + 'px · ' + V.seen.length + '/4 · ' + Math.round(V.fps) + 'fps');
    txt($('wbtn'), n ? n + (n == 1 ? ' warning' : ' warnings') : 'Log'); tog($('wbtn'), 'w', n);
    txt($('clock'), V.locked ? 'Unlock pose' : 'Lock pose');
    each(cam.querySelectorAll('[data-a=vtog]'), function (c) { tog(c, 'on', V.settings[c.getAttribute('data-s')]); });
    var c = V.click;
    show($('tvals'), !!c); show($('tnone'), !c);
    if (c) { txt($('tx'), f1(c.x)); txt($('ty'), f1(c.y)); txt($('tz'), f1(c.z)); }
    renderHold();
    tog($('cdet'), 'on', V.detect);
    tog(cam.querySelector('[data-a=pickd]'), 'dis', !(V.detect && V.dets.length) || held());
    var p = promptText(); txt($('cprompt'), p || 'tap to type objects…'); tog($('cprompt'), 'f', !p);
    var d = V.dets[0];
    txt($('cdets'), d ? d.name + ' ' + d.conf.toFixed(2) + ' · X ' + f0(d.x) + ' Y ' + f0(d.y) + (d.closing != null ? ' · jaws ' + f0(d.closing) + '°' : '') + (V.dets.length > 1 ? '   (+' + (V.dets.length - 1) + ' more)' : '')
      : (V.detect ? 'no detections' : 'detect is off'));
  }
  function renderHold() {
    if (!V) return;
    var h = held(), c = V.click, lo = cam.querySelector('[data-a=clow]'), pk = cam.querySelector('[data-a=cpick]');
    txt(lo, h ? 'Put down' : 'Lower'); txt(pk, h ? 'Bring here' : 'Pick here');
    tog(cam.querySelector('[data-a=chov]'), 'dis', !c); tog(lo, 'dis', !c && !h); tog(pk, 'dis', !c && !h);
    show($('cbusy'), V.busy || h);
    txt($('cbusy'), V.busy ? 'Pick running · STOP cancels' : h ? 'Holding ' + (S.held.name || 'it') + ' · Hover carries it' : '');
  }
  function renderSetup() {
    if (sub == 'mk') {
      each([0, 1, 2, 3], function (i) {
        var m = V.markers[i], t = V.touched[i], e = V.err[i];
        // touched entries are [x, y, z, tool_pitch, roll]; pitch/roll are absent on older records
        var pose = t && t.length > 4 && t[3] != null ? '\n@pitch ' + t[3] + '° roll ' + t[4] : '';
        txt($('cm' + i), (t ? 'measured ' + t.slice(0, 3).join(', ') + pose : 'stored ' + m.join(', ')) + (e != null ? '\n' + e.toFixed(1) + ' px' : ''));
      });
      var tc = V.tape_check;
      txt($('cmtapechk'), !tc ? 'No tape measurements in vision.json. Measuring the six distances between marker centres by hand is the only check on the arm that does not use the arm.'
        : 'tape vs model (shape only)\n' + tc.rows.map(function (r) { return r.pair + '  tape ' + pad(r.tape.toFixed(0), 4) + '  model ' + pad(r.model.toFixed(1), 6) + '  ' + (r.diff > 0 ? '+' : '') + r.diff.toFixed(1); }).join('\n')
        + '\nworst ' + tc.worst.toFixed(1) + ' mm ' + (tc.ok ? '- OK' : '- SHAPE ERROR, no trim can fix this'));
      txt($('cmfit'), V.fit_info || '');
    } else if (sub == 'roll') {
      txt($('crinfo'), 'roll zero = servo ' + V.roll.zero + '   servo up turns jaws ' + (V.roll.sign > 0 ? 'CCW' : 'CW') + ' (from above)' + (V.jaw != null ? '   jaws now close along ' + V.jaw.toFixed(0) + '° (table)' : ''));
    } else if (sub == 'set') {
      each(SET, function (a) { var v = V.settings[a[0]]; A.fieldShow('set_' + a[0], v == null ? 'keep' : String(v)); });
    } else if (sub == 'fol') {   // an older server has no follow_* keys: a dash, not a value
      each(FOL, function (a) { var v = V.settings[a[0]]; A.fieldShow('set_' + a[0], v != null ? String(v) : BLANK[a[0]] && a[0] in V.settings ? 'keep' : MINUS); });
    }
  }

  // ---------------- messages (index.html owns the connection; the server broadcasts to every client) ----------------
  A.listen(function (m) {
    if (m.type == 'hello') box.innerHTML = '';   // (re)connected: the server replays its log next
    else if (m.type == 'log') addLog(m);
    else if (m.type == 'vision') { V = m; vDirty = true; A.schedule(); }
    else if (m.type == 'state') { S = m; sDirty = true; A.schedule(); }
  });
  // nothing is drawn while hidden, and the camera tab is not re-rendered while the keyboard is open
  A.onRender(function () {
    if (A.tab() != 'cam' && !logOpen) return;
    if (KB.open) return;
    if (sDirty) { sDirty = false; renderState(); }
    if (vDirty) { vDirty = false; renderVision(); }
  });
})();
