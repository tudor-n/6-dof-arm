/* voice_panel.js - injected into index.html by server.py, after cam_panel.js. Adds the Voice tab: the
   listening mode (off / push to talk / wake word), a hold-to-talk button (V on a keyboard), the microphone
   and its level, what was heard and what it did, a guided calibration, and the settings. hearing.py does the
   listening; the server owns every state shown here, so two tablets always agree.
   Plain ES5 like index.html (Android 4.2 WebView), sharing its helpers through window.ARM. Same fixed
   1024x600 absolute layout. */
(function () {
  var A = window.ARM, $ = A.$, send = A.send, each = A.each, txt = A.txt, html = A.html, show = A.show, tog = A.tog, act = A.act;
  var VS = null, dirty = false, heard = [], hDirty = false, page = 'heard', step = 1, micOpen = false, mKey = null;
  function esc(s) { return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/"/g, '&quot;'); }
  function q(s) { return '"' + s + '"'; }
  function wakeWord() { var w = VS && VS.settings ? VS.settings.wake_word : 'dummy'; return w.charAt(0).toUpperCase() + w.slice(1); }

  var css = document.createElement('style');
  css.textContent = [
    '.vchip{height:26px;line-height:24px;padding:0 10px;border:1px solid #3a3a3a;border-radius:3px;color:#8a8a8a;font-weight:600;font-size:11px;letter-spacing:.1em;white-space:nowrap}',
    '.vchip.g{border-color:#30d158;color:#30d158}.vchip.a{border-color:#ffb020;color:#ffb020}.vchip.w{border-color:#fff;color:#fff}.vchip.r{border-color:#d92d20;color:#ff6a5c}',
    '.b.vptt{height:104px;line-height:102px;font-size:15px}.b.vptt.on{background:#30d158;border-color:#30d158;color:#000}',
    '.vmt{background:#262626;border-radius:2px;overflow:hidden}.vmf{position:absolute;left:0;top:0;height:100%;width:0;background:#8a8a8a}.vmf.sp{background:#30d158}',
    '.vmn{position:absolute;top:0;width:2px;height:100%;background:#000;display:none}',
    '.vrow{height:44px;cursor:pointer}.vrow .vrl{position:absolute;left:0;top:0;line-height:44px;font-size:14px;font-weight:500;white-space:nowrap}',
    '.vrow .vrh{position:absolute;left:78px;top:0;line-height:44px;font-size:12px;color:#5c5c5c;white-space:nowrap}.vrow.prs .vrl{color:#8a8a8a}',
    // the dot in the tab label shows the listening state from every tab
    '.vdot{display:none;width:7px;height:7px;border-radius:4px;margin-right:8px;vertical-align:middle;position:relative;top:-2px;background:#5c5c5c}',
    '.vdot.on{display:inline-block}.vdot.g{background:#30d158}.vdot.a{background:#ffb020}.vdot.w{background:#fff}.vdot.r{background:#d92d20}',
    '.vh{position:relative;height:58px;border-bottom:1px solid #1a1a1a}',
    '.vh .vt{position:absolute;left:0;top:11px;width:64px;font-size:11px;line-height:14px;color:#5c5c5c}',
    '.vh .v1{position:absolute;left:70px;top:8px;width:446px;font-size:16px;line-height:20px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.vh .v1.f{color:#5c5c5c}',
    '.vh .v2{position:absolute;left:70px;top:32px;width:446px;font-size:12px;line-height:16px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;color:#8a8a8a}',
    '.vh .v2.g{color:#30d158}.vh .v2.a{color:#ffb020}.vh .v2.r{color:#ff6a5c}.vh .v2.w{color:#fff}',
    '.vh .vms{position:absolute;right:54px;top:11px;font-size:11px;line-height:14px;color:#5c5c5c;text-align:right;white-space:nowrap}',
    '.vh .vpp{position:absolute;right:0;top:7px;width:44px;height:44px;line-height:42px;text-align:center;border:1px solid #3a3a3a;border-radius:3px;font-size:13px;cursor:pointer}.vh .vpp.prs{background:#fff;color:#000}',
    '.vst{position:absolute;left:0;width:200px;height:56px;border-bottom:1px solid #1a1a1a;cursor:pointer}',
    '.vst .dt{position:absolute;left:12px;top:24px;width:8px;height:8px;border-radius:4px;border:1px solid #5c5c5c}.vst.done .dt{background:#30d158;border-color:#30d158}',
    '.vst .nm{position:absolute;left:30px;top:10px;font-weight:500;font-size:14px;line-height:18px;color:#8a8a8a;white-space:nowrap}',
    '.vst .sb{position:absolute;left:30px;top:30px;font-size:11px;line-height:14px;color:#5c5c5c;white-space:nowrap;overflow:hidden;width:166px;text-overflow:ellipsis}',
    '.vst.cur{background:#fff}.vst.cur .nm{color:#000;font-weight:600}.vst.cur .sb{color:#3a3a3a}.vst.prs{background:#1a1a1a}.vst.cur.prs{background:#d0d0d0}',
    '.vct{font-weight:600;font-size:24px;line-height:28px}.vci{margin-top:8px;font-size:14px;line-height:20px;color:#8a8a8a}',
    '.vcv{font-size:14px;line-height:20px}.vcv.g{color:#30d158}.vcv.a{color:#ffb020}.vcv.r{color:#ff6a5c}',
    '.vcp{font-weight:600;font-size:30px;line-height:36px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.vcp.f{color:#5c5c5c;font-weight:400;font-size:18px}',
    '.vlr{display:inline-block;vertical-align:top;height:36px;line-height:34px;padding:0 6px 0 12px;margin:0 8px 8px 0;border:1px solid #3a3a3a;border-radius:18px;font-size:14px;cursor:pointer;white-space:nowrap}',
    '.vlr span{display:inline-block;width:24px;text-align:center;color:#8a8a8a}.vlr.prs{background:#fff;color:#000}',
    '.vmr{position:absolute;left:0;width:600px;height:52px;line-height:52px;padding:0 16px;border-bottom:1px solid #1a1a1a;font-size:15px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;cursor:pointer}',
    '.vmr.on{background:#fff;color:#000;font-weight:600}.vmr.prs{background:#1a1a1a}'
  ].join('\n');
  document.head.appendChild(css);
  function el(h) { var d = document.createElement('div'); d.innerHTML = h; return d.firstChild; }
  function B(cls, a, l, t, w, label, extra) {
    return '<div class="x b ' + cls + '" data-a="' + a + '" style="left:' + l + 'px;top:' + t + 'px' + (w ? ';width:' + w + 'px;padding:0' : '') + '"' + (extra || '') + '>' + label + '</div>';
  }
  function meter(id, l, t, w, h) {
    return '<div class="x vmt" style="left:' + l + 'px;top:' + t + 'px;width:' + w + 'px;height:' + h + 'px"><div id="' + id + 'f" class="vmf"></div><div id="' + id + 'n" class="vmn"></div></div>';
  }
  function sw(id, a, t, label, hint) {
    return '<div id="' + id + '" class="x vrow" data-a="' + a + '" style="left:16px;top:' + t + 'px;width:328px"><span class="vrl">' + label + '</span><span class="vrh">' + hint +
      '</span><span class="sw s2" style="right:0;top:9px"><i></i></span></div>';
  }

  // ---------------- the tab (body coordinates: y 0 = just under the top bar) ----------------
  var tab = document.querySelector('.tab[data-t=voice]');
  if (tab) tab.innerHTML = '<span id="vdot" class="vdot"></span>Voice';
  var view = el('<div id="voice" class="v" style="display:none"></div>');
  var STEPS = [[1, 'Mic check'], [2, 'Quiet room'], [3, 'Your voice']];
  var h = [
    // left: how it listens
    '<div class="x br" style="left:0;top:0;width:360px;height:472px">',
    ' <div class="x lb" style="left:16px;top:17px">Listening</div>',
    ' <div id="vstate" class="x vchip" style="right:16px;top:10px">OFF</div>',
    ' <div class="x pl" data-a="vmode" data-m="off" style="left:16px;top:44px;width:92px">Off</div>',
    ' <div class="x pl" data-a="vmode" data-m="ptt" style="left:114px;top:44px;width:112px">Push to talk</div>',
    ' <div class="x pl" data-a="vmode" data-m="wake" style="left:232px;top:44px;width:112px">Wake word</div>',
    ' ' + B('vptt', 'vptt', 16, 100, 328, 'Hold to talk', ' id="vptt"'),
    ' <div id="vpttn" class="x f" style="left:16px;top:210px;width:328px;font-size:12px;line-height:14px"></div>',
    ' <div class="x lb" style="left:16px;top:240px">Microphone</div>',
    ' <div id="vmic" class="x fld" data-a="vmic" style="left:16px;top:256px;width:328px;font-size:14px"></div>',
    ' ' + meter('vlm', 16, 314, 328, 10),
    ' <div id="vlmt" class="x m f" style="left:16px;top:330px;width:328px;font-size:11px;line-height:13px;white-space:nowrap"></div>',
    ' ' + sw('vdry', 'vdry', 352, 'Dry run', 'hear and show; only stop acts'),
    ' ' + sw('vsnd', 'vsnd', 398, 'Sounds', 'beep when it hears the name or acts'),
    ' <div id="vdev" class="x m f" style="left:16px;top:450px;width:328px;font-size:11px;line-height:13px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis"></div>',
    '</div>',
    // right: what it heard / calibration / settings
    '<div class="x" style="left:360px;top:0;width:664px;height:472px">',
    ' <div class="x pl" data-a="vpage" data-p="heard" style="left:16px;top:10px;width:120px">Heard</div>',
    ' <div class="x pl" data-a="vpage" data-p="cal" style="left:142px;top:10px;width:120px">Calibrate</div>',
    ' <div class="x pl" data-a="vpage" data-p="set" style="left:268px;top:10px;width:120px">Settings</div>',
    ' <div class="x pl" data-a="vpage" data-p="per" style="left:394px;top:10px;width:120px">Personality</div>',
    ' <div id="vclr" class="x tl" data-a="vclr" style="right:16px;top:10px;height:44px;line-height:44px;padding:0 4px">Clear</div>',
    // heard
    ' <div id="vpgheard" class="x" style="left:0;top:64px;width:664px;height:408px">',
    '  <div id="vsay" class="x" style="left:16px;top:6px;width:632px;font-size:14px;line-height:18px;color:#8a8a8a"></div>',
    '  <div id="vlist" class="x scroll" style="left:16px;top:34px;width:632px;height:366px"></div>',
    ' </div>',
    // calibrate: three steps on the left, the current one on the right
    ' <div id="vpgcal" class="x" style="left:0;top:64px;width:664px;height:408px;display:none">',
    '  <div class="x" style="left:16px;top:0;width:200px;height:168px">'
  ];
  each(STEPS, function (s, i) { h.push('<div id="vcs' + s[0] + '" class="vst" data-a="vstep" data-s="' + s[0] + '" style="top:' + (i * 56) + 'px"><span class="dt"></span><span class="nm">' + s[0] + '  ' + s[1] + '</span><span id="vcsb' + s[0] + '" class="sb"></span></div>'); });
  h.push('  </div>',
    '  <div id="vcc1" class="x" style="left:232px;top:0;width:416px;height:400px">',
    '   <div class="vct">Mic check</div>',
    '   <div class="vci">Turn on Push to talk or Wake word, then talk from where you usually stand. The bar should jump well past the dark noise mark. If it barely moves, pick another microphone or bring it closer.</div>',
    '   ' + meter('vcm', 0, 150, 416, 18),
    '   <div id="vcmt" class="x m f" style="left:0;top:176px;width:416px;font-size:12px;line-height:14px;white-space:nowrap"></div>',
    '   <div id="vcmv" class="x vcv" style="left:0;top:202px;width:416px"></div>',
    '   ' + B('', 'vmic', 0, 340, 200, 'Choose microphone') + B('pr', 'vstep', 296, 340, 120, 'Next', ' data-s="2"'),
    '  </div>',
    '  <div id="vcc2" class="x" style="left:232px;top:0;width:416px;height:400px;display:none">',
    '   <div class="vct">Quiet room</div>',
    '   <div class="vci">Press Start and stay quiet for 8 seconds. Let the arm move meanwhile if you can (jog it, or press Start pose): servo noise must not count as speech. It sets how sure it must be that someone is talking.</div>',
    '   <div id="vcqv" class="x vcv" style="left:0;top:170px;width:416px"></div>',
    '   ' + B('', 'vquiet', 0, 340, 200, 'Start · 8 s', ' id="vcqb"') + B('pr', 'vstep', 296, 340, 120, 'Next', ' data-s="3"'),
    '  </div>',
    '  <div id="vcc3" class="x" style="left:232px;top:0;width:416px;height:400px;display:none">',
    '   <div class="vct">Your voice</div>',
    '   <div class="vci">Say each phrase when it shows, the way you normally would, from where you normally stand. Two takes each. Where it hears you a little differently, it learns that.</div>',
    '   <div id="vcp" class="x vcp" style="left:0;top:130px;width:416px"></div>',
    '   <div id="vcpn" class="x m f" style="left:0;top:174px;width:416px;font-size:12px;line-height:14px"></div>',
    '   <div id="vcpl" class="x vcv" style="left:0;top:200px;width:416px;height:120px;overflow:hidden"></div>',
    '   <div id="vctb" class="x b" data-a="vtrain" style="left:0;top:340px;width:200px;padding:0">Start</div>',
    '   ' + B('pr', 'vmode', 216, 340, 200, 'Use wake word', ' id="vcuse" data-m="wake"'),
    '  </div>',
    ' </div>',
    // settings
    ' <div id="vpgset" class="x" style="left:0;top:64px;width:664px;height:408px;display:none">');
  // [key, label, hint, min, max, step, decimals, unit]
  var SLD = [['vad_threshold', 'Speech threshold', 'lower hears quieter speech, higher ignores more noise', 0.3, 0.9, 0.05, 2, ''],
    ['end_silence', 'Pause before acting', 'the quiet that ends what you said', 0.3, 1.2, 0.05, 2, ' s'],
    ['awake_window', 'Listen after the name', 'after just the name, how long it waits', 2, 10, 1, 0, ' s']];
  // the Personality page's two, the same way: the slider works in its own units, the server gets value / s[8]
  var PSLD = [['gesture_size', 'Size', 'how big the gestures are', 50, 150, 10, 0, '%', 100],
    ['gesture_speed', 'Gesture speed', 'whatever the speed slider says', 60, 180, 10, 0, ' \u00b0/s', 1]];
  function sv(s, v) { return (+v).toFixed(s[6]) + s[7]; }
  function sliderHTML(s, t) {
    return '<div class="x sl2" style="left:16px;top:' + t + 'px;width:200px;font-size:14px;line-height:18px;color:#fff">' + s[1] + '<br><span class="f" style="font-size:11px;line-height:14px">' + s[2] + '</span></div>' +
      A.hSliderHTML('vs_' + s[0], 228, t - 6, 330) + '<div id="vsv_' + s[0] + '" class="x m" style="left:568px;top:' + (t + 10) + 'px;width:80px;font-size:16px;line-height:20px;text-align:right"></div>';
  }
  function perSliders() { var o = ''; each(PSLD, function (s, i) { o += sliderHTML(s, 128 + i * 64); }); return o; }
  each(SLD, function (s, i) { h.push(sliderHTML(s, 4 + i * 74)); });
  h.push('  <div class="x sl2" style="left:16px;top:232px;width:200px;font-size:14px;line-height:18px;color:#fff">Speech model<br><span class="f" style="font-size:11px;line-height:14px">small is quick and plenty for commands</span></div>',
    '  <div class="x pl" data-a="vmodel" data-v="small.en" style="left:228px;top:228px;width:104px">Small</div>',
    '  <div class="x pl" data-a="vmodel" data-v="medium.en" style="left:338px;top:228px;width:104px">Medium</div>',
    '  <div class="x pl" data-a="vmodel" data-v="large-v3-turbo" style="left:448px;top:228px;width:104px">Turbo</div>',
    '  <div class="x lb" style="left:16px;top:300px">Learned from your voice</div>',
    '  <div id="vforall" class="x tl" data-a="vforall" style="right:16px;top:286px;height:40px;line-height:40px">Forget all</div>',
    '  <div id="vlearn" class="x" style="left:16px;top:322px;width:632px;height:84px;overflow:hidden"></div>',
    ' </div>',
    // personality: nod, shake, dance (gestures.py; the server runs them)
    ' <div id="vpgper" class="x" style="left:0;top:64px;width:664px;height:408px;display:none">',
    '  <div id="vper" class="x vrow" data-a="vper" style="left:16px;top:2px;width:300px"><span class="vrl">Personality</span><span class="vrh" style="left:104px">nod · shake · dance</span><span class="sw s2" style="right:0;top:9px"><i></i></span></div>',
    '  <div id="vclap" class="x vrow" data-a="vclap" style="left:348px;top:2px;width:300px"><span class="vrl">Gripper clap</span><span class="vrh" style="left:112px">in the dance</span><span class="sw s2" style="right:0;top:9px"><i></i></span></div>',
    '  <div class="x lb" style="left:16px;top:66px">Try</div>',
    '  ' + B('', 'vgest', 60, 52, 120, 'Nod', ' data-g="nod"') + B('', 'vgest', 188, 52, 120, 'Shake', ' data-g="shake"') + B('', 'vgest', 316, 52, 120, 'Dance', ' data-g="dance"'),
    '  <div id="vglast" class="x vcv" style="left:452px;top:54px;width:196px;height:44px;font-size:13px;line-height:22px;overflow:hidden"></div>',
    perSliders(),
    '  <div class="x lb" style="left:16px;top:250px">What he reacts to</div>',
    '  <div id="vreact" class="x" style="left:16px;top:268px;width:632px;font-size:14px;line-height:22px"></div>',
    '  <div class="x f" style="left:16px;top:362px;width:632px;font-size:12px;line-height:16px">Needs Live on. Each gesture is small, starts from wherever the arm is and comes back to it, and shrinks or is skipped near the table or a joint limit. STOP ends it where it is.</div>',
    ' </div>',
    '</div>',
    // microphone picker, over the whole tab
    '<div id="vmicp" class="x" style="left:0;top:0;width:1024px;height:472px;background:#000;display:none">',
    ' <div class="x lb" style="left:16px;top:20px">Microphone</div>',
    ' <div class="x f" style="left:140px;top:16px;font-size:13px;line-height:18px">the system default follows whatever input is selected in the sound settings</div>',
    ' ' + B('pr h44', 'vmicx', 892, 8, 116, 'Close'),
    ' <div id="vmicl" class="x scroll" style="left:16px;top:64px;width:600px;height:400px"></div>',
    '</div>');
  view.innerHTML = h.join('');
  $('app').insertBefore(view, $('abar'));
  A.addView('voice', view);

  // ---------------- controls ----------------
  function set(k, v) { send({ type: 'voice_set', key: k, value: v }); }
  act('vmode', { tap: function (e) { set('mode', e.getAttribute('data-m')); } });
  act('vdry', { tap: function () { set('dry_run', !(VS && VS.settings.dry_run)); } });
  act('vsnd', { tap: function () { set('sounds', !(VS && VS.settings.sounds)); } });
  act('vmodel', { tap: function (e) { set('model', e.getAttribute('data-v')); } });
  act('vpage', { tap: function (e) { page = e.getAttribute('data-p'); dirty = hDirty = true; A.schedule(); } });
  act('vstep', { tap: function (e) { step = +e.getAttribute('data-s'); dirty = true; A.schedule(); } });
  act('vclr', { tap: function () { heard = []; hDirty = true; A.schedule(); } });
  act('vquiet', { tap: function () { send({ type: 'voice_train', op: 'quiet' }); } });
  act('vtrain', { tap: function () { send({ type: 'voice_train', op: VS && VS.train && VS.train.on ? 'cancel' : 'start' }); } });
  act('vforget', { tap: function (e) { set('forget', [e.getAttribute('data-k'), e.getAttribute('data-v')]); } });
  act('vforall', { tap: function (e) { A.twice(e, function () { set('forget_all', true); }); } });
  act('vplay', { tap: function (e) { try { new Audio('/voice_clip?id=' + e.getAttribute('data-id')).play(); } catch (x) { } } });
  function micPick(on) { micOpen = on; show($('vmicp'), on); if (on) send({ type: 'voice_mics' }); mKey = null; dirty = true; A.schedule(); }
  act('vmic', { tap: function () { micPick(true); } });
  act('vmicx', { tap: function () { micPick(false); } });
  act('vmicsel', { tap: function (e) { set('mic', e.getAttribute('data-id')); micPick(false); } });
  // the server's values move the sliders too (Slider.set ignores them while a finger drags)
  var SL = {};
  each(SLD.concat(PSLD), function (s) {
    SL[s[0]] = A.Slider($('vs_' + s[0]), { size: 330, thumb: 32, min: s[3], max: s[4], step: s[5],
      onInput: function (v) { txt($('vsv_' + s[0]), sv(s, v)); }, onEnd: function (v) { set(s[0], v / (s[8] || 1)); } });
  });
  act('vper', { tap: function () { set('personality', !(VS && VS.settings.personality)); } });
  act('vclap', { tap: function () { set('dance_clap', !(VS && VS.settings.dance_clap)); } });
  act('vgest', { tap: function (e) { send({ type: 'gesture', name: e.getAttribute('data-g') }); } });
  // push to talk: resent every 100 ms while held, like creep, so a lost release ends it after the server's 0.4 s
  var pttRep = null;
  function pttDown() { if (pttRep) return; send({ type: 'voice_ptt', on: true }); pttRep = setInterval(function () { send({ type: 'voice_ptt', on: true }); }, 100); }
  function pttUp() { if (!pttRep) return; clearInterval(pttRep); pttRep = null; send({ type: 'voice_ptt', on: false }); }
  act('vptt', { down: pttDown, up: pttUp });
  // V on a keyboard is push to talk from any tab, unless something is being typed
  function typing(e) {
    var tn = e.target && e.target.tagName, kb = $('kb');
    return A.pad.id || (kb && kb.style.display != 'none') || tn == 'INPUT' || tn == 'TEXTAREA' || tn == 'SELECT';
  }
  document.addEventListener('keydown', function (e) {
    if (e.keyCode != 86 || e.ctrlKey || e.altKey || e.metaKey || e.repeat || typing(e)) return;
    if (!VS || VS.mode == 'off') return;
    pttDown(); e.preventDefault();
  });
  document.addEventListener('keyup', function (e) { if (e.keyCode == 86) pttUp(); });
  window.addEventListener('blur', pttUp);

  // ---------------- rendering ----------------
  var ST = { off: ['OFF', ''], loading: ['LOADING', 'a'], error: ['ERROR', 'r'], listening: ['LISTENING', 'g'], ready: ['READY', 'w'],
    hearing: ['HEARING', 'w'], thinking: ['THINKING', 'w'], awake: ['SAY IT', 'a'], training: ['CALIBRATING', 'a'], quiet: ['QUIET CHECK', 'a'] };
  function stateOf() { return !VS ? 'off' : !VS.ok ? 'error' : VS.state; }
  function renderDot() {
    var s = stateOf(), d = $('vdot');
    if (!d) return;
    var c = s == 'off' ? '' : s == 'error' ? 'r' : s == 'listening' ? 'g' : s == 'ready' ? '' : s == 'hearing' || s == 'thinking' ? 'w' : 'a';
    A.cls(d, 'vdot' + (s == 'off' ? '' : ' on') + (c ? ' ' + c : ''));
  }
  function db(v) { return v == null ? A.MINUS : A.f0(v) + ' dB'; }
  function levels() { return 'now ' + db(VS.level) + ' · room ' + db(VS.noise_db) + ' · last speech ' + db(VS.speech_db); }
  function fill(id, level, noise, speech) {
    var w = Math.max(0, Math.min(1, (level + 60) / 60)), f = $(id + 'f'), n = $(id + 'n');
    var pc = Math.round(w * 1000) / 10 + '%'; if (f._w !== pc) { f._w = pc; f.style.width = pc; }
    tog(f, 'sp', !!speech);
    show(n, noise != null);
    if (noise != null) { var np = Math.round(Math.max(0, Math.min(1, (noise + 60) / 60)) * 1000) / 10 + '%'; if (n._l !== np) { n._l = np; n.style.left = np; } }
  }
  function renderLeft() {
    var s = stateOf(), st = ST[s] || [s.toUpperCase(), ''], on = VS && VS.mode != 'off' && VS.ok, cf = VS ? VS.settings : null;
    txt($('vstate'), !VS ? 'OFF' : !VS.ok ? 'NOT INSTALLED' : st[0] + (cf && cf.dry_run && s != 'off' ? ' · DRY RUN' : ''));
    A.cls($('vstate'), 'x vchip' + (st[1] ? ' ' + st[1] : ''));
    each(view.querySelectorAll('[data-a=vmode]'), function (e) { if (e.id != 'vcuse') tog(e, 'on', !!VS && VS.mode == e.getAttribute('data-m')); });
    tog($('vptt'), 'dis', !on || s == 'loading' || s == 'error'); tog($('vptt'), 'on', !!VS && VS.ptt);
    txt($('vpttn'), VS && VS.mode == 'wake' ? 'or hold V on a keyboard. Holding it skips the name.' : 'or hold V on a keyboard, from any tab');
    var mic = VS ? VS.mics.filter(function (m) { return m.id == VS.settings.mic; })[0] : null;
    txt($('vmic'), mic ? mic.name : VS && VS.settings.mic ? VS.settings.mic + ' (not found)' : 'System default');
    var live = VS && VS.ok && s != 'off' && s != 'error';
    fill('vlm', live ? VS.level : -90, live ? VS.noise_db : null, live && VS.speech);
    txt($('vlmt'), live ? levels() : '');
    tog($('vdry'), 'on', !!cf && cf.dry_run); tog($('vsnd'), 'on', !!cf && cf.sounds);
    var bad = !VS || !VS.ok || s == 'error';
    var dev = !VS ? '' : bad ? VS.why : VS.note ? VS.note
      : VS.device ? cf.model + ' on the ' + (VS.device == 'cuda' ? 'GPU' : 'CPU') + (VS.asr_ms != null ? ' · ' + VS.asr_ms + ' ms per phrase' : '') : '';
    txt($('vdev'), dev); A.cls($('vdev'), 'x m' + (bad ? ' vcv r' : ' f'));
  }
  // one line per event: what was said, then what it meant / did
  function line(ev) {
    var k = ev.kind, src = ev.source == 'ptt' ? 'push to talk · ' : '', ww = q(wakeWord());
    if (k == 'command') return [ev.ok ? 'g' : 'a', src + (ev.cmd == 'yes' ? 'answer: yes' : ev.cmd == 'no' ? 'answer: no' : ev.say).toUpperCase() + ' ' + A.RARR + ' ' + ev.result];
    if (k == 'wake') return ['a', ww + ' ' + A.RARR + ' listening for the command' + (ev.result ? ' · ' + ev.result : '')];
    if (k == 'nowake') return ['', 'no ' + ww + ' first: ignored'];
    if (k == 'unknown') return [ev.wake || ev.source == 'ptt' ? 'a' : '', src + 'not a command'];
    if (k == 'noise') return ['', 'noise, not speech: ignored'];
    if (k == 'empty') return ['', src + 'heard nothing'];
    if (k == 'train') { var r = ev.train; return [r.ok ? 'g' : 'a', 'calibration: ' + (r.ok ? 'understood' : r.why) + learnedText(r.learned)]; }
    return ['', k];
  }
  function meaning(k) { return k == 'wake' ? wakeWord() : k == 'follow' ? 'come here' : k == 'home' ? 'start pose' : k; }
  function learnedList(l) { return l.map(function (x) { return q(x[1]) + ' = ' + meaning(x[0]); }).join(', '); }
  function learnedText(l) { return l && l.length ? ' · learned ' + learnedList(l) : ''; }
  function renderHeard() {
    var m = VS ? VS.mode : 'off', cmds = VS ? VS.commands : [], ww = wakeWord();
    txt($('vsay'), !VS || !VS.ok ? 'Voice needs faster-whisper: pip install -r requirements.txt, then restart the server.'
      : m == 'wake' ? 'Say ' + q(ww + ',') + ' then ' + cmds.map(function (c) { return q(c[1]); }).join(' · ') + '. Hold to talk skips the name.'
      : m == 'ptt' ? 'Hold the button and say ' + cmds.map(function (c) { return q(c[1]); }).join(' · ') + '. The name is optional.'
      : 'Voice is off. Pick Push to talk or Wake word on the left.');
    if (!hDirty) return; hDirty = false;
    if (!heard.length) { html($('vlist'), '<div class="f" style="font-size:14px;line-height:20px;padding-top:8px">Nothing heard yet.</div>'); return; }
    var o = '';
    each(heard, function (ev) {
      var l = line(ev), t = ev.text ? q(ev.text) : ev.kind == 'empty' ? '(silence)' : '';
      o += '<div class="vh"><span class="vt m">' + esc(ev.t) + '</span><span class="v1' + (ev.kind == 'noise' || ev.kind == 'nowake' ? ' f' : '') + '">' + esc(t) + '</span>' +
        '<span class="v2' + (l[0] ? ' ' + l[0] : '') + '">' + esc(l[1]) + '</span>' +
        '<span class="vms m">' + (ev.ms != null ? ev.ms + ' ms' : '') + '</span><span class="vpp" data-a="vplay" data-id="' + ev.id + '">' + A.G('▶', 'play') + '</span></div>';
    });
    html($('vlist'), o);
  }
  function renderCal() {
    each(STEPS, function (s) { tog($('vcs' + s[0]), 'cur', step == s[0]); show($('vcc' + s[0]), step == s[0]); });
    var running = VS && VS.ok && stateOf() != 'off' && stateOf() != 'error';
    // 1: mic check
    var diff = VS && VS.speech_db != null && VS.noise_db != null ? VS.speech_db - VS.noise_db : null;
    tog($('vcs1'), 'done', diff != null && diff >= 15);
    txt($('vcsb1'), diff != null ? 'speech ' + A.f0(diff) + ' dB over noise' : 'is it loud enough?');
    if (step == 1) {
      fill('vcm', running ? VS.level : -90, running ? VS.noise_db : null, running && VS.speech);
      txt($('vcmt'), running ? levels() : '');
      var v = !running ? ['', 'Listening is off: pick Push to talk or Wake word on the left first.'] : diff == null ? ['', 'Say something...']
        : diff >= 20 ? ['g', 'Good: your voice is ' + A.f0(diff) + ' dB above the room.'] : diff >= 12 ? ['a', 'OK: ' + A.f0(diff) + ' dB above the room. Closer would be more reliable.']
        : ['r', 'Too quiet: only ' + A.f0(diff) + ' dB above the room. Bring the mic closer or choose another one.'];
      txt($('vcmv'), v[1]); A.cls($('vcmv'), 'x vcv' + (v[0] ? ' ' + v[0] : ''));
    }
    // 2: quiet room
    var qr = VS && VS.quiet_result, qn = VS && VS.quiet;
    tog($('vcs2'), 'done', !!qr);
    txt($('vcsb2'), qr ? 'noise ' + db(qr.noise_db) + ' · threshold ' + qr.thr : 'so noise is ignored');
    if (step == 2) {
      tog($('vcqb'), 'dis', !!qn || !(VS && VS.ok));
      txt($('vcqb'), qn ? 'Quiet... ' + Math.ceil(qn.left) + ' s' : 'Start · 8 s');
      var t = qn ? 'Measuring. Stay quiet.' : qr ? 'Noise ' + db(qr.noise_db) + '. The loudest non-speech scored ' + qr.p99 + ' and ' + qr.starts + ' false start' + (qr.starts == 1 ? '' : 's') +
        ' happened. Speech threshold ' + (qr.thr == qr.old ? 'stays ' + qr.thr : qr.old + ' ' + A.RARR + ' ' + qr.thr) + '.' : stateOf() == 'loading' ? 'Loading the speech model first...' : '';
      txt($('vcqv'), t); A.cls($('vcqv'), 'x vcv' + (qr && !qn ? (qr.starts ? ' a' : ' g') : ''));
    }
    // 3: your voice
    var tr = VS && VS.train, sm = tr && tr.summary;
    tog($('vcs3'), 'done', !!sm);
    txt($('vcsb3'), sm ? sm.ok + '/' + sm.n + ' understood' : 'learn how it hears you');
    if (step == 3) {
      var on = !!(tr && tr.on), last = tr && tr.last;
      txt($('vcp'), on ? (stateOf() == 'loading' ? 'Loading the speech model...' : q(tr.say)) : sm ? 'Done: ' + sm.ok + ' of ' + sm.n + ' understood' : 'Press Start');
      tog($('vcp'), 'f', !on && !sm);
      txt($('vcpn'), on ? 'phrase ' + (tr.i + 1) + ' of ' + tr.n + (stateOf() == 'hearing' ? ' · hearing you' : stateOf() == 'thinking' ? ' · thinking' : '') : '');
      var lt = last ? (last.ok ? 'Understood ' : 'Not understood ') + (last.text ? q(last.text) : '') + (last.ok ? '' : ': ' + last.why) + learnedText(last.learned) : '';
      if (sm && !on) lt = (sm.learned.length ? 'Learned ' + learnedList(sm.learned) + '. ' : '') + (sm.ok == sm.n ? 'Everything was understood.' : 'Run it again, or try the phrases that failed with Push to talk.');
      txt($('vcpl'), lt); A.cls($('vcpl'), 'x vcv' + (last && on ? (last.ok ? ' g' : ' a') : ''));
      txt($('vctb'), on ? 'Stop' : sm ? 'Again' : 'Start'); tog($('vctb'), 'pr', !on && !sm); tog($('vctb'), 'dis', !(VS && VS.ok));
      show($('vcuse'), !!sm && !on && VS.mode != 'wake');
    }
  }
  function renderSet() {
    var cf = VS && VS.settings; if (!cf) return;
    each(SLD, function (s) { SL[s[0]].set(cf[s[0]]); if (SL[s[0]].drag == null) txt($('vsv_' + s[0]), sv(s, cf[s[0]])); });
    each(view.querySelectorAll('[data-a=vmodel]'), function (e) { tog(e, 'on', e.getAttribute('data-v') == cf.model); });
    var o = '', any = false, X_ = A.G('✕', 'x');
    function chip(k, a) { any = true; o += '<span class="vlr" data-a="vforget" data-k="' + esc(k) + '" data-v="' + esc(a) + '">' + esc(q(a) + ' = ' + meaning(k)) + '<span>' + X_ + '</span></span>'; }
    each(cf.wake_aliases, function (a) { chip('wake', a); });
    for (var c in cf.phrase_aliases) each(cf.phrase_aliases[c], function (a) { chip(c, a); });
    html($('vlearn'), any ? o : '<div class="f" style="font-size:13px;line-height:18px">Nothing yet. Calibrate ' + A.RARR + ' Your voice teaches it how Whisper hears you.</div>');
    show($('vforall'), any);
  }
  var REACT = [['Dummy', 'nods, then listens for the command'],
    ['Dummy, good boy', 'a happy dance (also: good job, well done, good robot)'],
    ['Dummy, are you dumb?', 'shakes his head (also: stupid, an idiot, broken, useless)'],
    ['Dummy, are you awake?', 'nods (also: ready, listening, there, can you hear me)']];
  function renderPer() {
    var cf = VS && VS.settings; if (!cf) return;
    tog($('vper'), 'on', !!cf.personality); tog($('vclap'), 'on', !!cf.dance_clap);
    each(view.querySelectorAll('[data-a=vgest]'), function (e) { tog(e, 'dis', !cf.personality); });
    each(PSLD, function (s) { SL[s[0]].set(cf[s[0]] * s[8]); if (SL[s[0]].drag == null) txt($('vsv_' + s[0]), sv(s, cf[s[0]] * s[8])); });
    var g = VS.gesture || {}, last = g.on ? 'moving...' : g.last || '';
    txt($('vglast'), last); A.cls($('vglast'), 'x vcv' + (g.on ? '' : /^(didn't|no room)/.test(last) ? ' a' : last ? ' g' : ''));
    var ww = wakeWord(), o = '';
    each(REACT, function (r) { o += '<div><b>' + esc(q(r[0].replace('Dummy', ww))) + '</b> <span class="d">' + A.RARR + ' ' + esc(r[1]) + '</span></div>'; });
    html($('vreact'), o);
  }
  function renderMics() {
    if (!VS) return;
    var k = JSON.stringify(VS.mics) + '|' + VS.settings.mic; if (k === mKey) return; mKey = k;
    var o = '';
    each(VS.mics, function (m, i) { o += '<div class="vmr' + (m.id == VS.settings.mic ? ' on' : '') + '" data-a="vmicsel" data-id="' + esc(m.id) + '" style="top:' + (i * 52) + 'px">' + esc(m.name) + '</div>'; });
    html($('vmicl'), o);
  }
  A.onTab(function (t) { if (t == 'voice') { dirty = hDirty = true; mKey = null; } else if (micOpen) micPick(false); });
  A.listen(function (m) {
    if (m.type == 'hello') { heard = []; hDirty = true; }
    else if (m.type == 'voice') { VS = m; dirty = true; }
    else if (m.type == 'voice_heard') { heard.unshift(m.ev); if (heard.length > 30) heard.length = 30; hDirty = dirty = true; }
    else return;
    A.schedule();
  });
  A.onRender(function () {
    if (!dirty) return;
    renderDot();
    if (A.tab() != 'voice') return;
    dirty = false;
    each(view.querySelectorAll('[data-a=vpage]'), function (e) { tog(e, 'on', e.getAttribute('data-p') == page); });
    show($('vpgheard'), page == 'heard'); show($('vpgcal'), page == 'cal'); show($('vpgset'), page == 'set'); show($('vpgper'), page == 'per');
    show($('vclr'), page == 'heard');
    renderLeft();
    if (micOpen) renderMics();
    if (page == 'heard') renderHeard();
    else if (page == 'cal') renderCal();
    else if (page == 'set') renderSet();
    else renderPer();
  });
})();
