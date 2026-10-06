// web_interface.cpp — phone-accessible Wi-Fi control panel for the cube.
//
// The ESP32 runs its own Wi-Fi ACCESS POINT (no router or internet needed):
// connect a phone to the "Cube-Control" network, then open the IP address
// printed on the Serial monitor (normally http://192.168.4.1).
//
// Design rules for this file:
//  * HTTP handlers must never call motor-control functions directly and
//    must never block.  They only read shared state (later phases will set
//    request flags that the main control loop acts on).
//  * The balancing loop in esp32_cube_enc.cpp stays fully independent:
//    loop() just calls handleWebInterface(), which returns immediately
//    when no client is waiting.
//
// Only built-in ESP32 Arduino libraries are used here.
#include "ESP32.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
// Build-time gzipped copy of DASHBOARD_HTML below (generated, git-ignored).
#include "dashboard_gz.h"

// Access-point credentials.  Change the password before real use;
// WPA2 requires it to be at least 8 characters long.
// The network name is per cube (cube_config.h), so a phone cannot end up on
// the wrong cube when more than one is powered.
const char* WIFI_NAME = CUBE_WIFI_NAME;
const char* WIFI_PASSWORD = "poop1234";
// mDNS name, so a laptop can use http://cube1.local etc. instead of the IP.
// Phones mostly ignore mDNS; the captive portal below is what serves them.
const char* MDNS_NAME = CUBE_MDNS_NAME;

// HTTP server on the standard port, so plain http://<ip> works.
WebServer webServer(80);

// Captive-portal DNS: answers EVERY name lookup with the cube's own address.
// Phones probe a known URL right after joining a network and show a "sign in"
// notification when the answer is not what they expected - so pointing those
// probes at this server is what makes the dashboard open by itself.
// Hijacking all names is safe here precisely because this AP has no route
// anywhere: there is nothing else on it to resolve.
DNSServer dnsServer;

// --- Tunable gains -----------------------------------------------------
// One table describes every gain: where it lives, the range the web
// interface will accept, and its compiled-in default.  Everything else
// (JSON output, validation, EEPROM save/load, Restore Defaults) loops over
// this table, so adding a gain means adding one row here and nothing else.
//
// The limits are generous around the defaults - they exist to stop a typo
// (a missing decimal point, a pasted value in the wrong field) from
// reaching the controller, not to constrain honest tuning.
struct GainDef {
  const char* name;
  float* ptr;         // the live variable used by the balancing loop
  float lo, hi;       // accepted range, inclusive
  float def;          // value restored by "Restore Defaults"
};
const GainDef GAIN_DEFS[] = {
  {"K1",  &K1,  0.0, 500.0, 180.0  },  // vertex: angle
  {"K2",  &K2,  0.0, 200.0,  30.0  },  // vertex: angular rate
  {"K3",  &K3,  0.0,  50.0,   1.6  },  // vertex: translational speed
  {"K4",  &K4,  0.0,   1.0,   0.008},  // vertex: motor speed
  {"zK2", &zK2, 0.0, 200.0,   8.0  },  // Z axis: angular rate
  {"zK3", &zK3, 0.0,  50.0,   0.30 },  // Z axis: motor speed
  {"eK1", &eK1, 0.0, 500.0, 190.0  },  // edge: angle
  {"eK2", &eK2, 0.0, 200.0,  31.0  },  // edge: angular rate
  {"eK3", &eK3, 0.0,  50.0,   2.5  },  // edge: motor 3 speed
  {"eK4", &eK4, 0.0,   1.0,   0.014},  // edge: motor speed
  // Auto-trim adaptation rate.  Default 0 = off, so the controller behaves
  // exactly as before until the user opts in.  The limit is small on
  // purpose: this is meant to track slow drift, not fight the balancing.
  // Negative values are allowed so the sign can be flipped from the web
  // interface if this hardware's encoder polarity is inverted.
  {"tK",  &tK, -0.5,   0.5,   0.0  },  // balance-point auto-trim rate
  // APPENDED, not inserted: loadGains() matches saved values to this table
  // by POSITION, so a new row may only go on the end or every gain after it
  // reads back the wrong saved number.  Appending needs no GAINS_ID bump -
  // the record stores its own count and older records simply leave this one
  // at the default (see the ESP32.h note above GAINS_ID).
  //
  // Outer heading loop, °/s of yaw commanded per ° of heading error.  1.0
  // approaches a target with a ~1 s time constant and saturates the
  // YAW_TURN_RATE clamp (20°/s) beyond 20° of error; the loop also ramps the
  // command at YAW_ACCEL.  0 disables heading hold entirely,
  // leaving the yaw command fully manual.
  {"zK1", &zK1,  0.0,  10.0,  1.0  },  // heading hold
  // Battery-voltage compensation (see vNom in ESP32.h), appended for the
  // same reason.  0 = off; otherwise the pack voltage the gains were tuned
  // at, e.g. 11.5.  Values well below a real pack voltage just hit the
  // compensation clamp.
  {"vNom", &vNom, 0.0,  13.0,  0.0  },  // battery compensation
  // Demo mode, appended likewise: 1 = arm at boot without the dashboard
  // (takes effect at the next boot, after Save), 0 = boot disarmed.
  {"autoArm", &autoArm, 0.0, 1.0, 0.0 },  // auto-arm at boot
  // Spin lag fix, appended likewise: zK3 scale while a spin is commanded.
  // 1 = unchanged, lower = less lag but faster wheel wind-up.
  {"zK3s", &zK3s, 0.0, 1.0, 1.0 },        // zK3 scale while spinning
};
// Keep the table and the EEPROM record in step at compile time.
static_assert(sizeof(GAIN_DEFS) / sizeof(GAIN_DEFS[0]) == NUM_GAINS,
              "GAIN_DEFS and NUM_GAINS disagree");
// Prove the gain record cannot overlap the calibration offsets at address 0,
// and that both fit inside the EEPROM allocation.  Getting this wrong would
// silently corrupt calibration, so let the compiler check it.
static_assert(sizeof(OffsetsObj) <= GAINS_EEPROM_ADDR,
              "gains would overwrite the calibration offsets");
static_assert(GAINS_EEPROM_ADDR + sizeof(GainsObj) <= EEPROM_SIZE,
              "gains do not fit within EEPROM_SIZE");

// The gains a restart would restore: copied at boot (after loading) and after
// every successful save.  The dashboard shows "unsaved changes" whenever the
// running gains differ, so a Save that was refused (balancing) or never sent
// can no longer look like it worked.  Trim is left out on purpose: auto-trim
// changes it continuously.
static float stored_v[NUM_GAINS];
static void snapshotGains() {
  for (int i = 0; i < NUM_GAINS; i++) stored_v[i] = *GAIN_DEFS[i].ptr;
}
static bool gainsUnsaved() {
  for (int i = 0; i < NUM_GAINS; i++)
    if (*GAIN_DEFS[i].ptr != stored_v[i]) return true;
  return false;
}

// Restore saved gains at startup.  Anything missing, corrupt, or outside
// the accepted range is ignored so the compiled-in default stays in force.
void loadGains() {
  GainsObj g;
  EEPROM.get(GAINS_EEPROM_ADDR, g);
  if (g.ID != GAINS_ID) {                // nothing saved yet: defaults stand
    snapshotGains();
    return;
  }
  // Only trust as many gains as the stored record actually contained.  A
  // record written by an older build with fewer gains stays valid; the ones
  // it did not know about keep their defaults.
  int n = g.count;
  if (n < 0) n = 0;
  if (n > NUM_GAINS) n = NUM_GAINS;
  for (int i = 0; i < n; i++) {
    float v = g.v[i];
    if (isnan(v) || v < GAIN_DEFS[i].lo || v > GAIN_DEFS[i].hi) continue;
    *GAIN_DEFS[i].ptr = v;
  }
  // Restore the learned balance point, range-checked the same way.  A bad
  // value simply leaves the trim at zero, which is the old behaviour.
  if (!isnan(g.trimX) && g.trimX >= -TRIM_MAX && g.trimX <= TRIM_MAX)
    trimX = g.trimX;
  if (!isnan(g.trimY) && g.trimY >= -TRIM_MAX && g.trimY <= TRIM_MAX)
    trimY = g.trimY;
  Serial.print("Loaded saved gains from EEPROM. Trim X ");
  Serial.print(trimX, 3); Serial.print(" Y "); Serial.println(trimY, 3);
  // List what is in force, so a Save can be checked from the boot log.
  // Gains the record did not contain show their compiled-in default.
  Serial.print("Gains ("); Serial.print(g.count); Serial.print(" stored):");
  for (int i = 0; i < NUM_GAINS; i++) {
    Serial.print(' '); Serial.print(GAIN_DEFS[i].name); Serial.print('=');
    Serial.print(*GAIN_DEFS[i].ptr, 4);
  }
  Serial.println();
  snapshotGains();
}

// Write the live gains to EEPROM.  Called only from the control loop, in
// response to an explicit Save from the dashboard - never on every edit,
// because EEPROM/NVS has a finite number of write cycles.
void saveGains() {
  GainsObj g;
  g.ID = GAINS_ID;
  g.count = NUM_GAINS;                   // so a future build knows how many
  for (int i = 0; i < NUM_GAINS; i++) g.v[i] = *GAIN_DEFS[i].ptr;
  // Store the balance point learned so far, so it is in force at next boot.
  g.trimX = trimX;
  g.trimY = trimY;
  EEPROM.put(GAINS_EEPROM_ADDR, g);
  if (EEPROM.commit()) {
    snapshotGains();                     // the dashboard turns "all saved"
    Serial.println("Saved tuning gains and trim to EEPROM.");
  } else {
    Serial.println("ERROR: saving gains to EEPROM failed.");
  }
}

// The dashboard page.  This readable literal is the SOURCE, but it is not
// what gets served: tools/gzip_dashboard.py compresses it into
// dashboard_gz.h at every build, and handleRoot() sends that.  The linker's
// --gc-sections drops this uncompressed copy from the binary since nothing
// references it.  Everything is inlined because the phone is connected to
// the cube's own access point and has no internet access to fetch assets.
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0e1216">
<title>Cube Control</title><style>
:root{--bg:#0e1216;--pnl:#161d24;--ln:#243039;--ink:#e8eff5;--dim:#7b8b99;
--live:#ffab1f;--ok:#3ecf8e;--stop:#ff453a}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--ink);font:400 16px/1.45
system-ui,-apple-system,sans-serif;-webkit-text-size-adjust:100%;
padding:16px 14px calc(20px + env(safe-area-inset-bottom))}
/* placard labels: the engineering-instrument voice */
.k{font-size:10px;text-transform:uppercase;letter-spacing:.16em;color:var(--dim)}
/* header ------------------------------------------------------------ */
header{display:flex;align-items:baseline;gap:8px;margin-bottom:16px}
h1{font-size:13px;font-weight:600;letter-spacing:.14em;text-transform:uppercase;
margin:0;flex:1}
#s{font-size:11px;color:var(--dim);text-align:right}
#lamp{width:7px;height:7px;border-radius:50%;background:var(--ln);flex:none;
align-self:center}
#lamp.a{background:var(--live);animation:p 1.6s ease-in-out infinite}
@keyframes p{50%{opacity:.25}}
/* signature: attitude target with the three wheels at their real 120° */
.inst{position:relative;width:100%;max-width:330px;margin:0 auto;
aspect-ratio:1;display:grid;place-items:center}
svg{width:76%;height:76%;overflow:visible}
/* One visualizer per pose. Vertex balancing is two-axis and uses all three
   wheels, so it gets the target. Edge balancing is a ONE-axis problem -
   only tilt X and motor 3 appear in its control law - so it gets a side-on
   pendulum instead, and the readouts that do not apply are hidden. */
#evis,body.e #vvis{display:none}
body.e #evis{display:block}
body.e .w.b,body.e .w.c{display:none}
body.e .w.a{top:auto;bottom:-4px}
body.e #ayw{opacity:.28}
.cube{fill:#1b242d;stroke:var(--live);stroke-width:2;stroke-linejoin:round;
transition:stroke .2s}
.cube.q{stroke:var(--ok)}
#ebody{transition:transform .12s linear}
.gnd{stroke:#33414d;stroke-width:2}
.piv{fill:var(--live)}
.ring{fill:none;stroke:var(--ln);stroke-width:1}
.ring.o{stroke:#33414d}
.ax{stroke:var(--ln);stroke-width:1;stroke-dasharray:2 5}
#dot{fill:var(--live);transition:cx .12s linear,cy .12s linear,fill .2s}
#dot.q{fill:var(--ok)}
.tick{font:9px ui-monospace,monospace;fill:var(--dim);letter-spacing:.05em}
/* three motor readouts, placed where the wheels actually are */
.w{position:absolute;width:74px;text-align:center}
.w.a{top:-2px;left:50%;transform:translateX(-50%)}      /* M3  top    */
.w.b{bottom:2px;left:-2px}                              /* M1  lower left  */
.w.c{bottom:2px;right:-2px}                             /* M2  lower right */
.w b{display:block;font:400 17px/1.1 ui-monospace,SFMono-Regular,Menlo,monospace;
font-variant-numeric:tabular-nums;margin-top:3px}
.bar{height:2px;background:var(--ln);margin-top:5px;border-radius:2px;
overflow:hidden}
.bar i{display:block;height:100%;width:0;background:var(--live);
transition:width .15s linear}
/* angle readouts ----------------------------------------------------- */
.ang{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:18px 0 4px}
.ang div{background:var(--pnl);border:1px solid var(--ln);border-radius:10px;
padding:10px 12px}
.ang b{display:block;font:300 30px/1.1 ui-monospace,SFMono-Regular,Menlo,monospace;
font-variant-numeric:tabular-nums;letter-spacing:-.03em;margin-top:4px}
/* status ------------------------------------------------------------- */
.st{display:flex;flex-wrap:wrap;gap:6px;margin:14px 0}
.p{font-size:11px;font-weight:600;letter-spacing:.1em;padding:5px 10px;
border-radius:6px;background:var(--pnl);border:1px solid var(--ln);
color:var(--dim)}
.p.on{color:var(--ok);border-color:#1f4d3a}
.p.live{color:var(--live);border-color:#5c4212}
.p.off{color:var(--stop);border-color:#5c231f}
/* controls ----------------------------------------------------------- */
button{font-family:inherit;border:0;border-radius:10px;color:var(--ink);
touch-action:manipulation;user-select:none}
#stop{width:100%;min-height:76px;font-size:19px;font-weight:700;
letter-spacing:.14em;color:#fff;background:var(--stop);
box-shadow:0 6px 20px -8px var(--stop)}
#stop:active{background:#c9302a;box-shadow:none}
.ab{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
.b{min-height:48px;padding:12px;font-size:14px;font-weight:600;
letter-spacing:.06em;background:var(--pnl);border:1px solid var(--ln)}
.b:active{background:#1f2831}
.b:active,#stop:active{transform:scale(.97)}
button:disabled{opacity:.5}
/* Result flash on the button that was pressed: green accepted, red rejected.
   Keyframed rules outrank the normal ones, so #stop's glow gives way. */
.ok{animation:fok .7s}
.er{animation:fer .7s}
@keyframes fok{0%,55%{box-shadow:inset 0 0 0 2px var(--ok)}}
@keyframes fer{0%,55%{box-shadow:inset 0 0 0 2px var(--stop)}}
.b.w1{grid-column:1/-1}
/* A button waiting for its second, confirming tap (see bind()). */
.b.cf{color:var(--live);border-color:var(--live);background:#2a2011}
/* collapsible sections ----------------------------------------------- */
details{background:var(--pnl);border:1px solid var(--ln);border-radius:10px;
margin-top:10px}
summary{padding:14px;font-size:12px;font-weight:600;letter-spacing:.14em;
text-transform:uppercase;cursor:pointer;list-style:none;display:flex;
align-items:center;gap:8px}
summary::-webkit-details-marker{display:none}
summary:after{content:'';width:6px;height:6px;border-right:1.5px solid var(--dim);
border-bottom:1.5px solid var(--dim);transform:rotate(45deg);margin-left:auto;
transition:transform .2s}
details[open] summary:after{transform:rotate(-135deg)}
.bd{padding:0 14px 14px}
.note{font-size:12px;color:var(--dim);margin:0 0 12px}
/* gain grid ---------------------------------------------------------- */
.gl{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.gr{display:flex;align-items:center;gap:7px}
.gr span{font:11px ui-monospace,monospace;color:var(--dim);width:2.6em;
letter-spacing:.04em}
.gr input{flex:1;min-width:0;background:var(--bg);color:var(--ink);
border:1px solid var(--ln);border-radius:7px;padding:9px 8px;font:14px
ui-monospace,SFMono-Regular,Menlo,monospace;font-variant-numeric:tabular-nums}
.gr input:focus{outline:2px solid var(--live);outline-offset:-1px;
border-color:transparent}
:focus-visible{outline:2px solid var(--live);outline-offset:2px}
/* yaw slider: full width, thumb sized for a fingertip */
input[type=range]{width:100%;margin:10px 0 2px;accent-color:var(--live);
height:28px}
.m{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;
font-variant-numeric:tabular-nums;font-size:15px}
@media(prefers-reduced-motion:reduce){*{animation:none!important;
transition:none!important}
/* no animation, so hold the outcome as a static ring until the next press */
.ok{box-shadow:inset 0 0 0 2px var(--ok)}
.er{box-shadow:inset 0 0 0 2px var(--stop)}}
</style></head><body>

<header><span id="lamp"></span><h1>Cube</h1><div id="s">connecting</div></header>

<!-- Attitude target. The outer ring is the real +/-7 deg disengage limit from
     angle_calc(); the radial scale is sqrt so the sub-degree angles seen while
     balancing are actually visible. Motors sit at their true 120 deg spacing. -->
<div class="inst">
<svg id="vvis" viewBox="0 0 200 200" aria-hidden="true">
<circle class="ring o" cx="100" cy="100" r="70"/>
<circle class="ring" cx="100" cy="100" r="46"/>
<circle class="ring" cx="100" cy="100" r="26"/>
<line class="ax" x1="18" y1="100" x2="182" y2="100"/>
<line class="ax" x1="100" y1="18" x2="100" y2="182"/>
<circle class="ring" cx="100" cy="100" r="4"/>
<text class="tick" x="103" y="70">3°</text>
<text class="tick" x="103" y="26">7°</text>
<circle id="dot" cx="100" cy="100" r="5"/>
</svg>
<!-- Edge view: the cube seen along the edge it stands on, so its outline is
     a square balanced on one corner. It pivots about the contact point,
     which is what the eK1 term is actually regulating. Rotation uses the
     same sqrt scale as the target above, so sub-degree tilts are visible;
     the true angle is on the Tilt X readout. -->
<svg id="evis" viewBox="0 0 200 200" aria-hidden="true">
<line class="ax" x1="100" y1="14" x2="100" y2="170"/>
<g id="ebody"><polygon id="ecube" class="cube"
points="100,170 158,112 100,54 42,112"/></g>
<line class="gnd" x1="14" y1="170" x2="186" y2="170"/>
<circle class="piv" cx="100" cy="170" r="3.5"/>
<text class="tick" x="105" y="20">upright</text>
</svg>
<div class="w a"><span class="k">M3</span><b id="m3">—</b>
<div class="bar"><i id="b3"></i></div></div>
<div class="w b"><span class="k">M1</span><b id="m1">—</b>
<div class="bar"><i id="b1"></i></div></div>
<div class="w c"><span class="k">M2</span><b id="m2">—</b>
<div class="bar"><i id="b2"></i></div></div>
</div>

<div class="ang">
<div><span class="k">Tilt X</span><b id="ax">—</b></div>
<div id="ayw"><span class="k">Tilt Y</span><b id="ay">—</b></div>
</div>

<div class="st">
<span class="p" id="armed">ARMED</span>
<span class="p" id="cal">CALIBRATION</span>
<span class="p" id="mode">MODE</span>
<span class="p" id="bv">— V</span>
</div>

<button id="stop">SAFE STOP</button>
<div class="ab"><button class="b" id="arm">ARM</button>
<button class="b" id="disarm">DISARM</button></div>

<details id="trc"><summary>Trace</summary><div class="bd">
<canvas id="tc" width="640" height="200"
 style="width:100%;height:150px;background:var(--bg);border:1px solid var(--ln);
 border-radius:8px"></canvas>
<div class="st" id="tl" style="margin:8px 0 0"></div>
<div class="ab">
<button class="b" id="tpause">Pause</button>
<button class="b" id="tsave">Download CSV</button>
<button class="b" id="tcopy">Copy CSV</button></div>
<textarea id="tcsv" readonly rows="6" style="display:none;width:100%;margin-top:8px;
 background:var(--bg);color:inherit;border:1px solid var(--ln);border-radius:8px;
 font:11px ui-monospace,monospace"></textarea>
<p class="note" id="tn">Live at the full 66.7 Hz control rate — the numeric
readouts above only sample at 3.3 Hz. Open while tuning: oscillation means
more damping (K2), slow wander means more angle gain (K1).</p>
</div></details>

<details><summary>Motion</summary><div class="bd">
<div style="display:flex;justify-content:space-between;align-items:baseline">
<span class="k">Yaw rate</span><b class="m" id="yv">0 °/s</b></div>
<input type="range" id="yaw" min="-20" max="20" step="5" value="0">
<button class="b w1" id="ystop" style="margin-top:8px">Stop spin</button>
<p class="note">Spins the cube about its vertical axis while balancing.
The spin ramps up and down gently, and yields to balancing when the wheels
are busy. Returns to zero on stop, disarm or arm.</p>
<div style="display:flex;justify-content:space-between;align-items:baseline">
<span class="k">Heading</span><b class="m" id="hv">—</b></div>
<div class="ab" style="margin:0">
<button class="b" id="hhold">Hold heading</button>
<button class="b" id="hfree">Release</button></div>
<div class="ab">
<button class="b" id="hl90">↺ 90°</button>
<button class="b" id="hr90">↻ 90°</button>
<button class="b" id="h180">↻ 180°</button></div>
<p class="note" id="hn">Turns are relative to where the cube is pointing now.
Heading is dead-reckoned from the gyro — no compass — so it drifts over
minutes and resets each time the cube stands up.</p>
<div style="display:flex;justify-content:space-between;align-items:baseline">
<span class="k">Learned trim</span><b class="m" id="tv">—</b></div>
<p class="note" id="tnote">Auto-trim is off.</p>
<div class="ab">
<button class="b" id="ttog">Auto-trim</button>
<button class="b" id="treset">Reset trim</button></div>
<p class="note">Saved with the gains, so it applies from the next boot.
Reset it after changing the cube's hardware, then save again.</p>
</div></details>

<details><summary>Calibration</summary><div class="bd">
<p class="note" id="ch">—</p>
<p class="note" id="cr" style="color:var(--live)"></p>
<p class="note" id="raw" style="font-family:ui-monospace,monospace;font-size:11px">
raw</p>
<div class="ab" style="margin:0">
<button class="b" id="cstart">Start</button>
<button class="b" id="ccap">Capture pose</button>
<button class="b w1" id="csave">Save calibration</button></div>
</div></details>

<details><summary>Gains</summary><div class="bd">
<div class="st" style="margin:0 0 8px"><span class="p" id="gst">—</span></div>
<div id="gl" class="gl">Loading…</div>
<div class="ab">
<button class="b" id="gapply">Apply</button>
<button class="b" id="gdef">Restore defaults</button>
<button class="b w1" id="gsave">Save gains + trim</button></div>
<p class="note" id="gsn" style="margin:8px 0 0;color:var(--live)"></p>
<p class="note" id="gm" style="margin:12px 0 0">Changes take effect at once.
They are lost on restart until you save.</p>
</div></details>

<!-- A phone that arrived through the captive-portal notification is showing
     this inside a cut-down sign-in browser, which suspends background
     polling and blocks storage.  Always offer the real address rather than
     trying to detect that browser - the detection is unreliable and the
     line is useful information either way. -->
<p class="note" style="text-align:center;margin:18px 0 4px">
Opened from the Wi-Fi sign-in screen? For live telemetry open
<a href="http://192.168.4.1/" style="color:var(--live)">192.168.4.1</a>
in your browser — or <b>cube.local</b> on a computer.</p>

<script>
var $=function(i){return document.getElementById(i);};
var busy=false;                        // one request at a time: the ESP32
                                       // WebServer serves a single client,
                                       // so never let polls pile up
var mmax=40;                           // motor bar full-scale, auto-ranging
function pill(el,cls,txt){el.textContent=txt;el.className='p '+cls;}
// Degrees -> radius. sqrt so 0.5 deg is visible while 7 deg lands on the ring.
function rad(a){
 var m=Math.min(Math.abs(a)/7,1);
 return (a<0?-1:1)*Math.sqrt(m)*70;
}
// Same curve for the edge pendulum, mapped to +/-22 deg of visible rotation
// so the cube reads as tipping without leaving the frame.
function tilt(a){return rad(a)/70*22;}
function poll(){
 if(busy)return; busy=true;
 fetch('/api/state',{cache:'no-store'}).then(function(r){return r.json();})
 .then(function(d){
  $('ax').textContent=d.robot_angleX.toFixed(2)+'°';
  $('ay').textContent=d.robot_angleY.toFixed(2)+'°';
  $('dot').setAttribute('cx',100+rad(d.robot_angleX));
  $('dot').setAttribute('cy',100-rad(d.robot_angleY));
  // Green inside the vertex capture window, amber once it is drifting.
  var q=Math.abs(d.robot_angleX)<0.4&&Math.abs(d.robot_angleY)<0.4;
  $('dot').setAttribute('class',q?'q':'');  // SVG: className is read-only
  // Edge pendulum. Both visualizers are updated unconditionally - it is two
  // attribute writes - and CSS shows whichever one matches the pose.
  $('ebody').setAttribute('transform',
    'rotate('+tilt(d.robot_angleX).toFixed(2)+',100,170)');
  $('ecube').setAttribute('class',
    'cube'+(Math.abs(d.robot_angleX)<0.4?' q':''));
  // Edge mode only when the firmware is actually in it; anything else keeps
  // the general-purpose attitude target.
  document.body.classList.toggle('e',d.vertical_edge&&!d.vertical_vertex);
  var s=[d.motor1_speed,d.motor2_speed,d.motor3_speed];
  mmax=Math.max(40,Math.abs(s[0]),Math.abs(s[1]),Math.abs(s[2]));
  for(var i=0;i<3;i++){
   $('m'+(i+1)).textContent=s[i];
   $('b'+(i+1)).style.width=(Math.abs(s[i])/mmax*100)+'%';
  }
  // Battery pill: green ok, amber low, red cutoff, plain on USB power.
  var bs=d.batt_state,bt=d.batt_voltage.toFixed(2)+' V'+
         (d.batt_comp!=1?' ×'+d.batt_comp.toFixed(2):'');
  pill($('bv'),bs=='cutoff'?'off':(bs=='low'?'live':(bs=='ok'?'on':'')),
       bs=='cutoff'?'BATTERY CUTOFF '+bt:(bs=='low'?'BATTERY LOW '+bt:
       (bs=='none'?'NO BATTERY':bt)));
  $('lamp').className=d.armed?'a':'';
  pill($('armed'),d.armed?'live':'off',d.armed?'ARMED':'DISARMED');
  pill($('cal'),d.calibrated?'on':'off',
       d.calibrating?'CALIBRATING':(d.calibrated?'CALIBRATED':'NOT CALIBRATED'));
  // Pose is detected even while disarmed, so only colour it when it can act.
  var m=d.vertical_vertex?'VERTEX':(d.vertical_edge?'EDGE':'IDLE');
  pill($('mode'),(d.vertical_vertex||d.vertical_edge)&&d.armed?'live':'',m);
  // Tell the user which calibration step comes next.  Calibration is
  // blocked while balancing, so say that instead when it applies.
  var bal=d.armed&&(d.vertical_vertex||d.vertical_edge)&&d.calibrated
          &&!d.calibrating;
  // Save state as the FIRMWARE reports it: green only once the running
  // gains really are in EEPROM, so a refused or skipped save shows.
  pill($('gst'),d.gains_unsaved?'live':'on',
       d.gains_unsaved?'UNSAVED CHANGES':'ALL GAINS SAVED');
  // The firmware refuses to save while balancing; say so up front.
  $('gsave').disabled=bal;
  $('gsn').textContent=bal?'Saving is blocked while the cube is balancing — '
                          +'disarm or lay it down first.':'';
  $('ch').textContent=
   bal?'Balancing. Press DISARM before calibrating.':
   (!d.calibrating?'Idle. Press Start to begin.':
   (!d.vertex_calibrated?'Step 1 — set the cube on a VERTEX, then capture.':
    'Step 2 — set the cube on an EDGE, then capture. Saves automatically.'));
  // Result of the last capture, plus the raw counts the pose test actually
  // uses. Both used to be Bluetooth-only.
  $('cr').textContent=d.cal_result||'';
  $('raw').textContent='raw  X '+d.acX+'   Y '+d.acY+'   Z '+d.acZ;
  // Learned balance-point trim. Only meaningful once tK is non-zero.
  $('tv').textContent=d.trimX.toFixed(2)+'° / '+d.trimY.toFixed(2)+'°';
  var tk=G&&G.tK?G.tK.v:0;
  $('ttog').textContent='Auto-trim: '+(tk!=0?'ON':'OFF');
  $('tnote').textContent=tk!=0
   ?'Active (tK '+tk+'). Values settle as the cube balances.'
   :'Off. Toggle on to learn the balance point while balancing.';
  // Reflect the firmware's actual yaw command unless the user is dragging.
  if(!dragging)$('yv').textContent=d.yaw_rate.toFixed(0)+' °/s';
  // Heading only exists in vertex mode; say so rather than showing a stale
  // number the firmware is not currently updating.
  $('hv').textContent=(d.vertical_vertex
   ?d.robot_yaw.toFixed(0)+'°'+(d.yaw_hold?' · holding':'')
   :'—')+(d.yaw_guard?' · wheels wound up ('+d.wheel_wind+')':'');
  // Don't erase a command result the user has not had time to read.
  if(Date.now()>msgUntil)$('s').textContent='live';
 }).catch(function(){$('s').textContent='no signal';})
 // Chain the trace fetch AFTER the state response so there is never more
 // than one request in flight - the ESP32 serves a single client.  Its
 // failure must not mark the whole link down, hence the inner catch.
 .then(function(){
  if($('trc').open)return pollTrace().catch(function(){});
 })
 .then(function(){busy=false;});
}
setInterval(poll,300);                 // 300 ms refresh (spec: 250-500 ms)
poll();
// --- telemetry trace --------------------------------------------------
// Samples accumulate here at the full 66.7 Hz control rate; the chart is a
// window onto the tail. Capped at 8000 samples (~2 min) to bound memory.
var T=[],tSeq=0,tPause=false;
// [csv column, colour, scale full-range, default on, label]
var CH=[['ax','#ffab1f',800,1,'tilt X'],  // centidegrees, +/-8 deg window
        ['ay','#3ecf8e',800,1,'tilt Y'],
        ['gx','#9aa3af',2500,0,'gyro X'], // tenths of deg/s
        ['gy','#58a6ff',2500,0,'gyro Y'],
        ['px','#ff453a',255,1,'pwm X'],
        ['py','#c084fc',255,0,'pwm Y'],
        ['m3','#f97316',450,0,'wheel 3'],
        ['gz','#22d3ee',250,0,'yaw rate'],  // tenths of deg/s, +/-25 deg/s
        ['yc','#facc15',250,0,'yaw cmd'],   // same scale, so they overlay
        ['pz','#f472b6',255,0,'pwm Z']];
function pollTrace(){
 return fetch('/api/trace?since='+tSeq,{cache:'no-store'})
 .then(function(r){return r.text();})
 .then(function(txt){
  var rows=txt.trim().split('\n');
  for(var i=1;i<rows.length;i++){       // row 0 is the header
   var v=rows[i].split(',').map(Number);
   if(v.length<14)continue;
   // A gap in seq means the ring lapped us; break the line honestly.
   if(T.length&&v[0]>tSeq+1)T.push(null);
   tSeq=v[0];
   T.push(v);
  }
  if(T.length>8000)T.splice(0,T.length-8000);
  if(!tPause)drawTrace();
 });
}
function drawTrace(){
 var c=$('tc'),g=c.getContext('2d'),W=c.width,H=c.height;
 g.clearRect(0,0,W,H);
 g.strokeStyle='#243039';g.beginPath();          // zero line
 g.moveTo(0,H/2);g.lineTo(W,H/2);g.stroke();
 var N=400,start=Math.max(0,T.length-N);         // ~6 s window
 CH.forEach(function(ch){
  if(!ch[3])return;
  var col={ax:2,ay:3,gx:4,gy:5,m1:6,m2:7,m3:8,px:9,py:10,
           gz:11,yc:12,pz:13}[ch[0]];
  g.strokeStyle=ch[1];g.beginPath();
  var pen=false;
  for(var i=start;i<T.length;i++){
   var s=T[i];
   if(!s){pen=false;continue;}                   // gap: lift the pen
   var x=(i-start)/N*W;
   var y=H/2-(s[col]/ch[2])*(H/2-4);
   if(pen)g.lineTo(x,y);else{g.moveTo(x,y);pen=true;}
  }
  g.stroke();
 });
}
// Legend chips double as channel toggles.
CH.forEach(function(ch,i){
 var b=document.createElement('span');
 b.className='p'+(ch[3]?' on':'');
 b.style.cursor='pointer';b.style.borderColor=ch[1];b.textContent=ch[4];
 b.onclick=function(){ch[3]=ch[3]?0:1;b.className='p'+(ch[3]?' on':'');drawTrace();};
 $('tl').appendChild(b);
});
$('tpause').onclick=function(){
 tPause=!tPause;this.textContent=tPause?'Resume':'Pause';
 if(!tPause)drawTrace();
};
function traceCSV(){
 // Rebuild CSV from everything accumulated, gaps marked as blank lines.
 var out='seq,t,ax,ay,gx,gy,m1,m2,m3,px,py,gz,yc,pz\n';
 T.forEach(function(s){out+=s?s.join(',')+'\n':'\n';});
 return out;
}
$('tsave').onclick=function(){
 var url=URL.createObjectURL(new Blob([traceCSV()],{type:'text/csv'}));
 var a=document.createElement('a');
 a.href=url;a.download='cube-trace.csv';
 document.body.appendChild(a);a.click();a.remove();
 // Keep the data alive while the browser saves it: Safari aborts the
 // download if the URL is revoked straight after the click.
 setTimeout(function(){URL.revokeObjectURL(url);},60000);
 say(T.length+' samples - if no file appears, use Copy CSV');
};
$('tcopy').onclick=function(){
 // For browsers that cannot download at all, such as the phone's
 // captive-portal sign-in window: show the CSV in a text box, select it and
 // copy it.  execCommand because the async clipboard API needs https.
 var t=$('tcsv');t.value=traceCSV();t.style.display='block';
 t.focus();t.select();t.setSelectionRange(0,t.value.length);
 var ok=false;try{ok=document.execCommand('copy');}catch(e){}
 say(ok?'trace copied - paste it into a note or message'
       :'select the text below and copy it');
};
// --- button feedback --------------------------------------------------
// A press has to be visibly acknowledged even over a slow AP link, so every
// command button greys out while its request is in flight, then flashes the
// outcome.  msgUntil holds the header text against the 300 ms poll.
var msgUntil=0;
function say(t){$('s').textContent=t;msgUntil=Date.now()+2500;}
function flash(b,ok){
 if(!b)return;
 b.disabled=false;
 b.classList.remove('ok','er');
 void b.offsetWidth;                   // reflow: restarts the animation when
 b.classList.add(ok?'ok':'er');        // the same button is pressed again
}
// Sends a command request only; the main control loop acts on it.
// cmd may carry extra form fields ("turn&deg=90"); label names it for the
// status line when the raw body would read badly.
// Resolves to {ok, err} so a caller can show the outcome where it matters.
function send(cmd,b,label){
 var n=(label||cmd).replace('_',' ');
 // SAFE STOP is never disabled - a hung request must not make it unpressable.
 // Everything else re-enables on a watchdog in case no response ever arrives.
 if(b&&b.id!='stop'){b.disabled=true;
  setTimeout(function(){b.disabled=false;},3000);}
 say(n+'…');
 return fetch('/api/command',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:'cmd='+cmd})
 .then(function(r){return r.text().then(function(t){
   // Show the firmware's own reason ("disarm first", "stop pending") rather
   // than a bare status code - that is the part worth reading.
   var e='';try{e=JSON.parse(t).error||'';}catch(x){}
   say(r.ok?n+' sent':n+' rejected — '+(e||r.status));
   flash(b,r.ok);
   return {ok:r.ok,err:e||String(r.status)};
  });})
 .catch(function(){say(n+' failed');flash(b,false);
                   return {ok:false,err:'no connection to the cube'};});
}
// One binder for every command button, with an optional confirmation.
//
// The confirmation is IN-PAGE rather than confirm().  A browser stops
// honouring confirm() once the user (or the browser itself) blocks further
// dialogs, which happens after a couple in quick succession - so saving a
// calibration and then pressing ARM left ARM silently dead: the handler
// returned before sending anything, with no request, no error, and nothing
// on screen.  Captive-portal sign-in browsers suppress dialogs outright,
// which the portal added below makes far more likely to be what you are
// looking at.  Two taps on the button itself cannot be suppressed by
// anything, and keep the deliberate second action that makes ARM safe.
function bind(id,cmd,ask,label){
 var b=$(id),txt=b.textContent,t=0;
 function reset(){
  if(t)clearTimeout(t);
  t=0;b.textContent=txt;b.classList.remove('cf');
 }
 b.onclick=function(){
  if(ask&&!t){                          // first tap: ask, then wait
   b.textContent='Confirm?';b.classList.add('cf');say(ask);
   t=setTimeout(reset,4000);            // times out rather than staying armed
   return;
  }
  reset();                              // second tap (or no confirmation)
  send(cmd,this,label);
 };
}
bind('stop','stop');
bind('disarm','disarm');
// Arming re-enables balancing, so require a deliberate confirmation.
bind('arm','arm','Arm the cube? Balancing will resume.');
// --- gain editing -----------------------------------------------------
// The form is built from /api/gains so the firmware's gain table stays the
// single source of truth: add a gain there and it appears here too.
var G=null;
function loadG(){
 fetch('/api/gains',{cache:'no-store'}).then(function(r){return r.json();})
 .then(function(g){
  G=g; var h='';
  for(var k in g){
   h+='<label class="gr"><span>'+k+'</span><input id="g_'+k+
      '" type="number" step="any" min="'+g[k].lo+'" max="'+g[k].hi+
      '" value="'+(+g[k].v.toFixed(4))+'"></label>';
  }
  $('gl').innerHTML=h;
 });
}
loadG();
// Resolves to true when the firmware accepted every field.
function applyG(btn){
 if(!G)return Promise.resolve(false);
 // Send every field; the firmware validates each one and rejects the whole
 // request if any is out of range.
 var b=[];
 for(var k in G){b.push(k+'='+$('g_'+k).value);}
 if(btn)btn.disabled=true;
 $('gm').textContent='Applying…';
 return fetch('/api/gains',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:b.join('&')})
 .then(function(r){return r.json().then(function(j){
   $('gm').textContent=r.ok?'Applied. Save to keep them after a restart.'
                           :('Rejected — '+j.error);
   flash(btn,r.ok);
   if(r.ok)loadG();                    // re-read what the firmware accepted
   return r.ok;
  });})
 .catch(function(){$('gm').textContent='Apply failed.';flash(btn,false);
                   return false;});
}
// True when a gain field holds something other than what the firmware runs.
function gainsEdited(){
 if(!G)return false;
 for(var k in G){if(+$('g_'+k).value!==+G[k].v.toFixed(4))return true;}
 return false;
}
$('gapply').onclick=function(){applyG(this);};
// Restore Defaults just fills the form with the firmware's defaults and
// applies them - still not saved until SAVE is pressed.  Two-tap confirm for
// the same reason as bind(): confirm() is not dependable here.
(function(){
 var b=$('gdef'),txt=b.textContent,t=0;
 b.onclick=function(){
  if(!G)return;
  if(!t){
   b.textContent='Confirm?';b.classList.add('cf');
   $('gm').textContent='Tap again to restore every gain to its default.';
   t=setTimeout(function(){t=0;b.textContent=txt;b.classList.remove('cf');},4000);
   return;
  }
  clearTimeout(t);t=0;b.textContent=txt;b.classList.remove('cf');
  for(var k in G){$('g_'+k).value=+G[k].d.toFixed(4);}
  applyG(this);
 };
})();
// Save writes what the firmware is RUNNING, so a value typed but not yet
// applied used to be silently lost (vNom, 2026-10-04).  Apply pending edits
// first, then save; a rejected edit cancels the save.
(function(){
 var b=$('gsave'),txt=b.textContent,t=0;
 b.onclick=function(){
  if(!t){
   b.textContent='Confirm?';b.classList.add('cf');
   say(gainsEdited()?'Apply the edited gains and save them to EEPROM?'
                    :'Save gains and the learned trim to EEPROM?');
   t=setTimeout(function(){t=0;b.textContent=txt;b.classList.remove('cf');},4000);
   return;
  }
  clearTimeout(t);t=0;b.textContent=txt;b.classList.remove('cf');
  // The outcome stays in the panel; the status line only shows it briefly.
  // "Sent" is not "saved": the indicator above turns green when the firmware
  // has written EEPROM.
  function save(){
   return send('gains_save',b,'save gains').then(function(r){
    $('gm').textContent=r.ok?'Saving… the indicator above turns green once '
                             +'the gains are stored.'
                            :'NOT saved — '+r.err+'.';
   });
  }
  if(!gainsEdited()){save();return;}
  applyG(null).then(function(ok){
   if(ok)save();
   else $('gm').textContent='NOT saved — fix the rejected gain first.';
  });
 };
})();
// --- yaw slider -------------------------------------------------------
// Dragging fires continuously, so the rate is sent at most every 150 ms.
// The ESP32 serves one client at a time; an unthrottled slider would queue
// requests faster than they drain and stall the telemetry poll.
var dragging=false,ySent=0,yPend=null;
function sendYaw(v){
 fetch('/api/command',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:'cmd=yaw&rate='+v})
 .catch(function(){$('s').textContent='yaw failed';});
}
function yawChanged(){
 var v=$('yaw').value;
 $('yv').textContent=v+' °/s';
 var now=Date.now();
 if(now-ySent>150){ySent=now;sendYaw(v);}
 else{clearTimeout(yPend);
      yPend=setTimeout(function(){ySent=Date.now();sendYaw($('yaw').value);},150);}
}
$('yaw').oninput=function(){dragging=true;yawChanged();};
$('yaw').onchange=function(){dragging=false;yawChanged();};
$('ystop').onclick=function(){$('yaw').value=0;dragging=false;yawChanged();
 flash(this,true);say('spin stopped');};
// Heading: deg=0 is "hold where you are", anything else is a relative turn.
// Each also zeroes the slider, since the firmware drops the manual rate.
function heading(id,deg,label){$(id).onclick=function(){
 $('yaw').value=0;$('yv').textContent='0 °/s';dragging=false;
 send('turn&deg='+deg,this,label);};}
heading('hhold',0,'hold heading');
heading('hl90',-90,'turn -90°');
heading('hr90',90,'turn +90°');
heading('h180',180,'turn 180°');
$('hfree').onclick=function(){$('yaw').value=0;$('yv').textContent='0 °/s';
 dragging=false;send('yaw_free',this,'heading released');};
bind('treset','trim_reset');
// Auto-trim toggle: writes tK through the normal gains endpoint so the
// firmware's range check still applies.  OFF stashes the current rate and
// ON restores it, so a rate tuned in the Gains panel round-trips.
// ponytail: 0.005 first-use default is a conservative guess - tune tK in
// the Gains panel if it learns too slowly, the toggle remembers it.
$('ttog').onclick=function(){
 if(!G)return;
 var b=this,on=G.tK.v!=0,v=0;
 if(on)localStorage.tkOn=G.tK.v;
 else v=+localStorage.tkOn||0.005;
 b.disabled=true;
 fetch('/api/gains',{method:'POST',
  headers:{'Content-Type':'application/x-www-form-urlencoded'},
  body:'tK='+v})
 .then(function(r){
   flash(b,r.ok);
   say(r.ok?'auto-trim '+(on?'off':'on'):'auto-trim rejected');
   if(r.ok)loadG();                     // refresh G so poll() sees the state
  })
 .catch(function(){flash(b,false);say('auto-trim failed');});
};
bind('cstart','cal_start');
bind('ccap','cal_capture');
// Saving writes EEPROM, so confirm before spending a write cycle.
bind('csave','cal_save','Save calibration to EEPROM?');
</script></body></html>)rawliteral";

// GET /  — serve the dashboard straight from flash, pre-gzipped at build
// time (see tools/gzip_dashboard.py).  handleClient() pushes the whole page
// synchronously from loop(), stalling the control loop for the duration of
// the transfer, so the ~2.7x smaller gzip body directly shortens that stall.
// Every browser since ~2000 accepts gzip; no fallback needed on a device
// whose only clients are phones pointed at its own access point.
void handleRoot() {
  webServer.sendHeader("Content-Encoding", "gzip");
  webServer.send_P(200, "text/html",
                   reinterpret_cast<const char*>(DASHBOARD_GZ),
                   DASHBOARD_GZ_LEN);
}

// Anything that is not a route we serve gets redirected to the dashboard.
// With the wildcard DNS above, this single handler catches every platform's
// connectivity probe (Android's /generate_204, Apple's /hotspot-detect.html,
// Windows' /ncsi.txt and friends) without naming any of them - they all
// resolve here, and none of them match a registered path.
void handleCaptive() {
  // Built from the AP's actual address rather than hard-coded, so changing
  // the softAP configuration cannot leave this pointing somewhere dead.
  IPAddress ip = WiFi.softAPIP();
  char url[32];
  snprintf(url, sizeof(url), "http://%u.%u.%u.%u/", ip[0], ip[1], ip[2], ip[3]);
  webServer.sendHeader("Location", url, true);
  webServer.send(302, "text/plain", "");
}

// GET /api/state  — read-only telemetry snapshot as JSON.
//
// This handler only READS shared state; it never commands the motors.
// No locking is needed: handleWebInterface() is called from the same loop()
// as the balancing code, so this can never interrupt a control iteration
// mid-update — the values below are always from a completed iteration.
void handleApiState() {
  // Fixed stack buffer instead of String concatenation: bounded memory and
  // no heap fragmentation on a long-running controller.
  // 640 rather than 512: the raw accelerometer values and cal_result string
  // added when Bluetooth was removed push the worst case past the old size,
  // and snprintf truncates silently - which would emit malformed JSON.
  // 960 since batt_state, batt_comp, wheel_wind, yaw_guard and
  // gains_unsaved were added.
  char json[960];
  int n = snprintf(json, sizeof(json),
    "{"
      "\"robot_angleX\":%.3f,"
      "\"robot_angleY\":%.3f,"
      "\"gyroXfilt\":%.3f,"
      "\"gyroYfilt\":%.3f,"
      "\"gyroZ\":%.3f,"
      "\"motor1_speed\":%d,"
      "\"motor2_speed\":%d,"
      "\"motor3_speed\":%d,"
      "\"speed_X\":%.3f,"
      "\"speed_Y\":%.3f,"
      "\"vertical_vertex\":%s,"
      "\"vertical_edge\":%s,"
      "\"calibrated\":%s,"
      "\"calibrating\":%s,"
      "\"vertex_calibrated\":%s,"
      "\"armed\":%s,"
      "\"batt_voltage\":%.2f,"
      "\"batt_state\":\"%s\","      // none | ok | low | cutoff
      "\"batt_comp\":%.3f,"         // motor compensation factor, 1 = off
      // Raw accelerometer counts.  During a first-time calibration the
      // corrected angles above are computed from EEPROM garbage, so these
      // are the only trustworthy numbers - and the pose accept/reject
      // thresholds are applied to exactly these values.
      "\"acX\":%d,"
      "\"acY\":%d,"
      "\"acZ\":%d,"
      // Learned balance-point offset and the active yaw command.
      "\"trimX\":%.3f,"
      "\"trimY\":%.3f,"
      "\"yaw_rate\":%.1f,"
      // Dead-reckoned heading and whether the outer loop is driving it.
      "\"robot_yaw\":%.1f,"
      "\"yaw_hold\":%s,"
      "\"wheel_wind\":%.0f,"        // smoothed average wheel speed, counts/tick
      "\"yaw_guard\":%s,"           // spin blocked until the wheels slow
      "\"gains_unsaved\":%s,"       // running gains differ from EEPROM
      "\"cal_result\":\"%s\""
    "}",
    robot_angleX, robot_angleY,
    gyroXfilt, gyroYfilt, gyroZ,
    motor1_speed, motor2_speed, motor3_speed,
    speed_X, speed_Y,
    // JSON has no C-style booleans, so emit the literals true/false.
    vertical_vertex ? "true" : "false",
    vertical_edge   ? "true" : "false",
    calibrated      ? "true" : "false",
    calibrating       ? "true" : "false",
    vertex_calibrated ? "true" : "false",
    armed             ? "true" : "false",
    batt_voltage, battStateName(), batt_comp,
    AcX, AcY, AcZ, trimX, trimY, yaw_rate_cmd,
    robot_yaw, yaw_hold ? "true" : "false",
    wheel_wind, yaw_guard ? "true" : "false",
    gainsUnsaved() ? "true" : "false",
    cal_result);
  // Truncated JSON would be malformed, so refuse to send it rather than let
  // the dashboard silently fail to parse.
  if (n < 0 || n >= (int)sizeof(json)) {
    webServer.send(500, "application/json",
                   "{\"ok\":false,\"error\":\"state buffer overflow\"}");
    return;
  }
  webServer.send(200, "application/json", json);
}

// GET /api/gains  — current value, accepted range, and default for each gain.
// Read-only; the dashboard uses it to build the editing form.
void handleApiGains() {
  char json[1024];
  int n = 0;
  n += snprintf(json + n, sizeof(json) - n, "{");
  for (int i = 0; i < NUM_GAINS && n < (int)sizeof(json); i++) {
    n += snprintf(json + n, sizeof(json) - n,
                  "%s\"%s\":{\"v\":%.4f,\"lo\":%.4f,\"hi\":%.4f,\"d\":%.4f}",
                  i ? "," : "", GAIN_DEFS[i].name, *GAIN_DEFS[i].ptr,
                  GAIN_DEFS[i].lo, GAIN_DEFS[i].hi, GAIN_DEFS[i].def);
  }
  n += snprintf(json + n, sizeof(json) - n, "}");
  if (n >= (int)sizeof(json)) {         // never send truncated JSON
    webServer.send(500, "application/json",
                   "{\"ok\":false,\"error\":\"gain buffer overflow\"}");
    return;
  }
  webServer.send(200, "application/json", json);
}

// POST /api/gains  — temporarily change one or more gains, e.g. "K1=185&K3=2".
//
// Changes apply in RAM only; they are lost on restart unless the user then
// presses Save.  Every supplied value is validated BEFORE any is applied, so
// a single bad field cannot leave the controller half-updated.
//
// Assigning the gains here is safe despite this being an HTTP handler:
// handleClient() is called from loop(), so this code cannot interrupt a
// control-loop iteration - it runs strictly between them.
void handleApiGainsSet() {
  float staged[NUM_GAINS];
  bool  present[NUM_GAINS] = {false};
  int   count = 0;

  // Pass 1: parse and validate everything.
  for (int i = 0; i < NUM_GAINS; i++) {
    if (!webServer.hasArg(GAIN_DEFS[i].name)) continue;
    String raw = webServer.arg(GAIN_DEFS[i].name);

    // strtod rather than toFloat(): toFloat() silently returns 0 for
    // garbage, which would quietly zero a gain instead of reporting an error.
    const char* s = raw.c_str();
    char* end;
    double v = strtod(s, &end);
    while (*end == ' ') end++;                 // tolerate trailing spaces
    bool bad = (end == s) || (*end != '\0') || isnan(v) || isinf(v);
    if (!bad && (v < GAIN_DEFS[i].lo || v > GAIN_DEFS[i].hi)) bad = true;

    if (bad) {
      char err[160];
      snprintf(err, sizeof(err),
               "{\"ok\":false,\"error\":\"%s must be a number between "
               "%.4f and %.4f\"}",
               GAIN_DEFS[i].name, GAIN_DEFS[i].lo, GAIN_DEFS[i].hi);
      webServer.send(400, "application/json", err);
      return;                                  // nothing applied
    }
    staged[i] = (float)v;
    present[i] = true;
    count++;
  }

  if (count == 0) {
    webServer.send(400, "application/json",
                   "{\"ok\":false,\"error\":\"no known gain supplied\"}");
    return;
  }

  // Pass 2: everything validated, so apply.
  for (int i = 0; i < NUM_GAINS; i++)
    if (present[i]) *GAIN_DEFS[i].ptr = staged[i];

  char json[64];
  snprintf(json, sizeof(json), "{\"ok\":true,\"applied\":%d}", count);
  webServer.send(200, "application/json", json);
}

// GET /api/trace?since=N  — control-loop telemetry as CSV, one line per
// 15 ms sample, starting after sequence number N.
//
// The browser polls this alongside /api/state and accumulates the stream,
// so each response only carries the samples since the previous poll
// (~20 lines, under one TCP segment).  It deliberately does NOT dump the
// whole ring on demand: a multi-kilobyte response would stall the control
// loop for several periods, which is exactly what the gzip work removed.
// The seq column lets the client detect dropped samples honestly.
void handleApiTrace() {
  uint32_t since = 0;
  if (webServer.hasArg("since"))
    since = strtoul(webServer.arg("since").c_str(), NULL, 10);

  // Clamp the request to what the ring still holds.  trace_seq is the next
  // sequence to be written, so valid history is [trace_seq-TRACE_LEN,
  // trace_seq).  A client that fell far behind just resumes from the oldest
  // sample and sees the gap in the seq column.
  uint32_t oldest = trace_seq > TRACE_LEN ? trace_seq - TRACE_LEN : 0;
  uint32_t from = since + 1;
  if (from < oldest) from = oldest;

  // Cap one response at 40 samples (~2.4 KB) so a lagging client catches up
  // over a few polls instead of provoking one long loop-stalling send.
  uint32_t upto = trace_seq;
  if (upto - from > 40) upto = from + 40;

  // Worst-case line: 10-digit seq + 10-digit t + twelve 6-char int16 fields
  // plus separators = 107 chars.  Sized for that, not the typical ~55, so a
  // batch is never silently shortened by large values.
  static char csv[40 * 108 + 80];
  int n = 0;
  n += snprintf(csv + n, sizeof(csv) - n,
                "seq,t,ax,ay,gx,gy,m1,m2,m3,px,py,gz,yc,pz\n");
  for (uint32_t q = from; q < upto && n < (int)sizeof(csv) - 108; q++) {
    TraceSample& s = trace_buf[q % TRACE_LEN];
    if (s.seq != q) continue;          // overwritten mid-read: skip honestly
    n += snprintf(csv + n, sizeof(csv) - n,
                  "%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                  (unsigned long)s.seq, (unsigned long)s.t_ms,
                  s.angX10, s.angY10, s.gyrX10, s.gyrY10,
                  s.m1, s.m2, s.m3, s.pwmX, s.pwmY,
                  s.gyrZ10, s.ycmd10, s.pwmZ);
  }
  webServer.send(200, "text/csv", csv);
}

// POST /api/command  — accepts "cmd=stop", "cmd=disarm" or "cmd=arm".
//
// SAFETY: this handler does not command the motors and does not change the
// balancing state itself.  It only records a request; the main control loop
// applies it at the start of its next cycle (within one 15 ms period).
void handleApiCommand() {
  if (!webServer.hasArg("cmd")) {
    webServer.send(400, "application/json",
                   "{\"ok\":false,\"error\":\"missing cmd\"}");
    return;
  }
  String cmd = webServer.arg("cmd");

  uint8_t req;
  // Commands that must not run while the motors are balancing: the
  // calibration steps, and anything that writes EEPROM.
  bool needs_idle = false;
  if (cmd == "stop")        req = WEB_CMD_STOP;
  else if (cmd == "disarm") req = WEB_CMD_DISARM;
  else if (cmd == "arm")    req = WEB_CMD_ARM;
  else if (cmd == "cal_start")   { req = WEB_CMD_CAL_START;   needs_idle = true; }
  else if (cmd == "cal_capture") { req = WEB_CMD_CAL_CAPTURE; needs_idle = true; }
  else if (cmd == "cal_save")    { req = WEB_CMD_CAL_SAVE;    needs_idle = true; }
  // Saving gains writes EEPROM, so it needs an idle cube for the same
  // reason the calibration commands do.
  else if (cmd == "gains_save")  { req = WEB_CMD_GAINS_SAVE;  needs_idle = true; }
  else if (cmd == "trim_reset")  req = WEB_CMD_TRIM_RESET;
  else if (cmd == "yaw") {
    // Yaw is a setpoint, not an action: record the requested rate and
    // return.  The control loop clamps and applies it on its next pass;
    // nothing here touches a motor.
    if (!webServer.hasArg("rate")) {
      webServer.send(400, "application/json",
                     "{\"ok\":false,\"error\":\"yaw needs a rate\"}");
      return;
    }
    String raw = webServer.arg("rate");
    const char* s = raw.c_str();
    char* end;
    double v = strtod(s, &end);
    while (*end == ' ') end++;
    if (end == s || *end != '\0' || isnan(v) || isinf(v)
        || v < -YAW_RATE_MAX || v > YAW_RATE_MAX) {
      char err[128];
      snprintf(err, sizeof(err),
               "{\"ok\":false,\"error\":\"rate must be a number between "
               "%.0f and %.0f deg/s\"}", -YAW_RATE_MAX, YAW_RATE_MAX);
      webServer.send(400, "application/json", err);
      return;
    }
    // Wind-up guard: no new spin until the wheels have slowed down.
    if (yaw_guard && v != 0) {
      webServer.send(409, "application/json",
                     "{\"ok\":false,\"error\":\"wheels wound up"
                     " - wait for them to slow down\"}");
      return;
    }
    yaw_rate_request = (float)v;
    // The slider is manual control, so taking it overrides any heading the
    // outer loop was chasing - otherwise the two would fight over
    // yaw_rate_cmd and the hold would silently win every iteration.
    yaw_hold = false;
    yaw_turn_new = false;
    webServer.send(200, "application/json", "{\"ok\":true,\"cmd\":\"yaw\"}");
    return;
  }
  else if (cmd == "turn") {
    // Turn a relative number of degrees and hold there; deg=0 means "hold
    // whatever heading you have right now".  Like yaw above this is only a
    // setpoint - the control loop reads it inside the vertex branch, which
    // is also the only place a heading exists to be relative to.
    if (!webServer.hasArg("deg")) {
      webServer.send(400, "application/json",
                     "{\"ok\":false,\"error\":\"turn needs deg\"}");
      return;
    }
    String raw = webServer.arg("deg");
    const char* s = raw.c_str();
    char* end;
    double v = strtod(s, &end);
    while (*end == ' ') end++;
    if (end == s || *end != '\0' || isnan(v) || isinf(v)
        || v < -YAW_TURN_MAX || v > YAW_TURN_MAX) {
      char err[128];
      snprintf(err, sizeof(err),
               "{\"ok\":false,\"error\":\"deg must be a number between "
               "%.0f and %.0f\"}", -YAW_TURN_MAX, YAW_TURN_MAX);
      webServer.send(400, "application/json", err);
      return;
    }
    // Wind-up guard: a hold (deg=0) is fine, a new turn waits.
    if (yaw_guard && v != 0) {
      webServer.send(409, "application/json",
                     "{\"ok\":false,\"error\":\"wheels wound up"
                     " - wait for them to slow down\"}");
      return;
    }
    // Order matters: the request must be in place before the loop is told
    // to look at it, or it could latch a stale value.
    yaw_turn_request = (float)v;
    yaw_turn_new = true;
    yaw_rate_request = 0;        // leave no manual rate to fall back into
    webServer.send(200, "application/json", "{\"ok\":true,\"cmd\":\"turn\"}");
    return;
  }
  else if (cmd == "yaw_free") {
    // Release the heading loop without commanding any rotation.
    yaw_hold = false;
    yaw_turn_new = false;
    yaw_rate_request = 0;
    webServer.send(200, "application/json", "{\"ok\":true,\"cmd\":\"yaw_free\"}");
    return;
  }
  else {
    // Reject anything unrecognised rather than silently ignoring it.
    webServer.send(400, "application/json",
                   "{\"ok\":false,\"error\":\"unknown cmd\"}");
    return;
  }

  // Never calibrate or write EEPROM while the motors are actively balancing.
  // Rejecting here
  // gives the user immediate feedback; the control loop re-checks before
  // acting, since the cube could start balancing in between.
  if (needs_idle && balancingActive()) {
    webServer.send(409, "application/json",
                   "{\"ok\":false,\"error\":\"not while balancing"
                   " - disarm first\"}");
    return;
  }

  // A battery cutoff blocks arming until the pack recovers; say why here
  // rather than let the control loop drop the request silently.
  if (req == WEB_CMD_ARM && batt_state == BATT_CUTOFF) {
    webServer.send(409, "application/json",
                   "{\"ok\":false,\"error\":\"battery cutoff"
                   " - charge or replace the pack\"}");
    return;
  }

  // A stop already waiting to be processed always wins.  Without this, an
  // arm arriving in the same 15 ms window could overwrite a pending stop
  // and the cube would never stop at all.
  if (web_cmd_pending == WEB_CMD_STOP && req != WEB_CMD_STOP) {
    webServer.send(409, "application/json",
                   "{\"ok\":false,\"error\":\"stop pending\"}");
    return;
  }
  web_cmd_pending = req;

  char json[64];
  snprintf(json, sizeof(json), "{\"ok\":true,\"cmd\":\"%s\"}", cmd.c_str());
  webServer.send(200, "application/json", json);
}

// Called once from setup(): bring up the access point, register routes,
// and start the HTTP server.
void startWebInterface() {
  WiFi.mode(WIFI_AP);                      // stand-alone AP, no router needed

  // softAP() returns false if the AP could not be started.  The most common
  // cause is a password shorter than the 8-character WPA2 minimum, which
  // makes the network silently never appear — so report failure loudly.
  if (!WiFi.softAP(WIFI_NAME, WIFI_PASSWORD)) {
    Serial.println("ERROR: Wi-Fi AP failed to start!"
                   "  (password must be at least 8 characters)");
    // No AP means no dashboard, and therefore no SAFE STOP button.  Do not
    // leave the cube able to spin up three reaction wheels with no reachable
    // way to stop it: disarm, and let the operator re-arm over USB serial
    // once they can see what is going on.
    armed = false;
    Serial.println("Balancing disarmed: no web interface available.");
    return;                                // no AP: skip starting the server
  }

  // Print the address the phone should open (the AP's own IP).
  Serial.print("Wi-Fi AP \"");
  Serial.print(WIFI_NAME);
  Serial.print("\" started.  Open http://");
  Serial.println(WiFi.softAPIP());

  // Wildcard DNS on port 53: every lookup answers with the cube's address,
  // which is what turns a phone's connectivity probe into "tap to open the
  // dashboard".  Failure here is not fatal - the IP still works.
  if (!dnsServer.start(53, "*", WiFi.softAPIP()))
    Serial.println("WARNING: captive-portal DNS failed to start"
                   " (dashboard still reachable by IP).");

  // mDNS for laptops: http://cube.local.  Also non-fatal.
  if (MDNS.begin(MDNS_NAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.print("Also reachable at http://");
    Serial.print(MDNS_NAME);
    Serial.println(".local");
  } else {
    Serial.println("WARNING: mDNS failed to start (use the IP address).");
  }

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/api/state", HTTP_GET, handleApiState);
  webServer.on("/api/command", HTTP_POST, handleApiCommand);
  webServer.on("/api/trace", HTTP_GET, handleApiTrace);
  webServer.on("/api/gains", HTTP_GET, handleApiGains);
  webServer.on("/api/gains", HTTP_POST, handleApiGainsSet);
  // Registered last so it can only ever catch paths none of the above claim.
  webServer.onNotFound(handleCaptive);
  webServer.begin();
}

// Called every pass of loop(): service at most one pending HTTP request.
// handleClient() is non-blocking and returns immediately when idle, so it
// does not disturb the 15 ms balancing period.
void handleWebInterface() {
  // Non-blocking: reads at most one waiting UDP packet and answers it.  A
  // DNS reply is a single small datagram, so this costs far less than the
  // HTTP serving below it.
  dnsServer.processNextRequest();
  webServer.handleClient();
}
