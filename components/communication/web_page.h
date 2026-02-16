static const char INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>

<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>ESP32 CAM Robot</title>
    <style>
        :root {
            --bg: #0b0f14;
            --panel: #111826;
            --panel2: #0f1622;
            --text: #e8eef7;
            --muted: #a9b4c2;
            --brand: #4247b7;
            --brand2: #1cb8bd;
            --danger: #ff3034;
            --border: rgba(255, 255, 255, 0.10);
            --shadow: 0 10px 30px rgba(0,0,0,.35);
            --r: 14px;
        }

        * { box-sizing: border-box; }
        html, body { height: 100%; }
        body {
            margin: 0;
            font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, Helvetica, sans-serif;
            background: var(--bg);
            color: var(--text);
            font-size: 16px;
        }

        .app {
            max-width: 1100px;
            margin: 0 auto;
            padding: 12px;
        }

        header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 12px;
            margin-bottom: 12px;
        }

        .title {
            display: flex;
            flex-direction: column;
            gap: 2px;
        }

        .title h1 {
            margin: 0;
            font-size: 18px;
            font-weight: 700;
            letter-spacing: .2px;
        }

        .title .sub {
            font-size: 12px;
            color: var(--muted);
        }

        .tabs {
            display: inline-flex;
            gap: 8px;
            background: var(--panel2);
            border: 1px solid var(--border);
            padding: 6px;
            border-radius: 999px;
            box-shadow: var(--shadow);
        }

        .tab {
            appearance: none;
            border: 0;
            background: transparent;
            color: var(--muted);
            padding: 8px 12px;
            border-radius: 999px;
            cursor: pointer;
            font-weight: 600;
            line-height: 1;
        }
        .tab[aria-selected="true"] {
            background: rgba(255,255,255,.08);
            color: var(--text);
        }

        .page { display: none; }
        .page.active { display: block; }

        .grid {
            display: grid;
            grid-template-columns: 1fr;
            gap: 12px;
        }
        @media (min-width: 900px) {
            .grid.cols2 { grid-template-columns: 1.2fr .8fr; }
        }

        .card {
            background: linear-gradient(180deg, var(--panel), var(--panel2));
            border: 1px solid var(--border);
            border-radius: var(--r);
            box-shadow: var(--shadow);
            padding: 12px;
        }
        .card h2 {
            margin: 0 0 10px;
            font-size: 14px;
            letter-spacing: .3px;
            text-transform: uppercase;
            color: var(--muted);
            font-weight: 700;
        }

        .row {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 10px;
            flex-wrap: wrap;
        }

        .btn {
            appearance: none;
            border: 1px solid var(--border);
            background: rgba(255,255,255,.06);
            color: var(--text);
            border-radius: 12px;
            padding: 10px 12px;
            cursor: pointer;
            font-weight: 700;
            line-height: 1;
            user-select: none;
            -webkit-user-select: none;
            touch-action: manipulation;
        }

        .btn.primary { background: var(--brand); border-color: transparent; }
        .btn.accent { background: var(--brand2); border-color: transparent; }
        .btn.danger { background: rgba(255,48,52,.12); border-color: rgba(255,48,52,.35); color: #ffd3d5; }
        .btn:active { transform: translateY(1px); }

        .controls {
            display: grid;
            grid-template-columns: 1fr;
            gap: 10px;
        }
        @media (min-width: 520px) {
            .controls { grid-template-columns: 1fr 1fr; }
        }

        .dpad {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            align-items: center;
        }
        .dpad .sp { visibility: hidden; }

        .slider {
            display: grid;
            grid-template-columns: 90px 1fr 40px;
            gap: 10px;
            align-items: center;
        }

        input[type="range"] { width: 100%; }
        input[type="number"] {
            width: 90px;
            padding: 8px 10px;
            border-radius: 12px;
            border: 1px solid var(--border);
            background: rgba(0,0,0,.15);
            color: var(--text);
        }

        .streamWrap {
            position: relative;
        }
        .streamBar {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 10px;
            margin-bottom: 10px;
        }
        .stream {
            width: 100%;
            display: block;
            border-radius: 12px;
            border: 1px solid var(--border);
            background: rgba(0,0,0,.25);
            min-height: 140px;
            object-fit: contain;
        }
        .close {
            width: 34px;
            height: 34px;
            border-radius: 999px;
            border: 1px solid rgba(255,48,52,.35);
            background: rgba(255,48,52,.12);
            color: #ffd3d5;
            cursor: pointer;
            font-weight: 900;
        }
        .hidden { display: none; }

        .servoGrid {
            display: grid;
            grid-template-columns: 1fr;
            gap: 8px;
        }
        @media (min-width: 600px) {
            .servoGrid { grid-template-columns: 1fr 1fr; }
        }
        .servoRow {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 8px;
            padding: 8px;
            border: 1px solid var(--border);
            border-radius: 12px;
            background: rgba(255,255,255,.03);
        }
        .servoRow .lbl {
            font-weight: 800;
            color: var(--muted);
            width: 34px;
        }

        pre {
            white-space: pre-wrap;
            word-break: break-word;
            margin: 0;
            padding: 10px;
            border-radius: 12px;
            border: 1px solid var(--border);
            background: rgba(0,0,0,.18);
            color: var(--text);
            font-size: 12px;
            line-height: 1.35;
        }

        .twoCol {
            display: grid;
            grid-template-columns: 1fr;
            gap: 12px;
        }
        @media (min-width: 900px) {
            .twoCol { grid-template-columns: 1fr 1fr; }
        }

        .legGrid {
            display: grid;
            grid-template-columns: 1fr;
            gap: 10px;
        }
        .legRow {
            display: grid;
            grid-template-columns: 56px 1fr 1fr 1fr 64px;
            gap: 8px;
            align-items: center;
            padding: 8px;
            border: 1px solid var(--border);
            border-radius: 12px;
            background: rgba(255,255,255,.03);
        }
        .legRow .legLbl {
            color: var(--muted);
            font-weight: 900;
            letter-spacing: .3px;
        }
        .legRow .hdr {
            color: var(--muted);
            font-size: 11px;
            font-weight: 800;
        }
        .legRow .cell {
            display: flex;
            flex-direction: column;
            gap: 4px;
        }
        .legRow .cell input { width: 100%; }
        @media (max-width: 520px) {
            .legRow {
                grid-template-columns: 52px 1fr 1fr;
                grid-template-areas:
                    "l a a"
                    "l b b"
                    "l c c"
                    "l btn btn";
            }
            .legRow .legLbl { grid-area: l; align-self: start; padding-top: 4px; }
            .legRow .cell.a { grid-area: a; }
            .legRow .cell.b { grid-area: b; }
            .legRow .cell.c { grid-area: c; }
            .legRow button { grid-area: btn; width: 100%; }
        }
    </style>
</head>

<body>
    <div class="app">
        <header>
            <div class="title">
                <h1>ESP32 CAM Robot</h1>
                <div class="sub">Control • Calibration • PWM</div>
            </div>
            <div class="tabs" role="tablist" aria-label="Pages">
                <button class="tab" id="tab-control" role="tab" aria-selected="true" aria-controls="page-control">Control</button>
                <button class="tab" id="tab-calib" role="tab" aria-selected="false" aria-controls="page-calib">Calibration</button>
            </div>
        </header>

        <section class="page active" id="page-control" role="tabpanel" aria-labelledby="tab-control">
            <div class="grid">
                <div class="card">
                    <h2>Stream</h2>
                    <div id="stream-container" class="streamWrap">
                        <div class="streamBar">
                            <button class="btn primary" id="toggle-stream">Start</button>
                            <button class="close" id="close-stream" title="Hide stream">×</button>
                        </div>
                        <img id="stream" class="stream" src="" alt="stream">
                    </div>
                </div>

                <div class="twoCol">
                    <div class="grid" style="gap:12px;">
                        <div class="card">
                            <h2>Height</h2>
                            <div class="slider">
                                <div style="color:var(--muted); font-weight:700;">Height</div>
                                <input id="height-slider" type="range" min="10" max="60" value="30">
                                <div id="height-value" style="text-align:right; font-weight:800;">30</div>
                            </div>
                        </div>

                        <div class="card">
                            <h2>Move</h2>
                            <div class="dpad">
                                <div class="sp">.</div>
                                <button class="btn accent" id="forward" data-hold-move="1" data-hold-cmd="50">Forward</button>
                                <div class="sp">.</div>

                                <button class="btn accent" id="turnleft" data-hold-move="2" data-hold-cmd="40" data-stop-move="6" data-stop-cmd="0">Left</button>
                                <button class="btn" id="steady" data-click-func="1" data-click-cmd="0">Steady</button>
                                <button class="btn accent" id="turnright" data-hold-move="4" data-hold-cmd="40" data-stop-move="6" data-stop-cmd="0">Right</button>

                                <div class="sp">.</div>
                                <button class="btn accent" id="backward" data-hold-move="5" data-hold-cmd="50">Reverse</button>
                                <div class="sp">.</div>
                            </div>
                        </div>

                        <div class="card">
                            <h2>Actions</h2>
                            <div class="controls">
                                <button class="btn" id="stayLow" data-click-func="9" data-click-cmd="0">Stand</button>
                                <button class="btn" id="handShake" data-click-func="1" data-click-cmd="40">Wave</button>
                                <button class="btn" id="Jump" data-click-func="9" data-click-cmd="0">Reset</button>

                                <button class="btn" id="actionA" data-click-func="5" data-click-cmd="0">Action A</button>
                                <button class="btn" id="actionB" data-click-func="6" data-click-cmd="0">Action B</button>
                                <button class="btn" id="actionC" data-click-func="7" data-click-cmd="0">Action C</button>

                                <button class="btn" id="initPos" data-click-func="8" data-click-cmd="0">Init Pos</button>
                                <button class="btn" id="middlePos" data-click-func="9" data-click-cmd="0">Middle Pos</button>
                            </div>
                        </div>
                    </div>

                    <div class="card">
                        <h2>Single Leg Control</h2>
                        <div class="legGrid">
                            <div class="legRow" data-leg="A" data-servo-a="8" data-servo-b="9" data-servo-c="10">
                                <div class="legLbl">Leg A</div>
                                <div class="cell a"><div class="hdr">FORE</div><input type="number" min="0" max="180" step="1" value="45" data-angle="a"></div>
                                <div class="cell b"><div class="hdr">BACK</div><input type="number" min="0" max="180" step="1" value="45" data-angle="b"></div>
                                <div class="cell c"><div class="hdr">WAVE</div><input type="number" min="0" max="180" step="1" value="45" data-angle="c"></div>
                                <button class="btn primary" data-apply-leg="A">Apply</button>
                            </div>

                            <div class="legRow" data-leg="B" data-servo-a="13" data-servo-b="14" data-servo-c="15">
                                <div class="legLbl">Leg B</div>
                                <div class="cell a"><div class="hdr">WAVE</div><input type="number" min="0" max="180" step="1" value="45" data-angle="a"></div>
                                <div class="cell b"><div class="hdr">FORE</div><input type="number" min="0" max="180" step="1" value="45" data-angle="b"></div>
                                <div class="cell c"><div class="hdr">BACK</div><input type="number" min="0" max="180" step="1" value="45" data-angle="c"></div>
                                <button class="btn primary" data-apply-leg="B">Apply</button>
                            </div>

                            <div class="legRow" data-leg="C" data-servo-a="7" data-servo-b="6" data-servo-c="5">
                                <div class="legLbl">Leg C</div>
                                <div class="cell a"><div class="hdr">FORE</div><input type="number" min="0" max="180" step="1" value="45" data-angle="a"></div>
                                <div class="cell b"><div class="hdr">BACK</div><input type="number" min="0" max="180" step="1" value="45" data-angle="b"></div>
                                <div class="cell c"><div class="hdr">WAVE</div><input type="number" min="0" max="180" step="1" value="45" data-angle="c"></div>
                                <button class="btn primary" data-apply-leg="C">Apply</button>
                            </div>

                            <div class="legRow" data-leg="D" data-servo-a="2" data-servo-b="1" data-servo-c="0">
                                <div class="legLbl">Leg D</div>
                                <div class="cell a"><div class="hdr">WAVE</div><input type="number" min="0" max="180" step="1" value="45" data-angle="a"></div>
                                <div class="cell b"><div class="hdr">FORE</div><input type="number" min="0" max="180" step="1" value="45" data-angle="b"></div>
                                <div class="cell c"><div class="hdr">BACK</div><input type="number" min="0" max="180" step="1" value="45" data-angle="c"></div>
                                <button class="btn primary" data-apply-leg="D">Apply</button>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </section>

        <section class="page" id="page-calib" role="tabpanel" aria-labelledby="tab-calib">
            <div class="grid">
                <div class="card">
                    <h2>NVS & Calibration</h2>
                    <div class="row" style="margin-bottom:10px;">
                        <button class="btn primary" id="btn-load-nvs">Load NVS</button>
                        <button class="btn danger" id="btn-reset-cal">Reset Cal</button>
                        <button class="btn" id="btn-refresh">Refresh</button>
                    </div>
                    <pre id="calib-view">Cal data: (fetching...)</pre>
                </div>

                <div class="card">
                    <h2>PWM</h2>
                    <div class="servoGrid">
                        <div class="servoRow"><div class="lbl">S0</div><input id="pwm0" type="number"><button class="btn" data-set-pwm="0">Set</button></div>
                        <div class="servoRow"><div class="lbl">S1</div><input id="pwm1" type="number"><button class="btn" data-set-pwm="1">Set</button></div>
                        <div class="servoRow"><div class="lbl">S2</div><input id="pwm2" type="number"><button class="btn" data-set-pwm="2">Set</button></div>
                        <div class="servoRow"><div class="lbl">S3</div><input id="pwm3" type="number"><button class="btn" data-set-pwm="3">Set</button></div>
                        <div class="servoRow"><div class="lbl">S4</div><input id="pwm4" type="number"><button class="btn" data-set-pwm="4">Set</button></div>
                        <div class="servoRow"><div class="lbl">S5</div><input id="pwm5" type="number"><button class="btn" data-set-pwm="5">Set</button></div>
                        <div class="servoRow"><div class="lbl">S6</div><input id="pwm6" type="number"><button class="btn" data-set-pwm="6">Set</button></div>
                        <div class="servoRow"><div class="lbl">S7</div><input id="pwm7" type="number"><button class="btn" data-set-pwm="7">Set</button></div>
                        <div class="servoRow"><div class="lbl">S8</div><input id="pwm8" type="number"><button class="btn" data-set-pwm="8">Set</button></div>
                        <div class="servoRow"><div class="lbl">S9</div><input id="pwm9" type="number"><button class="btn" data-set-pwm="9">Set</button></div>
                        <div class="servoRow"><div class="lbl">S10</div><input id="pwm10" type="number"><button class="btn" data-set-pwm="10">Set</button></div>
                        <div class="servoRow"><div class="lbl">S11</div><input id="pwm11" type="number"><button class="btn" data-set-pwm="11">Set</button></div>
                        <div class="servoRow"><div class="lbl">S12</div><input id="pwm12" type="number"><button class="btn" data-set-pwm="12">Set</button></div>
                        <div class="servoRow"><div class="lbl">S13</div><input id="pwm13" type="number"><button class="btn" data-set-pwm="13">Set</button></div>
                        <div class="servoRow"><div class="lbl">S14</div><input id="pwm14" type="number"><button class="btn" data-set-pwm="14">Set</button></div>
                        <div class="servoRow"><div class="lbl">S15</div><input id="pwm15" type="number"><button class="btn" data-set-pwm="15">Set</button></div>
                    </div>
                </div>
            </div>
        </section>
    </div>
    <script>
    document.addEventListener('DOMContentLoaded', function(){
        var baseHost = document.location.origin
        var streamUrl = `${document.location.protocol}//${document.location.hostname}:81`
        const calibView = document.getElementById('calib-view')
        const heightSlider = document.getElementById('height-slider')
        const heightValue = document.getElementById('height-value')

        const tabControl = document.getElementById('tab-control')
        const tabCalib = document.getElementById('tab-calib')
        const pageControl = document.getElementById('page-control')
        const pageCalib = document.getElementById('page-calib')

        const setPage = (page) => {
            const isControl = page === 'control'
            tabControl.setAttribute('aria-selected', isControl ? 'true' : 'false')
            tabCalib.setAttribute('aria-selected', isControl ? 'false' : 'true')
            pageControl.classList.toggle('active', isControl)
            pageCalib.classList.toggle('active', !isControl)
            if (!isControl) {
                refreshCalibration()
            }
        }

        tabControl.addEventListener('click', () => setPage('control'))
        tabCalib.addEventListener('click', () => setPage('calib'))

        const currentHeight = () => (heightSlider ? heightSlider.value : '95')
        if (heightSlider && heightValue) {
            heightValue.textContent = heightSlider.value
            heightSlider.addEventListener('input', () => {
                heightValue.textContent = heightSlider.value
            })
        }

        const sendCmd = (varName, val, cmd) => {
            const h = currentHeight()
            return fetch(`${baseHost}/control?var=${varName}&val=${val}&cmd=${cmd}&height=${h}`)
        }

        const sendMove = (val, cmd) => sendCmd('move', val, cmd)
        const sendFunc = (val, cmd) => sendCmd('funcMode', val, cmd)
        const sendServoAngle = (servoId, angle) => fetch(`${baseHost}/control?var=sangle&val=${servoId}&cmd=${angle}`)

        const view = document.getElementById('stream')
        const viewContainer = document.getElementById('stream-container')
        const streamButton = document.getElementById('toggle-stream')
        const closeButton = document.getElementById('close-stream')

        const stopStream = () => {
            window.stop();
            streamButton.innerHTML = 'Start'
        }

        const startStream = () => {
            view.src = `${streamUrl}/stream`
            streamButton.innerHTML = 'Stop'
        }

        const hideStream = () => {
            viewContainer.classList.add('hidden')
            stopStream()
        }

        const showStream = () => {
            viewContainer.classList.remove('hidden')
        }

        streamButton.onclick = () => {
            const streamEnabled = streamButton.innerHTML === 'Stop'
            if (streamEnabled) {
                stopStream()
            } else {
                showStream()
                startStream()
            }
        }

        closeButton.onclick = () => {
            hideStream()
        }

        const refreshCalibration = () => {
            fetch(`${baseHost}/servo/config`).then(r => r.json()).then(data => {
                calibView.textContent = `Middle: ${data.middle.join(', ')}\nDir: ${data.direction.join(', ')}`
                data.middle.forEach((m, idx) => {
                    const input = document.getElementById(`pwm${idx}`);
                    if (input) {
                        input.value = m;
                    }
                });
            }).catch(() => {
                calibView.textContent = 'Cal data: unavailable'
            })
        }

        const setPWM = (servoId) => {
            const input = document.getElementById(`pwm${servoId}`);
            const value = input.value;
            if (value !== '') {
                fetch(`${baseHost}/control?var=ssetval&val=${servoId}&cmd=${value}`);
            }
        }

        // Wire move/action buttons (mouse + touch via pointer events)
        const bindHoldMove = (btn) => {
            const startVal = parseInt(btn.getAttribute('data-hold-move'))
            const startCmd = parseInt(btn.getAttribute('data-hold-cmd'))
            const stopValAttr = btn.getAttribute('data-stop-move')
            const stopCmdAttr = btn.getAttribute('data-stop-cmd')
            const stopVal = stopValAttr ? parseInt(stopValAttr) : 3
            const stopCmd = stopCmdAttr ? parseInt(stopCmdAttr) : 0

            const down = (e) => {
                e.preventDefault()
                sendMove(startVal, startCmd)
            }
            const up = (e) => {
                e.preventDefault()
                sendMove(stopVal, stopCmd)
            }
            btn.addEventListener('pointerdown', down)
            btn.addEventListener('pointerup', up)
            btn.addEventListener('pointercancel', up)
            btn.addEventListener('pointerleave', (e) => {
                if (e.pressure === 0) return
                up(e)
            })
        }

        document.querySelectorAll('[data-hold-move]').forEach(bindHoldMove)
        document.querySelectorAll('[data-click-func]').forEach((btn) => {
            btn.addEventListener('click', () => {
                const v = parseInt(btn.getAttribute('data-click-func'))
                const c = parseInt(btn.getAttribute('data-click-cmd'))
                sendFunc(v, c)
            })
        })

        document.querySelectorAll('[data-set-pwm]').forEach((btn) => {
            btn.addEventListener('click', () => {
                const id = parseInt(btn.getAttribute('data-set-pwm'))
                setPWM(id)
            })
        })

        // Single leg control: apply 3 servo angles per leg
        const applyLegRow = (row) => {
            const sA = parseInt(row.getAttribute('data-servo-a'))
            const sB = parseInt(row.getAttribute('data-servo-b'))
            const sC = parseInt(row.getAttribute('data-servo-c'))
            const a = row.querySelector('input[data-angle="a"]')
            const b = row.querySelector('input[data-angle="b"]')
            const c = row.querySelector('input[data-angle="c"]')
            const angA = a && a.value !== '' ? parseFloat(a.value) : 45
            const angB = b && b.value !== '' ? parseFloat(b.value) : 45
            const angC = c && c.value !== '' ? parseFloat(c.value) : 45
            // Send sequentially to keep it simple and deterministic
            sendServoAngle(sA, angA)
                .then(() => sendServoAngle(sB, angB))
                .then(() => sendServoAngle(sC, angC))
                .catch(() => {})
        }

        document.querySelectorAll('[data-apply-leg]').forEach((btn) => {
            btn.addEventListener('click', () => {
                const row = btn.closest('.legRow')
                if (row) applyLegRow(row)
            })
        })

        const btnLoad = document.getElementById('btn-load-nvs')
        const btnReset = document.getElementById('btn-reset-cal')
        const btnRefresh = document.getElementById('btn-refresh')
        if (btnLoad) btnLoad.addEventListener('click', () => fetch(`${baseHost}/control?var=sload&val=0&cmd=0`).then(() => refreshCalibration()))
        if (btnReset) btnReset.addEventListener('click', () => fetch(`${baseHost}/control?var=sreset&val=0&cmd=0`).then(() => refreshCalibration()))
        if (btnRefresh) btnRefresh.addEventListener('click', () => refreshCalibration())

        window.refreshCalibration = refreshCalibration
        window.setPWM = setPWM
        window.sendMove = sendMove
        window.sendFunc = sendFunc

        refreshCalibration()
    });
    </script>
</body>

</html>
)rawliteral";