// =============================================================================
// FORKLIFT ROBOT - UNIFIED DASHBOARD CONTROLLER
// NVIDIA Jetson Dashboard + ESP32 Motor RC + LiDAR SLAM + Nav2 Target Dispatch
// =============================================================================

(() => {
    // ---------------------------------------------------------------------------
    // 1. ROS 2 WebSocket Connection via rosbridge
    // ---------------------------------------------------------------------------
    const ros = new ROSLIB.Ros({
        url: `ws://${window.location.hostname}:9090`
    });

    const rosDot = document.getElementById("rosDot");
    const connectionStatus = document.getElementById("connectionStatus");
    const espDot = document.getElementById("espDot");
    const espStatus = document.getElementById("espStatus");
    const motorLinkTag = document.getElementById("motorLinkTag");
    const cameraFeed = document.getElementById("cameraFeed");
    const cameraPlaceholder = document.getElementById("cameraPlaceholder");
    const mapStatus = document.getElementById("mapStatus");
    const navGoalStatus = document.getElementById("navGoalStatus");

    function updateDot(dotEl, labelEl, text, state) {
        if (!dotEl) return;
        dotEl.style.background = state === "ok" ? "#34d399" : state === "warn" ? "#f59e0b" : "#fb7185";
        dotEl.style.boxShadow = `0 0 10px ${dotEl.style.background}`;
        if (labelEl) labelEl.textContent = text;
    }

    ros.on("connection", () => {
        updateDot(rosDot, connectionStatus, "ROS 2: Online", "ok");
    });

    ros.on("error", (error) => {
        console.warn("ROS Bridge connection error:", error);
        updateDot(rosDot, connectionStatus, "ROS 2: Error", "err");
    });

    ros.on("close", () => {
        updateDot(rosDot, connectionStatus, "ROS 2: Disconnected", "err");
    });

    // ---------------------------------------------------------------------------
    // 2. Manual RC Motor & Forklift Controls (Communicating with ESP32 via Jetson)
    // ---------------------------------------------------------------------------
    const speedEl = document.getElementById("speed");
    const speedValue = document.getElementById("speedValue");
    let speed = Number(speedEl ? speedEl.value : 120);
    let driveAction = null;
    let liftAction = null;
    let lastMotorSend = 0;

    if (speedEl && speedValue) {
        speedEl.addEventListener("input", () => {
            speed = Number(speedEl.value);
            speedValue.textContent = speed;
            if (driveAction) sendMotor(driveAction);
            if (liftAction) sendMotor(liftAction);
        });
    }

    function sendMotor(action) {
        lastMotorSend = Date.now();
        const url = `/api/motor?a=${encodeURIComponent(action)}&s=${speed}`;
        fetch(url, { cache: "no-store" })
            .then(res => res.json())
            .then(data => {
                if (data.success) {
                    if (motorLinkTag) {
                        motorLinkTag.textContent = `LINK: ${action}`;
                        motorLinkTag.style.color = "#34d399";
                    }
                } else {
                    if (motorLinkTag) {
                        motorLinkTag.textContent = "LINK: ERROR";
                        motorLinkTag.style.color = "#fb7185";
                    }
                }
            })
            .catch(() => {
                if (motorLinkTag) {
                    motorLinkTag.textContent = "LINK: NO RESP";
                    motorLinkTag.style.color = "#fb7185";
                }
            });
    }

    function buttonFor(action) {
        return document.querySelector(`[data-drive="${action}"], [data-lift="${action}"]`);
    }

    function markButton(action, active) {
        const btn = buttonFor(action);
        if (btn) btn.classList.toggle("active", active);
    }

    function startDrive(action) {
        if (liftAction) {
            markButton(liftAction, false);
            liftAction = null;
            sendMotor("LS");
        }
        if (driveAction && driveAction !== action) markButton(driveAction, false);
        driveAction = action;
        markButton(action, true);
        sendMotor(action);
    }

    function endDrive(action) {
        if (driveAction !== action) return;
        driveAction = null;
        markButton(action, false);
        sendMotor("S");
    }

    function startLift(action) {
        if (driveAction) {
            markButton(driveAction, false);
            driveAction = null;
            sendMotor("S");
        }
        if (liftAction && liftAction !== action) markButton(liftAction, false);
        liftAction = action;
        markButton(action, true);
        sendMotor(action);
    }

    function endLift(action) {
        if (liftAction !== action) return;
        liftAction = null;
        markButton(action, false);
        sendMotor("LS");
    }

    function emergencyStop() {
        if (driveAction) markButton(driveAction, false);
        if (liftAction) markButton(liftAction, false);
        driveAction = null;
        liftAction = null;

        fetch("/api/estop", { method: "POST" })
            .then(res => res.json())
            .then(() => {
                if (motorLinkTag) {
                    motorLinkTag.textContent = "EMERGENCY STOPPED";
                    motorLinkTag.style.color = "#fb7185";
                }
            });
    }

    // Bind pointer events for drive pad
    document.querySelectorAll("[data-drive]").forEach(button => {
        const action = button.dataset.drive;
        button.addEventListener("pointerdown", event => {
            event.preventDefault();
            button.setPointerCapture(event.pointerId);
            startDrive(action);
        });
        ["pointerup", "pointercancel", "lostpointercapture"].forEach(name =>
            button.addEventListener(name, () => endDrive(action))
        );
        button.addEventListener("contextmenu", event => event.preventDefault());
    });

    // Bind pointer events for lift pad
    document.querySelectorAll("[data-lift]").forEach(button => {
        const action = button.dataset.lift;
        button.addEventListener("pointerdown", event => {
            event.preventDefault();
            button.setPointerCapture(event.pointerId);
            startLift(action);
        });
        ["pointerup", "pointercancel", "lostpointercapture"].forEach(name =>
            button.addEventListener(name, () => endLift(action))
        );
        button.addEventListener("contextmenu", event => event.preventDefault());
    });

    document.getElementById("rcStop")?.addEventListener("pointerdown", event => {
        event.preventDefault();
        if (driveAction) markButton(driveAction, false);
        driveAction = null;
        sendMotor("S");
    });

    document.getElementById("emergency")?.addEventListener("pointerdown", event => {
        event.preventDefault();
        emergencyStop();
    });

    document.getElementById("quickEstop")?.addEventListener("click", () => {
        emergencyStop();
    });

    // Keyboard shortcuts
    const keyMap = {
        w: "F", s: "B", a: "L", d: "R", r: "UP", f: "DOWN"
    };

    window.addEventListener("keydown", event => {
        if (event.code === "Space") {
            event.preventDefault();
            emergencyStop();
            return;
        }
        const key = event.key.toLowerCase();
        const action = keyMap[key];
        if (!action || event.repeat) return;
        event.preventDefault();
        if (action === "UP" || action === "DOWN") startLift(action);
        else startDrive(action);
    });

    window.addEventListener("keyup", event => {
        const key = event.key.toLowerCase();
        const action = keyMap[key];
        if (!action) return;
        if (action === "UP" || action === "DOWN") endLift(action);
        else endDrive(action);
    });

    // Refresh ESP32 watchdog (500ms timeout) while holding a button
    setInterval(() => {
        if (driveAction) sendMotor(driveAction);
        else if (liftAction) sendMotor(liftAction);
    }, 180);

    // Stop on blur
    window.addEventListener("blur", emergencyStop);
    window.addEventListener("pagehide", emergencyStop);
    document.addEventListener("visibilitychange", () => {
        if (document.hidden) emergencyStop();
    });

    // ---------------------------------------------------------------------------
    // 3. Live LiDAR Scan & SLAM Occupancy Map Rendering + Tap-to-Navigate
    // ---------------------------------------------------------------------------
    const scanContainer = document.getElementById("mapView");
    let latestScanPoints = [];
    let latestMap = null;
    let selectedGoal = null;
    let mapRenderMetrics = null;

    if (scanContainer) {
        scanContainer.innerHTML = "";
        const canvas = document.createElement("canvas");
        canvas.width = 900;
        canvas.height = 600;
        canvas.style.width = "100%";
        canvas.style.height = "100%";
        canvas.style.display = "block";
        scanContainer.appendChild(canvas);

        const ctx = canvas.getContext("2d");

        function drawScan() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = "#07111f";
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            const cx = canvas.width / 2;
            const cy = canvas.height / 2;
            const scale = 16;

            ctx.strokeStyle = "#23384d";
            ctx.lineWidth = 1;

            for (let r = 100; r <= 400; r += 100) {
                ctx.beginPath();
                ctx.arc(cx, cy, r, 0, Math.PI * 2);
                ctx.stroke();

                ctx.fillStyle = "#718096";
                ctx.font = "12px sans-serif";
                ctx.fillText(`${Math.round(r / scale)} m`, cx + r + 5, cy);
            }

            ctx.beginPath();
            ctx.moveTo(cx, 0); ctx.lineTo(cx, canvas.height);
            ctx.moveTo(0, cy); ctx.lineTo(canvas.width, cy);
            ctx.stroke();

            ctx.fillStyle = "#38bdf8";
            latestScanPoints.forEach(point => {
                const x = cx + point.x * scale;
                const y = cy - point.y * scale;
                if (x < 0 || x > canvas.width || y < 0 || y > canvas.height) return;
                ctx.beginPath();
                ctx.arc(x, y, 2.5, 0, Math.PI * 2);
                ctx.fill();
            });

            // Center robot marker
            ctx.fillStyle = "#34d399";
            ctx.beginPath();
            ctx.arc(cx, cy, 7, 0, Math.PI * 2);
            ctx.fill();

            ctx.fillStyle = "#e2e8f0";
            ctx.font = "13px sans-serif";
            ctx.fillText("LIVE LIDAR SCAN", 20, 30);
        }

        function drawOccupancyMap(map) {
            const info = map.info;
            const width = info.width;
            const height = info.height;
            const values = map.data;

            if (!width || !height || values.length !== width * height) return;

            const offscreen = document.createElement("canvas");
            offscreen.width = width;
            offscreen.height = height;
            const offCtx = offscreen.getContext("2d");
            const image = offCtx.createImageData(width, height);

            for (let y = 0; y < height; y++) {
                for (let x = 0; x < width; x++) {
                    const srcIdx = y * width + x;
                    const tgtIdx = ((height - 1 - y) * width + x) * 4;
                    const val = values[srcIdx];

                    let shade;
                    if (val < 0) shade = 100;         // Unknown space
                    else if (val >= 65) shade = 20;     // Occupied obstacle
                    else if (val <= 25) shade = 235;    // Free navigable space
                    else shade = Math.round(235 - (val / 100) * 215);

                    image.data[tgtIdx] = shade;
                    image.data[tgtIdx + 1] = shade;
                    image.data[tgtIdx + 2] = shade;
                    image.data[tgtIdx + 3] = 255;
                }
            }

            offCtx.putImageData(image, 0, 0);

            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = "#07111f";
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            const padding = 30;
            const scale = Math.min(
                (canvas.width - padding * 2) / width,
                (canvas.height - padding * 2) / height
            );

            const drawWidth = width * scale;
            const drawHeight = height * scale;
            const drawX = (canvas.width - drawWidth) / 2;
            const drawY = (canvas.height - drawHeight) / 2;

            // Cache metrics for click coordinate conversion
            mapRenderMetrics = { drawX, drawY, drawWidth, drawHeight, scale, width, height, info };

            ctx.imageSmoothingEnabled = false;
            ctx.drawImage(offscreen, drawX, drawY, drawWidth, drawHeight);

            ctx.strokeStyle = "#38bdf8";
            ctx.lineWidth = 1.5;
            ctx.strokeRect(drawX, drawY, drawWidth, drawHeight);

            // Draw user-selected navigation goal pin
            if (selectedGoal && mapRenderMetrics) {
                const goalGridX = (selectedGoal.x - info.origin.position.x) / info.resolution;
                const goalGridY = (selectedGoal.y - info.origin.position.y) / info.resolution;
                const gx = drawX + goalGridX * scale;
                const gy = drawY + (height - 1 - goalGridY) * scale;

                // Pulsing target pin
                ctx.fillStyle = "#fb7185";
                ctx.beginPath();
                ctx.arc(gx, gy, 8, 0, Math.PI * 2);
                ctx.fill();
                ctx.strokeStyle = "#ffffff";
                ctx.lineWidth = 2;
                ctx.stroke();

                ctx.fillStyle = "#ffffff";
                ctx.font = "bold 11px sans-serif";
                ctx.fillText("GOAL", gx + 12, gy + 4);
            }

            ctx.fillStyle = "#e2e8f0";
            ctx.font = "14px sans-serif";
            ctx.fillText(`OCCUPANCY MAP | ${width} × ${height} (${info.resolution}m/cell)`, 20, 25);
            ctx.font = "12px sans-serif";
            ctx.fillStyle = "#94a3b8";
            ctx.fillText("Tap anywhere on the map to set autonomous navigation target", 20, 45);
        }

        function refreshDisplay() {
            if (latestMap) drawOccupancyMap(latestMap);
            else drawScan();
        }

        // Tap/click on map to choose autonomous destination
        canvas.addEventListener("click", event => {
            if (!mapRenderMetrics || !latestMap) return;

            const rect = canvas.getBoundingClientRect();
            const clickX = (event.clientX - rect.left) * (canvas.width / rect.width);
            const clickY = (event.clientY - rect.top) * (canvas.height / rect.height);

            const m = mapRenderMetrics;
            if (clickX < m.drawX || clickX > m.drawX + m.drawWidth ||
                clickY < m.drawY || clickY > m.drawY + m.drawHeight) {
                return;
            }

            const pixelX = (clickX - m.drawX) / m.scale;
            const pixelY = (clickY - m.drawY) / m.scale;

            const gridX = pixelX;
            const gridY = m.height - 1 - pixelY;

            const worldX = m.info.origin.position.x + gridX * m.info.resolution;
            const worldY = m.info.origin.position.y + gridY * m.info.resolution;

            selectedGoal = { x: worldX, y: worldY };

            // Update input fields
            const navX = document.getElementById("nav-x");
            const navY = document.getElementById("nav-y");
            if (navX) navX.value = worldX.toFixed(2);
            if (navY) navY.value = worldY.toFixed(2);

            if (navGoalStatus) {
                navGoalStatus.textContent = `TARGET: (${worldX.toFixed(1)}, ${worldY.toFixed(1)})`;
                navGoalStatus.style.color = "#38bdf8";
            }

            drawOccupancyMap(latestMap);
        });

        // ROS Subscriptions
        const scanTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/scan",
            messageType: "sensor_msgs/msg/LaserScan"
        });

        scanTopic.subscribe(message => {
            const points = [];
            for (let i = 0; i < message.ranges.length; i++) {
                const d = message.ranges[i];
                if (!Number.isFinite(d) || d < message.range_min || d > message.range_max) continue;
                const angle = message.angle_min + i * message.angle_increment;
                points.push({ x: d * Math.cos(angle), y: d * Math.sin(angle) });
            }
            latestScanPoints = points;
            if (!latestMap) {
                drawScan();
                if (mapStatus) mapStatus.textContent = "LIVE LIDAR SCAN";
            }
        });

        const mapTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/map",
            messageType: "nav_msgs/msg/OccupancyGrid"
        });

        mapTopic.subscribe(message => {
            latestMap = message;
            refreshDisplay();
            if (mapStatus) {
                mapStatus.textContent = "SLAM MAP LIVE";
                mapStatus.style.color = "#34d399";
            }
        });

        drawScan();
    }

    // ---------------------------------------------------------------------------
    // 4. Camera Stream Handling
    // ---------------------------------------------------------------------------
    if (cameraFeed) {
        cameraFeed.style.display = "block";
        cameraFeed.style.width = "100%";
        cameraFeed.style.minHeight = "280px";
        cameraFeed.style.objectFit = "contain";

        cameraFeed.onload = () => {
            if (cameraPlaceholder) cameraPlaceholder.style.display = "none";
        };

        cameraFeed.onerror = () => {
            if (cameraPlaceholder) cameraPlaceholder.style.display = "flex";
        };

        cameraFeed.src = `http://${window.location.hostname}:8080/stream?topic=/camera/annotated/image_raw`;
    }

    // ---------------------------------------------------------------------------
    // 5. Mapping & Navigation API Requests
    // ---------------------------------------------------------------------------
    async function apiPost(endpoint, body = {}) {
        try {
            const response = await fetch(endpoint, {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify(body)
            });
            const result = await response.json();
            if (!response.ok || !result.success) {
                throw new Error(result.error || "Operation failed");
            }
            return result;
        } catch (err) {
            console.error("API Error:", err);
            alert("Error: " + err.message);
            return null;
        }
    }

    document.getElementById("startMapping")?.addEventListener("click", async () => {
        const res = await apiPost("/api/mapping/start");
        if (res) alert("Mapping started: LiDAR, TF, Odometry, and SLAM Toolbox active.");
    });

    document.getElementById("stopMapping")?.addEventListener("click", async () => {
        const res = await apiPost("/api/mapping/stop");
        if (res) alert("Mapping processes stopped.");
    });

    document.getElementById("saveMap")?.addEventListener("click", async () => {
        const res = await apiPost("/api/map/save");
        if (res) alert("Map saved to:\n" + res.files.join("\n"));
    });

    document.getElementById("startNav2")?.addEventListener("click", async () => {
        const res = await apiPost("/api/nav2/start");
        if (res) alert("Autonomous Nav2 stack & ESP32 velocity bridge started.");
    });

    document.getElementById("stopNav2")?.addEventListener("click", async () => {
        const res = await apiPost("/api/nav2/stop");
        if (res) alert("Nav2 stack stopped.");
    });

    // Autonomous goal dispatch
    document.getElementById("send-goal-btn")?.addEventListener("click", async () => {
        const x = parseFloat(document.getElementById("nav-x").value) || 0.0;
        const y = parseFloat(document.getElementById("nav-y").value) || 0.0;
        const yaw = parseFloat(document.getElementById("nav-yaw").value) || 0.0;

        const res = await apiPost("/api/navigate", { x, y, yaw });
        if (res) {
            if (navGoalStatus) {
                navGoalStatus.textContent = `NAVIGATING -> (${x.toFixed(1)}, ${y.toFixed(1)})`;
                navGoalStatus.style.color = "#34d399";
            }
        }
    });

    document.getElementById("cancel-goal-btn")?.addEventListener("click", async () => {
        const res = await apiPost("/api/navigate/cancel");
        if (res) {
            if (navGoalStatus) {
                navGoalStatus.textContent = "CANCELLED";
                navGoalStatus.style.color = "#fb7185";
            }
        }
    });

    // ---------------------------------------------------------------------------
    // 6. System Status Poller & ESP32 IP Configuration
    // ---------------------------------------------------------------------------
    const espIpInput = document.getElementById("espIpInput");
    const saveIpBtn = document.getElementById("saveIpBtn");

    saveIpBtn?.addEventListener("click", async () => {
        const newIp = espIpInput.value.trim();
        if (!newIp) return;
        const res = await apiPost("/api/config", { ip: newIp });
        if (res) alert(`ESP32 IP address updated to ${res.ip}`);
    });

    async function pollStatus() {
        try {
            const res = await fetch("/api/status");
            if (!res.ok) return;
            const data = await res.json();
            const procs = data.processes || {};

            // ESP32 Status
            if (data.esp32_online) {
                updateDot(espDot, espStatus, `ESP32: Connected (${data.esp32_ip})`, "ok");
                const mStatus = document.getElementById("espMotorStatus");
                if (mStatus) mStatus.textContent = "Online & Ready";
            } else {
                updateDot(espDot, espStatus, `ESP32: No Link (${data.esp32_ip})`, "err");
                const mStatus = document.getElementById("espMotorStatus");
                if (mStatus) mStatus.textContent = "Disconnected";
            }

            // LiDAR
            const lidarEl = document.getElementById("lidarStatus");
            if (lidarEl) lidarEl.textContent = procs.lidar ? "Running" : "Stopped";

            // Nav2
            const nav2El = document.getElementById("nav2Status");
            if (nav2El) nav2El.textContent = procs.navigation ? "Active" : "Inactive";

            // Mode
            const modeEl = document.getElementById("robotMode");
            if (modeEl) {
                if (data.navigating) modeEl.textContent = "Autonomous Nav2";
                else if (data.mapping) modeEl.textContent = "SLAM Mapping";
                else modeEl.textContent = "Manual RC";
            }
        } catch (e) {
            console.debug("Status poll:", e);
        }
    }

    pollStatus();
    setInterval(pollStatus, 2500);

    console.log("Unified Forklift Robot Control Center script initialized.");
})();
