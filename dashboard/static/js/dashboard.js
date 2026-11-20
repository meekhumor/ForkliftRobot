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
    let cachedOffscreenMap = null;
    let currentRobotPose = null;
    let robotTrail = [];
    let currentNavPath = [];

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

        function buildOccupancyCache(map) {
            const info = map.info;
            const width = info.width;
            const height = info.height;
            const values = map.data;

            if (!width || !height || values.length !== width * height) return null;

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
            return offscreen;
        }

        function drawOccupancyMap() {
            if (!latestMap || !cachedOffscreenMap) return;

            const info = latestMap.info;
            const width = info.width;
            const height = info.height;

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

            mapRenderMetrics = { drawX, drawY, drawWidth, drawHeight, scale, width, height, info };

            ctx.imageSmoothingEnabled = false;
            ctx.drawImage(cachedOffscreenMap, drawX, drawY, drawWidth, drawHeight);

            ctx.strokeStyle = "#38bdf8";
            ctx.lineWidth = 1.5;
            ctx.strokeRect(drawX, drawY, drawWidth, drawHeight);

            // 1. Draw Nav2 Planned Path
            if (currentNavPath.length > 1) {
                ctx.save();
                ctx.strokeStyle = "#38bdf8";
                ctx.lineWidth = 3;
                ctx.setLineDash([6, 4]);
                ctx.beginPath();
                currentNavPath.forEach((pt, idx) => {
                    const gx = (pt.x - info.origin.position.x) / info.resolution;
                    const gy = (pt.y - info.origin.position.y) / info.resolution;
                    const px = drawX + gx * scale;
                    const py = drawY + (height - 1 - gy) * scale;
                    if (idx === 0) ctx.moveTo(px, py);
                    else ctx.lineTo(px, py);
                });
                ctx.stroke();
                ctx.restore();
            }

            // 2. Draw Robot Breadcrumb Trajectory History
            if (robotTrail.length > 1) {
                ctx.save();
                ctx.strokeStyle = "rgba(52, 211, 153, 0.45)";
                ctx.lineWidth = 2;
                ctx.beginPath();
                robotTrail.forEach((pt, idx) => {
                    const gx = (pt.x - info.origin.position.x) / info.resolution;
                    const gy = (pt.y - info.origin.position.y) / info.resolution;
                    const px = drawX + gx * scale;
                    const py = drawY + (height - 1 - gy) * scale;
                    if (idx === 0) ctx.moveTo(px, py);
                    else ctx.lineTo(px, py);
                });
                ctx.stroke();
                ctx.restore();
            }

            // 3. Draw User Selected Navigation Goal Pin
            if (selectedGoal) {
                const goalGridX = (selectedGoal.x - info.origin.position.x) / info.resolution;
                const goalGridY = (selectedGoal.y - info.origin.position.y) / info.resolution;
                const gx = drawX + goalGridX * scale;
                const gy = drawY + (height - 1 - goalGridY) * scale;

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

            // 4. Draw Live Robot Position, Heading, and Forks
            if (currentRobotPose) {
                const rGridX = (currentRobotPose.x - info.origin.position.x) / info.resolution;
                const rGridY = (currentRobotPose.y - info.origin.position.y) / info.resolution;
                const rx = drawX + rGridX * scale;
                const ry = drawY + (height - 1 - rGridY) * scale;
                const yaw = currentRobotPose.yaw;

                ctx.save();
                ctx.translate(rx, ry);
                ctx.rotate(-yaw); // Invert yaw because canvas Y is top-down

                // Robot body (green chassis)
                ctx.fillStyle = "#10b981";
                ctx.strokeStyle = "#ffffff";
                ctx.lineWidth = 2;
                ctx.fillRect(-12, -8, 24, 16);
                ctx.strokeRect(-12, -8, 24, 16);

                // Front forks (orange tines pointing in +X heading direction)
                ctx.fillStyle = "#f59e0b";
                ctx.fillRect(12, -7, 9, 3);
                ctx.fillRect(12, 4, 9, 3);

                // Heading orientation line
                ctx.strokeStyle = "#38bdf8";
                ctx.lineWidth = 2.5;
                ctx.beginPath();
                ctx.moveTo(0, 0);
                ctx.lineTo(20, 0);
                ctx.stroke();

                ctx.restore();

                // Robot live coordinate badge
                ctx.fillStyle = "#34d399";
                ctx.font = "bold 11px sans-serif";
                ctx.fillText(`FORKLIFT (${currentRobotPose.x.toFixed(2)}, ${currentRobotPose.y.toFixed(2)})`, rx + 14, ry - 14);
            }

            // Header titles
            ctx.fillStyle = "#e2e8f0";
            ctx.font = "14px sans-serif";
            ctx.fillText(`OCCUPANCY MAP | ${width} × ${height} (${info.resolution}m/cell)`, 20, 25);
            ctx.font = "12px sans-serif";
            ctx.fillStyle = "#94a3b8";
            ctx.fillText("Tap anywhere on the map to set autonomous navigation target", 20, 45);
        }

        function refreshDisplay() {
            if (latestMap) drawOccupancyMap();
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

            drawOccupancyMap();
        });

        // -----------------------------------------------------------------------
        // ROS Subscriptions: LiDAR, Map, AMCL Pose, Planned Path, Odometry
        // -----------------------------------------------------------------------
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
            cachedOffscreenMap = buildOccupancyCache(message);
            refreshDisplay();
            if (mapStatus) {
                mapStatus.textContent = "SLAM MAP LIVE";
                mapStatus.style.color = "#34d399";
            }
        });

        // AMCL Robot Pose Subscription (Primary Navigation Localization)
        const amclPoseTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/amcl_pose",
            messageType: "geometry_msgs/msg/PoseWithCovarianceStamped"
        });

        amclPoseTopic.subscribe(message => {
            const pos = message.pose.pose.position;
            const ori = message.pose.pose.orientation;
            // Planar yaw from quaternion
            const yaw = 2 * Math.atan2(ori.z, ori.w);

            currentRobotPose = { x: pos.x, y: pos.y, yaw: yaw };

            // Append to breadcrumb trail if moved > 5cm
            const lastPt = robotTrail[robotTrail.length - 1];
            if (!lastPt || Math.hypot(pos.x - lastPt.x, pos.y - lastPt.y) > 0.05) {
                robotTrail.push({ x: pos.x, y: pos.y });
                if (robotTrail.length > 200) robotTrail.shift();
            }

            if (latestMap) drawOccupancyMap();
        });

        // Odometry Pose Subscription (Fallback during mapping before AMCL)
        const odomTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/odom_rf2o",
            messageType: "nav_msgs/msg/Odometry"
        });

        odomTopic.subscribe(message => {
            // Only use odometry if AMCL is not active
            if (!currentRobotPose) {
                const pos = message.pose.pose.position;
                const ori = message.pose.pose.orientation;
                const yaw = 2 * Math.atan2(ori.z, ori.w);
                currentRobotPose = { x: pos.x, y: pos.y, yaw: yaw };

                const lastPt = robotTrail[robotTrail.length - 1];
                if (!lastPt || Math.hypot(pos.x - lastPt.x, pos.y - lastPt.y) > 0.05) {
                    robotTrail.push({ x: pos.x, y: pos.y });
                    if (robotTrail.length > 200) robotTrail.shift();
                }

                if (latestMap) drawOccupancyMap();
            }
        });

        // Nav2 Planned Global Path Subscription
        const planTopic = new ROSLIB.Topic({
            ros: ros,
            name: "/plan",
            messageType: "nav_msgs/msg/Path"
        });

        planTopic.subscribe(message => {
            if (message.poses && message.poses.length > 0) {
                currentNavPath = message.poses.map(p => ({
                    x: p.pose.position.x,
                    y: p.pose.position.y
                }));
            } else {
                currentNavPath = [];
            }
            if (latestMap) drawOccupancyMap();
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

    // ---------------------------------------------------------------------------
    // 7. Live System & Process Logs Viewer
    // ---------------------------------------------------------------------------
    const logsSection = document.getElementById("logsSection");
    const toggleLogsBtn = document.getElementById("toggleLogsBtn");
    const headerLogsBtn = document.getElementById("headerLogsBtn");
    const closeLogsBtn = document.getElementById("closeLogsBtn");
    const refreshLogsBtn = document.getElementById("refreshLogsBtn");
    const autoRefreshCheckbox = document.getElementById("autoRefreshLogs");
    const logTabs = document.querySelectorAll(".log-tab");
    const logContent = document.getElementById("logContent");
    const logHeaderTitle = document.getElementById("logHeaderTitle");
    const logLinesCount = document.getElementById("logLinesCount");

    let activeLogName = "slam";
    let logPollTimer = null;

    function openLogs(targetLog) {
        if (!logsSection) return;
        logsSection.style.display = "block";
        logsSection.scrollIntoView({ behavior: "smooth", block: "nearest" });
        if (targetLog) {
            activeLogName = targetLog;
            logTabs.forEach(tab => {
                tab.classList.toggle("active", tab.dataset.log === targetLog);
            });
        }
        fetchCurrentLog();
        startLogPolling();
    }

    function closeLogs() {
        if (!logsSection) return;
        logsSection.style.display = "none";
        stopLogPolling();
    }

    function toggleLogs() {
        if (!logsSection) return;
        if (logsSection.style.display === "none" || logsSection.style.display === "") {
            openLogs();
        } else {
            closeLogs();
        }
    }

    async function fetchCurrentLog() {
        if (!logContent) return;
        try {
            const res = await fetch(`/api/logs?name=${encodeURIComponent(activeLogName)}&lines=120`);
            const data = await res.json();
            if (logHeaderTitle) {
                logHeaderTitle.textContent = `${activeLogName}.log — Real-time output`;
            }
            if (data.success) {
                logContent.textContent = data.content || "(Log is currently empty)";
                if (logLinesCount) logLinesCount.textContent = `${data.lines} lines`;
            } else {
                logContent.textContent = data.content || data.error || "No log entries found.";
                if (logLinesCount) logLinesCount.textContent = "0 lines";
            }
            // Auto scroll to bottom
            logContent.scrollTop = logContent.scrollHeight;
        } catch (err) {
            logContent.textContent = `Error loading ${activeLogName}.log: ${err.message}`;
        }
    }

    function startLogPolling() {
        stopLogPolling();
        logPollTimer = setInterval(() => {
            if (logsSection && logsSection.style.display !== "none" && autoRefreshCheckbox && autoRefreshCheckbox.checked) {
                fetchCurrentLog();
            }
        }, 2000);
    }

    function stopLogPolling() {
        if (logPollTimer) {
            clearInterval(logPollTimer);
            logPollTimer = null;
        }
    }

    toggleLogsBtn?.addEventListener("click", toggleLogs);
    headerLogsBtn?.addEventListener("click", toggleLogs);
    closeLogsBtn?.addEventListener("click", closeLogs);
    refreshLogsBtn?.addEventListener("click", fetchCurrentLog);

    logTabs.forEach(tab => {
        tab.addEventListener("click", () => {
            logTabs.forEach(t => t.classList.remove("active"));
            tab.classList.add("active");
            activeLogName = tab.dataset.log;
            fetchCurrentLog();
        });
    });

    console.log("Unified Forklift Robot Control Center script initialized.");
})();
