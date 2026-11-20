
// ==========================================
// FORKLIFT ROBOT DASHBOARD
// Live LiDAR + SLAM Occupancy Map + Camera
// ==========================================

const ros = new ROSLIB.Ros({
    url: `ws://${window.location.hostname}:9090`
});

const connectionStatus = document.getElementById("connectionStatus");
const cameraFeed = document.getElementById("cameraFeed");
const cameraPlaceholder = document.getElementById("cameraPlaceholder");
const cameraStatus = document.getElementById("cameraStatus");
const mapStatus = document.getElementById("mapStatus");

function setStatus(element, text, connected) {
    if (!element) return;

    element.textContent = text;
    element.classList.toggle("connected", connected);
    element.classList.toggle("disconnected", !connected);
}

// ==========================================
// ROS CONNECTION
// ==========================================

ros.on("connection", () => {
    setStatus(connectionStatus, "ROS Connected", true);
});

ros.on("error", (error) => {
    console.error("ROS error:", error);
    setStatus(connectionStatus, "ROS Error", false);
});

ros.on("close", () => {
    setStatus(connectionStatus, "ROS Disconnected", false);
});

// ==========================================
// MAP AND LIDAR DISPLAY
// ==========================================

const scanContainer = document.getElementById("mapView");

let latestScanPoints = [];
let latestMap = null;

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

    // --------------------------------------
    // Draw live LiDAR scan
    // --------------------------------------

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
            ctx.font = "12px Arial";
            ctx.fillText(`${Math.round(r / scale)} m`, cx + r + 5, cy);
        }

        ctx.beginPath();
        ctx.moveTo(cx, 0);
        ctx.lineTo(cx, canvas.height);
        ctx.moveTo(0, cy);
        ctx.lineTo(canvas.width, cy);
        ctx.stroke();

        ctx.fillStyle = "#38bdf8";

        latestScanPoints.forEach(point => {

            const x = cx + point.x * scale;
            const y = cy - point.y * scale;

            if (
                x < 0 || x > canvas.width ||
                y < 0 || y > canvas.height
            ) return;

            ctx.beginPath();
            ctx.arc(x, y, 2.5, 0, Math.PI * 2);
            ctx.fill();
        });

        // Robot position in the live scan view
        ctx.fillStyle = "#22c55e";
        ctx.beginPath();
        ctx.arc(cx, cy, 7, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = "#e2e8f0";
        ctx.font = "14px Arial";
        ctx.fillText("LIVE LIDAR SCAN", 20, 30);
    }

    // --------------------------------------
    // Convert occupancy grid into an image
    // --------------------------------------

    function drawOccupancyMap(map) {

        const info = map.info;
        const width = info.width;
        const height = info.height;
        const values = map.data;

        if (!width || !height || values.length !== width * height) {
            console.warn("Invalid occupancy map received");
            return;
        }

        const offscreen = document.createElement("canvas");
        offscreen.width = width;
        offscreen.height = height;

        const offCtx = offscreen.getContext("2d");
        const image = offCtx.createImageData(width, height);

        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {

                const sourceIndex = y * width + x;

                // Flip vertically for canvas coordinates
                const targetIndex =
                    ((height - 1 - y) * width + x) * 4;

                const occupancy = values[sourceIndex];

                let shade;

                if (occupancy < 0) {
                    // Unknown space
                    shade = 115;
                } else if (occupancy >= 65) {
                    // Occupied space
                    shade = 20;
                } else if (occupancy <= 25) {
                    // Free space
                    shade = 238;
                } else {
                    // Intermediate occupancy
                    shade = Math.round(
                        238 - (occupancy / 100) * 218
                    );
                }

                image.data[targetIndex] = shade;
                image.data[targetIndex + 1] = shade;
                image.data[targetIndex + 2] = shade;
                image.data[targetIndex + 3] = 255;
            }
        }

        offCtx.putImageData(image, 0, 0);

        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = "#07111f";
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Fit the map while preserving its proportions
        const padding = 35;

        const scale = Math.min(
            (canvas.width - padding * 2) / width,
            (canvas.height - padding * 2) / height
        );

        const drawWidth = width * scale;
        const drawHeight = height * scale;

        const drawX = (canvas.width - drawWidth) / 2;
        const drawY = (canvas.height - drawHeight) / 2;

        ctx.imageSmoothingEnabled = false;

        ctx.drawImage(
            offscreen,
            drawX,
            drawY,
            drawWidth,
            drawHeight
        );

        ctx.strokeStyle = "#38bdf8";
        ctx.lineWidth = 2;
        ctx.strokeRect(drawX, drawY, drawWidth, drawHeight);

        ctx.fillStyle = "#e2e8f0";
        ctx.font = "14px Arial";
        ctx.fillText(
            `SLAM OCCUPANCY MAP | ${width} × ${height} cells`,
            20,
            25
        );

        ctx.font = "12px Arial";
        ctx.fillStyle = "#cbd5e1";
        ctx.fillText(
            `Resolution: ${info.resolution.toFixed(3)} m/cell`,
            20,
            45
        );
    }

    // --------------------------------------
    // Display whichever data is available
    // --------------------------------------

    function refreshMapDisplay() {

        if (latestMap) {
            drawOccupancyMap(latestMap);
        } else {
            drawScan();
        }
    }

    // --------------------------------------
    // Subscribe to LiDAR
    // --------------------------------------

    const scanTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/scan",
        messageType: "sensor_msgs/msg/LaserScan"
    });

    scanTopic.subscribe(message => {

        const points = [];

        for (let i = 0; i < message.ranges.length; i++) {

            const distance = message.ranges[i];

            if (
                !Number.isFinite(distance) ||
                distance < message.range_min ||
                distance > message.range_max
            ) continue;

            const angle =
                message.angle_min +
                i * message.angle_increment;

            points.push({
                x: distance * Math.cos(angle),
                y: distance * Math.sin(angle)
            });
        }

        latestScanPoints = points;

        if (!latestMap) {
            refreshMapDisplay();
            setStatus(mapStatus, "LIVE LIDAR SCAN", true);
        }
    });

    // --------------------------------------
    // Subscribe to SLAM occupancy map
    // --------------------------------------

    const mapTopic = new ROSLIB.Topic({
        ros: ros,
        name: "/map",
        messageType: "nav_msgs/msg/OccupancyGrid"
    });

    mapTopic.subscribe(message => {

        latestMap = message;

        refreshMapDisplay();

        setStatus(mapStatus, "SLAM MAP LIVE", true);

        console.log(
            "SLAM map received:",
            message.info.width,
            "x",
            message.info.height
        );
    });

    // Initial screen
    drawScan();

    setStatus(mapStatus, "WAITING FOR SLAM MAP", false);
}

// ==========================================
// ANNOTATED CAMERA STREAM
// ==========================================

if (cameraFeed) {

    cameraFeed.style.display = "block";
    cameraFeed.style.width = "100%";
    cameraFeed.style.height = "100%";
    cameraFeed.style.minHeight = "300px";
    cameraFeed.style.objectFit = "contain";

    cameraFeed.onload = () => {
        cameraFeed.style.display = "block";

        if (cameraPlaceholder) {
            cameraPlaceholder.style.display = "none";
        }

        setStatus(cameraStatus, "Camera Streaming", true);
    };

    cameraFeed.onerror = () => {
        setStatus(cameraStatus, "Stream Unavailable", false);

        if (cameraPlaceholder) {
            cameraPlaceholder.style.display = "flex";
        }
    };

    cameraFeed.src =
        `http://${window.location.hostname}:8080/stream?topic=/camera/annotated/image_raw`;
}

console.log("Forklift dashboard initialized.");


// ==========================================
// MAPPING CONTROL API
// ==========================================

async function mappingRequest(endpoint) {
    try {
        const response = await fetch(endpoint, {
            method: "POST",
            headers: { "Content-Type": "application/json" }
        });

        const result = await response.json();

        if (!response.ok || !result.success) {
            throw new Error(result.error || "Request failed");
        }

        console.log(result.message || "Request completed");
        return result;

    } catch (error) {
        console.error("Mapping request error:", error);
        alert("Mapping error: " + error.message);
        return null;
    }
}

document.getElementById("startMapping")?.addEventListener("click", async () => {
    const result = await mappingRequest("/api/mapping/start");
    if (result) updateMappingStatus();
});

document.getElementById("stopMapping")?.addEventListener("click", async () => {
    const result = await mappingRequest("/api/mapping/stop");
    if (result) updateMappingStatus();
});

document.getElementById("saveMap")?.addEventListener("click", async () => {
    const result = await mappingRequest("/api/map/save");

    if (result) {
        alert("Map saved successfully:\n" + result.files.join("\n"));
    }
});

async function updateMappingStatus() {
    try {
        const response = await fetch("/api/status");
        if (!response.ok) throw new Error("Status request failed");

        const result = await response.json();
        const state = result.processes || {};

        const setText = (id, value) => {
            const element = document.getElementById(id);
            if (element) element.textContent = value;
        };

        setText("lidarStatus", state.lidar ? "Running" : "Stopped");
        setText("rosStatus", result.status === "online" ? "Online" : "Offline");

        setText(
            "robotMode",
            result.mapping ? "Mapping" : "Idle"
        );

    } catch (error) {
        console.error("Status update failed:", error);
    }
}

updateMappingStatus();
setInterval(updateMappingStatus, 3000);
