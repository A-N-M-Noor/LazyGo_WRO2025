// static/main.js

// 1) Establish WebSocket
const protocol = window.location.protocol === "https:" ? "wss" : "ws";
const wsURL = protocol + "://" + window.location.host + "/ws";
const socket = new WebSocket(wsURL);

// Maintain a map of float values in JS so we can update the <ul> cleanly:
const floatData = {};

// Utility to re-render the <ul id="float_list"> whenever data arrives
function renderFloatList() {
    const ul = document.getElementById("float_list");
    ul.innerHTML = ""; // clear

    for (const [topic, value] of Object.entries(floatData)) {
        const li = document.createElement("li");
        li.textContent = `${topic}: ${value.toFixed(3)}`;
        ul.appendChild(li);
    }
}

// 2) On receiving a message from WS
socket.onmessage = function (event) {
    try {
        const msg = JSON.parse(event.data);
        if (msg.type === "float_update") {
            // e.g. { type:"float_update", topic:"/float_topic_1", value: 0.123 }
            floatData[msg.topic] = msg.value;
            renderFloatList();
        }
    } catch (err) {
        console.error("Error parsing WS message:", err);
    }
};

// 3) Hook click on the <img> tag
window.addEventListener("DOMContentLoaded", () => {
    const img = document.getElementById("video_stream");

    img.addEventListener("click", (e) => {
        // Compute coordinates relative to the image’s bounding box
        const rect = img.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;

        // Send JSON over WS
        const payload = {
            type: "click",
            x: Math.round(x),
            y: Math.round(y),
        };
        socket.send(JSON.stringify(payload));
    });
});
