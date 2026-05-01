const socket = io();

// RPM Gauge
const rpmCtx = document.getElementById("rpmGauge").getContext("2d");
const rpmGauge = new Chart(rpmCtx, {
    type: "doughnut",
    data: {
        datasets: [{
            data: [0, 10000],
            backgroundColor: ["#60a5fa", "#1f2937"],
            borderWidth: 0
        }]
    },
    options: {
        cutout: '80%',
        rotation: -90,
        circumference: 180,
        plugins: {
            doughnutlabel: {
                labels: [
                    { text: "0", font: { size: 20 }, color: "#fff" },
                    { text: "RPM", color: "#9ca3af" }
                ]
            },
            legend: { display: false }
        }
    }
});

// Speed Gauge
const speedCtx = document.getElementById("speedGauge").getContext("2d");
const speedGauge = new Chart(speedCtx, {
    type: "doughnut",
    data: {
        datasets: [{
            data: [0, 250],
            backgroundColor: ["#34d399", "#1f2937"],
            borderWidth: 0
        }]
    },
    options: {
        cutout: '80%',
        rotation: -90,
        circumference: 180,
        plugins: {
            doughnutlabel: {
                labels: [
                    { text: "0", font: { size: 20 }, color: "#fff" },
                    { text: "km/h", color: "#9ca3af" }
                ]
            },
            legend: { display: false }
        }
    }
});

// Receive real-time data
socket.on("update", function(data) {
    if (data.RPM !== undefined) {
        rpmGauge.data.datasets[0].data[0] = data.RPM;
        rpmGauge.data.datasets[0].data[1] = 10000 - data.RPM;
        rpmGauge.options.plugins.doughnutlabel.labels[0].text = data.RPM.toString();
        rpmGauge.update();
        document.getElementById("rpmValue").innerText = data.RPM;
    }

    if (data.Speed !== undefined) {
        speedGauge.data.datasets[0].data[0] = data.Speed;
        speedGauge.data.datasets[0].data[1] = 250 - data.Speed;
        speedGauge.options.plugins.doughnutlabel.labels[0].text = data.Speed.toString();
        speedGauge.update();
        document.getElementById("speedValue").innerText = data.Speed;
    }

    const keys = ["Engine_Temp", "SOC", "SOH", "Battery_Temp", "Voltage", "Current"];
    keys.forEach(key => {
        if (data[key] !== undefined) {
            document.getElementById(key).innerText = data[key];
        }
    });
});
