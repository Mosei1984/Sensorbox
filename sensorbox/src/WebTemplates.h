#ifndef WEB_TEMPLATES_H
#define WEB_TEMPLATES_H

// HTML header with CSS styles
const char HTML_HEADER[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>SensorBox Dashboard</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 20px;
      background-color: #f5f5f5;
    }
    .container {
      max-width: 800px;
      margin: 0 auto;
      background-color: white;
      padding: 20px;
      border-radius: 5px;
      box-shadow: 0 2px 5px rgba(0,0,0,0.1);
    }
    .header {
      background-color: #333;
      color: white;
      padding: 10px 20px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .header h1 {
      margin: 0;
      font-size: 24px;
    }
    .card {
      background-color: white;
      border-radius: 5px;
      box-shadow: 0 2px 5px rgba(0,0,0,0.1);
      padding: 20px;
      margin-bottom: 20px;
    }
    .card h2 {
      margin-top: 0;
      color: #333;
      border-bottom: 1px solid #eee;
      padding-bottom: 10px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fill, minmax(280px, 1fr));
      gap: 20px;
    }
    .sensor-value {
      display: flex;
      justify-content: space-between;
      margin-bottom: 10px;
      font-size: 18px;
    }
    .sensor-name {
      font-weight: bold;
    }
    .sensor-reading {
      color: #0066cc;
    }
    .footer {
      text-align: center;
      padding: 20px;
      color: #666;
      font-size: 14px;
    }
    .nav {
      display: flex;
      gap: 10px;
    }
    .nav a {
      color: white;
      text-decoration: none;
    }
    .alert-box {
      background-color: #ff3333;
      color: white;
      padding: 15px;
      margin-bottom: 20px;
      border-radius: 5px;
      animation: blink 1s infinite;
    }
    @keyframes blink {
      50% { opacity: 0.8; }
    }
    @media (max-width: 600px) {
      .header {
        flex-direction: column;
        text-align: center;
      }
      .nav {
        margin-top: 10px;
      }
    }
  </style>
  <script>
    function refreshData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          // Update sensor values
          document.getElementById('temp').textContent = data.temperature.toFixed(1) + ' °C';
          document.getElementById('hum').textContent = data.humidity.toFixed(1) + ' %';
          document.getElementById('pres').textContent = data.pressure.toFixed(0) + ' hPa';
          document.getElementById('alt').textContent = data.altitude.toFixed(1) + ' m';
          document.getElementById('mq2').textContent = data.mq2.toFixed(0);
          document.getElementById('mq135').textContent = data.mq135.toFixed(0);
          document.getElementById('lux').textContent = data.lux.toFixed(1) + ' lx';
          document.getElementById('db').textContent = data.db.toFixed(1) + ' dB';
          document.getElementById('vib').textContent = data.vibMag.toFixed(2) + ' m/s²';
          document.getElementById('eco2').textContent = data.eco2 + ' ppm';
          document.getElementById('tvoc').textContent = data.tvoc + ' ppb';
          document.getElementById('airq').textContent = data.airQualityScore.toFixed(1) + ' / 100';
          
          // Update location if available
          if (data.latitude && data.longitude) {
            document.getElementById('lat').textContent = data.latitude.toFixed(6);
            document.getElementById('lon').textContent = data.longitude.toFixed(6);
            document.getElementById('sat').textContent = data.satellites;
          }
          
          // Handle alert state
          const alertBox = document.getElementById('alertBox');
          if (data.alertState) {
            alertBox.style.display = 'block';
            document.getElementById('alertReason').textContent = data.alarmReason;
          } else {
            alertBox.style.display = 'none';
          }
          
          document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
        })
        .catch(error => {
          console.error('Error fetching data:', error);
        });
    }
    
    // Refresh data every 5 seconds
    setInterval(refreshData, 5000);
    
    // Initial data load
    document.addEventListener('DOMContentLoaded', refreshData);
  </script>
</head>
<body>
  <div class="header">
    <h1>SensorBox Dashboard</h1>
    <div class="nav">
      <a href="/">Dashboard</a>
      <a href="/wifi">WiFi Setup</a>
    </div>
  </div>
  <div class="container">
)rawliteral";

// HTML footer
const char HTML_FOOTER[] PROGMEM = R"rawliteral(
  </div>
  <div class="footer">
    <p>Last updated: <span id="lastUpdate">-</span></p>
  </div>
</body>
</html>
)rawliteral";

// Dashboard content
const char DASHBOARD_CONTENT[] PROGMEM = R"rawliteral(
<div id="alertBox" class="alert-box" style="display:none;">
  <h2 style="margin-top:0;">⚠️ ALERT!</h2>
  <p>Triggered by: <span id="alertReason">-</span></p>
</div>

<div class="grid">
  <div class="card">
    <h2>Environmental</h2>
    <div class="sensor-value">
      <span class="sensor-name">Temperature:</span>
      <span class="sensor-reading" id="temp">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">Humidity:</span>
      <span class="sensor-reading" id="hum">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">Pressure:</span>
      <span class="sensor-reading" id="pres">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">Altitude:</span>
      <span class="sensor-reading" id="alt">-</span>
    </div>
  </div>
  
  <div class="card">
    <h2>Air Quality</h2>
    <div class="sensor-value">
      <span class="sensor-name">MQ-135:</span>
      <span class="sensor-reading" id="mq135">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">MQ-2:</span>
      <span class="sensor-reading" id="mq2">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">eCO2:</span>
      <span class="sensor-reading" id="eco2">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">TVOC:</span>
      <span class="sensor-reading" id="tvoc">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">Air Quality:</span>
      <span class="sensor-reading" id="airq">-</span>
    </div>
  </div>
  
  <div class="card">
    <h2>Physical</h2>
    <div class="sensor-value">
      <span class="sensor-name">Light:</span>
      <span class="sensor-reading" id="lux">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">Sound:</span>
      <span class="sensor-reading" id="db">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">Vibration:</span>
      <span class="sensor-reading" id="vib">-</span>
    </div>
  </div>
  
  <div class="card">
    <h2>Location</h2>
    <div class="sensor-value">
      <span class="sensor-name">Latitude:</span>
      <span class="sensor-reading" id="lat">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">Longitude:</span>
      <span class="sensor-reading" id="lon">-</span>
    </div>
    <div class="sensor-value">
      <span class="sensor-name">Satellites:</span>
      <span class="sensor-reading" id="sat">-</span>
    </div>
  </div>
</div>
)rawliteral";

#endif // WEB_TEMPLATES_H
