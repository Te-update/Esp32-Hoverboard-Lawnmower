// *******************************************************************
//  Author: Osemen Testimony Marvellous
//  Date: July 2025
//  Version: 1.0
//  ESP32 WiFi Lawn Mower Controller - Basic Ui
//  This Arduino sketch controls a hoverboard motor powered lawnmower using an Esp32 microcontroller.
//  Combines WiFi joystick control with lawn mower UART communication
//  Based on Emmanuel Feru's hoverboard-firmware-hack-FOC protocol
//  The relay controls a coling fan attached to the top of the hoverboard driver's enclosure.
//  The relay is activated when the hoverboard driver's temperature goes above 45 degrees.
//  Comment out parts of the code you absolutely don't need.
//  You MAY want to remove all the serial prints when you're done debugging.  
// *******************************************************************

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // Baud rate for LawnMowerSerial (to lawn mower)
#define SERIAL_BAUD         115200      // Baud rate for debugging Serial (Serial Monitor)
#define START_FRAME         0xABCD      // Start frame for reliable communication
#define TIME_SEND           100         // Sending interval in ms
#define RXD2                16          // ESP32 GPIO for LawnMowerSerial RX
#define TXD2                17          // ESP32 GPIO for rd

// Create server and WebSocket instance
AsyncWebServer server(81);
AsyncWebSocket ws("/ws");
HardwareSerial LawnMowerSerial(1);      // Use UART1LawnMowerSerial TX
#define LED_BUILTIN         2
#define relay               4           // EsP32 GPIO for driver cooling

#define ECHO_LEFT    34                 // Ultrasonic sensor pins (AJR04 only needs echo pins)
#define ECHO_RIGHT   35                 // Ultrasonic sensor pins (AJR04 only needs echo pins)
#define CUTTING_MOTOR_PIN 5             // Pin to control cutting motor relay

#define TRIG_LEFT_SIDE   32             // Side ultrasonic sensor pins (normal HC-SR04 type)
#define ECHO_LEFT_SIDE   33             // Side ultrasonic sensor pins (normal HC-SR04 type)
#define TRIG_RIGHT_SIDE  25             // Side ultrasonic sensor pins (normal HC-SR04 type)
#define ECHO_RIGHT_SIDE  26             // Side ultrasonic sensor pins (normal HC-SR04 type)

// Autonomous mode variables
bool autonomousMode = false;
bool emergencyStop = false;
bool cuttingMotorEnabled = false;
unsigned long lastObstacleCheck = 0;
const int OBSTACLE_CHECK_INTERVAL = 100;  // Check every 100ms
const int OBSTACLE_DISTANCE = 30;         // Stop if obstacle within 30cm
const int BACKUP_TIME = 2000;             // Backup for 2 seconds
const int TURN_TIME = 1500;               // Turn for 1.5 seconds
const int SIDE_OBSTACLE_DISTANCE = 20;    // Side sensors trigger at 20cm   
unsigned long backupStartTime = 0;
unsigned long turnStartTime = 0;
enum AutonomousState { AUTO_FORWARD, AUTO_BACKUP, AUTO_TURN, AUTO_STOP };
AutonomousState autoState = AUTO_FORWARD;        

// WiFi credentials
const char* ssid = "Lawn Mower";        // Your AP SSID
const char* password = "12345678";      // Your AP password

// Control parameters
int maxSpeed = 300;         // Maximum speed value (adjustable)
int maxSteer = 300;         // Maximum steering value (adjustable)
const float DEADZONE = 0.1; // Center deadzone threshold

// ########################## STRUCTURES ##########################
typedef struct {
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## VARIABLES ##########################
uint8_t idx = 0;
uint16_t bufStartFrame;
byte *p;
byte incomingByte;
byte incomingBytePrev;

unsigned long iTimeSend = 0;
int currentSpeed = 0;
int currentSteer = 0;

float actualTemp = 0.0;  // Global variable to store temperature

// Spiral cutting variables
bool spiralCutEnabled = false;
unsigned long spiralStartTime = 0;
float spiralRadius = 0.0;
float spiralAngle = 0.0;
const float SPIRAL_GROWTH_RATE = 0.05;  // How fast spiral expands (adjust as needed)(Higher value = faster GROWTH_RATE)
const float SPIRAL_ANGULAR_VELOCITY = 0.1;  // Radians per calculation cycle
const int SPIRAL_BASE_SPEED = 150;  // Base forward speed for spiral
unsigned long lastSpiralUpdate = 0;
const int SPIRAL_UPDATE_INTERVAL = 50;  // Update spiral every 50ms

// Basic HTML webpage with modern design and light/dark mode toggle
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>Lawn Mower Controller</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="preconnect" href="https://fonts.googleapis.com">
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap" rel="stylesheet">
    <style>
        :root {
            --primary-color: #10b981;
            --primary-dark: #059669;
            --primary-light: #6ee7b7;
            --secondary-color: #3b82f6;
            --danger-color: #ef4444;
            --warning-color: #f59e0b;
            --success-color: #10b981;
        }

        [data-theme="dark"] {
            --background: #0f172a;
            --surface: #1e293b;
            --surface-light: #334155;
            --text-primary: #f8fafc;
            --text-secondary: #cbd5e1;
            --text-muted: #94a3b8;
            --border: #334155;
            --shadow: 0 25px 50px -12px rgba(0, 0, 0, 0.6);
            --shadow-lg: 0 20px 25px -5px rgba(0, 0, 0, 0.4);
            --gradient-primary: linear-gradient(135deg, #10b981 0%, #059669 100%);
            --gradient-danger: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
            --gradient-warning: linear-gradient(135deg, #f59e0b 0%, #d97706 100%);
        }

        [data-theme="light"] {
            --background: #f8fafc;
            --surface: #ffffff;
            --surface-light: #f1f5f9;
            --text-primary: #1e293b;
            --text-secondary: #475569;
            --text-muted: #64748b;
            --border: #e2e8f0;
            --shadow: 0 25px 50px -12px rgba(0, 0, 0, 0.15);
            --shadow-lg: 0 20px 25px -5px rgba(0, 0, 0, 0.1);
            --gradient-primary: linear-gradient(135deg, #10b981 0%, #059669 100%);
            --gradient-danger: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
            --gradient-warning: linear-gradient(135deg, #f59e0b 0%, #d97706 100%);
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
            background: var(--background);
            color: var(--text-primary);
            min-height: 100vh;
            padding: 1rem;
            transition: all 0.3s ease;
        }

        [data-theme="dark"] body {
            background-image: 
                radial-gradient(circle at 25% 25%, #10b98120 0%, transparent 50%),
                radial-gradient(circle at 75% 75%, #3b82f620 0%, transparent 50%);
        }

        [data-theme="light"] body {
            background-image: 
                radial-gradient(circle at 25% 25%, #10b98108 0%, transparent 50%),
                radial-gradient(circle at 75% 75%, #3b82f608 0%, transparent 50%);
        }

        .container {
            max-width: 1200px;
            margin: 0 auto;
            display: grid;
            grid-template-columns: 1fr 400px;
            gap: 2rem;
            align-items: start;
        }

        @media (max-width: 1024px) {
            .container {
                grid-template-columns: 1fr;
                gap: 1.5rem;
            }
        }

        .header {
            grid-column: 1 / -1;
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 2rem;
        }

        .header-content h1 {
            font-size: 2.5rem;
            font-weight: 700;
            background: var(--gradient-primary);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            margin-bottom: 0.5rem;
        }

        .theme-toggle {
            background: var(--surface);
            border: 2px solid var(--border);
            border-radius: 50px;
            padding: 0.5rem;
            cursor: pointer;
            display: flex;
            align-items: center;
            gap: 0.5rem;
            transition: all 0.3s ease;
            color: var(--text-primary);
            font-size: 1.1rem;
        }

        .header-controls {
            display: flex;
            gap: 1rem;
            align-items: center;
        }

        .spiral-toggle {
            background: var(--surface);
            border: 2px solid var(--border);
            border-radius: 50px;
            padding: 0.5rem 1rem;
            cursor: pointer;
            display: flex;
            align-items: center;
            gap: 0.5rem;
            transition: all 0.3s ease;
            color: var(--text-primary);
            font-size: 1.1rem;
        }

        .spiral-toggle:hover {
            background: var(--surface-light);
            transform: scale(1.05);
        }

        .spiral-toggle.active {
            background: var(--gradient-primary);
            color: white;
            border-color: var(--primary-color);
        }

        .spiral-toggle.active:hover {
            background: var(--gradient-primary);
        }

        .card {
            background: var(--surface);
            border-radius: 1.5rem;
            padding: 2rem;
            box-shadow: var(--shadow);
            border: 1px solid var(--border);
            backdrop-filter: blur(10px);
            transition: all 0.3s ease;
        }

        .card:hover {
            transform: translateY(-2px);
            box-shadow: var(--shadow-lg);
        }

        .card-title {
            font-size: 1.5rem;
            font-weight: 600;
            margin-bottom: 1.5rem;
            display: flex;
            align-items: center;
            gap: 0.75rem;
        }

        .card-title::before {
            content: '';
            width: 4px;
            height: 1.5rem;
            background: var(--gradient-primary);
            border-radius: 2px;
        }

        .main-control {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 2rem;
        }

        #status {
            padding: 1rem 2rem;
            border-radius: 1rem;
            font-weight: 500;
            text-align: center;
            transition: all 0.3s ease;
            border: 2px solid transparent;
            font-size: 1rem;
        }

        #status.connected {
            background: rgba(16, 185, 129, 0.1);
            border-color: var(--success-color);
            color: var(--success-color);
        }

        #status.disconnected {
            background: rgba(239, 68, 68, 0.1);
            border-color: var(--danger-color);
            color: var(--danger-color);
        }

        #joystickContainer {
            width: 350px;
            height: 350px;
            position: relative;
            border-radius: 50%;
            touch-action: none;
            background: radial-gradient(circle, var(--surface-light) 0%, var(--surface) 100%);
            box-shadow: 
                inset 0 0 50px rgba(0, 0, 0, 0.1),
                0 10px 30px rgba(0, 0, 0, 0.1);
            border: 3px solid var(--border);
            overflow: hidden;
        }

        [data-theme="dark"] #joystickContainer {
            box-shadow: 
                inset 0 0 50px rgba(0, 0, 0, 0.3),
                0 10px 30px rgba(0, 0, 0, 0.4);
        }

        #joystickContainer::before {
            content: '';
            position: absolute;
            top: 50%;
            left: 50%;
            width: 2px;
            height: 80%;
            background: var(--border);
            transform: translate(-50%, -50%);
        }

        #joystickContainer::after {
            content: '';
            position: absolute;
            top: 50%;
            left: 50%;
            width: 80%;
            height: 2px;
            background: var(--border);
            transform: translate(-50%, -50%);
        }

        #joystick {
            width: 80px;
            height: 80px;
            background: var(--gradient-primary);
            border-radius: 50%;
            position: absolute;
            left: 135px;
            top: 135px;
            cursor: grab;
            box-shadow: 
                0 8px 25px rgba(16, 185, 129, 0.4),
                inset 0 2px 10px rgba(255, 255, 255, 0.2);
            transition: all 0.2s ease;
            border: 3px solid rgba(255, 255, 255, 0.3);
        }

        #joystick:active {
            cursor: grabbing;
            transform: scale(1.1);
            box-shadow: 
                0 12px 35px rgba(16, 185, 129, 0.6),
                inset 0 2px 10px rgba(255, 255, 255, 0.3);
        }

        #joystick.battery-warning {
            background: var(--gradient-warning);
            box-shadow: 0 8px 25px rgba(245, 158, 11, 0.4);
        }

        #joystick.battery-danger {
            background: var(--gradient-danger);
            box-shadow: 0 8px 25px rgba(239, 68, 68, 0.4);
        }

        .controls-display {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1rem;
            width: 100%;
            max-width: 400px;
        }

        .control-item {
            background: var(--surface-light);
            padding: 1.5rem;
            border-radius: 1rem;
            text-align: center;
            border: 1px solid var(--border);
            transition: all 0.3s ease;
        }

        .control-item:hover {
            background: var(--border);
        }

        .control-label {
            font-size: 0.875rem;
            font-weight: 500;
            color: var(--text-muted);
            margin-bottom: 0.5rem;
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }

        .control-value {
            font-size: 2rem;
            font-weight: 700;
            color: var(--text-primary);
            font-family: 'SF Mono', 'Monaco', monospace;
        }

        .coordinates {
            grid-column: 1 / -1;
            text-align: center;
            padding: 1rem;
            background: var(--surface-light);
            border-radius: 0.75rem;
            font-family: 'SF Mono', 'Monaco', monospace;
            font-size: 1rem;
            color: var(--text-secondary);
        }

        .sidebar {
            display: flex;
            flex-direction: column;
            gap: 1.5rem;
        }

        .settings-grid {
            display: flex;
            flex-direction: column;
            gap: 1.5rem;
        }

        .slider-container {
            background: var(--surface-light);
            padding: 1.5rem;
            border-radius: 1rem;
            border: 1px solid var(--border);
            transition: all 0.3s ease;
        }

        .slider-container:hover {
            background: var(--border);
        }

        .slider-label {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1rem;
            font-weight: 500;
        }

        .slider-value {
            font-family: 'SF Mono', 'Monaco', monospace;
            font-weight: 600;
            color: var(--primary-color);
            font-size: 1.1rem;
        }

        .slider {
            width: 100%;
            height: 8px;
            border-radius: 4px;
            background: var(--surface);
            outline: none;
            -webkit-appearance: none;
            appearance: none;
            cursor: pointer;
            transition: all 0.3s ease;
        }

        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background: var(--gradient-primary);
            cursor: pointer;
            box-shadow: 0 4px 12px rgba(16, 185, 129, 0.4);
            transition: all 0.2s ease;
        }

        .slider::-webkit-slider-thumb:hover {
            transform: scale(1.2);
            box-shadow: 0 6px 16px rgba(16, 185, 129, 0.6);
        }

        .slider::-moz-range-thumb {
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background: var(--gradient-primary);
            cursor: pointer;
            border: none;
            box-shadow: 0 4px 12px rgba(16, 185, 129, 0.4);
        }

        .feedback-section {
            font-family: 'SF Mono', 'Monaco', monospace;
            font-size: 0.9rem;
            line-height: 1.6;
        }

        .feedback-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1rem;
            margin-top: 1rem;
        }

        .feedback-item {
            background: var(--surface-light);
            padding: 1rem;
            border-radius: 0.75rem;
            border: 1px solid var(--border);
        }

        .feedback-label {
            font-size: 0.75rem;
            color: var(--text-muted);
            margin-bottom: 0.25rem;
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }

        .feedback-value {
            font-size: 1.1rem;
            font-weight: 600;
            color: var(--text-primary);
        }

        .battery-section {
            text-align: center;
        }

        .battery-display {
            display: flex;
            align-items: center;
            gap: 1rem;
            margin: 1.5rem 0;
        }

        .battery-icon {
            width: 60px;
            height: 30px;
            border: 3px solid var(--border);
            border-radius: 6px;
            position: relative;
            background: var(--surface-light);
        }

        .battery-icon::after {
            content: '';
            position: absolute;
            right: -8px;
            top: 50%;
            transform: translateY(-50%);
            width: 6px;
            height: 12px;
            background: var(--border);
            border-radius: 0 2px 2px 0;
        }

        .battery-fill {
            position: absolute;
            left: 2px;
            top: 2px;
            bottom: 2px;
            border-radius: 2px;
            transition: all 0.5s ease;
            background: var(--gradient-primary);
        }

        .battery-fill.warning {
            background: var(--gradient-warning);
        }

        .battery-fill.danger {
            background: var(--gradient-danger);
        }

        .battery-text {
            font-family: 'SF Mono', 'Monaco', monospace;
            font-weight: 600;
            font-size: 1.2rem;
        }

        .battery-percentage {
            font-size: 0.9rem;
            color: var(--text-secondary);
            margin-left: 0.5rem;
        }

        .pulse {
            animation: pulse 2s cubic-bezier(0.4, 0, 0.6, 1) infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .fade-in {
            animation: fadeIn 0.5s ease-in-out;
        }

        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(10px); }
            to { opacity: 1; transform: translateY(0); }
        }

        /* Responsive adjustments */
        @media (max-width: 768px) {
            body {
                padding: 0.5rem;
            }
            
            .header {
                flex-direction: column;
                gap: 1rem;
                text-align: center;
            }
            
            .header-content h1 {
                font-size: 2rem;
            }
            
            #joystickContainer {
                width: 300px;
                height: 300px;
            }
            
            #joystick {
                width: 70px;
                height: 70px;
                left: 115px;
                top: 115px;
            }
            
            .card {
                padding: 1.5rem;
            }
            
            .controls-display {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body data-theme="dark">
    <div class="container">
        <div class="header">
    <div class="header-content">
        <h1>Lawn Mower Controller</h1>
    </div>
    <div class="header-controls">
        <button class="spiral-toggle" id="spiralToggle">
            <span id="spiralIcon"></span>
            <span id="spiralText">Spiral Cut</span>
        </button>
        <button class="spiral-toggle" id="autoToggle">
            <span id="autoIcon"></span>
            <span id="autoText">Auto Mode</span>
        </button>
        <button class="spiral-toggle" id="stopToggle">
            <span id="stopIcon"></span>
            <span id="stopText">E-Stop</span>
        </button>
        <button class="spiral-toggle" id="cutToggle">
            <span id="cutIcon"></span>
            <span id="cutText">Cut Motor</span>
        </button>
        <button class="theme-toggle" id="themeToggle">
            <span id="themeIcon"></span>
            <span id="themeText">Light</span>
        </button>
    </div>
</div>

        <div class="main-control">
            <div class="card">
                <div id="status" class="disconnected">WebSocket: Disconnected</div>
            </div>

            <div class="card">
                <div class="card-title">Control Joystick</div>
                <div id="joystickContainer">
                    <div id="joystick"></div>
                </div>
            </div>

            <div class="card">
                <div class="card-title">Current Commands</div>
                <div class="controls-display">
                    <div class="control-item">
                        <div class="control-label">Speed</div>
                        <div class="control-value" id="speedValue">0</div>
                    </div>
                    <div class="control-item">
                        <div class="control-label">Steering</div>
                        <div class="control-value" id="steerValue">0</div>
                    </div>
                    <div class="coordinates" id="coordinates">X: 0.00, Y: 0.00</div>
                </div>
            </div>
        </div>

        <div class="sidebar">
            <div class="card">
                <div class="card-title">Settings</div>
                <div class="settings-grid">
                    <div class="slider-container">
                        <div class="slider-label">
                            <span>Max Speed</span>
                            <span class="slider-value" id="maxSpeedValue">300</span>
                        </div>
                        <input type="range" id="maxSpeedSlider" class="slider" min="50" max="1000" value="300">
                    </div>
                    <div class="slider-container">
                        <div class="slider-label">
                            <span>Max Steering</span>
                            <span class="slider-value" id="maxSteerValue">300</span>
                        </div>
                        <input type="range" id="maxSteerSlider" class="slider" min="50" max="1000" value="300">
                    </div>
                </div>
            </div>

            <div class="card">
                <div class="card-title">System Status</div>
                <div class="feedback-section">
                    <div class="feedback-grid">
                        <div class="feedback-item">
                            <div class="feedback-label">Speed R</div>
                            <div class="feedback-value" id="speedRValue">--</div>
                        </div>
                        <div class="feedback-item">
                            <div class="feedback-label">Speed L</div>
                            <div class="feedback-value" id="speedLValue">--</div>
                        </div>
                        <div class="feedback-item">
                            <div class="feedback-label">Temperature</div>
                            <div class="feedback-value" id="tempValue">--°C</div>
                        </div>
                        <div class="feedback-item">
                            <div class="feedback-label">Status</div>
                            <div class="feedback-value" id="statusValue">Waiting...</div>
                        </div>
                    </div>
                </div>
            </div>

            <div class="card">
                <div class="card-title">Battery Status</div>
                <div class="battery-section">
                    <div class="battery-display">
                        <div class="battery-icon">
                            <div id="batteryFill" class="battery-fill" style="width: 100%;"></div>
                        </div>
                        <div>
                            <span id="batteryText" class="battery-text">--.-V</span>
                            <span id="batteryPercentage" class="battery-percentage">(---%)</span>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        var gateway = `ws://${window.location.hostname}:81/ws`;
        var websocket;
        var joystick = document.getElementById('joystick');
        var container = document.getElementById('joystickContainer');
        var status = document.getElementById('status');
        var coordinates = document.getElementById('coordinates');
        var speedDisplay = document.getElementById('speedValue');
        var steerDisplay = document.getElementById('steerValue');
        var maxSpeedSlider = document.getElementById('maxSpeedSlider');
        var maxSteerSlider = document.getElementById('maxSteerSlider');
        var maxSpeedValue = document.getElementById('maxSpeedValue');
        var maxSteerValue = document.getElementById('maxSteerValue');
        var speedRValue = document.getElementById('speedRValue');
        var speedLValue = document.getElementById('speedLValue');
        var tempValue = document.getElementById('tempValue');
        var statusValue = document.getElementById('statusValue');
        var batteryFill = document.getElementById('batteryFill');
        var batteryText = document.getElementById('batteryText');
        var batteryPercentage = document.getElementById('batteryPercentage');
        var themeToggle = document.getElementById('themeToggle');
        var themeIcon = document.getElementById('themeIcon');
        var themeText = document.getElementById('themeText')
        var spiralToggle = document.getElementById('spiralToggle');
        var spiralIcon = document.getElementById('spiralIcon');
        var spiralText = document.getElementById('spiralText');
        var spiralEnabled = false;
        
        var isDragging = false;
        var centerX = container.offsetWidth / 2 - joystick.offsetWidth / 2;
        var centerY = container.offsetHeight / 2 - joystick.offsetHeight / 2;
        var maxDistance = container.offsetWidth / 2 - joystick.offsetWidth / 2;
        var maxSpeed = 300;
        var maxSteer = 300;

        // Theme toggle functionality
        function toggleTheme() {
            const body = document.body;
            const currentTheme = body.getAttribute('data-theme');
            const newTheme = currentTheme === 'dark' ? 'light' : 'dark';
            
            body.setAttribute('data-theme', newTheme);
            
            if (newTheme === 'light') {
                themeIcon.textContent = '🌙';
                themeText.textContent = 'Dark';
            } else {
                themeIcon.textContent = '☀️';
                themeText.textContent = 'Light';
            }
            
            // Save theme preference
            localStorage.setItem('theme', newTheme);
        }

        // Load saved theme
        function loadTheme() {
            const savedTheme = localStorage.getItem('theme') || 'dark';
            document.body.setAttribute('data-theme', savedTheme);
            
            if (savedTheme === 'light') {
                themeIcon.textContent = '🌙';
                themeText.textContent = 'Dark';
            } else {
                themeIcon.textContent = '☀️';
                themeText.textContent = 'Light';
            }
        }

        themeToggle.addEventListener('click', toggleTheme);
        function toggleSpiral() {
            spiralEnabled = !spiralEnabled;
    
            if (spiralEnabled) {
                spiralToggle.classList.add('active');
                spiralIcon.textContent = '⏹️';
                spiralText.textContent = 'Stop Spiral';
        
                // Disable manual joystick
                joystick.style.pointerEvents = 'none';
                joystick.style.opacity = '0.5';
                
                // Send spiral activation command
            if (websocket.readyState === WebSocket.OPEN) {
                websocket.send(JSON.stringify({spiralCut: true}));
            }
        } else {
            spiralToggle.classList.remove('active');
            spiralIcon.textContent = '🌀';
            spiralText.textContent = 'Spiral Cut';
        
            // Re-enable manual joystick
            joystick.style.pointerEvents = 'auto';
            joystick.style.opacity = '1';
        
            // Send spiral deactivation command
        if (websocket.readyState === WebSocket.OPEN) {
            websocket.send(JSON.stringify({spiralCut: false}));
        }
        
        // Reset joystick to center
        handleEnd();
    }
}

spiralToggle.addEventListener('click', toggleSpiral);
var autoToggle = document.getElementById('autoToggle');
var autoIcon = document.getElementById('autoIcon');
var autoText = document.getElementById('autoText');
var autoEnabled = false;

var stopToggle = document.getElementById('stopToggle');
var stopIcon = document.getElementById('stopIcon');
var stopText = document.getElementById('stopText');
var stopEnabled = false;

var cutToggle = document.getElementById('cutToggle');
var cutIcon = document.getElementById('cutIcon');
var cutText = document.getElementById('cutText');
var cutEnabled = false;

// Autonomous mode toggle
function toggleAuto() {
    autoEnabled = !autoEnabled;
    
    if (autoEnabled) {
        autoToggle.classList.add('active');
        autoIcon.textContent = '⏹️';
        autoText.textContent = 'Stop Auto';
        
        // Disable manual joystick and spiral
        joystick.style.pointerEvents = 'none';
        joystick.style.opacity = '0.5';
        spiralToggle.style.pointerEvents = 'none';
        spiralToggle.style.opacity = '0.5';
        
    } else {
        autoToggle.classList.remove('active');
        autoIcon.textContent = '🤖';
        autoText.textContent = 'Auto Mode';
        
        // Re-enable controls
        joystick.style.pointerEvents = 'auto';
        joystick.style.opacity = '1';
        spiralToggle.style.pointerEvents = 'auto';
        spiralToggle.style.opacity = '1';
        
        handleEnd();
    }
    
    if (websocket.readyState === WebSocket.OPEN) {
        websocket.send(JSON.stringify({autonomousMode: autoEnabled}));
    }
}

// Emergency stop toggle
function toggleStop() {
    stopEnabled = !stopEnabled;
    
    if (stopEnabled) {
        stopToggle.classList.add('active');
        stopIcon.textContent = '▶️';
        stopText.textContent = 'Resume';
    } else {
        stopToggle.classList.remove('active');
        stopIcon.textContent = '⛔';
        stopText.textContent = 'E-Stop';
    }
    
    if (websocket.readyState === WebSocket.OPEN) {
        websocket.send(JSON.stringify({emergencyStop: stopEnabled}));
    }
}

// Cutting motor toggle
function toggleCut() {
    cutEnabled = !cutEnabled;
    
    if (cutEnabled) {
        cutToggle.classList.add('active');
        cutIcon.textContent = '⚡';
        cutText.textContent = 'Cut ON';
    } else {
        cutToggle.classList.remove('active');
        cutIcon.textContent = '🔌';
        cutText.textContent = 'Cut OFF';
    }
    
    if (websocket.readyState === WebSocket.OPEN) {
        websocket.send(JSON.stringify({cuttingMotor: cutEnabled}));
    }
}

autoToggle.addEventListener('click', toggleAuto);
stopToggle.addEventListener('click', toggleStop);
cutToggle.addEventListener('click', toggleCut);

        function initWebSocket() {
            console.log('Trying to connect to: ' + gateway);
            websocket = new WebSocket(gateway);
            websocket.onopen = onOpen;
            websocket.onclose = onClose;
            websocket.onmessage = onMessage;
        }

        function onOpen(event) {
            console.log('Connection opened');
            status.innerHTML = "WebSocket: Connected";
            status.className = "connected fade-in";
        }

        function onClose(event) {
            console.log('Connection closed');
            status.innerHTML = "WebSocket: Disconnected";
            status.className = "disconnected pulse";
            setTimeout(initWebSocket, 2000);
        }

        function onMessage(event) {
            try {
                const data = JSON.parse(event.data);
                if (data.feedback) {
                    const voltage = data.feedback.batVoltage;
                    const temp = data.feedback.boardTemp;
                    
                    speedRValue.textContent = data.feedback.speedR;
                    speedLValue.textContent = data.feedback.speedL;
                    tempValue.textContent = temp.toFixed(1) + '°C';
                    statusValue.textContent = 'Active';
                    
                    // Update battery indicator
                    updateBatteryIndicator(voltage);
                }
            } catch (e) {
                console.log('Received: ' + event.data);
            }
        }

        function updateBatteryIndicator(voltage) {
            // Clamp voltage between 20V and 25V
            const minVolt = 20;
            const maxVolt = 25;
            const clampedVolt = Math.max(minVolt, Math.min(maxVolt, voltage));
            
            // Calculate percentage
            const percentage = ((clampedVolt - minVolt) / (maxVolt - minVolt)) * 100;
            
            // Update battery display
            batteryFill.style.width = percentage + '%';
            batteryText.textContent = voltage.toFixed(1) + 'V';
            batteryPercentage.textContent = '(' + Math.round(percentage) + '%)';
            
            // Update colors based on battery level
            batteryFill.className = 'battery-fill';
            joystick.className = '';
            
            if (percentage > 70) {
                // Green - good
            } else if (percentage > 40) {
                batteryFill.className += ' warning';
                joystick.className = 'battery-warning';
            } else {
                batteryFill.className += ' danger';
                joystick.className = 'battery-danger';
            }
        }

        function sendCommand(speed, steer, updateMaxValues = false) {
            if (websocket.readyState === WebSocket.OPEN) {
                const command = {
                    speed: speed,
                    steer: steer
                };
                
                if (updateMaxValues) {
                    command.maxSpeed = maxSpeed;
                    command.maxSteer = maxSteer;
                }
                
                websocket.send(JSON.stringify(command));
            }
        }

        function sendPosition(x, y) {
            // Convert normalized values (-1 to 1) to hoverboard commands
            const speed = Math.round(y * maxSpeed); // Invert Y so up is positive
            const steer = Math.round(x * maxSteer);
            
            // Update displays
            speedDisplay.textContent = speed;
            steerDisplay.textContent = steer;
            
            sendCommand(speed, steer);
        }

        // Slider event handlers
        maxSpeedSlider.addEventListener('input', function() {
            maxSpeed = parseInt(this.value);
            maxSpeedValue.textContent = maxSpeed;
            sendCommand(0, 0, true); // Send max values update
        });

        maxSteerSlider.addEventListener('input', function() {
            maxSteer = parseInt(this.value);
            maxSteerValue.textContent = maxSteer;
            sendCommand(0, 0, true); // Send max values update
        });

        // Joystick event handlers
        function handleStart(event) {
            isDragging = true;
            handleMove(event);
        }

        function handleMove(event) {
            if (!isDragging) return;

            event.preventDefault();
            
            let clientX, clientY;
            if (event.touches) {
                clientX = event.touches[0].clientX;
                clientY = event.touches[0].clientY;
            } else {
                clientX = event.clientX;
                clientY = event.clientY;
            }

            const rect = container.getBoundingClientRect();
            let x = clientX - rect.left - centerX - joystick.offsetWidth / 2;
            let y = clientY - rect.top - centerY - joystick.offsetHeight / 2;

            const distance = Math.sqrt(x * x + y * y);
            if (distance > maxDistance) {
                x = (x / distance) * maxDistance;
                y = (y / distance) * maxDistance;
            }

            joystick.style.left = (centerX + x) + 'px';
            joystick.style.top = (centerY + y) + 'px';

            const normalizedX = x / maxDistance;
            const normalizedY = -y / maxDistance;

            coordinates.innerHTML = `X: ${normalizedX.toFixed(2)}, Y: ${normalizedY.toFixed(2)}`;
            sendPosition(normalizedX, normalizedY);
        }

        function handleEnd() {
            isDragging = false;
            joystick.style.left = centerX + 'px';
            joystick.style.top = centerY + 'px';
            sendPosition(0, 0);
            coordinates.innerHTML = "X: 0.00, Y: 0.00";
            speedDisplay.textContent = "0";
            steerDisplay.textContent = "0";
        }

        // Event listeners
        joystick.addEventListener('mousedown', handleStart);
        document.addEventListener('mousemove', handleMove);
        document.addEventListener('mouseup', handleEnd);
        joystick.addEventListener('touchstart', handleStart);
        document.addEventListener('touchmove', handleMove);
        document.addEventListener('touchend', handleEnd);

        // Recalculate joystick center on window resize
        window.addEventListener('resize', function() {
            centerX = container.offsetWidth / 2 - joystick.offsetWidth / 2;
            centerY = container.offsetHeight / 2 - joystick.offsetHeight / 2;
            maxDistance = container.offsetWidth / 2 - joystick.offsetWidth / 2;
            
            if (!isDragging) {
                joystick.style.left = centerX + 'px';
                joystick.style.top = centerY + 'px';
            }
        });

        window.addEventListener('load', initWebSocket);
    </script>
</body>
</html>)rawliteral";

// ########################## SEND FUNCTION ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  Command.start    = START_FRAME;
  Command.steer    = uSteer;
  Command.speed    = uSpeed;
  Command.checksum = Command.start ^ Command.steer ^ Command.speed;

  LawnMowerSerial.write((uint8_t*)&Command, sizeof(Command)); 
}

// ########################## RECEIVE FUNCTION ##########################
void Receive()
{
  if (LawnMowerSerial.available()) {
    incomingByte = LawnMowerSerial.read();
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;
  } else {
    return;
  }

  if (bufStartFrame == START_FRAME) {
    p = (byte*)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
    *p++ = incomingByte;
    idx++;
  }

  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum = NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
                        NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
                        NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed;

    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
      // Map voltage from 12-bit (0-4095) to actual voltage (0-25V)
      float actualVoltage = map(Feedback.batVoltage, 0, 4095, 0, 4200) / 100.0; // Convert to volts with decimals
      
      // Convert temperature (divide by 100)
      actualTemp = Feedback.boardTemp / 10.0;
      
      // Send feedback to web interface
      StaticJsonDocument<300> feedbackDoc;
      feedbackDoc["feedback"]["speedR"] = Feedback.speedR_meas;
      feedbackDoc["feedback"]["speedL"] = Feedback.speedL_meas;
      feedbackDoc["feedback"]["batVoltage"] = actualVoltage;
      feedbackDoc["feedback"]["boardTemp"] = actualTemp;
      
      String feedbackStr;
      serializeJson(feedbackDoc, feedbackStr);
      ws.textAll(feedbackStr);
      
      Serial.print("SpeedR: "); Serial.print(Feedback.speedR_meas);
      Serial.print(" SpeedL: "); Serial.print(Feedback.speedL_meas);
      Serial.print(" BatV: "); Serial.print(actualVoltage);
      Serial.print(" Temp: "); Serial.println(actualTemp);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;
  }

  incomingBytePrev = incomingByte;
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, data);
    
    if (error) {
      Serial.println("JSON parsing failed!");
      return;
    }
    
    // Update max values if provided
    if (doc.containsKey("maxSpeed")) {
      maxSpeed = doc["maxSpeed"];
      Serial.printf("Max speed updated to: %d\n", maxSpeed);
    }
    if (doc.containsKey("maxSteer")) {
      maxSteer = doc["maxSteer"];
      Serial.printf("Max steer updated to: %d\n", maxSteer);
    }
    // Add this after the existing maxSteer update:
  if (doc.containsKey("spiralCut")) {
  spiralCutEnabled = doc["spiralCut"];
  if (spiralCutEnabled) {
    spiralStartTime = millis();
    spiralRadius = 0.0;
    spiralAngle = 0.0;
    Serial.println("Spiral cut enabled");
  } else {
    Serial.println("Spiral cut disabled");
  }
}


// Only process manual commands if spiral cut is disabled
  

// Handle autonomous mode toggle
if (doc.containsKey("autonomousMode")) {
  autonomousMode = doc["autonomousMode"];
  if (autonomousMode) {
    autoState = AUTO_FORWARD;
    Serial.println("Autonomous mode enabled");
  } else {
    currentSpeed = 0;
    currentSteer = 0;
    Serial.println("Autonomous mode disabled");
  }
}

// Handle emergency stop
if (doc.containsKey("emergencyStop")) {
  emergencyStop = doc["emergencyStop"];
  if (emergencyStop) {
    currentSpeed = 0;
    currentSteer = 0;
    Serial.println("Emergency stop activated");
  } else {
    Serial.println("Emergency stop deactivated");
  }
}

// Handle cutting motor toggle
// I used an arduino nano to receive the digital ON/OFF (HIGH/LOW) states. i then received this digital state on an arduino pin and used the servo library on the 
// arduino nano to turn a brushless esc that controls the cutting motor on or off. Ive not been very succesfull at generating clean pwm signals on the esp32.
if (doc.containsKey("cuttingMotor")) {
  cuttingMotorEnabled = doc["cuttingMotor"];
  digitalWrite(CUTTING_MOTOR_PIN, cuttingMotorEnabled ? HIGH : LOW);
  Serial.printf("Cutting motor %s\n", cuttingMotorEnabled ? "ON" : "OFF");
}
// Only process manual commands if autonomous mode and spiral cut are disabled
if (!spiralCutEnabled && !autonomousMode && !emergencyStop) {
  // Get speed and steer commands
  int speed = doc["speed"];
  int steer = doc["steer"];
  
  // Apply deadzone (convert to normalized values for deadzone check)
  float normalizedSpeed = (float)speed / maxSpeed;
  float normalizedSteer = (float)steer / maxSteer;
  
  if (abs(normalizedSpeed) < DEADZONE) speed = 0;
  if (abs(normalizedSteer) < DEADZONE) steer = 0;
  
  // Constrain values to required range range
  speed = constrain(speed, -maxSpeed, maxSpeed);
  steer = constrain(steer, -maxSteer, maxSteer);
  
  currentSpeed = speed;
  currentSteer = steer;
  
  Serial.printf("Manual - Speed: %d, Steer: %d\n", speed, steer);
 }
}
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void calculateSpiralMovement() {
  if (!spiralCutEnabled) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastSpiralUpdate < SPIRAL_UPDATE_INTERVAL) return;
  
  lastSpiralUpdate = currentTime;
  
  // Calculate spiral position (Archimedean spiral: r = a * θ). It doesn't work really well tho. You may need to adjust the GROWTH_RATE and spiralRadius to get 
  // better results. 
  spiralAngle += SPIRAL_ANGULAR_VELOCITY;
  spiralRadius = SPIRAL_GROWTH_RATE * spiralAngle;
  
  // Convert polar coordinates to movement commands
  float x = spiralRadius * cos(spiralAngle);
  float y = spiralRadius * sin(spiralAngle);
  
  // Calculate differential steering for the spiral
  // This creates the curved path by varying wheel speeds
  currentSpeed = SPIRAL_BASE_SPEED;
  
  // Steering calculation for spiral curve
  // Positive steer = right turn, negative = left turn
  float curvature = SPIRAL_ANGULAR_VELOCITY / (spiralRadius + 0.1); // Avoid division by zero
  currentSteer = (int)(curvature * maxSteer * 0.5); // Scale and limit steering
  
  // Constrain values
  currentSpeed = constrain(currentSpeed, -maxSpeed, maxSpeed);
  currentSteer = constrain(currentSteer, -maxSteer, maxSteer);
  
  Serial.printf("Spiral: Angle=%.2f, Radius=%.2f, Speed=%d, Steer=%d\n", 
                spiralAngle, spiralRadius, currentSpeed, currentSteer);
}
// AJR04 sensor function (The one i used needs only echo pin)
float getDistance(int echoPin) {
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999; // No echo received
  
  float distance = (duration * 0.034) / 2;
  return distance;
}
// Normal ultrasonic sensor function (with trig and echo pins)
float getDistanceNormal(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  
  float distance = (duration * 0.034) / 2;
  return distance;
}

// Autonomous navigation function


void handleAutonomousMode() {
  if (!autonomousMode || emergencyStop) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastObstacleCheck < OBSTACLE_CHECK_INTERVAL) return;
  
  lastObstacleCheck = currentTime;
  
  // Read all sensors
  float frontLeft = getDistance(ECHO_LEFT);
  float frontRight = getDistance(ECHO_RIGHT);
  float sideLeft = getDistanceNormal(TRIG_LEFT_SIDE, ECHO_LEFT_SIDE);
  float sideRight = getDistanceNormal(TRIG_RIGHT_SIDE, ECHO_RIGHT_SIDE);
  
  Serial.printf("Distances - FL: %.1f, FR: %.1f, SL: %.1f, SR: %.1f\n", 
                frontLeft, frontRight, sideLeft, sideRight);
  
  // Check for obstacles
  bool frontObstacle = (frontLeft < OBSTACLE_DISTANCE || frontRight < OBSTACLE_DISTANCE);
  bool sideObstacle = (sideLeft < SIDE_OBSTACLE_DISTANCE || sideRight < SIDE_OBSTACLE_DISTANCE);
  bool allBlocked = frontObstacle && sideObstacle;
  
  switch (autoState) {
    case AUTO_FORWARD:
      if (allBlocked) {
        // All sensors detect obstacles - backup
        Serial.println("All blocked - starting backup");
        autoState = AUTO_BACKUP;
        backupStartTime = currentTime;
        currentSpeed = -100;  // Negative for backward
        currentSteer = 0;
      } else if (frontObstacle) {
        // Front obstacle - check sides for turning
        Serial.println("Front obstacle detected - turning");
        autoState = AUTO_TURN;
        turnStartTime = currentTime;
        currentSpeed = 0;
        
        // Decide turn direction based on side sensor readings
        if (sideLeft > sideRight) {
          currentSteer = -150;  // Turn left (more space on left)
          Serial.println("Turning left");
        } else {
          currentSteer = 150;   // Turn right (more space on right)
          Serial.println("Turning right");
        }
      } else if (sideObstacle) {
        // Only side obstacles - adjust steering while moving slowly
        if (sideLeft < SIDE_OBSTACLE_DISTANCE) {
          currentSpeed = 100;    // Slower speed
          currentSteer = 150;   // Steer away from left obstacle
          Serial.println("Avoiding left side obstacle");
        } else if (sideRight < SIDE_OBSTACLE_DISTANCE) {
          currentSpeed = 100;    // Slower speed
          currentSteer = -150;  // Steer away from right obstacle
          Serial.println("Avoiding right side obstacle");
        }
      } else {
        // No obstacles - move forward normally
        currentSpeed = 100;
        currentSteer = 0;
        Serial.println("Path clear - moving forward");
      }
      break;
      
    case AUTO_BACKUP:
      // Check if we've backed up long enough
      if (currentTime - backupStartTime >= BACKUP_TIME) {
        Serial.println("Backup complete - starting turn");
        autoState = AUTO_TURN;
        turnStartTime = currentTime;
        currentSpeed = 0;
        
        // Choose turn direction based on which side has more space
        if (sideLeft > sideRight) {
          currentSteer = -200;  // Turn left
          Serial.println("After backup - turning left");
        } else {
          currentSteer = 200;   // Turn right
          Serial.println("After backup - turning right");
        }
      } else {
        // Continue backing up
        currentSpeed = -100;
        currentSteer = 0;
        Serial.printf("Backing up... %lu ms remaining\n", 
                     BACKUP_TIME - (currentTime - backupStartTime));
      }
      break;
      
    case AUTO_TURN:
      // Check if we've turned long enough
      if (currentTime - turnStartTime >= TURN_TIME) {
        Serial.println("Turn complete - resuming forward");
        autoState = AUTO_FORWARD;
        currentSpeed = 0;  // Will be set in next cycle based on sensor readings
        currentSteer = 0;
      } else {
        // Continue turning (maintain current steer direction)
        currentSpeed = 0;
        Serial.printf("Turning... %lu ms remaining\n", 
                     TURN_TIME - (currentTime - turnStartTime));
      }
      break;
      
    case AUTO_STOP:
      // Emergency stop state to stop things from goimg out of control! - keep everything at zero
      currentSpeed = 0;
      currentSteer = 0;
      Serial.println("AUTO_STOP state - all motors stopped");
      // You could add logic here to transition back to AUTO_FORWARD
      // after certain conditions are met
      break;
  }
  
  // Safety limits - ensure values don't exceed bounds
  currentSpeed = constrain(currentSpeed, -maxSpeed, maxSpeed);
  currentSteer = constrain(currentSteer, -maxSteer, maxSteer);
  
  Serial.printf("Auto State: %d, Speed: %d, Steer: %d\n", 
                autoState, currentSpeed, currentSteer);
}
// ########################## SETUP ##########################
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.println("ESP32 WiFi Lawn Mower Controller v2.0 - Enhanced UI");
  pinMode(relay, OUTPUT);
  // AJR04 sensor pins setup (only echo pins needed)
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);
  // Side ultrasonic sensor pins setup
  pinMode(TRIG_LEFT_SIDE, OUTPUT);
  pinMode(ECHO_LEFT_SIDE, INPUT);
  pinMode(TRIG_RIGHT_SIDE, OUTPUT);
  pinMode(ECHO_RIGHT_SIDE, INPUT);
  pinMode(CUTTING_MOTOR_PIN, OUTPUT);
  digitalWrite(CUTTING_MOTOR_PIN, LOW);  // Start with cutting motor off

  // Initialize UART communication with lawn mower
  LawnMowerSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, RXD2, TXD2);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set up WiFi access point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // WebSocket setup
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Serve HTML page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // Start the server
  server.begin();
  
  Serial.println("Enhanced Server started");
  Serial.println("Connect to WiFi: Lawn Mower");
  Serial.println("Navigate to: http://192.168.4.1");
  Serial.println("Features: Basic UI, Battery Monitoring, Real-time Feedback");
}

// ########################## MAIN LOOP ##########################
void loop() {
  unsigned long timeNow = millis();

  // Receive feedback from lawn mower
  Receive();

  // Send commands to lawn mower at regular intervals
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  // Calculate spiral movement if enabled
  calculateSpiralMovement();
  // Handle autonomous navigation
  handleAutonomousMode();
  
  Send(currentSteer, currentSpeed);

  // Control cooling fan based on temperature.
  if (actualTemp > 45){
    digitalWrite(relay, HIGH);
  }
  if (actualTemp < 38){
    digitalWrite(relay, LOW);
  }
  //digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
  
  // Clean up WebSocket connections
  //ws.cleanupClients();
}