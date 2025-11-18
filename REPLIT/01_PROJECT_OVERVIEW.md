# 🌱 PlantGuard IoT

> **Real-time IoT Plant Monitoring System for Greenhouse & Agricultural Applications**

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.0-blue.svg)](https://www.typescriptlang.org/)
[![React](https://img.shields.io/badge/React-18-61dafb.svg)](https://reactjs.org/)
[![Express](https://img.shields.io/badge/Express-4.x-000000.svg)](https://expressjs.com/)

PlantGuard is a comprehensive IoT monitoring system designed for our **Senior Project at Kennesaw State University**, providing real-time environmental monitoring with ESP32 hardware, LoRa communication, and dual dashboards for optimal plant health management.

[PlantGuard Dashboard](https://farm-sensor-moralesmarlon50.replit.app)

<img width="668" height="1204" alt="image" src="https://github.com/user-attachments/assets/e8f6ad2b-3043-4ae4-8db9-ba3d66c3c4cd" />


## 🎯 Project Overview

PlantGuard monitors critical environmental conditions for greenhouse and agricultural applications:

- 🌡️ **Temperature & Humidity** monitoring
- 💧 **Soil Moisture** tracking
- 🧪 **pH Levels** measurement
- ⚡ **Electrical Conductivity (EC)** sensing
- 🌿 **NPK (Nitrogen, Phosphorus, Potassium)** analysis
- 📡 **Real-time LoRa Communication** (915 MHz)
- 🔋 **Battery-powered SLAVE nodes** with deep sleep (2-4 weeks runtime)
- 📊 **Live data visualization** with WebSocket updates
- 🚨 **Automated alerts** for out-of-range conditions

---

## 🏗️ System Architecture

### **Hardware Setup**
```
┌─────────────────┐         LoRa 915MHz           ┌─────────────────┐
│  SLAVE Node(s)  │ ──────────────────────────>   │  MASTER Gateway │
│  (Sensors)      │         RS485 Data            │  (WiFi/MQTT)    │
│  - Battery      │                               │  - Wall Power   │
│  - RS485 Sensor │                               │  - OLED Display │
│  - LoRa Radio   │                               │  - LoRa Radio   │
└─────────────────┘                               └─────────────────┘
                                                          │
                                                          │ WiFi/MQTT
                                                          ▼
                                              ┌─────────────────────┐
                                              │   Cloud Server      │
                                              │   - PostgreSQL DB   │
                                              │   - WebSocket       │
                                              │   - REST API        │
                                              └─────────────────────┘
                                                          │
                                      ┌───────────────────┴───────────────────┐
                                      ▼                                       ▼
                            ┌──────────────────┐                  ┌──────────────────┐
                            │  Web Dashboard   │                  │  Node-RED Flow   │
                            │  (React PWA)     │                  │  (Automation)    │
                            └──────────────────┘                  └──────────────────┘
```

### **Software Stack**

#### **Frontend**
- **React 18** with TypeScript
- **Shadcn/UI** + Tailwind CSS for modern, responsive design
- **TanStack Query** for server state management
- **Chart.js** for historical data visualization
- **WebSockets** for real-time updates
- **PWA Support** for mobile access

#### **Backend**
- **Express.js** with TypeScript
- **PostgreSQL** (Neon serverless) with Drizzle ORM
- **WebSocket Server** for live data broadcasting
- **MQTT Integration** for ESP32 communication
- **Zod** for data validation
- **RESTful API** with rate limiting

#### **IoT Firmware**
- **ESP32** (Heltec WiFi LoRa 32 V3.2)
- **LoRa Radio** (915 MHz, SX1262)
- **MQTT Protocol** for cloud communication
- **RS485** sensor interface
- **Deep Sleep** power management

---

## ✨ Key Features

### **Real-time Monitoring**
- Live sensor data updates via WebSockets
- Historical data visualization with interactive charts
- Connection quality tracking (RSSI/SNR)
- Data rate monitoring

### **Smart Alerts**
- Automated threshold-based alerts
- Customizable alert rules
- Alert history and management
- Node-RED integration for custom automation

### **Power Optimization**
- Deep sleep mode for SLAVE nodes
- 2-4 weeks battery life
- OLED auto-sleep timeout
- Remote configuration via MQTT

### **Dual Dashboard**
- **Replit Web App**: Full-featured dashboard with charts and analytics
- **Node-RED Flow**: Visual programming for automation and alerts

### **Remote Configuration**
- Adjust sensor update intervals
- Configure LoRa radio settings (frequency, SF, bandwidth, TX power)
- Update deep sleep timings
- OLED display timeout control

### **Performance Optimized**
- Rate limiting (5s window for sensor data)
- WebSocket throttling (1-second batching)
- Automatic database cleanup (keeps last 7 days)
- 96% faster query performance
- 99.9% faster request processing

---

## 🚀 Quick Start

### **Prerequisites**
- Node.js 18+
- PostgreSQL database
- MQTT broker (optional, for ESP32 integration)

### **Installation**

1. **Clone the repository**
   ```bash
   git clone https://github.com/yourusername/plantguard-iot.git
   cd plantguard-iot
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

3. **Set up environment variables**
   ```bash
   # Create .env file
   DATABASE_URL=postgresql://...
   MQTT_BROKER_URL=mqtt://broker.example.com:1883
   MQTT_USERNAME=your_username
   MQTT_PASSWORD=your_password
   ```

4. **Initialize database**
   ```bash
   npm run db:push
   ```

5. **Start the application**
   ```bash
   npm run dev
   ```

6. **Access the dashboard**
   - Web: `http://localhost:5000`
   - Mobile PWA: `http://localhost:5000/mobile`

---

## 🔧 ESP32 Firmware Setup

### **Hardware Requirements**
- **Heltec WiFi LoRa 32 V3.2** (ESP32-S3 based)
- RS485 Soil Sensor (with NPK, pH, moisture, EC, temp)
- 0.96" OLED Display (built-in)
- LoRa Radio SX1262 (built-in, 915 MHz)

### **Firmware Files**

#### **MASTER Node (Gateway)**
- File: `ESP32_PlantGuard_MASTER.ino`
- Function: Receives LoRa data from SLAVEs, forwards to MQTT
- Power: Wall-powered (USB or 5V)
- Features: OLED display, WiFi, MQTT client

#### **SLAVE Node (Sensor)**
- File: `ESP32_PlantGuard_SLAVE.ino`
- Function: Reads RS485 sensors, transmits via LoRa
- Power: Battery-powered (18650 Li-ion recommended)
- Features: Deep sleep, low power mode, remote configuration

### **Upload Instructions**
See [MASTER_SLAVE_SETUP_GUIDE.md](./MASTER_SLAVE_SETUP_GUIDE.md) for detailed setup instructions.

---

## 📚 Documentation

- **[Configuration Guide](./CONFIGURATION_GUIDE.md)** - Server and ESP32 configuration
- **[MASTER/SLAVE Setup](./MASTER_SLAVE_SETUP_GUIDE.md)** - ESP32 firmware installation
- **[Node-RED Integration](./NODE_RED_INTEGRATION.md)** - Automation setup
- **[MASTER Display Troubleshooting](./MASTER_DISPLAY_TROUBLESHOOTING.md)** - Display issues

---

## 📡 API Endpoints

### **Sensor Data**
- `GET /api/sensor-nodes` - List all sensor nodes
- `GET /api/sensor-readings/history/:nodeId` - Get historical data
- `POST /api/node-red/sensor-data` - Submit sensor data (Node-RED/ESP32)

### **Alerts**
- `GET /api/alerts` - Get all alerts
- `POST /api/node-red/alert` - Create alert from Node-RED
- `PATCH /api/alerts/:id` - Update alert status

### **Configuration**
- `GET /api/esp32-configs/:nodeId` - Get ESP32 configuration
- `PUT /api/esp32-configs/:nodeId` - Update ESP32 configuration
- `GET /api/lora-configs/:nodeId` - Get LoRa radio settings

### **Admin**
- `POST /api/admin/cleanup` - Manually trigger database cleanup

---

## 🎨 Screenshots

### Dashboard (Desktop)
*Real-time monitoring with historical charts and alert management*

### Mobile PWA
*Touch-friendly interface optimized for smartphones and tablets*

### Node-RED Flow
*Visual automation and custom alert workflows*

---

## 🔒 Security

- Environment variables for sensitive credentials
- MQTT authentication support
- Session management with PostgreSQL store
- Input validation with Zod schemas
- CORS configuration for API protection

---

## 📊 Performance

- **Query Speed**: 39-64ms (96% faster than initial implementation)
- **Request Processing**: 0-1ms for duplicate detection (99.9% improvement)
- **WebSocket Updates**: Batched updates (1-3 per second)
- **Database Size**: Auto-cleanup keeps last 7 days of data
- **Power Efficiency**: 2-4 weeks battery life for SLAVE nodes

---

## 🤝 Contributing

This is a senior project for **Kennesaw State University (KSU)**. Contributions, suggestions, and feedback are welcome!

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👥 Authors

**KSU Senior Project Teammate - Marlon Morales**
- Project: PlantGuard IoT Monitoring System
- Hardware: ESP32 (Heltec WiFi LoRa 32 V3.2)
- Frequency: 915 MHz LoRa
- Architecture: MASTER/SLAVE communication

---

## 🙏 Acknowledgments

- **Kennesaw State University** - Academic support
- **Replit** - Development platform
- **Heltec Automation** - ESP32 hardware
- **Semtech** - LoRa technology

---

## 📞 Support

For questions or issues:
- Open an [Issue](https://github.com/yourusername/plantguard-iot/issues)
- Check existing [Documentation](./CONFIGURATION_GUIDE.md)
- Review [Troubleshooting Guide](./MASTER_DISPLAY_TROUBLESHOOTING.md)

---

<div align="center">

**Built with ❤️ for sustainable agriculture and plant health optimization**

[⭐ Star this repo](https://github.com/yourusername/plantguard-iot) • [🐛 Report Bug](https://github.com/yourusername/plantguard-iot/issues) • [💡 Request Feature](https://github.com/yourusername/plantguard-iot/issues)

</div>
