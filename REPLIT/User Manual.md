# 🌱 PlantGuard IoT - User Manual

**Welcome to PlantGuard!** This guide will show you how to use the dashboard to monitor your plants and manage sensor alerts.

---

## 📱 Getting Started

### **Accessing PlantGuard**

**Desktop/Laptop:**
- Open your web browser (Chrome, Firefox, Safari, or Edge)
- Go to your PlantGuard web address
- The dashboard will load automatically

**Mobile (Phone/Tablet):**
- Open your web browser
- Go to your PlantGuard web address
- Add `/mobile` at the end for the mobile-friendly version
- **Tip:** Add to home screen for app-like experience!

---

## 🏠 Dashboard Overview

When you first open PlantGuard, you'll see the main dashboard with:

### **Top Navigation Bar**
- **🏠 Dashboard** - Main overview screen
- **📊 Historian** - View historical data and trends
- **⚙️ Settings** - Configure nodes and alerts
- **🔔 Alerts** - View and manage alert notifications
- **📱 Mobile** - Mobile-friendly interface

### **Main Dashboard Cards**
Each sensor node shows:
- 🌡️ **Temperature** - Current air temperature
- 💧 **Soil Moisture** - How wet the soil is (%)
- 🧪 **pH Level** - Soil acidity/alkalinity
- ⚡ **EC (Electrical Conductivity)** - Nutrient levels
- 🌿 **NPK Values** - Nitrogen, Phosphorus, Potassium
- 🔋 **Battery Level** - Power remaining (for battery nodes)
- 📡 **Signal Quality** - Connection strength (RSSI/SNR)

---

## 📊 Viewing Sensor Data

### **Real-time Updates**
- Data updates **automatically** as sensors send readings
- No need to refresh the page!
- Watch values change in real-time

### **Understanding the Values**

#### **Temperature** 🌡️
- Shows in Celsius (°C)
- **Good:** 18-25°C for most plants
- **Warning:** Below 10°C or above 30°C

#### **Soil Moisture** 💧
- Shows as percentage (%)
- **Dry:** 0-30% (needs watering!)
- **Good:** 40-60% (perfect)
- **Wet:** 70-100% (might be overwatered)

#### **pH Level** 🧪
- Shows as 0-14 scale
- **Acidic:** Below 6.0
- **Neutral:** 6.0-7.5 (ideal for most plants)
- **Alkaline:** Above 7.5

#### **EC (Electrical Conductivity)** ⚡
- Shows nutrient concentration
- **Low:** 0-500 (add fertilizer)
- **Good:** 500-2000 (healthy)
- **High:** Above 2000 (too much fertilizer!)

#### **NPK Values** 🌿
- **N (Nitrogen):** Leaf growth
- **P (Phosphorus):** Root and flower development
- **K (Potassium):** Overall plant health

#### **Battery Level** 🔋
- Shows percentage remaining
- **Good:** Above 50%
- **Low:** 20-50% (plan to recharge soon)
- **Critical:** Below 20% (recharge immediately!)

#### **Signal Quality** 📡
- **RSSI:** Signal strength (-50 is excellent, -100 is poor)
- **SNR:** Signal-to-noise ratio (higher is better)
- **Green:** Good connection
- **Yellow:** Weak connection
- **Red:** Poor connection (move node closer to gateway)

---

## 📈 Historical Data (Historian Page)

### **View Past Trends**
1. Click **"Historian"** in the top menu
2. Select a **sensor node** from the dropdown
3. Choose a **time range** (Last Hour, 24 Hours, 7 Days, etc.)
4. View the **interactive chart**

### **Reading the Charts**
- **X-axis:** Time (dates/times)
- **Y-axis:** Sensor values
- **Hover** over any point to see exact values
- **Zoom** by selecting an area on the chart
- **Compare** multiple sensors by selecting them

### **What to Look For**
- **Steady lines:** Healthy, stable conditions
- **Sudden drops/spikes:** Possible issues (check alerts!)
- **Gradual changes:** Normal day/night cycles
- **Patterns:** Daily watering effects, temperature cycles

---

## 🔔 Managing Alerts

### **Viewing Alerts**
1. Click **"Alerts"** in the top menu (or the 🔔 bell icon)
2. See all active and past alerts
3. Alerts show:
   - ⚠️ **Warning:** Minor issue (yellow)
   - 🚨 **Critical:** Serious issue (red)
   - ℹ️ **Info:** General notification (blue)

### **Alert Types**

**Common Alerts:**
- 🌡️ **High/Low Temperature** - Outside safe range
- 💧 **Low Soil Moisture** - Plant needs water
- 💦 **High Soil Moisture** - Possible overwatering
- 🧪 **pH Out of Range** - Soil too acidic/alkaline
- ⚡ **High/Low EC** - Nutrient imbalance
- 🔋 **Low Battery** - Sensor needs recharging
- 📡 **Connection Lost** - Sensor offline

### **Responding to Alerts**

**When You Get an Alert:**
1. Read the alert message carefully
2. Check the affected sensor node
3. Take appropriate action (see below)
4. Mark alert as "Read" when handled

**Common Solutions:**
- **Low Moisture:** Water your plants
- **High Moisture:** Reduce watering, improve drainage
- **High Temperature:** Add shade, increase ventilation
- **Low Temperature:** Move to warmer location, add heat
- **pH Issues:** Add pH adjusters to soil
- **High EC:** Flush soil with water, reduce fertilizer
- **Low EC:** Add fertilizer
- **Low Battery:** Recharge or replace battery
- **Connection Lost:** Check if node is powered on, move closer to gateway

---

## ⚙️ Settings & Configuration

### **Managing Sensor Nodes**

1. Click **"Settings"** in the top menu
2. Select a **sensor node**
3. You can configure:

#### **Basic Settings**
- **Node Name:** Give it a friendly name (e.g., "Tomato Bed", "Greenhouse A")
- **Location:** Where this sensor is placed
- **Plant Type:** What you're growing (helps with alert thresholds)

#### **Update Interval**
- How often sensor reads data
- **Frequent:** Every 1-5 minutes (uses more battery)
- **Normal:** Every 10-30 minutes (balanced)
- **Power Saver:** Every 1-2 hours (longer battery life)

#### **Alert Thresholds**
Set when you want to be notified:
- **Temperature:** Min/Max acceptable values
- **Moisture:** Min/Max acceptable values
- **pH:** Min/Max acceptable values
- **EC:** Min/Max acceptable values

#### **Power Management** (Battery Nodes Only)
- **Deep Sleep:** Enable to extend battery life
- **Display Timeout:** Turn off OLED screen after X minutes

#### **LoRa Settings** (Advanced)
- **Frequency:** Radio frequency (usually 915 MHz)
- **Spreading Factor:** Range vs speed tradeoff
- **Bandwidth:** Signal bandwidth
- **TX Power:** Transmission power (higher = longer range, more battery)

**Note:** Only change LoRa settings if you know what you're doing!

---

## 📱 Mobile Version

### **Features**
- Touch-friendly interface
- Swipe to navigate between nodes
- Quick access to current values
- Simplified charts
- Alert notifications

### **Using Mobile Dashboard**
1. Add `/mobile` to your PlantGuard URL
2. **Swipe left/right** to switch between sensor nodes
3. **Tap cards** to see details
4. **Pull down** to refresh data
5. **Tap alerts icon** to view notifications

### **Adding to Home Screen (iOS)**
1. Open Safari and go to PlantGuard mobile
2. Tap the **Share** button (box with arrow)
3. Scroll down and tap **"Add to Home Screen"**
4. Name it "PlantGuard" and tap **Add**

### **Adding to Home Screen (Android)**
1. Open Chrome and go to PlantGuard mobile
2. Tap the **three-dot menu** (⋮)
3. Tap **"Add to Home screen"**
4. Name it "PlantGuard" and tap **Add**

---

## 🔍 Troubleshooting

### **No Data Showing**
**Problem:** Dashboard shows "No data" or empty cards

**Solutions:**
1. Check if sensor node is powered on
2. Verify battery isn't dead (check OLED display on node)
3. Make sure WiFi/LoRa connection is working
4. Wait 1-2 minutes for first reading

---

### **Data Not Updating**
**Problem:** Values stay the same, no real-time updates

**Solutions:**
1. Refresh your browser page
2. Check if WebSocket is connected (look for "Connected" status)
3. Verify sensor node is still powered on
4. Check your internet connection

---

### **Getting Too Many Alerts**
**Problem:** Alert notifications every few minutes

**Solutions:**
1. Go to Settings → Select node
2. Adjust alert thresholds to be less sensitive
3. Increase update interval to reduce alert frequency
4. Disable alerts temporarily while troubleshooting

---

### **Not Getting Alerts**
**Problem:** No alerts even when values are abnormal

**Solutions:**
1. Check alert settings are enabled
2. Verify threshold values are set correctly
3. Make sure browser allows notifications
4. Check if alerts are being marked as "Read" automatically

---

### **Poor Connection Quality**
**Problem:** Signal keeps dropping, values update slowly

**Solutions:**
1. Move sensor node closer to gateway/MASTER node
2. Remove obstacles between nodes (metal, concrete walls)
3. Check if gateway is powered on and connected to WiFi
4. Verify LoRa antenna is attached properly
5. In Settings, increase TX Power (uses more battery but improves range)

---

### **Battery Draining Too Fast**
**Problem:** Battery dies in days instead of weeks

**Solutions:**
1. Enable Deep Sleep mode in Settings
2. Increase update interval (read less frequently)
3. Reduce display timeout (turn off OLED sooner)
4. Lower TX Power if signal quality is already good
5. Check battery health (might need replacement)

---

## 💡 Tips & Best Practices

### **Daily Routine**
1. ✅ Check dashboard once per day
2. ✅ Review any new alerts
3. ✅ Water plants if moisture is low
4. ✅ Note any unusual patterns

### **Weekly Tasks**
1. ✅ Review historical trends (Historian page)
2. ✅ Check battery levels
3. ✅ Clean sensor probes if dirty
4. ✅ Verify all nodes are still connected

### **Monthly Maintenance**
1. ✅ Calibrate pH sensors if readings seem off
2. ✅ Clean OLED displays
3. ✅ Check alert thresholds still make sense
4. ✅ Update node names/locations if plants moved

### **Optimal Alert Thresholds**

**For Most Vegetables:**
- Temperature: 15-28°C
- Moisture: 40-70%
- pH: 6.0-7.0
- EC: 800-1500

**For Herbs:**
- Temperature: 18-24°C
- Moisture: 30-50% (prefer drier)
- pH: 6.0-7.5
- EC: 600-1200

**For Flowers:**
- Temperature: 18-26°C
- Moisture: 50-70%
- pH: 6.0-6.8
- EC: 1000-2000

**For Succulents:**
- Temperature: 15-30°C
- Moisture: 20-40% (very dry!)
- pH: 6.0-7.5
- EC: 400-1000

---

## 📞 Getting Help

### **Common Questions**

**Q: How often should I water based on moisture readings?**
A: Water when moisture drops below 40% for most plants. Let it dry to 20-30% for succulents.

**Q: What does "EC too high" mean?**
A: Too much fertilizer/nutrients. Flush soil with plain water and reduce fertilizer amount.

**Q: My pH is always high, what should I do?**
A: Add sulfur or acidic amendments to lower pH. Most plants prefer 6.0-7.0.

**Q: How do I know if my sensor is accurate?**
A: Compare with a manual soil tester. Calibrate if readings are consistently off.

**Q: Can I add more sensor nodes?**
A: Yes! Configure new ESP32 nodes and they'll appear automatically on the dashboard.

**Q: Does this work without internet?**
A: The sensors work locally, but you need internet to view the dashboard remotely.

---

## 🎓 Understanding Your Plants

### **Reading Plant Signals**

**Healthy Plant:**
- ✅ Steady moisture (40-60%)
- ✅ Stable temperature (18-25°C)
- ✅ Neutral pH (6.0-7.0)
- ✅ Moderate EC (800-1500)

**Overwatered Plant:**
- ⚠️ Moisture above 70% constantly
- ⚠️ Yellowing leaves
- ⚠️ Possible high EC (nutrients not being absorbed)

**Underwatered Plant:**
- ⚠️ Moisture below 30%
- ⚠️ Wilting leaves
- ⚠️ Slow growth

**Nutrient Deficiency:**
- ⚠️ Low EC (below 500)
- ⚠️ Yellowing leaves
- ⚠️ Slow growth
- **Solution:** Add balanced fertilizer

**Nutrient Burn:**
- ⚠️ High EC (above 2000)
- ⚠️ Brown leaf tips
- ⚠️ Stunted growth
- **Solution:** Flush with water, reduce fertilizer

---

## ✅ Quick Reference Card

**Perfect Conditions (Most Plants):**
- 🌡️ Temperature: 18-25°C
- 💧 Moisture: 40-60%
- 🧪 pH: 6.0-7.0
- ⚡ EC: 800-1500
- 🔋 Battery: Above 50%
- 📡 Signal: RSSI > -80 dBm

**When to Take Action:**
- Water: Moisture < 40%
- Add Fertilizer: EC < 600
- Flush Soil: EC > 2000
- Adjust pH: pH < 5.5 or > 7.5
- Recharge Battery: Battery < 20%

---

## 🌟 You're All Set!

You now know how to:
- ✅ Read the dashboard
- ✅ Understand sensor values
- ✅ Respond to alerts
- ✅ View historical data
- ✅ Configure settings
- ✅ Troubleshoot issues
- ✅ Keep your plants healthy!

**Happy Growing! 🌱**

---

*For technical documentation and developer setup, see README.md*
