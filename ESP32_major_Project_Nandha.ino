#include <WiFi.h>
#include <HTTPClient.h>
//#include "model.h"   // Include your TinyML model header (Edge Impulse / TensorFlow Lite Micro)


/* ----------- Wi-Fi & Server Config ----------- */
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const char* SERVER_URL;

/* ----------- Global Variables ----------- */
extern String incomingData;
extern bool dataReady;
extern unsigned long lastSendTime;
extern const unsigned long SEND_INTERVAL;

// Wi-Fi / Dashboard config
#define DASHBOARD_IP   "192.168.1.100"  // Replace with your dashboard server IP
#define DASHBOARD_PORT 5000             // Replace with your dashboard server port

// Wi-Fi credentials
#define WIFI_SSID     "YourWiFiSSID"     // Replace with your Wi-Fi network name
#define WIFI_PASSWORD "YourWiFiPassword" // Replace with your Wi-Fi password

// TinyML / Anomaly detection threshold
#define SOME_THRESHOLD 0.5  // Set according to your trained model's scale 


/* ----------- Function Declarations ----------- */

// 1️⃣ Initialize Wi-Fi
void WiFi_Init();

// 2️⃣ Send JSON data to IoT dashboard
void Send_To_Dashboard(const String& jsonPayload);

// 3️⃣ Process incoming data from STM32
void Process_Incoming_Data(const String& msg);

// 4️⃣ Trigger local alert (LED, buzzer)
void Local_Alert(const String& alertMsg);

// 5️⃣ Receive data via UART from STM32
void UART_Receive_Loop();

// 6️⃣ Check Wi-Fi connection status and reconnect if needed
bool Check_WiFi();

// 7️⃣ Optional: parse EEG data into channels
void Parse_EEG_Data(const String& rawData, float* channelData, uint8_t numChannels);

// 8️⃣ Optional: compute bandpower or features from EEG
void Compute_Bandpower(float* channelData, uint8_t numChannels, float* bandPower);

// 9️⃣ Optional: detect anomaly/seizure from processed data
bool Detect_Anomaly(float* features, uint8_t numFeatures);

// 🔟 Optional: send alert to IoT or user
void WiFi_Send_Alert(const char* alertMsg);

// 11️⃣ Optional: initialization of peripherals (LEDs, buzzer)
void Peripheral_Init();

/* ----------- Setup & Loop ----------- */
void setup() {
    // -----------------------------
    // 1. Initialize serial for debugging
    // -----------------------------
    Serial.begin(115200);
    while (!Serial);  // Wait until Serial is ready

    Serial.println("Starting ESP32 EEG Monitoring System...");

    // -----------------------------
    // 2. Initialize all peripherals
    // -----------------------------
    Peripheral_Init();  // ADC, timers, sensors, etc.
    Serial.println("Peripherals initialized.");

    // -----------------------------
    // 3. Initialize Wi-Fi
    // -----------------------------
    WiFi_Init();  // Configure SSID, password, etc.
    Serial.println("Wi-Fi initialization done.");

    // -----------------------------
    // 4. Check Wi-Fi connection with retries
    // -----------------------------
    uint8_t attempts = 0;
    const uint8_t maxAttempts = 5;

    while (!Check_WiFi() && attempts < maxAttempts) {
        Serial.println("Wi-Fi not connected! Retrying...");
        delay(1000);  // Wait 1 second before retry
        attempts++;
    }

    if (Check_WiFi()) {
        Serial.println("Wi-Fi connected successfully.");
    } else {
        Serial.println("Wi-Fi connection failed after retries.");
        Local_Alert("Wi-Fi connection failed!"); // Optional local notification
    }

    // -----------------------------
    // 5. Ready message
    // -----------------------------
    Serial.println("ESP32 EEG monitoring system ready.");
}

void loop() {
    // 1. Continuously check if Wi-Fi is connected
    if (!Check_WiFi()) {
        WiFi_Init(); // Reconnect if disconnected
    }

    // 2. Process incoming EEG data from STM32 via UART
    UART_Receive_Loop();

    // 3. Optional: other periodic tasks
    // e.g., check battery, log system status, etc.
    
    delay(1); // Small delay to avoid blocking
}


// 1️⃣ Initialize Wi-Fi
void WiFi_Init() {
    Serial.println("Initializing Wi-Fi...");

    // Connect to Wi-Fi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    // Wait until connected
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.print("Connected to Wi-Fi. IP Address: ");
    Serial.println(WiFi.localIP());
}


// 2️⃣ Send JSON data to IoT dashboard
void Send_To_Dashboard(const String& jsonPayload) {
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client;
        String url = String("http://") + DASHBOARD_IP + ":" + DASHBOARD_PORT + "/update";

        if (client.connect(DASHBOARD_IP, DASHBOARD_PORT)) {
            // Construct HTTP POST request
            client.println("POST " + String("/update") + " HTTP/1.1");
            client.println("Host: " + String(DASHBOARD_IP));
            client.println("Content-Type: application/json");
            client.print("Content-Length: ");
            client.println(jsonPayload.length());
            client.println();
            client.println(jsonPayload);

            Serial.println("Data sent to dashboard:");
            Serial.println(jsonPayload);
        } else {
            Serial.println("Failed to connect to dashboard!");
        }
        client.stop();
    } else {
        Serial.println("Wi-Fi not connected. Cannot send data.");
    }
}

// 3️⃣ Process incoming data from STM32
void Process_Incoming_Data(const String& msg) {
    // -----------------------------
    // 1. Debug: print the raw message
    // -----------------------------
    Serial.print("Received message: ");
    Serial.println(msg);

    // -----------------------------
    // 2. Parse the message
    // Example: if STM32 sends CSV: "RMS,VAR,HJ_A,HJ_M,HJ_C,DEL,TH,AL,BE,ENTROPY"
    // -----------------------------
    // Split message by comma
    // Use Arduino's String functions or a helper function
    // Example:
    // String values[10];
    // int index = 0;
    // int from = 0;
    // while ((index = msg.indexOf(',', from)) != -1) {
    //     values[i++] = msg.substring(from, index);
    //     from = index + 1;
    // }
    // values[i] = msg.substring(from);

    // -----------------------------
    // 3. Convert parsed values to floats
    // -----------------------------
    // float rms = values[0].toFloat();
    // float variance = values[1].toFloat();
    // ... etc

    // -----------------------------
    // 4. Check for alerts or commands
    // Example: if msg contains "SEIZURE" then trigger dashboard alert
    // -----------------------------
    if (msg.indexOf("SEIZURE") >= 0) {
        Serial.println("ALERT: Seizure detected!");
        // Optionally forward to dashboard or trigger buzzer/LED
    }

    // -----------------------------
    // 5. Store or buffer data for dashboard
    // -----------------------------
    // Example: append to a JSON string for periodic sending
}

// 4️⃣ Trigger local alert (LED, buzzer)
void Local_Alert(const String& alertMsg) {
    // -----------------------------
    // 1. Debug: print alert message
    // -----------------------------
    Serial.print("Local Alert: ");
    Serial.println(alertMsg);

    // -----------------------------
    // 2. Trigger buzzer or LED
    // -----------------------------
    // Example: if using a buzzer on pin BUZZER_PIN
    // digitalWrite(BUZZER_PIN, HIGH);
    // delay(500); // half-second buzz
    // digitalWrite(BUZZER_PIN, LOW);

    // Example: if using an LED on pin LED_PIN
    // digitalWrite(LED_PIN, HIGH);
    // delay(1000); // LED on for 1 second
    // digitalWrite(LED_PIN, LOW);

    // -----------------------------
    // 3. Optional: display on local screen
    // -----------------------------
    // e.g., if using OLED or LCD
    // display.clearDisplay();
    // display.setCursor(0,0);
    // display.print(alertMsg);
    // display.display();
}

// 5️⃣ Receive data via UART from STM32
void UART_Receive_Loop() {
    // -----------------------------
    // 1. Check if data is available on UART
    // -----------------------------
    while (Serial1.available()) {
        // 2. Read incoming bytes
        String incomingMsg = Serial1.readStringUntil('\n'); // read until newline

        // 3. Optional: debug print
        Serial.print("Received UART: ");
        Serial.println(incomingMsg);

        // 4. Process the received data
        Process_Incoming_Data(incomingMsg);
    }

    // -----------------------------
    // 5. Optional: add small delay to avoid hogging CPU
    // -----------------------------
    delay(5); // adjust as needed
}

// 6️⃣ Check Wi-Fi connection status and reconnect if needed
bool Check_WiFi() {
    // -----------------------------
    // 1. Check current Wi-Fi connection status
    // -----------------------------
    if (WiFi.status() == WL_CONNECTED) {
        // Optional: debug print
        Serial.println("Wi-Fi connected");
        return true;
    } else {
        Serial.println("Wi-Fi not connected");
        // Optionally, try to reconnect here
        // WiFi.reconnect();
        return false;
    }
}

// 7️⃣ Optional: parse EEG data into channels
void Parse_EEG_Data(const String& rawData, float* channelData, uint8_t numChannels) {
    // -----------------------------
    // 1. Assume rawData is a comma-separated string of EEG values
    //    e.g., "0.12,0.15,0.10,0.20"
    // -----------------------------

    int startIndex = 0;
    int commaIndex = 0;
    for (uint8_t i = 0; i < numChannels; i++) {
        commaIndex = rawData.indexOf(',', startIndex);

        String valueStr;
        if (commaIndex != -1) {
            valueStr = rawData.substring(startIndex, commaIndex);
            startIndex = commaIndex + 1;
        } else {
            // Last value (no comma)
            valueStr = rawData.substring(startIndex);
        }

        // Convert to float and store in array
        channelData[i] = valueStr.toFloat();
    }

    // -----------------------------
    // 2. Optional: Debug print each channel
    // -----------------------------
    for (uint8_t i = 0; i < numChannels; i++) {
        Serial.print("Channel "); Serial.print(i); 
        Serial.print(": "); Serial.println(channelData[i]);
    }
}

// 8️⃣ Optional: compute bandpower or features from EEG
void Compute_Bandpower(float* channelData, uint8_t numChannels, float* bandPower) {
    // -----------------------------
    // 1. Assume we have 4 EEG bands: Delta, Theta, Alpha, Beta
    //    bandPower should be of size numChannels * 4
    // -----------------------------

    // Placeholder for FFT arrays per channel
    // (You can replace with actual ArduinoFFT or other FFT library)
    // double vReal[BUFFER_SIZE];
    // double vImag[BUFFER_SIZE];

    for (uint8_t ch = 0; ch < numChannels; ch++) {
        // -----------------------------
        // 2. Extract data for this channel
        // -----------------------------
        // Fill vReal with channelData samples for this channel

        // -----------------------------
        // 3. Apply FFT to compute frequency spectrum
        // -----------------------------
        // FFT.Compute()
        // FFT.ComplexToMagnitude()

        // -----------------------------
        // 4. Sum power within each frequency band
        //    For example: Delta 0.5-4Hz, Theta 4-8Hz, Alpha 8-13Hz, Beta 13-30Hz
        // -----------------------------
        // bandPower[ch*4 + 0] = delta power
        // bandPower[ch*4 + 1] = theta power
        // bandPower[ch*4 + 2] = alpha power
        // bandPower[ch*4 + 3] = beta power

        // -----------------------------
        // 5. Optional: normalize or scale band powers
        // -----------------------------
    }
}

// 9️⃣ Optional: detect anomaly/seizure from processed data
bool Detect_Anomaly(float* features, uint8_t numFeatures) {
    // ---------- TinyML inference ----------
    // Step 1: Prepare input buffer for the model
    // Example: input_data is float array same as features
    float input_data[numFeatures];
    for (uint8_t i = 0; i < numFeatures; i++) {
        input_data[i] = features[i];
    }

    // Step 2: Run inference
    // Replace 'run_model_inference' with your actual TinyML function
    // Example return: probability or anomaly score (0 to 1)

    /* float anomalyScore = run_model_inference(input_data, numFeatures);

    // Step 3: Compare with model threshold
    const float MODEL_THRESHOLD = 0.8; // Adjust according to your TinyML training
    if (anomalyScore > MODEL_THRESHOLD) {
        return true;  // Anomaly detected by model
    }
    */

    // ---------- Fallback threshold check ----------
    for (uint8_t i = 0; i < numFeatures; i++) {
        if (features[i] > SOME_THRESHOLD) {
            return true;  // Anomaly detected by simple threshold
        }
    }
    return false;  // No anomaly
}

// 🔟 Optional: send alert to IoT or user
void WiFi_Send_Alert(const char* alertMsg) {
    // -----------------------------
    // 1. Convert the alert message to JSON or plain string
    // -----------------------------
    // Example: {"alert":"Seizure detected!"}
    String jsonPayload = "{\"alert\":\"";
    jsonPayload += alertMsg;
    jsonPayload += "\"}";

    // -----------------------------
    // 2. Send the alert to dashboard / server
    // -----------------------------
    // Use your preferred method:
    // a) Serial1 (UART) to ESP32 / WiFi module
    // b) HTTP POST / MQTT via WiFi connection
    // c) Any cloud IoT dashboard
    // -----------------------------
    
    Send_To_Dashboard(jsonPayload);  // placeholder function

    // -----------------------------
    // 3. Optional: Print to local Serial for debugging
    // -----------------------------
    Serial.println("[ALERT SENT]: " + jsonPayload);
}

// 11️⃣ Optional: initialization of peripherals (LEDs, buzzer)
void Peripheral_Init() {
    // -----------------------------
    // 1. Initialize Serial for debugging
    // -----------------------------
    Serial.begin(115200);
    while(!Serial); // Wait for Serial to be ready

    // -----------------------------
    // 2. Initialize Wi-Fi / ESP module
    // -----------------------------
    WiFi_Init();

    // -----------------------------
    // 3. Initialize ADC / EEG sensor interface
    // -----------------------------
    // e.g., configure ADC pins, I2C/SPI if using external EEG board
    // ADC_Init(); // placeholder

    // -----------------------------
    // 4. Initialize any other sensors / peripherals
    // -----------------------------
    // GPIOs, buttons, LEDs, buzzer, etc.
    // pinMode(LED_PIN, OUTPUT);
    // pinMode(BUZZER_PIN, OUTPUT);

    // -----------------------------
    // 5. Initialize timers if needed
    // -----------------------------
    // Timer_Init(); // placeholder
}