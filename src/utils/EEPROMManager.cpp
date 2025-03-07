/*
 * ESP32 High-Precision Motion Control System
 * EEPROM Manager Implementation
 */

 #include "EEPROMManager.h"

 EEPROMManager::EEPROMManager()
     : m_initialized(false)
     , m_configValid(false)
     , m_configVersion(CONFIG_EEPROM_VERSION)
 {
 }
 
 bool EEPROMManager::initialize() {
     // Initialize EEPROM
     if (!EEPROM.begin(CONFIG_EEPROM_SIZE)) {
         return false;
     }
     
     m_initialized = true;
     
     // Check if configuration is valid
     uint16_t magicMarker;
     readValue(ADDR_MAGIC_MARKER, magicMarker);
     
     if (magicMarker == CONFIG_EEPROM_MAGIC_MARKER) {
         // Read configuration version
         readValue(ADDR_VERSION, m_configVersion);
         m_configValid = true;
     } else {
         // No valid configuration found
         m_configValid = false;
         // Initialize with default values if needed
         resetToDefaults();
     }
     
     return true;
 }
 
 bool EEPROMManager::isConfigValid() const {
     return m_initialized && m_configValid;
 }
 
 bool EEPROMManager::commit() {
     if (!m_initialized) {
         return false;
     }
     
     return EEPROM.commit();
 }
 
 bool EEPROMManager::resetToDefaults() {
     if (!m_initialized) {
         return false;
     }
     
     // Write magic marker and version
     writeValue(ADDR_MAGIC_MARKER, CONFIG_EEPROM_MAGIC_MARKER);
     writeValue(ADDR_VERSION, CONFIG_EEPROM_VERSION);
     
     // Reset motor configurations
     for (uint8_t i = 0; i < CONFIG_MAX_MOTORS; i++) {
         // PID parameters
         savePIDParameters(i, CONFIG_DEFAULT_PID_KP, CONFIG_DEFAULT_PID_KI, 
                           CONFIG_DEFAULT_PID_KD, CONFIG_DEFAULT_PID_FF);
         
         // Profile parameters
         saveProfileParameters(i, CONFIG_DEFAULT_MAX_VELOCITY, CONFIG_DEFAULT_ACCELERATION,
                              CONFIG_DEFAULT_DECELERATION, CONFIG_DEFAULT_MAX_JERK);
         
         // Soft limits
         saveSoftLimits(i, -1000000, 1000000, true);
     }
     
     // Reset system configuration
     saveSystemConfig(CONFIG_LOG_LEVEL, CONFIG_DEBUG_ENABLED, CONFIG_STATUS_UPDATE_FREQUENCY_HZ);
     
     // Reset safety configuration
     saveSafetyConfig(CONFIG_SAFETY_POSITION_TOLERANCE, CONFIG_SAFETY_VELOCITY_TOLERANCE,
                     CONFIG_SAFETY_MAX_TEMPERATURE_C);
     
     // Commit changes to EEPROM
     bool success = commit();
     
     m_configValid = success;
     
     return success;
 }
 
 bool EEPROMManager::loadPIDParameters(uint8_t motorIndex, float& kp, float& ki, float& kd, float& ff) {
     if (!m_initialized || !m_configValid || !isValidMotorIndex(motorIndex)) {
         return false;
     }
     
     uint16_t baseAddr = getMotorAddress(motorIndex, 0);
     readValue(baseAddr, kp);
     readValue(baseAddr + sizeof(float), ki);
     readValue(baseAddr + 2*sizeof(float), kd);
     readValue(baseAddr + 3*sizeof(float), ff);
     
     return true;
 }
 
 bool EEPROMManager::savePIDParameters(uint8_t motorIndex, float kp, float ki, float kd, float ff) {
     if (!m_initialized || !isValidMotorIndex(motorIndex)) {
         return false;
     }
     
     uint16_t baseAddr = getMotorAddress(motorIndex, 0);
     writeValue(baseAddr, kp);
     writeValue(baseAddr + sizeof(float), ki);
     writeValue(baseAddr + 2*sizeof(float), kd);
     writeValue(baseAddr + 3*sizeof(float), ff);
     
     return true;
 }
 
 bool EEPROMManager::loadProfileParameters(uint8_t motorIndex, float& maxVelocity, float& acceleration, 
                                         float& deceleration, float& jerk) {
     if (!m_initialized || !m_configValid || !isValidMotorIndex(motorIndex)) {
         return false;
     }
     
     uint16_t baseAddr = getMotorAddress(motorIndex, 16); // 16-byte offset from start of motor block
     readValue(baseAddr, maxVelocity);
     readValue(baseAddr + sizeof(float), acceleration);
     readValue(baseAddr + 2*sizeof(float), deceleration);
     readValue(baseAddr + 3*sizeof(float), jerk);
     
     return true;
 }
 
 bool EEPROMManager::saveProfileParameters(uint8_t motorIndex, float maxVelocity, float acceleration, 
                                         float deceleration, float jerk) {
     if (!m_initialized || !isValidMotorIndex(motorIndex)) {
         return false;
     }
     
     uint16_t baseAddr = getMotorAddress(motorIndex, 16); // 16-byte offset from start of motor block
     writeValue(baseAddr, maxVelocity);
     writeValue(baseAddr + sizeof(float), acceleration);
     writeValue(baseAddr + 2*sizeof(float), deceleration);
     writeValue(baseAddr + 3*sizeof(float), jerk);
     
     return true;
 }
 
 bool EEPROMManager::loadSoftLimits(uint8_t motorIndex, int32_t& minLimit, int32_t& maxLimit, bool& enabled) {
     if (!m_initialized || !m_configValid || !isValidMotorIndex(motorIndex)) {
         return false;
     }
     
     uint16_t baseAddr = getMotorAddress(motorIndex, 32); // 32-byte offset from start of motor block
     readValue(baseAddr, minLimit);
     readValue(baseAddr + sizeof(int32_t), maxLimit);
     
     uint8_t enabledValue;
     readValue(baseAddr + 2*sizeof(int32_t), enabledValue);
     enabled = (enabledValue != 0);
     
     return true;
 }
 
 bool EEPROMManager::saveSoftLimits(uint8_t motorIndex, int32_t minLimit, int32_t maxLimit, bool enabled) {
     if (!m_initialized || !isValidMotorIndex(motorIndex)) {
         return false;
     }
     
     uint16_t baseAddr = getMotorAddress(motorIndex, 32); // 32-byte offset from start of motor block
     writeValue(baseAddr, minLimit);
     writeValue(baseAddr + sizeof(int32_t), maxLimit);
     
     uint8_t enabledValue = enabled ? 1 : 0;
     writeValue(baseAddr + 2*sizeof(int32_t), enabledValue);
     
     return true;
 }
 
 bool EEPROMManager::loadSystemConfig(uint8_t& logLevel, bool& debugEnabled, uint8_t& statusUpdateFrequency) {
     if (!m_initialized || !m_configValid) {
         return false;
     }
     
     uint16_t baseAddr = ADDR_SYSTEM_CONFIG_START;
     readValue(baseAddr, logLevel);
     
     uint8_t debugValue;
     readValue(baseAddr + 1, debugValue);
     debugEnabled = (debugValue != 0);
     
     readValue(baseAddr + 2, statusUpdateFrequency);
     
     return true;
 }
 
 bool EEPROMManager::saveSystemConfig(uint8_t logLevel, bool debugEnabled, uint8_t statusUpdateFrequency) {
     if (!m_initialized) {
         return false;
     }
     
     uint16_t baseAddr = ADDR_SYSTEM_CONFIG_START;
     writeValue(baseAddr, logLevel);
     
     uint8_t debugValue = debugEnabled ? 1 : 0;
     writeValue(baseAddr + 1, debugValue);
     
     writeValue(baseAddr + 2, statusUpdateFrequency);
     
     return true;
 }
 
 bool EEPROMManager::loadSafetyConfig(uint32_t& positionErrorThreshold, float& velocityErrorThreshold, 
                                    float& maxTemperature) {
     if (!m_initialized || !m_configValid) {
         return false;
     }
     
     uint16_t baseAddr = ADDR_SAFETY_CONFIG_START;
     readValue(baseAddr, positionErrorThreshold);
     readValue(baseAddr + sizeof(uint32_t), velocityErrorThreshold);
     readValue(baseAddr + sizeof(uint32_t) + sizeof(float), maxTemperature);
     
     return true;
 }
 
 bool EEPROMManager::saveSafetyConfig(uint32_t positionErrorThreshold, float velocityErrorThreshold, 
                                    float maxTemperature) {
     if (!m_initialized) {
         return false;
     }
     
     uint16_t baseAddr = ADDR_SAFETY_CONFIG_START;
     writeValue(baseAddr, positionErrorThreshold);
     writeValue(baseAddr + sizeof(uint32_t), velocityErrorThreshold);
     writeValue(baseAddr + sizeof(uint32_t) + sizeof(float), maxTemperature);
     
     return true;
 }
 
 int EEPROMManager::loadUserData(void* data, size_t size) {
     if (!m_initialized || !m_configValid || data == nullptr) {
         return -1;
     }
     
     // Limit size to available space
     if (size > ADDR_USER_DATA_SIZE) {
         size = ADDR_USER_DATA_SIZE;
     }
     
     // Read data byte-by-byte
     uint8_t* byteData = static_cast<uint8_t*>(data);
     for (size_t i = 0; i < size; i++) {
         byteData[i] = EEPROM.read(ADDR_USER_DATA_START + i);
     }
     
     return size;
 }
 
 int EEPROMManager::saveUserData(const void* data, size_t size) {
     if (!m_initialized || data == nullptr) {
         return -1;
     }
     
     // Limit size to available space
     if (size > ADDR_USER_DATA_SIZE) {
         size = ADDR_USER_DATA_SIZE;
     }
     
     // Write data byte-by-byte
     const uint8_t* byteData = static_cast<const uint8_t*>(data);
     for (size_t i = 0; i < size; i++) {
         EEPROM.write(ADDR_USER_DATA_START + i, byteData[i]);
     }
     
     return size;
 }
 
 uint16_t EEPROMManager::getMotorAddress(uint8_t motorIndex, uint16_t offset) const {
     return ADDR_MOTOR_CONFIG_START + (motorIndex * ADDR_MOTOR_CONFIG_SIZE) + offset;
 }
 
 template<typename T>
 void EEPROMManager::writeValue(uint16_t address, const T& value) {
     if (address + sizeof(T) > CONFIG_EEPROM_SIZE) {
         return;
     }
     
     // Write value byte-by-byte
     const uint8_t* byteData = reinterpret_cast<const uint8_t*>(&value);
     for (size_t i = 0; i < sizeof(T); i++) {
         EEPROM.write(address + i, byteData[i]);
     }
 }
 
 template<typename T>
 void EEPROMManager::readValue(uint16_t address, T& value) {
     if (address + sizeof(T) > CONFIG_EEPROM_SIZE) {
         return;
     }
     
     // Read value byte-by-byte
     uint8_t* byteData = reinterpret_cast<uint8_t*>(&value);
     for (size_t i = 0; i < sizeof(T); i++) {
         byteData[i] = EEPROM.read(address + i);
     }
 }
 
 bool EEPROMManager::isValidMotorIndex(uint8_t motorIndex) const {
     return motorIndex < CONFIG_MAX_MOTORS;
 }