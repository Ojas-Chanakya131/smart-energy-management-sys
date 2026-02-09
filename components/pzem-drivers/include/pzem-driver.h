#include <stdio.h>
#include <stdbool.h>


/**
 * @brief Initializes the pzem module
 *
 * @param txPin Tx pin for module
 * @param rxPin Rx Pin for module
 */
void initialize_pzem(uint8_t txPin, uint8_t rxPin);

/**
 * @brief update the value internally
 *
 * @param addr address of the device
 * @return true request completed
 * @return false
 */
bool updateValues(uint8_t addr);

/**
 * @brief update address
 *
 * @param addr address of the device
 * @param new_addr new address
 * @return true request completed
 * @return false
 */
bool setAddr(uint8_t addr, uint8_t new_addr);

/**
 * @brief Get the Voltage
 *
 * @return float
 */
float getVoltage();
/**
 * @brief Get the Current
 *
 * @return float
 */
float getCurrent();
/**
 * @brief Get the Power
 *
 * @return float
 */
float getPower();
/**
 * @brief Get the Energy
 *
 * @return float
 */
float getEnergy();
/**
 * @brief Get the Frequency
 *
 * @return float
 */
float getFrequency();
/**
 * @brief Get the power factor
 *
 * @return float
 */
float getPF();

/**
 * @brief Get the Alarm value if any
 *
 * @return uint16_t
 */
uint16_t getAlarm();
