#ifndef WIFI_BLE_ONBOARDING_H
#define WIFI_BLE_ONBOARDING_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Wi-Fi connection from NVS.
 * 
 * If no credentials exist or connection fails after timeout,
 * starts a BLE GATT server named "Antigravity_Agent_BLE" to accept
 * new credentials. Also starts UDP listener for network handover.
 */
void wifi_ble_onboarding_init(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_BLE_ONBOARDING_H */
