/**
 * @file kalman_filter.c
 * @brief Implementación del filtro de Kalman para velocidades de encoders
 * @author Yonathan López Mejía, Santiago Vargas, Mauricio Aguas
 * @date 2025
 */

#include "kalman_filter_1d.h"

void kalman_init(kalman_filter_t *kf, float process_noise, float measurement_noise) {
    // Inicializar estado a cero
    kf->x = 0.0f;
    
    // Inicializar covarianza con incertidumbre inicial
    kf->p = 1.0f;
    
    // Configurar ruidos
    kf->q = process_noise;      // Ruido del proceso
    kf->r = measurement_noise;  // Ruido de medición
}

float kalman_update(kalman_filter_t *kf, float measurement) {
    // Predecir el siguiente estado (asumimos velocidad constante)
    float x_pred = kf->x;
    
    // Predecir la covarianza del error
    float p_pred = kf->p + kf->q;
    
    // Calcular la ganancia de Kalman
    // K = P_pred / (P_pred + R)
    float k = p_pred / (p_pred + kf->r);
    
    // Actualizar la estimación con la medición
    // x = x_pred + K * (medición - x_pred)
    kf->x = x_pred + k * (measurement - x_pred);
    
    // Actualizar la covarianza del error
    // P = (1 - K) * P_pred
    kf->p = (1.0f - k) * p_pred;
    
    // retorna el estado filtrado
    return kf->x;
}

void kalman_reset(kalman_filter_t *kf) {
    // Resetear estado y covarianza
    kf->x = 0.0f;
    kf->p = 1.0f;
    // Los parámetros q y r se mantienen
}