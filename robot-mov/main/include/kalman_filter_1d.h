/**
 * @file kalman_filter_1d.h
 * @brief Filtro de Kalman simple para velocidades de encoders
 * @author Yonathan López Mejía, Santiago Vargas, Mauricio Aguas
 * @date 2025
 * 
 * Filtro de Kalman unidimensional para suavizar lecturas de velocidad
 * de encoders en robot con ESP32-S3
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

/**
 * @brief Estructura del filtro de Kalman
 * 
 * Contiene el estado interno del filtro necesario para
 * el procesamiento secuencial de mediciones
 */
typedef struct {
    float x;           // Estado estimado (velocidad filtrada)
    float p;           // Covarianza del error de estimación
    float q;           // Ruido del proceso (process noise)
    float r;           // Ruido de medición (measurement noise)
} kalman_filter_t;

/**
 * @brief Inicializa el filtro de Kalman
 * 
 * Debe llamarse una vez antes de usar el filtro. Configura los
 * parámetros de ruido que determinan el comportamiento del filtro.
 * 
 * @param kf Puntero a la estructura del filtro
 * @param process_noise Ruido del proceso Q
 *                      - Valores típicos: 0.001 - 0.01
 *                      - Más bajo: respuesta más suave y lenta
 *                      - Más alto: respuesta más rápida, menos filtrado
 * @param measurement_noise Ruido de medición R
 *                          - Valores típicos: 0.5 - 2.0
 *                          - Más bajo: confía más en las mediciones
 *                          - Más alto: más filtrado
 * 
 * @note Empezar con Q=0.005 y R=1.0, luego ajustar según necesidad
 */
void kalman_init(kalman_filter_t *kf, float process_noise, float measurement_noise);

/**
 * @brief Actualiza el filtro con nueva medición de velocidad
 * 
 * Debe llamarse periódicamente (cada 2ms recomendado) con cada
 * nueva medición de velocidad del encoder.
 * 
 * @param kf Puntero a la estructura del filtro
 * @param measurement Velocidad medida sin filtrar (en tus unidades)
 * @return Velocidad filtrada (en las mismas unidades)
 * 
 * @note Esta función implementa las ecuaciones del filtro de Kalman:
 *       - Predicción: estima el estado siguiente
 *       - Actualización: corrige con la medición
 */
float kalman_update(kalman_filter_t *kf, float measurement);

/**
 * @brief Resetea el filtro a su estado inicial
 * 
 * Útil cuando se necesita reiniciar el filtro, por ejemplo:
 * - Al cambiar de modo de operación
 * - Después de una parada completa
 * - Al detectar una condición de error
 * 
 * @param kf Puntero a la estructura del filtro
 */
void kalman_reset(kalman_filter_t *kf);

#endif // KALMAN_FILTER_H