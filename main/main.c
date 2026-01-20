#include <stdio.h> // Funciones estándar de entrada/salida (printf)
#include <math.h> // Funciones matemáticas (pow para cálculo de potencia)
#include "freertos/FreeRTOS.h" // Núcleo del sistema operativo en tiempo real
#include "freertos/task.h" // Funciones para manejo de tareas y retardos
#include "driver/i2c.h" // Controlador de interfaz I2C para comunicación con periféricos

// PLACA Y MÓDULOS
// ESP32-WROOM-32 2AC7Z ← HW-611 (BMP280): 3V3, GND, G21 (SDA), G22 (SCL)

// CONFIGURACIÓN I2C
#define I2C_MASTER_PORT I2C_NUM_0 // Puerto I2C maestro número 0
#define I2C_SDA_PIN 21 // Pin GPIO21 para línea de datos SDA
#define I2C_SCL_PIN 22 // Pin GPIO22 para línea de reloj SCL
#define I2C_FREQ_HZ 100000 // Frecuencia de comunicación I2C (100 kHz)

// BMP280
#define BMP280_ADDR 0x76 // Dirección I2C del sensor BMP280 (7 bits)
#define REG_CALIB 0x88 // Registro inicial de coeficientes de calibración
#define REG_CTRL_MEAS 0xF4 // Registro de control de mediciones
#define REG_CONFIG 0xF5 // Registro de configuración (standby, filtro IIR)
#define REG_PRESS_MSB 0xF7 // Registro inicial de datos de presión

// VARIABLES GLOBALES (COEFICIENTES DE CALIBRACIÓN BMP280)
uint16_t coef_T1; // Coeficiente de temperatura 1 (sin signo)
int16_t coef_T2; // Coeficiente de temperatura 2 (con signo)
int16_t coef_T3; // Coeficiente de temperatura 3 (con signo)

uint16_t coef_P1; // Coeficiente de presión 1 (sin signo)
int16_t coef_P2; // Coeficiente de presión 2 (con signo)
int16_t coef_P3; // Coeficiente de presión 3 (con signo)
int16_t coef_P4; // Coeficiente de presión 4 (con signo)
int16_t coef_P5; // Coeficiente de presión 5 (con signo)
int16_t coef_P6; // Coeficiente de presión 6 (con signo)
int16_t coef_P7; // Coeficiente de presión 7 (con signo)
int16_t coef_P8; // Coeficiente de presión 8 (con signo)
int16_t coef_P9; // Coeficiente de presión 9 (con signo)

int32_t ajuste_t_fine; // Variable ajustada de temperatura para compensación de presión

// FUNCIONES BÁSICAS I2C
// Escribe un byte de datos en un registro específico del sensor
static void i2c_escribir_byte(uint8_t registro, uint8_t dato)
{
    i2c_cmd_handle_t comando = i2c_cmd_link_create(); // Crear manejador de comando I2C

    i2c_master_start(comando); // Generar condición START en bus I2C
    i2c_master_write_byte(comando, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true); // Enviar dirección + bit escritura
    i2c_master_write_byte(comando, registro, true); // Enviar dirección de registro
    i2c_master_write_byte(comando, dato, true); // Enviar dato a escribir
    i2c_master_stop(comando); // Generar condición STOP en bus I2C

    i2c_master_cmd_begin(I2C_MASTER_PORT, comando, pdMS_TO_TICKS(100)); // Ejecutar comando con timeout
    i2c_cmd_link_delete(comando); // Liberar memoria del comando
}

// Lee múltiples bytes desde un registro específico del sensor
static void i2c_leer_bytes(uint8_t registro, uint8_t *buffer_datos, uint8_t longitud)
{
    i2c_cmd_handle_t comando = i2c_cmd_link_create(); // Crear manejador de comando I2C

    i2c_master_start(comando); // Generar condición START en bus I2C
    i2c_master_write_byte(comando, (BMP280_ADDR << 1) | I2C_MASTER_WRITE, true); // Enviar dirección + bit escritura
    i2c_master_write_byte(comando, registro, true); // Enviar dirección de registro a leer

    i2c_master_start(comando); // Generar condición START repetida
    i2c_master_write_byte(comando, (BMP280_ADDR << 1) | I2C_MASTER_READ, true); // Enviar dirección + bit lectura

    if (longitud > 1) // Si se leen más de un byte
    {
        i2c_master_read(comando, buffer_datos, longitud - 1, I2C_MASTER_ACK); // Leer bytes con ACK
    }

    i2c_master_read_byte(comando, buffer_datos + longitud - 1, I2C_MASTER_NACK); // Último byte con NACK
    i2c_master_stop(comando); // Generar condición STOP en bus I2C

    i2c_master_cmd_begin(I2C_MASTER_PORT, comando, pdMS_TO_TICKS(100)); // Ejecutar comando con timeout
    i2c_cmd_link_delete(comando); // Liberar memoria del comando
}

// CALIBRACIÓN
// Lee y almacena los coeficientes de calibración del sensor BMP280
static void bmp280_leer_coeficientes_calibracion()
{
    uint8_t buffer_calib[24]; // Buffer para 24 bytes de datos de calibración
    i2c_leer_bytes(REG_CALIB, buffer_calib, 24); // Leer coeficientes desde registro 0x88

    coef_T1 = (buffer_calib[1] << 8) | buffer_calib[0];
    coef_T2 = (buffer_calib[3] << 8) | buffer_calib[2];
    coef_T3 = (buffer_calib[5] << 8) | buffer_calib[4];

    coef_P1 = (buffer_calib[7] << 8) | buffer_calib[6];
    coef_P2 = (buffer_calib[9] << 8) | buffer_calib[8];
    coef_P3 = (buffer_calib[11] << 8) | buffer_calib[10];
    coef_P4 = (buffer_calib[13] << 8) | buffer_calib[12];
    coef_P5 = (buffer_calib[15] << 8) | buffer_calib[14];
    coef_P6 = (buffer_calib[17] << 8) | buffer_calib[16];
    coef_P7 = (buffer_calib[19] << 8) | buffer_calib[18];
    coef_P8 = (buffer_calib[21] << 8) | buffer_calib[20];
    coef_P9 = (buffer_calib[23] << 8) | buffer_calib[22];
}

// COMPENSACIONES
static void bmp280_compensar_temperatura(int32_t valor_adc_temperatura)
{
    int32_t termino_1 = ((((valor_adc_temperatura >> 3) - ((int32_t)coef_T1 << 1))) * coef_T2) >> 11;
    int32_t termino_2 = (((((valor_adc_temperatura >> 4) - coef_T1) * ((valor_adc_temperatura >> 4) - coef_T1)) >> 12) * coef_T3) >> 14;

    ajuste_t_fine = termino_1 + termino_2;
}

static uint32_t bmp280_compensar_presion(int32_t valor_adc_presion)
{
    int64_t variable_v1 = (int64_t)ajuste_t_fine - 128000;
    int64_t variable_v2 = variable_v1 * variable_v1 * coef_P6 + ((variable_v1 * coef_P5) << 17) + ((int64_t)coef_P4 << 35);

    variable_v1 = ((variable_v1 * variable_v1 * coef_P3) >> 8) + ((variable_v1 * coef_P2) << 12);
    variable_v1 = (((int64_t)1 << 47) + variable_v1) * coef_P1 >> 33;

    if (variable_v1 == 0) return 0;

    uint64_t presion_temp = 1048576 - valor_adc_presion;
    presion_temp = ((presion_temp << 31) - variable_v2) * 3125 / variable_v1;
    presion_temp += ((int64_t)coef_P9 * (presion_temp >> 13) * (presion_temp >> 13)) >> 25;
    presion_temp += ((int64_t)coef_P8 * presion_temp) >> 19;

    return (uint32_t)((presion_temp >> 8) + ((int64_t)coef_P7 << 4));
}

// LECTURA Y CÁLCULO
static float bmp280_calcular_altitud()
{
    uint8_t buffer_datos[6];
    i2c_leer_bytes(REG_PRESS_MSB, buffer_datos, 6);

    int32_t valor_adc_presion = (buffer_datos[0] << 12) | (buffer_datos[1] << 4) | (buffer_datos[2] >> 4);
    int32_t valor_adc_temperatura = (buffer_datos[3] << 12) | (buffer_datos[4] << 4) | (buffer_datos[5] >> 4);

    bmp280_compensar_temperatura(valor_adc_temperatura);

    float presion_hpa = bmp280_compensar_presion(valor_adc_presion) / 25600.0;

    return 44330.0 * (1.0 - pow(presion_hpa / 1013.25, 0.1903));
}

// PUNTO DE PARTIDA
void app_main()
{
    // CONFIGURACIÓN I2C
    i2c_config_t config_i2c =
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };

    i2c_param_config(I2C_MASTER_PORT, &config_i2c);
    i2c_driver_install(I2C_MASTER_PORT, config_i2c.mode, 0, 0, 0);

    // INICIALIZACIÓN BMP280
    i2c_escribir_byte(REG_CONFIG, 0xA0); // Standby 1000 ms, filtro IIR x4
    i2c_escribir_byte(REG_CTRL_MEAS, 0x27); // Modo normal, oversampling x1
    bmp280_leer_coeficientes_calibracion();

    // BUCLE PRINCIPAL
    while (1)
    {
        printf("Altitude: %.1f m\n", bmp280_calcular_altitud());
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}