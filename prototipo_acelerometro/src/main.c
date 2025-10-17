#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>

// Opción 1: Definir manualmente la estructura i2c_dt_spec
static const struct i2c_dt_spec mma = {
    .bus = DEVICE_DT_GET(DT_NODELABEL(i2c1)),
    .addr = 0x1D,
};


#define REG_CTRL_REG1 0x2A
#define REG_CTRL_REG2 0x2B
#define REG_OUT_X_MSB 0x01
#define REG_OUT_X_LSB 0x02
#define REG_OUT_Y_MSB 0x03
#define REG_OUT_Y_LSB 0x04
#define REG_OUT_Z_MSB 0x05
#define REG_OUT_Z_LSB 0x06
#define REG_WHO_AM_I  0x0D //Esto sirve para confirmar que estamos hablando con el sensor correcto

//Configuramos los rangos de medidas
#define FS_2G 0x00
#define FS_4G 0x01
#define FS_8G 0x02

//Configuramos el rango que escogemos
#define REG_XYZ_DATA_CFG 0x0E

#define CTRL_REG1_ACTIVE_BIT (1 << 0)
#define WHO_AM_I_ID 0x1A  //Y esto sirve para identificar el registro donde escribir

static int mma_read_regs(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t *buf, size_t len){
    return i2c_write_read_dt(dev, &reg, 1, buf, len);
}

// Chequeamos el sensor
static int mma8451_check_id(const struct i2c_dt_spec *dev){
    uint8_t whoami;
    int ret = mma_read_regs(dev, REG_WHO_AM_I, &whoami, 1);
    if (ret < 0) {
        printk("Failed to read WHO_AM_I: %d\n", ret);
        return ret;
    }
    
    printk("WHO_AM_I: 0x%02X (esperado: 0x%02X)\n", whoami, WHO_AM_I_ID);
    
    if (whoami == WHO_AM_I_ID) {
        printk("✓ Sensor MMA8451 identificado correctamente\n");
        return 0;
    } else {
        printk("✗ ERROR: Dispositivo desconocido\n");
        return -EIO;
    }
}

static int mma_write_reg(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg, val};
    return i2c_write_dt(dev, buf, sizeof(buf));
}


static int mma8451_init(const struct i2c_dt_spec *dev){
    int ret;
    uint8_t ctrl1;

    if(!i2c_is_ready_dt(dev)){
        printk("I2C device not ready\n");
        return -ENODEV;
    }

    printk("MMA8451 found at address 0x%02X\n", dev->addr);

    ret=mma8451_check_id(dev);
    if(ret<0){
        printk("Failed to identify the sensor");
        return ret;
    }

    ret = mma_read_regs(dev, REG_CTRL_REG1, &ctrl1, 1);
    if (ret < 0) {
        printk("Failed to read CTRL_REG1: %d\n", ret);
        return ret;
    }

    printk("Initial CTRL_REG1: 0x%02X\n", ctrl1);

    // Desactivar el sensor para configurar
    ctrl1 &= ~CTRL_REG1_ACTIVE_BIT;
    ret = mma_write_reg(dev, REG_CTRL_REG1, ctrl1);
    if(ret < 0) {
        printk("Failed to deactivate sensor: %d\n", ret);
        return ret;
    }

    k_sleep(K_MSEC(10));

    // Reactivar el sensor
    ctrl1 |= CTRL_REG1_ACTIVE_BIT;
    ret = mma_write_reg(dev, REG_CTRL_REG1, ctrl1);
    if(ret < 0) {
        printk("Failed to activate sensor: %d\n", ret);
        return ret;
    }

    k_sleep(K_MSEC(10));

    // Verificar que se activó correctamente
    ret = mma_read_regs(dev, REG_CTRL_REG1, &ctrl1, 1);
    if (ret < 0) {
        printk("Failed to verify activation: %d\n", ret);
        return ret;
    }

    printk("Final CTRL_REG1: 0x%02X\n", ctrl1);

    if (!(ctrl1 & CTRL_REG1_ACTIVE_BIT)) {
        printk("Sensor not active after initialization\n");
        return -EIO;
    }

    return 0;
}

static int mma8451_set_range(const struct i2c_dt_spec *dev, uint8_t range){
    int ret;
    uint8_t ctrl1;

    ret=mma_read_regs(dev, REG_CTRL_REG1, &ctrl1, 1);
    if (ret<0) return ret;

    ctrl1 &= ~CTRL_REG1_ACTIVE_BIT;
    ret=mma_write_reg(dev, REG_CTRL_REG1, ctrl1);
    if(ret<0) return ret;

    k_sleep(K_MSEC(1));

    ctrl1 |=CTRL_REG1_ACTIVE_BIT;
    ret= mma_write_reg(dev, REG_CTRL_REG1, ctrl1);
    if(ret<0) return ret;

    k_sleep(K_MSEC(1));
    return 0;
}

static int mma8451_read_xyz(const struct i2c_dt_spec *dev, int16_t *x, int16_t *y, int16_t *z){
    uint8_t buf[6]; //DOS POR EJE (entrada, salida)
    int ret = mma_read_regs(dev, REG_OUT_X_MSB, buf, sizeof(buf)); //Solo ponemos este registro por que I2C pasa automáticamente a la siguiente posición, es decir de 0x01 pasa a 0x02, 0x03, looo...
    if (ret < 0) {
        printk("Failed to read X axis: %d\n", ret);
        return ret;
    }

    // Estamos usando 16 bits, pero el registro es de 14, hay que modificar el dato recibido: [0]=BMS, [1]=BLS
    *x = (int16_t)(((buf[0] << 8) | buf[1]) >> 2); //Desplazamos 8 posiciones a la izq (<-) los BMS, después lo combinamos con los BLS, y desplazamos dos bits a la derecha (->), para usar el registro de 14 bits 
    *y = (int16_t)(((buf[2] << 8) | buf[3]) >> 2);
    *z = (int16_t)(((buf[4] << 8) | buf[5]) >> 2);
    return 0;
}

static void convert_to_g(int16_t raw, uint8_t range, float *g_value){
    float sensitivity;

    switch(range & 0x03){
        case FS_2G: sensitivity=4096.0f; break;
        case FS_4G: sensitivity=2048.0f; break;
        case FS_8G: sensitivity=1024.0f; break;
        default:    sensitivity=4096.0f; break;
    }

    *g_value=(float)raw/sensitivity;
}


int main(void){
    int16_t x, y, z;
    float x_g, y_g, z_g;
    int ret;

    static uint8_t current_range = FS_2G;  // Cambia a FS_4G o FS_8G manualmente

    printk("MMA8451 Accelerometer Test Starting\n");

    ret = mma8451_init(&mma);
    if(ret < 0){
        printk("MMA8451 initialization failed: %d\n", ret);
        return 0;
    }

    ret = mma8451_set_range(&mma, current_range);
    if(ret < 0){
        printk("Error initial configuration\n");
        return 0;
    }

    printk("MMA8451 initialized successfully - Rango: ");
    switch(current_range){
        case FS_2G: printk("±2G\n"); break;
        case FS_4G: printk("±4G\n"); break;
        case FS_8G: printk("±8G\n"); break;
    }

    while(1){
        ret = mma8451_read_xyz(&mma, &x, &y, &z);
        if (ret < 0){
            printk("Error reading accelerometer: %d\n", ret);
        } else {
            convert_to_g(x, current_range, &x_g);
            convert_to_g(y, current_range, &y_g);
            convert_to_g(z, current_range, &z_g);
            
            printk("X: %6.3fg  Y: %6.3fg  Z: %6.3fg  (Raw: %6d, %6d, %6d)\n", x_g, y_g, z_g, x, y, z);
            printk("Current_range: %d", current_range);
        }
        
        k_sleep(K_MSEC(500));
    }

    return 0;
}