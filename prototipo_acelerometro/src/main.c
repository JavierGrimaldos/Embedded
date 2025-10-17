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
#define REG_OUT_X_MSB 0x01
#define REG_OUT_X_LSB 0x02
#define REG_OUT_Y_MSB 0x03
#define REG_OUT_Y_LSB 0x04
#define REG_OUT_Z_MSB 0x05
#define REG_OUT_Z_LSB 0x06

#define CTRL_REG1_ACTIVE_BIT (1 << 0)

static int mma_write_reg(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg, val};
    return i2c_write_dt(dev, buf, sizeof(buf));
}

static int mma_read_regs(const struct i2c_dt_spec *dev, uint8_t reg, uint8_t *buf, size_t len){
    return i2c_write_read_dt(dev, &reg, 1, buf, len);
}

static int mma8451_init(const struct i2c_dt_spec *dev){
    int ret;
    uint8_t ctrl1;

    if(!i2c_is_ready_dt(dev)){
        printk("I2C device not ready\n");
        return -ENODEV;
    }

    printk("MMA8451 found at address 0x%02X\n", dev->addr);

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


int main(void){
    int16_t x;
    int16_t y;
    int16_t z;
    int ret;

    printk("MMA8451 Accelerometer Test Starting\n");

    ret = mma8451_init(&mma);
    if(ret < 0){
        printk("MMA8451 initialization failed: %d\n", ret);
        return 0;
    }

    printk("MMA8451 initialized successfully\n");

    while(1){
        ret = mma8451_read_xyz(&mma, &x, &y, &z);
        if (ret < 0){
            printk("Error reading accelerometer: %d\n", ret);
        } else {
            printk("X acceleration: %d\tY acceleration: %d\tZ acceleration: %d\n", x, y,z);
        }
        
        k_sleep(K_MSEC(500));
    }

    return 0;
}