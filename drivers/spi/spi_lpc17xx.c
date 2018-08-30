/*
 * Copyright (c) 2018 Zilogic Systems.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

#include <errno.h>
#include <device.h>
#include <spi.h>
#include <soc.h>
#include <board.h>
#include <string.h>

#include "spi_lpc17xx.h"

#define PCONP 0x400FC0C4

enum reg_offset {
        SPI_CR0_OFFSET   = 0x00,
        SPI_CR1_OFFSET   = 0x04,
        SPI_DR_OFFSET    = 0x08,
        SPI_SR_OFFSET    = 0x0C,
        SPI_CPSR_OFFSET  = 0x10,
        SPI_IMSC_OFFSET  = 0x14,
        SPI_RIS_OFFSET   = 0x18,
        SPI_MIS_OFFSET   = 0x1C,
        SPI_ICR_OFFSET   = 0x20,
        SPI_DMACR_OFFSET = 0x24
};

#define SPI_PCLK (CONFIG_SYS_CRYSTAL_FREQ / 4)
#define SPI_REG_ADDR(addr, offset) (addr + offset)

static void sys_set_bits(u32_t address,
                         u32_t mask, u32_t shift, u32_t data)
{
        u32_t temp;

        temp = sys_read32(address);
        temp &= ~(mask << shift);
        temp |= (data & mask) << shift;
        sys_write32(temp, address);
}

static void rx_buffer_flush(u32_t baddr)
{
        u32_t stat_reg = SPI_REG_ADDR(baddr, SPI_SR_OFFSET);

        /* Read data until Rx FIFO is Empty */
        while (sys_test_bit(stat_reg, 2))
                sys_read32(SPI_REG_ADDR(baddr, SPI_DR_OFFSET));
}

static void wait_for_sync(u32_t baddr)
{
        u32_t stat_reg = SPI_REG_ADDR(baddr, SPI_SR_OFFSET);

        /* wait until busy flag is clear */
        while (sys_test_bit(stat_reg, 4))
                ;
}

static bool spi_transfer_complete(struct device *dev)
{
        struct spi_lpc17xx_data *data = dev->driver_data;

        if (spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx)) {
                return 1;
        }
        return 0;
}

static void pull_data(struct device *dev)
{
        const struct spi_lpc17xx_config *cfg = dev->config->config_info;
        struct spi_lpc17xx_data *data = dev->driver_data;
        u32_t baddr = cfg->baddr;
        u16_t value;

        /* Check Rx FIFO is Not Empty */
        while (sys_test_bit(SPI_REG_ADDR(baddr, SPI_SR_OFFSET), 2)) {

                value = sys_read16(SPI_REG_ADDR(baddr, SPI_DR_OFFSET));

                if (spi_context_rx_buf_on(&data->ctx)) {
                        UNALIGNED_PUT(value, (u8_t *)data->ctx.rx_buf);
                }
                spi_context_update_rx(&data->ctx, 1, 1);
        }
}

static void push_data(struct device *dev)
{
        const struct spi_lpc17xx_config *cfg = dev->config->config_info;
        struct spi_lpc17xx_data *data = dev->driver_data;
        u32_t baddr = cfg->baddr;
        u16_t value;

        /* Check Tx FIFO is Not Full */
        if (sys_test_bit(SPI_REG_ADDR(baddr, SPI_SR_OFFSET), 1)) {
                value = 0;

                /* Hold, if Rx FIFO Full */
                if (sys_test_bit(SPI_REG_ADDR(baddr, SPI_SR_OFFSET), 3)) {
                        return;
                }

                if (spi_context_tx_buf_on(&data->ctx)) {
                        value = UNALIGNED_GET((u8_t *)(data->ctx.tx_buf));
                }

                sys_write16(value, SPI_REG_ADDR(baddr, SPI_DR_OFFSET));
                spi_context_update_tx(&data->ctx, 1, 1);
        }
}

static void set_frequency(uint32_t freq, spi_config_regs *regs)
{

        u32_t i, j;

        for (i = 2; i <= 254; i += 2)
                for (j = 1; j < 256; j++)
                        if (freq >= ((SPI_PCLK) / (i * j))) {
                                /* Set optimal div value */
                                sys_set_bits((u32_t)&regs->cpsr, 0xFF, 0, i);

                                /* Set optimal SCR value */
                                sys_set_bits((u32_t)&regs->cr0, 0xFF, 8, j - 1);
                                return;
                        }
        /* Set div value to default if frequency is less*/
        sys_set_bits((u32_t)&regs->cpsr, 0xFF, 0, 254);

        /* Set SCR value to default if frequency is less */
        sys_set_bits((u32_t)&regs->cr0, 0xFF, 8, 255);
}

static int spi_lpc17xx_configure(struct device *dev,
                                 const struct spi_config *config)
{
        const struct spi_lpc17xx_config *cfg = dev->config->config_info;
        struct spi_lpc17xx_data *data = dev->driver_data;
        u32_t baddr = cfg->baddr;
        spi_config_regs *cfg_regs = &data->regs;
        spi_config_regs regs = {0};

        if (spi_context_configured(&data->ctx, config)) {
                /* Nothing to do */
                return 0;
        }

        if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
                /* Slave mode is not implemented. */
                return -ENOTSUP;
        }

        /* Setting SSP in Master Mode */
        sys_clear_bit((u32_t)&regs.cr1, 2);

        /* Setting Frame Format to SPI */
        sys_set_bits((u32_t)&regs.cr0, 0x3, 4, 0x0);

        if ((config->operation & SPI_MODE_CPOL) != 0) {
                /* Set CPOL if configured */
                sys_set_bit((u32_t)&regs.cr0, 6);
        }

        if ((config->operation & SPI_MODE_CPHA) != 0) {
                /* Set CPHA if configured */
                sys_set_bit((u32_t)&regs.cr0, 7);
        }

        if ((config->operation & SPI_MODE_LOOP) != 0) {
                /* Setting SSP to Loopback Mode */
                sys_set_bit((u32_t)&regs.cr1, 0);
        }

        if (SPI_WORD_SIZE_GET(config->operation) != 8) {
                return -ENOTSUP;
        }

        /* 8 bits frame per transfer */
        sys_set_bits((u32_t)&regs.cr0, 0xF, 0, 0x7);

        /* Setting the configured frequency */
        set_frequency(config->frequency, &regs);

        if (memcmp(&regs, cfg_regs, sizeof(spi_config_regs))) {
                /* Writing value to registers if config changed */
                sys_write32(regs.cr0, SPI_REG_ADDR(baddr, SPI_CR0_OFFSET));
                sys_write32(regs.cr1, SPI_REG_ADDR(baddr, SPI_CR1_OFFSET));
                sys_write32(regs.cpsr, SPI_REG_ADDR(baddr, SPI_CPSR_OFFSET));

                /* Preserving config in structure */
                memcpy(cfg_regs, &regs, sizeof(spi_config_regs));
        }

        /* At this point, it's mandatory to set this on the context! */
        data->ctx.config = config;

        spi_context_cs_configure(&data->ctx);

        return 0;
}

static int spi_lpc17xx_transceive(struct device *dev,
                                  const struct spi_config *config,
                                  const struct spi_buf_set *tx_bufs,
                                  const struct spi_buf_set *rx_bufs)
{
        const struct spi_lpc17xx_config *cfg = dev->config->config_info;
        struct spi_lpc17xx_data *data = dev->driver_data;
        u32_t baddr = cfg->baddr;
        int err;

        spi_context_lock(&data->ctx, false, NULL);

        /* Wait for previous transfer complete */
        wait_for_sync(baddr);

        rx_buffer_flush(baddr);

        err = spi_lpc17xx_configure(dev, config);
        if (err != 0) {
                goto done;
        }

        spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

        spi_context_cs_control(&data->ctx, true);

        /* Set SSE bit, enable SSP */
        sys_set_bit(SPI_REG_ADDR(baddr, SPI_CR1_OFFSET), 1);

        do {
                push_data(dev);
                pull_data(dev);
        } while (spi_transfer_complete(dev));
done:
        spi_context_release(&data->ctx, err);
        return err;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_lpc17xx_transceive_async(struct device *dev,
                                        const struct spi_config *config,
                                        const struct spi_buf_set *tx_bufs,
                                        const struct spi_buf_set *rx_bufs,
                                        struct k_poll_signal *async)
{
        return -ENOTSUP;
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_lpc17xx_release(struct device *dev,
                               const struct spi_config *config)
{
        const struct spi_lpc17xx_config *cfg = dev->config->config_info;
        struct spi_lpc17xx_data *data = dev->driver_data;
        u32_t baddr = cfg->baddr;

        /* Wait for previous transfer complete */
        wait_for_sync(baddr);

        spi_context_unlock_unconditionally(&data->ctx);

        return 0;
}

static int spi_lpc17xx_ssp_init(struct device *dev)
{
        const struct spi_lpc17xx_config *cfg = dev->config->config_info;
        struct spi_lpc17xx_data *data = dev->driver_data;

        /* Enabling Power to SSP */
        sys_set_bit(PCONP, cfg->clock);

        spi_context_unlock_unconditionally(&data->ctx);

        return 0;
}

static const struct spi_driver_api spi_lpc17xx_driver_api = {
        .transceive = spi_lpc17xx_transceive,
#ifdef CONFIG_SPI_ASYNC
        .transceive_async = spi_lpc17xx_transceive_async,
#endif
        .release = spi_lpc17xx_release,
};

#ifdef CONFIG_SPI_0

static const struct spi_lpc17xx_config spi_lpc17xx_ssp0_cfg = {
        .baddr = CONFIG_SSP0_BASE_ADDRESS,
        .clock = 21
};

static struct spi_lpc17xx_data spi_lpc17xx_ssp0_data = {
        SPI_CONTEXT_INIT_LOCK(spi_lpc17xx_ssp0_data, ctx),
        SPI_CONTEXT_INIT_SYNC(spi_lpc17xx_ssp0_data, ctx),
};

DEVICE_AND_API_INIT(spi_lpc17xx_ssp0, CONFIG_SSP0_NAME, &spi_lpc17xx_ssp_init,
                    &spi_lpc17xx_ssp0_data, &spi_lpc17xx_ssp0_cfg,
                    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
                    &spi_lpc17xx_driver_api);


#endif /* CONFIG_SPI_0 */

#ifdef CONFIG_SPI_1

static const struct spi_lpc17xx_config spi_lpc17xx_ssp1_cfg = {
        .baddr = CONFIG_SSP1_BASE_ADDRESS,
        .clock = 10
};

static struct spi_lpc17xx_data spi_lpc17xx_ssp1_data = {
        SPI_CONTEXT_INIT_LOCK(spi_lpc17xx_ssp1_data, ctx),
        SPI_CONTEXT_INIT_SYNC(spi_lpc17xx_ssp1_data, ctx),
};

DEVICE_AND_API_INIT(spi_lpc17xx_ssp1, CONFIG_SSP1_NAME, &spi_lpc17xx_ssp_init,
                    &spi_lpc17xx_ssp1_data, &spi_lpc17xx_ssp1_cfg,
                    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
                    &spi_lpc17xx_driver_api);

#endif /* CONFIG_SPI_1 */
