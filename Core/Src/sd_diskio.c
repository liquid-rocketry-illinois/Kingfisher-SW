/*
 * sd_diskio.c
 *
 * SPI-mode SD/SDHC/SDXC disk I/O driver for FatFs R0.12c on STM32H7.
 *
 * Hardware (Kingfisher / STM32H753VIT6):
 *   SPI6  — PA5=SCK  PA6=MISO  PA7=MOSI
 *   CS    — PA4  (SD_CS1_Pin  / SD_CS1_GPIO_Port)
 *   DET   — PA3  (SD_DET_Pin  / SD_DET_GPIO_Port, active-LOW = inserted)
 *
 * Design notes:
 *   - All transfers are CPU-polled (no DMA) to avoid D-Cache coherency issues
 *     endemic to the STM32H7.
 *   - Baud rate is set to SPI_BAUDRATEPRESCALER_256 (~468 kHz at 120 MHz APB4)
 *     for card initialisation, then raised to SPI_BAUDRATEPRESCALER_8 (~15 MHz)
 *     for data transfers.  Both changes are made by directly modifying CFG1.MBR
 *     while SPE=0; the HAL handle is kept in sync.
 *   - FreeRTOS osDelay() is used inside wait loops so the RTOS scheduler is not
 *     starved during card power-up / erase operations.
 */

#include "sd_diskio.h"
#include "main.h"       /* SD_CS1_Pin, SD_DET_Pin, port defines  */
#include "spi.h"        /* extern SPI_HandleTypeDef hspi6        */
#include "cmsis_os.h"   /* osDelay, osKernelGetTickCount         */
#include <string.h>
#include <stdbool.h>

/* ── Timeouts (ms) ──────────────────────────────────────────────────────── */
#define SD_INIT_TIMEOUT_MS    1000u
#define SD_BUSY_TIMEOUT_MS     500u
#define SD_TOKEN_TIMEOUT_MS    200u

/* ── SD SPI tokens ──────────────────────────────────────────────────────── */
#define TOKEN_START_BLOCK    0xFEu   /* CMD17 / CMD18 / CMD24 data start   */
#define TOKEN_MBW_START      0xFCu   /* CMD25 multi-block write start      */
#define TOKEN_STOP_TRAN      0xFDu   /* CMD25 stop transmission            */
#define DATA_RESP_MASK       0x1Fu
#define DATA_RESP_ACCEPTED   0x05u

/* ── SD commands ────────────────────────────────────────────────────────── */
#define CMD0    0u   /* GO_IDLE_STATE       */
#define CMD8    8u   /* SEND_IF_COND        */
#define CMD9    9u   /* SEND_CSD            */
#define CMD12  12u   /* STOP_TRANSMISSION   */
#define CMD13  13u   /* SEND_STATUS         */
#define CMD16  16u   /* SET_BLOCKLEN        */
#define CMD17  17u   /* READ_SINGLE_BLOCK   */
#define CMD18  18u   /* READ_MULTIPLE_BLOCK */
#define CMD24  24u   /* WRITE_BLOCK         */
#define CMD25  25u   /* WRITE_MULTIPLE_BLOCK*/
#define CMD55  55u   /* APP_CMD             */
#define CMD58  58u   /* READ_OCR            */
#define ACMD41 41u   /* SD_SEND_OP_COND     */

/* ── Card-type flags (ORed together) ────────────────────────────────────── */
#define CT_SD1   0x01u          /* SDv1  — byte-addressed SDSC             */
#define CT_SD2   0x02u          /* SDv2  — may be SDSC or SDHC             */
#define CT_SDHC  0x04u          /* SDHC/SDXC — block-addressed (or'd w/SD2)*/

/* ── Module state ───────────────────────────────────────────────────────── */
static DSTATUS s_stat     = STA_NOINIT;
static uint8_t s_cardtype = 0u;

/*
 * Static 512-byte buffer of 0xFF bytes used as the dummy TX source whenever
 * we only care about the received data (e.g. reading a sector).  Allocated
 * statically to keep it off the FreeRTOS task stack.
 */
static uint8_t s_ff_buf[512];

/* ── SPI helpers ────────────────────────────────────────────────────────── */

static inline void CS_Assert(void)
{
    HAL_GPIO_WritePin(SD_CS1_GPIO_Port, SD_CS1_Pin, GPIO_PIN_RESET);
}

static inline void CS_Deassert(void)
{
    HAL_GPIO_WritePin(SD_CS1_GPIO_Port, SD_CS1_Pin, GPIO_PIN_SET);
}

/* Send one byte, return received byte. */
static uint8_t SPI_Xfer(uint8_t tx)
{
    uint8_t rx = 0xFFu;
    HAL_SPI_TransmitReceive(&hspi6, &tx, &rx, 1u, HAL_MAX_DELAY);
    return rx;
}

/*
 * Reconfigure SPI6 baud-rate prescaler at runtime without re-initialising GPIO.
 *
 * CFG1.MBR may only be written while SPE=0.  The HAL sets SPE at the start of
 * every transfer and clears it (via the H7's automatic EOT mechanism) at the
 * end, so SPE is already 0 between our calls.  We clear it explicitly just to
 * be safe.  The HAL handle's Init field is updated so any future HAL_SPI_Init
 * call starts from the correct prescaler.
 */
static void SPI_SetPrescaler(uint32_t prescaler)
{
    CLEAR_BIT(hspi6.Instance->CR1, SPI_CR1_SPE);
    MODIFY_REG(hspi6.Instance->CFG1, SPI_CFG1_MBR_Msk, prescaler);
    hspi6.Init.BaudRatePrescaler = prescaler;
}

/* ── SD-layer helpers ───────────────────────────────────────────────────── */

/*
 * Poll until MISO returns 0xFF (card not busy).  CS must already be asserted.
 * Returns true when ready, false on timeout.
 */
static bool SD_WaitReady(uint32_t timeout_ms)
{
    uint32_t t0 = osKernelGetTickCount();
    while ((osKernelGetTickCount() - t0) < timeout_ms) {
        if (SPI_Xfer(0xFFu) == 0xFFu) return true;
        osDelay(1u);
    }
    return false;
}

/*
 * Send a 6-byte SD command frame with CS already asserted.
 * Waits for card-ready first (except CMD12 which gets a stuff byte instead).
 * Returns the R1 response byte, or 0xFF on error/timeout.
 */
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg)
{
    if (cmd != CMD12) {
        if (!SD_WaitReady(SD_BUSY_TIMEOUT_MS)) return 0xFFu;
    }

    uint8_t frame[6] = {
        (uint8_t)(0x40u | cmd),
        (uint8_t)(arg >> 24u),
        (uint8_t)(arg >> 16u),
        (uint8_t)(arg >>  8u),
        (uint8_t)(arg),
        0x01u   /* dummy CRC — only CMD0 and CMD8 need a real one */
    };
    if (cmd == CMD0) frame[5] = 0x95u;
    if (cmd == CMD8) frame[5] = 0x87u;

    HAL_SPI_Transmit(&hspi6, frame, 6u, HAL_MAX_DELAY);

    /* CMD12: skip one stuff byte before the response */
    if (cmd == CMD12) SPI_Xfer(0xFFu);

    /* Poll for a valid R1 byte (MSB = 0), up to 8 tries */
    uint8_t r1 = 0xFFu;
    for (uint8_t i = 0; i < 8u; i++) {
        r1 = SPI_Xfer(0xFFu);
        if (!(r1 & 0x80u)) break;
    }
    return r1;
}

/*
 * Wait for the 0xFE data-start token then receive 512 data bytes + 2 CRC bytes.
 * CS must be asserted.
 */
static DRESULT SD_RxBlock(uint8_t *buff)
{
    uint32_t t0 = osKernelGetTickCount();
    uint8_t  tok;
    do {
        tok = SPI_Xfer(0xFFu);
        if (tok == TOKEN_START_BLOCK) break;
        if (tok != 0xFFu) return RES_ERROR;   /* error token from card */
        osDelay(1u);
    } while ((osKernelGetTickCount() - t0) < SD_TOKEN_TIMEOUT_MS);

    if (tok != TOKEN_START_BLOCK) return RES_NOTRDY;

    /* Receive 512 bytes; transmit dummy 0xFF on MOSI */
    HAL_SPI_TransmitReceive(&hspi6, s_ff_buf, buff, 512u, HAL_MAX_DELAY);

    /* Discard two CRC bytes */
    SPI_Xfer(0xFFu);
    SPI_Xfer(0xFFu);
    return RES_OK;
}

/*
 * Transmit a 512-byte data block for a write command.
 * token = TOKEN_START_BLOCK (CMD24) or TOKEN_MBW_START (CMD25).
 * CS must be asserted and the card must be ready before calling.
 */
static DRESULT SD_TxBlock(const uint8_t *buff, uint8_t token)
{
    if (!SD_WaitReady(SD_BUSY_TIMEOUT_MS)) return RES_NOTRDY;

    SPI_Xfer(token);   /* data-start token */

    /* Transmit 512 bytes of payload */
    HAL_SPI_Transmit(&hspi6, (uint8_t *)buff, 512u, HAL_MAX_DELAY);

    /* Dummy CRC */
    SPI_Xfer(0xFFu);
    SPI_Xfer(0xFFu);

    /* Check data-response token */
    uint8_t resp = SPI_Xfer(0xFFu) & DATA_RESP_MASK;
    if (resp != DATA_RESP_ACCEPTED) return RES_ERROR;

    /* Wait for write to complete */
    if (!SD_WaitReady(SD_BUSY_TIMEOUT_MS)) return RES_NOTRDY;
    return RES_OK;
}

/* ── FatFs disk interface ───────────────────────────────────────────────── */

DSTATUS SD_disk_initialize(BYTE pdrv)
{
    if (pdrv != 0u) return STA_NOINIT;

    /* Fill the dummy-TX buffer once */
    memset(s_ff_buf, 0xFFu, sizeof(s_ff_buf));

    /*
     * Card-detect is active-LOW on most microSD sockets (card shorts the pin
     * to GND when inserted).  If your socket uses active-HIGH, flip the test
     * to GPIO_PIN_RESET.
     */
    if (HAL_GPIO_ReadPin(SD_DET_GPIO_Port, SD_DET_Pin) == GPIO_PIN_SET) {
        s_stat = STA_NODISK | STA_NOINIT;
        return s_stat;
    }

    /* ── Slow clock for initialisation ──────────────────────────────────── */
    CS_Deassert();
    SPI_SetPrescaler(SPI_BAUDRATEPRESCALER_256);   /* ~468 kHz @ 120 MHz APB4 */

    /* Power-up: ≥74 clock pulses with CS de-asserted */
    uint8_t clk_buf[10];
    memset(clk_buf, 0xFFu, sizeof(clk_buf));
    HAL_SPI_Transmit(&hspi6, clk_buf, sizeof(clk_buf), HAL_MAX_DELAY);

    /* ── CMD0: enter SPI mode ────────────────────────────────────────────── */
    CS_Assert();
    uint8_t r1 = SD_SendCmd(CMD0, 0u);
    CS_Deassert();
    if (r1 != 0x01u) { s_stat = STA_NOINIT; return s_stat; }

    /* ── CMD8: probe for SDv2 ────────────────────────────────────────────── */
    uint8_t cardtype;
    CS_Assert();
    r1 = SD_SendCmd(CMD8, 0x1AAu);
    if (r1 == 0x01u) {
        /* SDv2 path: read the 4-byte R7 response */
        uint8_t r7[4];
        for (int i = 0; i < 4; i++) r7[i] = SPI_Xfer(0xFFu);
        CS_Deassert();

        /* Echo pattern 0xAA on bits[7:0], voltage range 0x1 on bits[11:8] */
        if (r7[2] == 0x01u && r7[3] == 0xAAu) {
            cardtype = CT_SD2;
        } else {
            s_stat = STA_NOINIT;    /* voltage not supported */
            return s_stat;
        }
    } else {
        CS_Deassert();
        cardtype = CT_SD1;          /* SDv1 */
    }

    /* ── ACMD41: wait for card to leave idle state ───────────────────────── */
    /* HCS bit (bit 30) tells the card we support SDHC addressing */
    uint32_t hcs = (cardtype == CT_SD2) ? (1u << 30u) : 0u;
    uint32_t t0  = osKernelGetTickCount();
    do {
        CS_Assert();
        SD_SendCmd(CMD55, 0u);      /* APP_CMD prefix */
        CS_Deassert();
        CS_Assert();
        r1 = SD_SendCmd(ACMD41, hcs);
        CS_Deassert();
        if (r1 == 0x00u) break;
        osDelay(1u);
    } while ((osKernelGetTickCount() - t0) < SD_INIT_TIMEOUT_MS);

    if (r1 != 0x00u) { s_stat = STA_NOINIT; return s_stat; }

    /* ── CMD58: read OCR, determine SDHC vs SDSC ─────────────────────────── */
    if (cardtype == CT_SD2) {
        CS_Assert();
        r1 = SD_SendCmd(CMD58, 0u);
        uint8_t ocr[4];
        for (int i = 0; i < 4; i++) ocr[i] = SPI_Xfer(0xFFu);
        CS_Deassert();

        if (r1 == 0x00u && (ocr[0] & 0x40u)) {
            cardtype |= CT_SDHC;    /* CCS=1: block-addressed SDHC/SDXC */
        }
    }

    /* ── CMD16: lock block length to 512 for byte-addressed SDSC ────────── */
    if (!(cardtype & CT_SDHC)) {
        CS_Assert();
        r1 = SD_SendCmd(CMD16, 512u);
        CS_Deassert();
        if (r1 != 0x00u) { s_stat = STA_NOINIT; return s_stat; }
    }

    s_cardtype = cardtype;
    s_stat     = 0;   /* card ready */

    /* ── Fast clock for data transfers ──────────────────────────────────── */
    SPI_SetPrescaler(SPI_BAUDRATEPRESCALER_8);     /* ~15 MHz @ 120 MHz APB4 */

    return s_stat;
}

DSTATUS SD_disk_status(BYTE pdrv)
{
    return (pdrv == 0u) ? s_stat : STA_NOINIT;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0u || count == 0u) return RES_PARERR;
    if (s_stat & STA_NOINIT)       return RES_NOTRDY;

    /* SDSC uses byte addressing; SDHC/SDXC uses block addressing */
    if (!(s_cardtype & CT_SDHC)) sector *= 512u;

    uint8_t cmd = (count == 1u) ? CMD17 : CMD18;

    CS_Assert();
    if (SD_SendCmd(cmd, sector) != 0x00u) {
        CS_Deassert();
        return RES_ERROR;
    }

    DRESULT res = RES_OK;
    while (count--) {
        res = SD_RxBlock(buff);
        if (res != RES_OK) break;
        buff += 512u;
    }

    if (cmd == CMD18) SD_SendCmd(CMD12, 0u);   /* stop multi-block read */

    CS_Deassert();
    SPI_Xfer(0xFFu);   /* release bus */
    return res;
}

DRESULT SD_disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    if (pdrv != 0u || count == 0u) return RES_PARERR;
    if (s_stat & STA_NOINIT)       return RES_NOTRDY;

    if (!(s_cardtype & CT_SDHC)) sector *= 512u;

    DRESULT res = RES_OK;
    CS_Assert();

    if (count == 1u) {
        if (SD_SendCmd(CMD24, sector) == 0x00u) {
            res = SD_TxBlock(buff, TOKEN_START_BLOCK);
        } else {
            res = RES_ERROR;
        }
    } else {
        if (SD_SendCmd(CMD25, sector) == 0x00u) {
            while (count--) {
                res = SD_TxBlock(buff, TOKEN_MBW_START);
                if (res != RES_OK) break;
                buff += 512u;
            }
            /* Send stop-transmission token and wait for card to finish */
            SPI_Xfer(TOKEN_STOP_TRAN);
            /* SD_WaitReady(SD_BUSY_TIMEOUT_MS); */
            if (!SD_WaitReady(SD_BUSY_TIMEOUT_MS)) res = RES_NOTRDY;
        } else {
            res = RES_ERROR;
        }
    }

    CS_Deassert();
    SPI_Xfer(0xFFu);   /* release bus */
    return res;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    if (pdrv != 0u)          return RES_PARERR;
    if (s_stat & STA_NOINIT) return RES_NOTRDY;

    DRESULT res = RES_ERROR;

    switch (cmd) {

    case CTRL_SYNC:
        /* Flush — just confirm the card is not busy */
        CS_Assert();
        res = SD_WaitReady(SD_BUSY_TIMEOUT_MS) ? RES_OK : RES_ERROR;
        CS_Deassert();
        break;

    case GET_SECTOR_COUNT: {
        /*
         * Read CSD (CMD9) and parse the capacity field.
         * CSD v2 (SDHC/SDXC):  sectors = (C_SIZE + 1) × 1024
         * CSD v1 (SDSC):       sectors = (C_SIZE+1) × 2^(C_SIZE_MULT+2)
         *                                × 2^READ_BL_LEN / 512
         */
        uint8_t csd[16];
        CS_Assert();
        if (SD_SendCmd(CMD9, 0u) == 0x00u) {
            uint32_t t1 = osKernelGetTickCount();
            while ((osKernelGetTickCount() - t1) < SD_TOKEN_TIMEOUT_MS) {
                uint8_t tok = SPI_Xfer(0xFFu);
                if (tok == TOKEN_START_BLOCK) {
                    HAL_SPI_TransmitReceive(&hspi6, s_ff_buf, csd, 16u, HAL_MAX_DELAY);
                    SPI_Xfer(0xFFu); SPI_Xfer(0xFFu);   /* CRC */
                    res = RES_OK;
                    break;
                }
                if (tok != 0xFFu) break;   /* error token */
            }
        }
        CS_Deassert();

        if (res == RES_OK) {
            DWORD sectors;
            if ((csd[0] >> 6u) == 1u) {
                /* CSD v2 */
                uint32_t c_size = ((uint32_t)(csd[7] & 0x3Fu) << 16u) |
                                  ((uint32_t) csd[8]           <<  8u) |
                                   (uint32_t) csd[9];
                sectors = (c_size + 1u) * 1024u;
            } else {
                /* CSD v1 */
                uint32_t c_size      = ((uint32_t)(csd[6]  & 0x03u) << 10u) |
                                       ((uint32_t) csd[7]            <<  2u) |
                                       ((uint32_t)(csd[8]  & 0xC0u) >>  6u);
                uint32_t c_size_mult = ((uint32_t)(csd[9]  & 0x03u) <<  1u) |
                                       ((uint32_t)(csd[10] & 0x80u) >>  7u);
                uint32_t blk_len     =  (uint32_t)(csd[5]  & 0x0Fu);
                sectors = (c_size + 1u) << (c_size_mult + 2u + blk_len - 9u);
            }
            *(DWORD *)buff = sectors;
        }
        break;
    }

    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512u;
        res = RES_OK;
        break;

    case GET_BLOCK_SIZE:
        /* Report 1 sector — safe and conservative for any card */
        *(DWORD *)buff = 1u;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
        break;
    }

    return res;
}