/*
* AVB switch driver module for SJA1105
* Copyright (C) 2016 - 2018 NXP Semiconductors
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/
/**
*
* \file  sja1105p_spi_linux.c
*
* \author NXP Semiconductors
*
* \date 2017-07-31
*
* \brief Linux platform dependent SPI read and write functions
*
*****************************************************************************/
#include <linux/kernel.h>

#include "sja1105p_init.h"
#include "NXP_SJA1105P_spi.h"
#include "sja1105p_spi_linux.h"
#include "NXP_SJA1105P_config.h"

/*
 * Local constants and macros
 */
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

/* is initialized during SJA1105P probing */
static struct spi_device *g_spi_h[SJA1105P_N_SWITCHES];

/* prototypes */
uint8_t sja1105p_spi_read32(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue);
uint8_t sja1105p_spi_write32(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue);

/* helper function, the word order needs to be switched for some platforms */
u32 preprocess_words(u32 cmd)
{
#if SPI_SWITCH_WORDS == 1
	u32 upper, lower;
	upper = (cmd & 0x0000FFFF) << 16;
	lower = (cmd & 0xFFFF0000) >> 16;
	cmd = upper | lower;
#endif

	return cmd;
}

/************************** HAL Callback registration *************************/
void unregister_spi_callback(int active_switches)
{
	/* reference counting of registered spi devices */
	if (active_switches == 0) {
		SJA1105P_registerSpiRead32CB(NULL);
		SJA1105P_registerSpiWrite32CB(NULL);
	}
}

void register_spi_callback(struct spi_device *spi, int active_switches)
{
	/**
	 * spi context stored to be used by contextless HAL callbacks.
	 * Value of active_switches is equal to device_select of the spi device.
	 */
	g_spi_h[active_switches] = spi;

	/* reference counting of registered spi devices */
	if (active_switches == 0) {
		SJA1105P_registerSpiRead32CB(sja1105p_spi_read32);
		SJA1105P_registerSpiWrite32CB(sja1105p_spi_write32);
	}
}

/*************************** Platform dependent read **************************/

/**
 * Callback, is called when an SPI transfer is completed
 */
static void read_complete(void *context)
{
	struct sja1105p_context_data *sw_ctx = context;

	complete(&sw_ctx->spi_tf_done);
}

/**
 * sja1105p_read_reg32  - read 32bit register from slave
 * @dev: The chip state (device)
 * @reg_addr: The register address to start from
 *
 * @return: value read
 */
u32 sja1105p_read_reg32(struct spi_device *spi, u32 reg_addr)
{
	u32 cmd[2];
	u32 resp[2];
	struct spi_message m;
	struct spi_transfer t;
	struct sja1105p_context_data *sw_ctx = spi_get_drvdata(spi);
	const unsigned long tmo = jiffies + msecs_to_jiffies(20); /* transfer timeout */
	int rc;
	bool done;

	cmd[0] = cpu_to_le32 (CMD_ENCODE_RWOP(CMD_RD_OP) | CMD_ENCODE_ADDR(reg_addr) | CMD_ENCODE_WRD_CNT(1));
	cmd[1] = 0;

	cmd[0] = preprocess_words(cmd[0]);

	spi_message_init(&m);
	m.context = sw_ctx;
	m.complete = read_complete;

	memset(&t, 0, sizeof(t));
	t.tx_buf = cmd;
	t.rx_buf = resp;
	t.len = 8;
	t.bits_per_word = SPI_BITS_PER_WORD_MSG;

	if (verbosity > 6) dev_info(&spi->dev, "reading 4bytes @%08x tlen %d t.bits_per_word %d\n", reg_addr, t.len, t.bits_per_word);

	spi_message_add_tail(&t, &m);

	rc = spi_async(spi, &m);
	if (rc) {
		dev_info(&spi->dev, "spi_sync rc %d\n", rc);
		goto spi_read_err;
	}

	/**
	 * poll for completion, do not sleep:
	 * Some function calls (e.g. to ndo_get_stats that results from executing the userspace app 'ifconfig')
	 * should execute in atomic context. The synchronous spi_sync() may sleep, so it is not suitable here.
	 *
	 * Use the asynchronous spi_async() and busy-wait (~33us on average) for its completion.
	 * cf. https://lists.yoctoproject.org/pipermail/linux-yocto/2016-November/006023.html
	 *
	 * Alternative: Poll stats in background and retrieve most recent buffer when needed
	 * cf. https://patchwork.ozlabs.org/patch/116458/
	 */
	do {
		done = try_wait_for_completion(&sw_ctx->spi_tf_done);
	} while (!done && time_before(jiffies, tmo));
	if (!done) {
		dev_err(&spi->dev, "Error: SPI Read of register @%08x failed (Timeout)!\n", reg_addr);
		goto spi_read_err;
	}

	resp[1] = preprocess_words(resp[1]);

	return le32_to_cpu(resp[1]);

spi_read_err:
	return 0;
}

/**
 *sja1105p_spi_write32 - this function is used by the HAL to read data from SJA1105
 */
uint8_t sja1105p_spi_read32(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue)
{
	struct spi_device *spi = g_spi_h[deviceSelect];
	int i = 0;

	if (verbosity > 6) dev_info(&spi->dev, "%s: device %d wordCount=%d, registerAddress=%08x\n", __func__, deviceSelect, wordCount, registerAddress);

	while (i < wordCount) {
		*p_registerValue = sja1105p_read_reg32(spi, registerAddress);
		if (verbosity > 6) dev_info(&spi->dev, "%s: wordCount %d i %d registerAddress=%08x registerValue=%08x\n", __func__, wordCount, i, registerAddress, *p_registerValue);
		p_registerValue++;
		registerAddress++;
		i++;
	}


	return 0;
}


/************************** Platform dependent write **************************/

/**
 * sja1105p_spi_write - write N words to chip
 * @spi: The spi device
 * @cmd: The command buffer including address and data
 * @nb_words: Number of words to write
 *
 */
int sja1105p_spi_write(struct spi_device *spi, u32 *cmd, u8 nb_words)
{
	struct spi_message m;
	struct spi_transfer t;
	int rc;

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.tx_buf = cmd;
	t.rx_buf = NULL;
	t.len = (nb_words << 2);
	t.bits_per_word = SPI_BITS_PER_WORD_MSG;

	if (verbosity > 6) dev_info(&spi->dev, "writing: %d cmd words with bits_per_word=%d, len=%d\n", nb_words, t.bits_per_word, t.len);

	spi_message_add_tail(&t, &m);
	rc = spi_sync(spi, &m);

	if (rc) dev_err(&spi->dev, "spi_sync rc %d\n", rc);

	return rc;
}

/**
 * __sja1105p_cfg_block_write - write max SPI_CFG_BLOCKS 32bits words to SJA1105
 * @dev: The chip state (device)
 * @reg_addr: The register address to write to
 * @data: The pointer to buffer of 32bits words
 * @nb_words: number of 32bits words to write
 *
 * @return: value read
 */
int __sja1105p_cfg_block_write(struct spi_device *spi, u32 reg_addr, u32 *data, int nb_words)
{
	u32 cmd[SJA1105P_CONFIG_WORDS_PER_BLOCK+1];
	int i = 0;

	cmd[0] = cpu_to_le32 (CMD_ENCODE_RWOP(CMD_WR_OP) | CMD_ENCODE_ADDR(reg_addr));
	cmd[0] = preprocess_words(cmd[0]);

	while (i < nb_words) {
		cmd[i+1] = *data++;
		cmd[i+1] = preprocess_words(cmd[i+1]);
		if (verbosity > 6) dev_info(&spi->dev, "0x%08x\n", cmd[i+1]);
		i++;
	}

	return sja1105p_spi_write(spi, cmd, nb_words + 1);
}

/**
 * sja1105p_cfg_block_write  - write multiple 32bits words to SJA1105
 * Wrapper function, that splits nb_words many words into chunks of max
 * SPI_CFG_BLOCKS words and writes them separately.
 *
 * @dev: The chip state (device)
 * @reg_addr: The register address to write to
 * @data: The pointer to buffer of 32bits words
 * @nb_words: number of 32bits words to write
 *
 * @return: value read
 */
int sja1105p_cfg_block_write(struct spi_device *spi, u32 reg_addr, u32 *data, int nb_words)
{
	int i, err, remaining_words;
	int dev_addr = reg_addr;
	u32 *cfg_data = data;

	/* total number of words to write */
	remaining_words = nb_words;

	i = 0;
	while (remaining_words > 0) {
		//TODO: should use SJA1105P_CONFIG_WORDS_PER_BLOCK many words per transfer
		int block_size_words = MIN(SPI_CFG_BLOCKS, remaining_words);

		if (verbosity > 6) dev_info(&spi->dev, "block_size_words %d remaining_words %d\n", block_size_words, remaining_words);

		err = __sja1105p_cfg_block_write(spi, dev_addr, cfg_data, block_size_words);
		if (err < 0)
			return err;

		if (verbosity > 6) dev_info(&spi->dev, "Loaded block %d @%08x\n", i, dev_addr);

		dev_addr += block_size_words;
		cfg_data += block_size_words;
		remaining_words -= block_size_words;
		i++;
	}

	return 0;
}

/**
 * sja1105p_spi_write32 - this function is used by the HAL to write data to SJA1105
 */
uint8_t sja1105p_spi_write32(uint8_t deviceSelect, uint8_t wordCount, uint32_t registerAddress, uint32_t *p_registerValue)
{
	struct spi_device *spi = g_spi_h[deviceSelect];

	if (verbosity > 6) dev_info(&spi->dev, "%s: device %d wordCount=%d, registerAddress=%08x registerValue=%08x\n", __func__, deviceSelect, wordCount, registerAddress, *p_registerValue);

	sja1105p_cfg_block_write(spi, registerAddress, p_registerValue, wordCount);

	return 0;
}
