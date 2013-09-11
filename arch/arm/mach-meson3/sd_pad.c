#include <mach/card_io.h>
#include <linux/cardreader/card_block.h>
#include <linux/cardreader/cardreader.h>


static unsigned sd_backup_input_val = 0;
static unsigned sd_backup_output_val = 0;
static unsigned SD_BAKUP_INPUT_REG = (unsigned)&sd_backup_input_val;
static unsigned SD_BAKUP_OUTPUT_REG = (unsigned)&sd_backup_output_val;

unsigned SD_CMD_OUTPUT_EN_REG;
unsigned SD_CMD_OUTPUT_EN_MASK;
unsigned SD_CMD_INPUT_REG;
unsigned SD_CMD_INPUT_MASK;
unsigned SD_CMD_OUTPUT_REG;
unsigned SD_CMD_OUTPUT_MASK;

unsigned SD_CLK_OUTPUT_EN_REG;
unsigned SD_CLK_OUTPUT_EN_MASK;
unsigned SD_CLK_OUTPUT_REG;
unsigned SD_CLK_OUTPUT_MASK;

unsigned SD_DAT_OUTPUT_EN_REG;
unsigned SD_DAT0_OUTPUT_EN_MASK;
unsigned SD_DAT0_3_OUTPUT_EN_MASK;
unsigned SD_DAT_INPUT_REG;
unsigned SD_DAT_OUTPUT_REG;
unsigned SD_DAT0_INPUT_MASK;
unsigned SD_DAT0_OUTPUT_MASK;
unsigned SD_DAT0_3_INPUT_MASK;
unsigned SD_DAT0_3_OUTPUT_MASK;
unsigned SD_DAT_INPUT_OFFSET;
unsigned SD_DAT_OUTPUT_OFFSET;

unsigned SD_INS_OUTPUT_EN_REG;
unsigned SD_INS_OUTPUT_EN_MASK;
unsigned SD_INS_INPUT_REG;
unsigned SD_INS_INPUT_MASK;

unsigned SD_WP_OUTPUT_EN_REG;
unsigned SD_WP_OUTPUT_EN_MASK;
unsigned SD_WP_INPUT_REG;
unsigned SD_WP_INPUT_MASK;

unsigned SD_PWR_OUTPUT_EN_REG;
unsigned SD_PWR_OUTPUT_EN_MASK;
unsigned SD_PWR_OUTPUT_REG;
unsigned SD_PWR_OUTPUT_MASK;
unsigned SD_PWR_EN_LEVEL;

unsigned SD_WORK_MODE;

extern int using_sdxc_controller;

void sd_io_init(struct memory_card *card)
{
	struct aml_card_info *aml_card_info = card->card_plat_info;
	SD_WORK_MODE = aml_card_info->work_mode;

	if ((aml_card_info->io_pad_type == SDXC_CARD_0_5) || 
		(aml_card_info->io_pad_type == SDXC_BOOT_0_11) ||
		(aml_card_info->io_pad_type == SDXC_GPIOX_0_9))
		using_sdxc_controller = 1;
	
	switch (aml_card_info->io_pad_type) {

		case SDIO_A_GPIOX_0_3:
			SD_CMD_OUTPUT_EN_REG = EGPIO_GPIOX_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_9_MASK;
			SD_CMD_INPUT_REG = EGPIO_GPIOX_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_9_MASK;
			SD_CMD_OUTPUT_REG = EGPIO_GPIOX_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_9_MASK;

			SD_CLK_OUTPUT_EN_REG = EGPIO_GPIOX_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_8_MASK;
			SD_CLK_OUTPUT_REG = EGPIO_GPIOX_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_8_MASK;

			SD_DAT_OUTPUT_EN_REG = EGPIO_GPIOX_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_0_3_MASK;
			SD_DAT_INPUT_REG = EGPIO_GPIOX_INPUT;
			SD_DAT_OUTPUT_REG = EGPIO_GPIOX_OUTPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_OUTPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_INPUT_OFFSET = 0;
			SD_DAT_OUTPUT_OFFSET = 0;
			break;

		case SDIO_B_CARD_0_5:
			SD_CMD_OUTPUT_EN_REG = CARD_GPIO_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_28_MASK;
			SD_CMD_INPUT_REG = CARD_GPIO_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_28_MASK;
			SD_CMD_OUTPUT_REG = CARD_GPIO_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_28_MASK;

			SD_CLK_OUTPUT_EN_REG = CARD_GPIO_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_27_MASK;
			SD_CLK_OUTPUT_REG = CARD_GPIO_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_27_MASK;

			SD_DAT_OUTPUT_EN_REG = CARD_GPIO_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_23_26_MASK;
			SD_DAT_INPUT_REG = CARD_GPIO_INPUT;
			SD_DAT_OUTPUT_REG = CARD_GPIO_OUTPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_23_MASK;
			SD_DAT0_OUTPUT_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_23_26_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_23_26_MASK;
			SD_DAT_INPUT_OFFSET = 23;
			SD_DAT_OUTPUT_OFFSET = 23;
			break;

		case SDIO_C_BOOT_0_3:
			SD_CMD_OUTPUT_EN_REG = BOOT_GPIO_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_10_MASK;
			SD_CMD_INPUT_REG = BOOT_GPIO_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_10_MASK;
			SD_CMD_OUTPUT_REG = BOOT_GPIO_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_10_MASK;

			SD_CLK_OUTPUT_EN_REG = BOOT_GPIO_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_11_MASK;
			SD_CLK_OUTPUT_REG = BOOT_GPIO_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_11_MASK;

			SD_DAT_OUTPUT_EN_REG = BOOT_GPIO_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_0_3_MASK;
			SD_DAT_INPUT_REG = BOOT_GPIO_INPUT;
			SD_DAT_OUTPUT_REG = BOOT_GPIO_OUTPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_OUTPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_INPUT_OFFSET = 0;
			SD_DAT_OUTPUT_OFFSET = 0;
			break;

#define SDHC_B_ENABLE	CBUS_REG_ADDR(0x201b)
#define SDHC_B_OUTPUT	CBUS_REG_ADDR(0x201c)
#define SDHC_B_INPUT	CBUS_REG_ADDR(0x201d)
		case SDHC_CARD_0_5:		//SDHC-B
			SD_CMD_OUTPUT_EN_REG = SDHC_B_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_28_MASK;
			SD_CMD_OUTPUT_REG = SDHC_B_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_28_MASK;
			SD_CMD_INPUT_REG = SDHC_B_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_28_MASK;

			SD_CLK_OUTPUT_EN_REG = SDHC_B_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_27_MASK;
			SD_CLK_OUTPUT_REG = SDHC_B_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_27_MASK;


			SD_DAT_OUTPUT_EN_REG = SDHC_B_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_23_26_MASK;
			
			SD_DAT_OUTPUT_REG = SDHC_B_OUTPUT;
			SD_DAT0_OUTPUT_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_23_26_MASK;
			SD_DAT_OUTPUT_OFFSET = 23;
			
			SD_DAT_INPUT_REG = SDHC_B_INPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_23_26_MASK;
			SD_DAT_INPUT_OFFSET = 23;
			break;

#define SDHC_C_ENABLE	CBUS_REG_ADDR(0x2015)
#define SDHC_C_OUTPUT	CBUS_REG_ADDR(0x2016)
#define SDHC_C_INPUT	CBUS_REG_ADDR(0x2017)
		case SDHC_BOOT_0_11:		//SDHC-C
			SD_CMD_OUTPUT_EN_REG = SDHC_C_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_10_MASK;
			SD_CMD_OUTPUT_REG = SDHC_C_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_10_MASK;
			SD_CMD_INPUT_REG = SDHC_C_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_10_MASK;

			SD_CLK_OUTPUT_EN_REG = SDHC_C_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_11_MASK;
			SD_CLK_OUTPUT_REG = SDHC_C_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_11_MASK;


			SD_DAT_OUTPUT_EN_REG = SDHC_C_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_0_3_MASK;
			
			SD_DAT_OUTPUT_REG = SDHC_C_OUTPUT;
			SD_DAT0_OUTPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_OUTPUT_OFFSET = 0;
			
			SD_DAT_INPUT_REG = SDHC_C_INPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_INPUT_OFFSET = 0;
			break;

#define SDHC_A_ENABLE	CBUS_REG_ADDR(0x2018)
#define SDHC_A_OUTPUT	CBUS_REG_ADDR(0x2019)
#define SDHC_A_INPUT	CBUS_REG_ADDR(0x201a)
		case SDHC_GPIOX_0_9:		//SDHC-A
			SD_CMD_OUTPUT_EN_REG = SDHC_A_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_9_MASK;
			SD_CMD_OUTPUT_REG = SDHC_A_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_9_MASK;
			SD_CMD_INPUT_REG = SDHC_A_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_9_MASK;

			SD_CLK_OUTPUT_EN_REG = SDHC_A_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_8_MASK;
			SD_CLK_OUTPUT_REG = SDHC_A_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_8_MASK;


			SD_DAT_OUTPUT_EN_REG = SDHC_A_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_0_3_MASK;
			
			SD_DAT_OUTPUT_REG = SDHC_A_OUTPUT;
			SD_DAT0_OUTPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_OUTPUT_OFFSET = 0;
			
			SD_DAT_INPUT_REG = SDHC_A_INPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_INPUT_OFFSET = 0;
			break;

#define SDXC_B_ENABLE	CBUS_REG_ADDR(0x201b)
#define SDXC_B_OUTPUT	CBUS_REG_ADDR(0x201c)
#define SDXC_B_INPUT	CBUS_REG_ADDR(0x201d)
		case SDXC_CARD_0_5:		//SDXC-B
			SD_CMD_OUTPUT_EN_REG = SDXC_B_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_28_MASK;
			SD_CMD_OUTPUT_REG = SDXC_B_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_28_MASK;
			SD_CMD_INPUT_REG = SDXC_B_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_28_MASK;

			SD_CLK_OUTPUT_EN_REG = SDXC_B_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_27_MASK;
			SD_CLK_OUTPUT_REG = SDXC_B_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_27_MASK;


			SD_DAT_OUTPUT_EN_REG = SDXC_B_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_23_26_MASK;
			
			SD_DAT_OUTPUT_REG = SDXC_B_OUTPUT;
			SD_DAT0_OUTPUT_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_23_26_MASK;
			SD_DAT_OUTPUT_OFFSET = 23;
			
			SD_DAT_INPUT_REG = SDXC_B_INPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_23_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_23_26_MASK;
			SD_DAT_INPUT_OFFSET = 23;
			break;

#define SDXC_C_ENABLE	CBUS_REG_ADDR(0x2015)
#define SDXC_C_OUTPUT	CBUS_REG_ADDR(0x2016)
#define SDXC_C_INPUT	CBUS_REG_ADDR(0x2017)
		case SDXC_BOOT_0_11:		//SDXC-C
			SD_CMD_OUTPUT_EN_REG = SDXC_C_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_10_MASK;
			SD_CMD_OUTPUT_REG = SDXC_C_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_10_MASK;
			SD_CMD_INPUT_REG = SDXC_C_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_10_MASK;

			SD_CLK_OUTPUT_EN_REG = SDXC_C_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_11_MASK;
			SD_CLK_OUTPUT_REG = SDXC_C_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_11_MASK;


			SD_DAT_OUTPUT_EN_REG = SDXC_C_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_0_3_MASK;
			
			SD_DAT_OUTPUT_REG = SDXC_C_OUTPUT;
			SD_DAT0_OUTPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_OUTPUT_OFFSET = 0;
			
			SD_DAT_INPUT_REG = SDXC_C_INPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_INPUT_OFFSET = 0;
			break;

#define SDXC_A_ENABLE	CBUS_REG_ADDR(0x2018)
#define SDXC_A_OUTPUT	CBUS_REG_ADDR(0x2019)
#define SDXC_A_INPUT	CBUS_REG_ADDR(0x201a)
		case SDXC_GPIOX_0_9:		//SDXC-A
			SD_CMD_OUTPUT_EN_REG = SDXC_A_ENABLE;
			SD_CMD_OUTPUT_EN_MASK = PREG_IO_9_MASK;
			SD_CMD_OUTPUT_REG = SDXC_A_OUTPUT;
			SD_CMD_OUTPUT_MASK = PREG_IO_9_MASK;
			SD_CMD_INPUT_REG = SDXC_A_INPUT;
			SD_CMD_INPUT_MASK = PREG_IO_9_MASK;

			SD_CLK_OUTPUT_EN_REG = SDXC_A_ENABLE;
			SD_CLK_OUTPUT_EN_MASK = PREG_IO_8_MASK;
			SD_CLK_OUTPUT_REG = SDXC_A_OUTPUT;
			SD_CLK_OUTPUT_MASK = PREG_IO_8_MASK;


			SD_DAT_OUTPUT_EN_REG = SDXC_A_ENABLE;
			SD_DAT0_OUTPUT_EN_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_EN_MASK = PREG_IO_0_3_MASK;
			
			SD_DAT_OUTPUT_REG = SDXC_A_OUTPUT;
			SD_DAT0_OUTPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_OUTPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_OUTPUT_OFFSET = 0;
			
			SD_DAT_INPUT_REG = SDXC_A_INPUT;
			SD_DAT0_INPUT_MASK = PREG_IO_0_MASK;
			SD_DAT0_3_INPUT_MASK = PREG_IO_0_3_MASK;
			SD_DAT_INPUT_OFFSET = 0;
			break;

        default:
			printk("Warning couldn`t find any valid hw io pad!!!\n");
            break;
	}

	if (aml_card_info->card_ins_en_reg) {
		SD_INS_OUTPUT_EN_REG = aml_card_info->card_ins_en_reg;
		SD_INS_OUTPUT_EN_MASK = aml_card_info->card_ins_en_mask;
		SD_INS_INPUT_REG = aml_card_info->card_ins_input_reg;
		SD_INS_INPUT_MASK = aml_card_info->card_ins_input_mask;
	}
	else {
		SD_INS_OUTPUT_EN_REG = SD_BAKUP_OUTPUT_REG;
		SD_INS_OUTPUT_EN_MASK = 1;
		SD_INS_INPUT_REG = SD_BAKUP_INPUT_REG;
		SD_INS_INPUT_MASK =
		SD_WP_INPUT_MASK = 1;
	}

	if (aml_card_info->card_power_en_reg) {
		SD_PWR_OUTPUT_EN_REG = aml_card_info->card_power_en_reg;
		SD_PWR_OUTPUT_EN_MASK = aml_card_info->card_power_en_mask;
		SD_PWR_OUTPUT_REG = aml_card_info->card_power_output_reg;
		SD_PWR_OUTPUT_MASK = aml_card_info->card_power_output_mask;
		SD_PWR_EN_LEVEL = aml_card_info->card_power_en_lev;
	}
	else {
		SD_PWR_OUTPUT_EN_REG = SD_BAKUP_OUTPUT_REG;
		SD_PWR_OUTPUT_EN_MASK = 1;
		SD_PWR_OUTPUT_REG = SD_BAKUP_OUTPUT_REG;
		SD_PWR_OUTPUT_MASK = 1;
		SD_PWR_EN_LEVEL = 0;	
	}

	if (aml_card_info->card_wp_en_reg) {
		SD_WP_OUTPUT_EN_REG = aml_card_info->card_wp_en_reg;
		SD_WP_OUTPUT_EN_MASK = aml_card_info->card_wp_en_mask;
		SD_WP_INPUT_REG = aml_card_info->card_wp_input_reg;
		SD_WP_INPUT_MASK = aml_card_info->card_wp_input_mask;
	}
	else {
		SD_WP_OUTPUT_EN_REG = SD_BAKUP_OUTPUT_REG;
		SD_WP_OUTPUT_EN_MASK = 1;
		SD_WP_INPUT_REG = SD_BAKUP_INPUT_REG;
		SD_WP_INPUT_MASK = 1;
	}

	return;
}


void sd_sdio_enable(SDIO_Pad_Type_t io_pad_type)
{
	switch (io_pad_type) {

		case SDIO_A_GPIOX_0_3:
			SET_CBUS_REG_MASK(CARD_PIN_MUX_8, (0x3F<<0));
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (0));
			break;

		case SDIO_B_CARD_0_5:
			SET_CBUS_REG_MASK(CARD_PIN_MUX_2, (0x3F<<10));
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		case SDIO_C_BOOT_0_3:
			SET_CBUS_REG_MASK(CARD_PIN_MUX_6, (0x3F<<24));
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (2));
			break;

		case SDHC_CARD_0_5 :	//SDHC-B
			SET_CBUS_REG_MASK(CARD_PIN_MUX_2, (0x3F<<10));
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		case SDHC_BOOT_0_11 :	//SDHC-C
			SET_CBUS_REG_MASK(CARD_PIN_MUX_6, (0x3F<<24));
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (2));
			break;

		case SDHC_GPIOX_0_9 :	//SDHC-A
			SET_CBUS_REG_MASK(CARD_PIN_MUX_8, 0x3F);
			SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (0));
			break;

		case SDXC_CARD_0_5 :	//SDXC-B
			SET_CBUS_REG_MASK(CARD_PIN_MUX_2, (0xF<<4));
			//SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		case SDXC_BOOT_0_11 :	//SDXC-C
			SET_CBUS_REG_MASK(CARD_PIN_MUX_4, (0x1F<<26));
			//SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (2));
			break;

		case SDXC_GPIOX_0_9 :	//SDXC-A
			SET_CBUS_REG_MASK(CARD_PIN_MUX_5, (0x1F<<10));
			//SET_CBUS_REG_MASK(SDIO_MULT_CONFIG, (0));
			break;

		default :
			printk("invalid hw io pad!!!\n");
			break;
	}
	
	return;
}

void sd_gpio_enable(SDIO_Pad_Type_t io_pad_type)
{
	switch (io_pad_type) {

		case SDIO_A_GPIOX_0_3:
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_8, (0x3F<<0));
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (0));
			break;

		case SDIO_B_CARD_0_5:
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_2, (0x3F<<10));
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		case SDIO_C_BOOT_0_3:
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_6, (0x3F<<24));
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (2));
			break;

		case SDHC_CARD_0_5 :	//SDHC-B
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_2, (0x3F<<10));
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		case SDHC_BOOT_0_11 :	//SDHC-C
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_6, (0x3F<<24));
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (2));
			break;

		case SDHC_GPIOX_0_9 :	//SDHC-A
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_8, 0x3F);
			CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (0));
			break;

		case SDXC_CARD_0_5 :	//SDXC-B
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_2, (0xF<<4));
			//CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (1));
			break;

		case SDXC_BOOT_0_11 :	//SDXC-C
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_4, (0x1F<<26));
			//CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (2));
			break;

		case SDXC_GPIOX_0_9 :	//SDXC-A
			CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_5, (0x1F<<10));
			//CLEAR_CBUS_REG_MASK(SDIO_MULT_CONFIG, (0));
			break;

		default :
			printk("invalid hw io pad!!!\n");
			break;
	}
	
	return;
}

/*set SDIO_MULT_CONFIG 0, SDIO CLK(SDIOA) temp be 50M, unwork on samsung NRX600 wifi*/
void sd_gpio_enable_sdioa(void)
{
    CLEAR_CBUS_REG_MASK(CARD_PIN_MUX_8, (0x3F<<0));
}

