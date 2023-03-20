/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Littlefs File System on SD Card
*              and QSPI NOR Flash code example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "cyhal.h"
#include "cycfg_pins.h"
#include "lfs.h"
#include "lfs_sd_bd.h"
#include "lfs_spi_flash_bd.h"
#include <zephyr/device.h>


/*******************************************************************************
* Macros
********************************************************************************/
/* When set to 1, SD card will be used instead of the NOR flash */
#define STORAGE_DEVICE_SD_CARD              (0)

#define LITTLEFS_TASK_STACK_SIZE            (2048U)
#define LITTLEFS_TASK_PRIORITY              (6U)
#define USER_BUTTON_INTERRUPT_PRIORITY      (7U)

/* Debounce delay for the user button. */
#define DEBOUNCE_DELAY_MS                   (50U)

/* Copied BSP defines */
#define CYBSP_BTN_OFF                       (1U)


/*******************************************************************************
 * Function prototypes
 ******************************************************************************/
static void littlefs_task(void *dummy1, void *dummy2, void *dummy3);


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
K_SEM_DEFINE(btn_isr_sema, 0, 1);
K_THREAD_DEFINE(littlefs_task_id, LITTLEFS_TASK_STACK_SIZE, littlefs_task,
            NULL, NULL, NULL, LITTLEFS_TASK_PRIORITY, 0, -1);
cyhal_gpio_callback_data_t gpio_btn_callback_data;


/*******************************************************************************
* Function Name: check_status
****************************************************************************//**
* Summary:
*  Prints the message, indicates the non-zero status (error condition) by
*  turning the LED on, and asserts the non-zero status.
*
* Parameters:
*  message - message to print if status is non-zero.
*  status - status for evaluation.
*
*******************************************************************************/
static void check_status(char *message, uint32_t status)
{
    if (0u != status)
    {
        printk("\n================================================================================\n");
        printk("\nFAIL: %s\n", message);
        printk("Error Code: 0x%08"PRIx32"\n", status);
        printk("\n================================================================================\n");

        while(true);
    }
}


/*******************************************************************************
* Function Name: print_block_device_parameters
********************************************************************************
* Summary:
*   Prints the block device parameters such as the block count, block size, and
*   program (page) size to the UART terminal.
*
* Parameters:
*  lfs_cfg - pointer to the lfs_config structure.
*
*******************************************************************************/
static void print_block_device_parameters(struct lfs_config *lfs_cfg)
{
    printk("Number of blocks: %"PRIu32"\n", lfs_cfg->block_count);
    printk("Erase block size: %"PRIu32" bytes\n", lfs_cfg->block_size);
    printk("Prog size: %"PRIu32" bytes\n\n", lfs_cfg->prog_size);
}


/*******************************************************************************
* Function Name: user_button_interrupt_handler
********************************************************************************
* Summary:
*   User button interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_event_t (unused)
*
*******************************************************************************/
static void user_button_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    /* Remove warning for unused parameter */
	ARG_UNUSED(handler_arg);
    ARG_UNUSED(event);

    /* notify thread that data is available */
    k_sem_give(&btn_isr_sema);
}


/*******************************************************************************
* Function Name: increment_boot_count
********************************************************************************
* Summary:
*   Mounts the filesystem in the memory and performs basic file I/O operations.
*   And then reads a 32-bit value from a file, increments the value, writes
*   the value back to the file in the memory, and finally prints the value to the
*   UART terminal.
*
* Parameters:
*  lfs - pointer to the lfs_t structure.
*  lfs_cfg - pointer to the lfs_config structure.
*
*******************************************************************************/
static void increment_boot_count(lfs_t *lfs, struct lfs_config *lfs_cfg)
{
    uint32_t boot_count = 0;
    lfs_file_t file;

    /* Mount the filesystem */
    int err = lfs_mount(lfs, lfs_cfg);

    /* Reformat if we cannot mount the filesystem.
     * This should only happen when littlefs is set up on the storage device for
     * the first time.
     */
    if (err) {
        printk("\nError in mounting. This could be the first time littlefs is used on the storage device.\n");
        printk("Formatting the block device...\n\n");

        lfs_format(lfs, lfs_cfg);
        lfs_mount(lfs, lfs_cfg);
    }

    /* Read the current boot count. */
    lfs_file_open(lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(lfs, &file, &boot_count, sizeof(boot_count));

    /* Update the boot count. */
    boot_count += 1;
    lfs_file_rewind(lfs, &file);
    lfs_file_write(lfs, &file, &boot_count, sizeof(boot_count));

    /* The storage is not updated until the file_sd is closed successfully. */
    lfs_file_close(lfs, &file);

    /* Release any resources we were using. */
    lfs_unmount(lfs);

    /* Print the boot count. */
    printk("boot_count: %"PRIu32"\n\n", boot_count);
}


/*******************************************************************************
* Function Name: littlefs_task
********************************************************************************
* Summary:
*   Initializes the block device, prints the block device parameters to the UART
*   terminal, calls the function that performs simple file I/O operations, and
*   waits for user button press. When the user button is pressed, formats the
*   memory and deinitializes the block device.
*
* Parameters:
*  arg - Unused.
*
*******************************************************************************/
static void littlefs_task(void *dummy1, void *dummy2, void *dummy3)
{
    cy_rslt_t result;
    lfs_t lfs;
    struct lfs_config lfs_cfg;

    /* Remove warning for unused parameter */
    ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

    /* Step 1: Get the default configuration for the block device.
     * Step 2: Initialize the lfs_config structure to zero (not required if it
     *         is a global variable)
     * Step 3: Create the block device
     * Step 4: Print the block device parameters such as erase block size
     * Step 5: Perform file system operations to increment the boot count
     */

#if(STORAGE_DEVICE_SD_CARD)
    lfs_sd_bd_config_t sd_bd_cfg;

    printk("Incrementing the boot count on SD Card...\n\n");

    /* Get the default configuration for the SD card block device. */
    lfs_sd_bd_get_default_config(&sd_bd_cfg);

    /* Initialize the pointers in lfs_cfg to NULL. */
    memset(&lfs_cfg, 0, sizeof(lfs_cfg));

    /* Create the SD card block device. */
    result = lfs_sd_bd_create(&lfs_cfg, &sd_bd_cfg);
    check_status("Creating SD card block device failed", result);
#else
    lfs_spi_flash_bd_config_t spi_flash_bd_cfg;

    printk("\nIncrementing the boot count on SPI flash...\n\n");

    /* Get the default configuration for the SPI flash block device. */
    lfs_spi_flash_bd_get_default_config(&spi_flash_bd_cfg);

    /* Initialize the pointers in lfs_cfg to NULL. */
    memset(&lfs_cfg, 0, sizeof(lfs_cfg));

    /* Create the SPI flash block device. */
    result = lfs_spi_flash_bd_create(&lfs_cfg, &spi_flash_bd_cfg);
    check_status("Creating SPI flash block device failed", result);
#endif /* #if(STORAGE_DEVICE_SD_CARD) */

    print_block_device_parameters(&lfs_cfg);
    increment_boot_count(&lfs, &lfs_cfg);

    /* Enable the user button interrupt */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, USER_BUTTON_INTERRUPT_PRIORITY, true);

    printk("Press the user button to format the block device or press reset to increment the boot count again\n\n");

    /* Wait until the user button press is notified through the interrupt */
    while (true)
    {
        if (k_sem_take(&btn_isr_sema, K_MSEC(DEBOUNCE_DELAY_MS)) != 0)
        {
            /* Debounce the button press. */
            k_msleep(DEBOUNCE_DELAY_MS);

            if(!cyhal_gpio_read(CYBSP_USER_BTN)) { break; }
        }
    }

    /* User button is pressed. Format the block device. */
    printk("Formatting the block device...\n");
    lfs_format(&lfs, &lfs_cfg);
    printk("Formatting completed...\n");

#if(STORAGE_DEVICE_SD_CARD)
    lfs_sd_bd_destroy(&lfs_cfg);
#else
    lfs_spi_flash_bd_destroy(&lfs_cfg);
#endif /* #if(STORAGE_DEVICE_SD_CARD) */

    printk("Press reset to continue...\n");
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It does...
*    1. Initializes the UART for redirecting printf output
*    2. Intializes the user button GPIO
*    3. Creates a FreeRTOS task to perform file I/O operations
*    4. Starts the FreeRTOS scheduler
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
void main(void)
{
    cy_rslt_t result;

    /* Initialize the pins */
    init_cycfg_pins();

    /* Initialize the user button used for erasing the block device. */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    CY_ASSERT (result == CY_RSLT_SUCCESS);

    /* Configure & the user button interrupt */
    gpio_btn_callback_data.callback = user_button_interrupt_handler;
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &gpio_btn_callback_data);


    /* Enable global interrupts */
    __enable_irq();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printk("\x1b[2J\x1b[;H");

    printk("************* "
           "Littlefs File System on SD Card and QSPI NOR Flash "
           "************* \n\n");

    /* Started the created threads */
    k_thread_start(littlefs_task_id);
}


/* [] END OF FILE */
