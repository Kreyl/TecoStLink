/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Provides main entry point. Initialise subsystems and enter GDB protocol loop. */

#include "general.h"
#include "platform.h"
#include "gdb_if.h"
#include "gdb_main.h"
#include "target.h"
#include "exception.h"
#include "gdb_packet.h"
#include "command.h"
#include "rtt.h"
#include <libopencm3/stm32/usart.h>

/* This has to be aligned so the remote protocol can re-use it without causing Problems */
static char pbuf[GDB_PACKET_BUFFER_SIZE + 1U] __attribute__((aligned(8)));

char* gdb_packet_buffer() {
    return pbuf;
}

typedef enum {kPwrOffUartOff, kPwrOffUartOn, kPwrOnUartOn} PwrAndUartState;

static void TaskButton() {
    static uint32_t start = 0;
    static bool btn_was_pressed = false;
    static PwrAndUartState pwr_uart_sta = kPwrOffUartOn;

    uint32_t now = platform_time_ms();
    if(now - start >= 54UL) {
        start = now;
        // Check button
        bool btn_is_pressed = (gpio_get(BTN_PORT, BTN_PIN) == 0);
        if(btn_is_pressed && !btn_was_pressed) { // Btn press occured, switch power
            btn_was_pressed = true;
            switch(pwr_uart_sta) {
                case kPwrOffUartOff:
                    pwr_uart_sta = kPwrOffUartOn;
                    // Power off
                    platform_target_set_power(false);
                    // Activate UART
                    gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN);
                    gpio_set_mode(USBUSART_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, USBUSART_RX_PIN);
                    gpio_set(USBUSART_PORT, USBUSART_RX_PIN); // Enable Pull-up
                    aux_serial_drain_receive_buffer();
                    usart_enable(USBUSART);
                    // Indicate
                    gpio_set(LED_PORT, LED_UART_EN);
                    break;

                case kPwrOffUartOn:
                    pwr_uart_sta = kPwrOnUartOn;
                    // Power On
                    platform_target_set_power(true);
                    // UART already on, do not touch it
                    break;

                case kPwrOnUartOn:
                    pwr_uart_sta = kPwrOffUartOff;
                    // Power off
                    platform_target_set_power(false);
                    // Disable UART
                    usart_disable(USBUSART);
                    aux_serial_drain_receive_buffer();
                    gpio_set_mode(USBUSART_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, USBUSART_TX_PIN | USBUSART_RX_PIN);
                    // Indicate
                    gpio_clear(LED_PORT, LED_UART_EN);
                    break;
            } // switch
        }
        else if(!btn_is_pressed && btn_was_pressed) { // Btn release occured
            btn_was_pressed = false;
        }
    }
}

int main(void) {
    platform_init();

    while(true) {
        volatile exception_s e;
        TRY_CATCH(e, EXCEPTION_ALL) {
            SET_IDLE_STATE(false);
            while(gdb_target_running && cur_target) {
                gdb_poll_target();
                // Check again, as `gdb_poll_target()` may alter these variables.
                if(!gdb_target_running || !cur_target) break;
                char c = gdb_if_getchar_to(0);
                if(c == '\x03' || c == '\x04') target_halt_request(cur_target);
                if(rtt_enabled) poll_rtt(cur_target);
                TaskButton();
            }

            SET_IDLE_STATE(true);
            size_t size = gdb_getpacket(pbuf, GDB_PACKET_BUFFER_SIZE);
            // If port closed and target detached, stay idle
            if(pbuf[0] != '\x04' || cur_target) SET_IDLE_STATE(false);
            gdb_main(pbuf, GDB_PACKET_BUFFER_SIZE, size);
            TaskButton();
        }
        if(e.type) {
            gdb_putpacketz("EFF");
            target_list_free();
            gdb_outf("Uncaught exception: %s\n", e.msg);
        }
    }

    target_list_free();
    return 0;
}
