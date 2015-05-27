/*
 * Copyright (c) 2014, 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file uart_test.c
 * @brief Ara Toshiba bridge ASIC uart test program
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

//need to reobe


/*
 * These commands mirror the Greybus uart protocol ops as of version
 * v0.1.
 *
 * The following Greybus uart operations have no counterparts here:
 *
 */
 
#define INVALID -1

enum {
    HELP,
    PROTOCOL_VERSION,
    SEND_DATA,
    RECEIVE_DATA,
    SET_LINE_CODING,
    SET_CONTROL_LINE_STATE,
    SEND_BREAK,
    SERIAL_STATE,
    MAX_CMD,
};

struct command {
    const char shortc;
    const char *longc;
    const char *argspec;
    const char *help;
};

static const struct command commands[] = {
    [HELP]					= {'h', "help", NULL, "Print this message and exit"},
    [PROTOCOL_VERSION]		= {'p', "protocol-version", NULL, "Print number of protocol version\n" },
    [SEND_DATA]				= {'s', "send-data", "<uart> [...]", "Send data to UART"},
    [RECEIVE_DATA]			= {'r', "receive-data","<uart> [...]", "receive data from UART"},
    [SET_LINE_CODING]		= {'l', "set-line-coding", "<uart> [...]","set line coding to UART"},
    [SET_CONTROL_LINE_STATE]= {'c', "set-control-state", "<uart> [...]","set control line to UART"},
    [SEND_BREAK]			= {'b', "send-break", "<uart> [...]", "send break to UART"},
    [SERIAL_STATE]			= {'e', "serial-state", "<uart> [...]", "Get serial state value"},
};


static void print_usage(void)
{
    int i;
    printf("uart: usage:\n");
    for (i = 0; i < MAX_CMD; i++) {
        const char *argspec = commands[i].argspec;
        const char *space = " ";
        if (!argspec) {
            space = "";
            argspec = "";
        }
        printf(" uart_test [%c|%s]%s%s: %s\n",
               commands[i].shortc, commands[i].longc,
               space, argspec,
               commands[i].help);
    }
    printf("\n"
           "<uart> values range from 0 to the line count minus one.\n");
}

static void usage(int exit_status)
{
    print_usage();
    exit(exit_status);
}

static void get_protocol_version(void)
{

	printf("get_protocol_version\n");
    //printf("uart line count: %u\n", gpio_line_count());
}



#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int uart_test_main(int argc, char *argv[])
#endif
{
    int i;
    int cmd = INVALID;
    const char *cmd_str;
    int rc = 0;

    /* Parse arguments. */
    if (argc < 2) {
        rc = EXIT_FAILURE;
        goto done;
    }
    cmd_str = argv[1];
    for (i = 0; i < MAX_CMD; i++) {
        if (!strcmp(cmd_str, commands[i].longc)) {
            cmd = i;
            break;
        } else if (strlen(cmd_str) == 1 &&
                   cmd_str[0] == commands[i].shortc) {
            cmd = i;
            break;
        }
    }

    /* Parse command arguments and run command. */
    argc -= 2;
    void (*handler)(uint8_t*, size_t);
    switch (cmd) {
    /* These are special cases. */
    case HELP:
        print_usage();
        goto done;
    case PROTOCOL_VERSION:
        get_protocol_version();
        goto done;
        
        
 default:
        rc = EXIT_FAILURE;
        goto done;
    case SEND_DATA:
  /*      if (argc == 0) {
            fprintf(stderr,
                    "You must specify at least one GPIO and value to set.\n");
            rc = EXIT_FAILURE;
            goto done;
        } else if (argc & 1) {
            fprintf(stderr, "You must specify one value per GPIO\n");
            rc = EXIT_FAILURE;
            goto done;
        } else {
            printf("Setting (gpio, value):");
            for (i = 0; i < argc / 2; i++) {
                gpios[i] = (uint8_t)atoi(argv[2 + 2 * i]);
                values[i] = !!(uint8_t)atoi(argv[3 + 2 * i]);
                printf(" (%u, %u)", gpios[i], values[i]);
            }
            printf("\n");
            do_set_value(gpios, values, (size_t)argc / 2);
        }
        goto done;*/
   
    /* The rest are all parsed in the same way. */
    case RECEIVE_DATA:
    
        break;
    case SET_LINE_CODING:
        
        break;
    case SET_CONTROL_LINE_STATE:
        
        break;
    case SEND_BREAK:
        
        break;
    case SERIAL_STATE:
        
        break;
   
    }

    if (argc == 0) {
        fprintf(stderr, "You must specify at least one cpmmand.\n");
        rc = EXIT_FAILURE;
        goto done;
    }
   
 /*   printf("UARTs:");
    for (i = 0; i < argc; i++) {
        gpios[i] = (uint8_t)atoi(argv[2 + i]);
        printf(" %u", gpios[i]);
    }
    printf("\n");
    handler(gpios, (size_t)argc);*/
 done:
   
    if (rc) {
        usage(rc);
    }
    return 0;
}
