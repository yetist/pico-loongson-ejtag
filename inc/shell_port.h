
#pragma once

#include "shell.h"

extern Shell shell;

short shell_port_read_usb(char *data, unsigned short len);
short shell_port_write_usb(char *data, unsigned short len);
