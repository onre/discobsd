#include "sys/types.h"
#include "sys/kconfig.h"

#define C (char *)


struct conf_ctlr conf_ctlr_init[] = {
   /* driver,		unit,	addr,		pri,	flags */
    { 0 }
};

struct conf_device conf_device_init[] = {
   /* driver,		ctlr driver,	unit,	ctlr,	drive,	flags,	pins */
    { 0 }
};

struct conf_service conf_service_init[] = {
    { 0 }
};
