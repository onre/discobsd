/*
 * Kernel memory linker script for TEENSY35.
 */

MEMORY {
    FLASH  (rx)   : ORIGIN = 0x00000000, LENGTH = 512K
    RAM    (rwx) : ORIGIN = 0x1FFF0000, LENGTH = 58K 
    U0AREA (rwx) : ORIGIN = 0x1FFFE800, LENGTH = 3K  
    UAREA  (rwx) : ORIGIN = 0x1FFFF400, LENGTH = 3K

    USERRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 191K
}
