MOV R0, SW_BASE
LDR R0, [R0] // R0 = 0x140
LDR R2, [R0] // R2 = value on SW0 through SW7 on DE1-SoC
MOV R3, R2, LSL #1 // R3 = R2 << 1 (which is 2*R2)
MOV R1, LEDR_BASE
LDR R1, [R1] // R1 = 0x100
STR R3, [R1] // display contents of R3 on red LEDs
HALT
SW_BASE:
.word 0x0140
LEDR_BASE:
.word 0x0100