#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

// --- Definições do Simulador ---
#define MEMORY_SIZE (128 * 1024)
#define NUM_REGISTERS 32
#define PC_START_ADDRESS 0x80000000

// --- Mapa de Memória dos Periféricos ---
#define CLINT_BASE 0x02000000
#define PLIC_BASE  0x0C000000
#define UART_BASE  0x10000000
#define UART_IRQ 10
// --- Variáveis Globais de Estado da CPU ---
uint32_t pc;
uint32_t regs[NUM_REGISTERS];
uint8_t memory[MEMORY_SIZE];
int halt_flag = 0;
int trap_pending_print = 0;

// --- CSRs ---
uint32_t mstatus = 0x00001800;
uint32_t mie = 0, mtvec = 0, mepc = 0, mcause = 0, mtval = 0, mscratch = 0;
uint32_t mip = 0;
uint32_t misa = 0x40101101;

// --- Periféricos ---
uint64_t mtime = 0;
// --- CORREÇÃO 2: Inicializado para o valor máximo para não disparar imediatamente.
uint64_t mtimecmp = -1; 
uint32_t plic_pending = 0;
uint32_t plic_enable = 0;
uint8_t uart_ier = 0;
uint8_t uart_lsr = 1 << 5;

// Ponteiro de arquivo para a saída da UART
FILE* uart_outfile = NULL;
FILE* uart_infile = NULL;

const char *abi_name[NUM_REGISTERS] = {
    "zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2",
    "s0", "s1", "a0", "a1", "a2", "a3", "a4", "a5",
    "a6", "a7", "s2", "s3", "s4", "s5", "s6", "s7",
    "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"};

// --- Funções Auxiliares ---
const char *get_csr_name(uint32_t csr_addr) {
    switch (csr_addr) {
        case 0x300: return "mstatus"; case 0x304: return "mie";
        case 0x305: return "mtvec";   case 0x341: return "mepc";
        case 0x342: return "mcause";  case 0x343: return "mtval";
        case 0x340: return "mscratch";case 0x301: return "misa";
        case 0x344: return "mip";
        default: return "unknown_csr";
    }
}

const char *get_trap_name(uint32_t cause) {
    if (cause & 0x80000000) {
        switch (cause & 0x7FFFFFFF) {
            case 3: return "interrupt:software";
            case 7: return "interrupt:timer";
            case 11: return "interrupt:external";
            default: return "interrupt:unknown";
        }
    } else {
        switch (cause) {
            case 1: return "exception:instruction_fault";
            case 2: return "exception:illegal_instruction";
            case 5: return "exception:load_fault";
            case 7: return "exception:store_fault";
            case 11: return "exception:environment_call";
            default: return "exception:unknown";
        }
    }
}

uint32_t read_csr(uint32_t addr) {
    switch (addr) {
        case 0x300: return mstatus; case 0x301: return misa;
        case 0x304: return mie;     case 0x305: return mtvec;
        case 0x340: return mscratch;case 0x341: return mepc;
        case 0x342: return mcause;  case 0x343: return mtval;
        case 0x344: return mip;
        default: return 0;
    }
}

void write_csr(uint32_t addr, uint32_t value) {
    switch (addr) {
        case 0x300: mstatus = value; break; case 0x301: misa = value; break;
        case 0x304: mie = value; break;     case 0x305: mtvec = value; break;
        case 0x340: mscratch = value; break;case 0x341: mepc = value; break;
        case 0x342: mcause = value; break;  case 0x343: mtval = value; break;
        case 0x344: mip = value; break;
    }
}

void trigger_trap(uint32_t cause, uint32_t tval, uint32_t trap_pc) {
    trap_pending_print = 1;
    uint32_t mstatus_val = mstatus;
    mstatus &= ~(1 << 3); // Desabilita interrupções globais (bit MIE)
    mstatus |= ((mstatus_val >> 3) & 1) << 7; // Salva o estado anterior do MIE no MPIE
    
    mepc = trap_pc;   // Salva o PC da instrução que causou a falha
    mcause = cause;   // Salva a causa da falha
    mtval = tval;     // Salva o valor associado à falha (ex: endereço inválido)

    // --- LÓGICA ALTERADA ---
    // Se o programa não configurou um handler de exceção (mtvec == 0),
    // o simulador irá simplesmente pular a instrução que causou a falha.
    // Isso evita a "Double Fault" e permite que a execução continue.
    if (mtvec == 0) {
        pc = mepc + 4;
    } else {
        // Se um handler foi configurado, pula para ele.
        pc = mtvec & ~0x3;
    }
}

// --- Periféricos (MMIO) ---
uint32_t uart_read(uint32_t address) {
    uint32_t offset = address - UART_BASE;

    // Lendo o Line Status Register (LSR)
    if (offset == 5) {
        // O bit 5 (Transmitter Empty) indica que o transmissor está pronto para um novo caractere.
        uint8_t status = (1 << 5); 
        
        // Verifica se o arquivo de entrada foi aberto
        if (uart_infile) {
            // Espia o próximo caractere sem consumi-lo do stream
            int c = fgetc(uart_infile);
            if (c != EOF) {
                ungetc(c, uart_infile); // Devolve o caractere para que a leitura do RBR possa pegá-lo
                status |= 1;            // Seta o bit 0 (Data Ready), informando que há dados para ler.
            }
        }
        return status;
    }

    // Lendo o Receive Buffer Register (RBR)
    if (offset == 0) {
        // Verifica se o arquivo de entrada foi aberto
        if (uart_infile) {
            int c = fgetc(uart_infile); // Lê o próximo caractere do arquivo
            if (c != EOF) {
                return (uint8_t)c;
            }
        }
        return 0; // Retorna 0 se não houver arquivo ou se chegou ao fim (EOF)
    }
    
    // Lendo o Interrupt Identification Register (IIR)
    if (offset == 2) return 1;

    return 0;
}

void uart_write(uint32_t address, uint32_t value, uint8_t size) {
    uint32_t offset = address - UART_BASE;
    if (offset == 0 && size == 1) {
        if (uart_outfile) {
            fputc((char)value, uart_outfile);
            fflush(uart_outfile);
        } else {
            printf("%c", (char)value);
            fflush(stdout);
        }
        plic_pending |= (1 << UART_IRQ);
    } else if (offset == 1 && size == 1) {
        uart_ier = value;
    }
}

uint32_t clint_read(uint32_t address) {
    if (address == CLINT_BASE + 0xBFF8) return (uint32_t)mtime;
    if (address == CLINT_BASE + 0xBFFC) return (uint32_t)(mtime >> 32);
    return 0;
}

// --- CORREÇÃO 2: LÓGICA DO CLINT ---
void clint_write(uint32_t address, uint32_t value) {
    uint32_t offset = address - CLINT_BASE;
    if (offset == 0x4000) { // mtimecmp low
        mtimecmp = (mtimecmp & 0xFFFFFFFF00000000) | value;
        mip &= ~(1 << 7); // 
    } else if (offset == 0x4004) { // mtimecmp high
        mtimecmp = (mtimecmp & 0x00000000FFFFFFFF) | ((uint64_t)value << 32);
        mip &= ~(1 << 7);
    } else if (offset == 0) { // msip
        if (value & 1) {
            mip |= (1 << 3); // 
        } else {
            mip &= ~(1 << 3); // 
        }
    }
}

uint32_t plic_read(uint32_t address) {
    if (address == PLIC_BASE + 0x200004) {
        if ((plic_pending & plic_enable) & (1 << UART_IRQ)) {
            //plic_pending &= ~(1 << UART_IRQ); // 
            //mip &= ~(1 << 11); //
            return UART_IRQ; // 
        }
    }
    return 0;
}

void plic_write(uint32_t address, uint32_t value) {
     if (address >= PLIC_BASE + 0x2000 && address < PLIC_BASE + 0x2080) {
        plic_enable = value;
    } else if (address == PLIC_BASE + 0x200004) { 
        if(value == UART_IRQ) {
             plic_pending &= ~(1 << UART_IRQ);
        }
    } else if (address >= PLIC_BASE + 4 && address < PLIC_BASE + 0x1000) {
        // Prioridade das fontes de interrupção
    }
}

// --- Acesso à Memória ---
uint8_t memory_read_byte(uint32_t address, uint32_t current_pc) {
    if (address >= UART_BASE && address < UART_BASE + 8) return uart_read(address);
    if (address >= PC_START_ADDRESS && (address - PC_START_ADDRESS) < MEMORY_SIZE) {
        return memory[address - PC_START_ADDRESS];
    }
    trigger_trap(5, address, current_pc); return 0;
}

uint16_t memory_read_halfword(uint32_t address, uint32_t current_pc) {
    if (address % 2 != 0) {
        trigger_trap(5, address, current_pc); // Load access fault
        return 0;
    }
    if (address >= PC_START_ADDRESS && (address - PC_START_ADDRESS + 1) < MEMORY_SIZE) {
        return (uint16_t)memory[address - PC_START_ADDRESS] | ((uint16_t)memory[address - PC_START_ADDRESS + 1] << 8);
    }
    trigger_trap(5, address, current_pc); return 0;
}

uint32_t memory_read_word(uint32_t address, uint32_t current_pc) {
    if (address % 4 != 0) {
        trigger_trap(5, address, current_pc); // Load access fault
        return 0;
    }
    if (address >= CLINT_BASE && address < CLINT_BASE + 0x10000) return clint_read(address);
    if (address >= PLIC_BASE && address < PLIC_BASE + 0x4000000) return plic_read(address);
    if (address >= PC_START_ADDRESS && (address - PC_START_ADDRESS + 3) < MEMORY_SIZE) {
        return ((uint32_t)memory[address - PC_START_ADDRESS + 0] << 0) |
               ((uint32_t)memory[address - PC_START_ADDRESS + 1] << 8) |
               ((uint32_t)memory[address - PC_START_ADDRESS + 2] << 16) |
               ((uint32_t)memory[address - PC_START_ADDRESS + 3] << 24);
    }
    trigger_trap(5, address, current_pc); return 0;
}

void memory_write_byte(uint32_t address, uint8_t value, uint32_t current_pc) {
    if (address >= UART_BASE && address < UART_BASE + 8) { uart_write(address, value, 1); return; }
    if (address >= PC_START_ADDRESS && (address - PC_START_ADDRESS) < MEMORY_SIZE) {
        memory[address - PC_START_ADDRESS] = value;
    } else {
        trigger_trap(7, address, current_pc);
    }
}

void memory_write_halfword(uint32_t address, uint16_t value, uint32_t current_pc) {
    if (address % 2 != 0) {
        trigger_trap(7, address, current_pc); // Store/AMO access fault
        return;
    }
    if (address >= PC_START_ADDRESS && (address - PC_START_ADDRESS + 1) < MEMORY_SIZE) {
        memory[address - PC_START_ADDRESS + 0] = (uint8_t)(value & 0xFF);
        memory[address - PC_START_ADDRESS + 1] = (uint8_t)((value >> 8) & 0xFF);
    } else {
        trigger_trap(7, address, current_pc);
    }
}

void memory_write_word(uint32_t address, uint32_t value, uint32_t current_pc) {
    if (address % 4 != 0) {
        trigger_trap(7, address, current_pc); // Store/AMO access fault
        return;
    }
    if (address >= CLINT_BASE && address < CLINT_BASE + 0x10000) { clint_write(address, value); return; }
    if (address >= PLIC_BASE && address < PLIC_BASE + 0x4000000) { plic_write(address, value); return; }
    if (address >= PC_START_ADDRESS && (address - PC_START_ADDRESS + 3) < MEMORY_SIZE) {
         memory[address - PC_START_ADDRESS + 0] = (uint8_t)(value & 0xFF);
        memory[address - PC_START_ADDRESS + 1] = (uint8_t)((value >> 8) & 0xFF);
        memory[address - PC_START_ADDRESS + 2] = (uint8_t)((value >> 16) & 0xFF);
        memory[address - PC_START_ADDRESS + 3] = (uint8_t)((value >> 24) & 0xFF);
    } else {
        trigger_trap(7, address, current_pc);
    }
}

// --- Funções de Decodificação ---
uint32_t fetch_instruction_from_pc() {
    if (pc < PC_START_ADDRESS || (pc + 3) >= (PC_START_ADDRESS + MEMORY_SIZE) || (pc % 4 != 0)) {
        trigger_trap(1, pc, pc);
        return 0;
    }
    return memory_read_word(pc, pc);
}

uint32_t get_opcode(uint32_t i) { return i & 0x7F; }
uint32_t get_rd(uint32_t i) { return (i >> 7) & 0x1F; }
uint32_t get_rs1(uint32_t i) { return (i >> 15) & 0x1F; }
uint32_t get_rs2(uint32_t i) { return (i >> 20) & 0x1F; }
uint32_t get_funct3(uint32_t i) { return (i >> 12) & 0x7; }
uint32_t get_funct7(uint32_t i) { return (i >> 25) & 0x7F; }

int32_t get_imm_I(uint32_t instruction) {
    return (int32_t)(instruction) >> 20;
}

int32_t get_imm_S(uint32_t instruction) {
    int32_t imm = ((instruction >> 25) << 5) | ((instruction >> 7) & 0x1F);
    return (int32_t)(imm << 20) >> 20;
}

int32_t get_imm_B(uint32_t instruction) {
    int32_t imm = (((instruction >> 31) & 0x1) << 12) |
                  (((instruction >> 7) & 0x1) << 11) |
                  (((instruction >> 25) & 0x3F) << 5)|
                  (((instruction >> 8) & 0xF) << 1);
    return (int32_t)(imm << 19) >> 19;
}

int32_t get_imm_U(uint32_t instruction) {
    return (int32_t)(instruction & 0xFFFFF000);
}

int32_t get_imm_J(uint32_t instruction) {
    int32_t imm = (((instruction >> 31) & 0x1) << 20) |
                  (((instruction >> 12) & 0xFF) << 12) |
                  (((instruction >> 20) & 0x1) << 11) |
                  (((instruction >> 21) & 0x3FF) << 1);
    return (int32_t)(imm << 11) >> 11;
}

// --- Decodificação e Execução ---
void decode_and_execute(uint32_t instruction, uint32_t current_pc, char* details_buffer) {
    uint32_t opcode = get_opcode(instruction), rd = get_rd(instruction), rs1 = get_rs1(instruction),
             rs2 = get_rs2(instruction), funct3 = get_funct3(instruction), funct7 = get_funct7(instruction);

    int32_t imm_i_sext = get_imm_I(instruction);
    int32_t imm_s_sext = get_imm_S(instruction);
    int32_t imm_b_sext = get_imm_B(instruction);
    int32_t imm_u_sext = get_imm_U(instruction);
    int32_t imm_j_sext = get_imm_J(instruction);

    uint32_t original_rs1_val = regs[rs1], original_rs2_val = regs[rs2];
    uint32_t effective_address;
    switch (opcode) {
        case 0x37:
            if (rd != 0) regs[rd] = imm_u_sext;
            sprintf(details_buffer, "lui    %s,0x%05x          %s=0x%08x", abi_name[rd], (imm_u_sext >> 12) & 0xFFFFF, abi_name[rd], rd != 0 ? regs[rd] : 0);
            break;
        case 0x17:
            if (rd != 0) regs[rd] = current_pc + imm_u_sext;
            sprintf(details_buffer, "auipc  %s,0x%05x          %s=0x%08x+0x%08x=0x%08x", abi_name[rd], (imm_u_sext >> 12) & 0xFFFFF, abi_name[rd], current_pc, imm_u_sext, rd != 0 ? regs[rd] : 0);
            break;
        case 0x6F:
            if (rd != 0) regs[rd] = current_pc + 4;
            pc = current_pc + imm_j_sext;
            sprintf(details_buffer, "jal    %s,0x%05x        pc=0x%08x,%s=0x%08x", abi_name[rd], (uint32_t)imm_j_sext & 0x1FFFFF, pc, abi_name[rd], rd != 0 ? regs[rd] : 0);
            break;
        case 0x67:
            {
                uint32_t target_pc = (original_rs1_val + imm_i_sext) & ~1;
                if (rd != 0) regs[rd] = current_pc + 4;
                pc = target_pc;
                sprintf(details_buffer, "jalr   %s,%s,0x%03x       pc=0x%08x+0x%08x,%s=0x%08x", abi_name[rd], abi_name[rs1], (uint32_t)imm_i_sext & 0xFFF, original_rs1_val, (uint32_t)imm_i_sext, abi_name[rd], rd != 0 ? regs[rd] : 0);
            }
            break;
        case 0x63:
             {
                int taken = 0; const char* op_str = "?"; const char* op_name="?";
                switch (funct3) {
                    case 0x0: op_name="beq";  if (original_rs1_val == original_rs2_val) taken = 1; op_str = "=="; break;
                    case 0x1: op_name="bne";  if (original_rs1_val != original_rs2_val) taken = 1; op_str = "!="; break;
                    case 0x4: op_name="blt";  if ((int32_t)original_rs1_val < (int32_t)original_rs2_val) taken = 1; op_str = "<"; break;
                    case 0x5: op_name="bge";  if ((int32_t)original_rs1_val >= (int32_t)original_rs2_val) taken = 1; op_str = ">="; break;
                    case 0x6: op_name="bltu"; if (original_rs1_val < original_rs2_val) taken = 1; op_str = "<"; break;
                    case 0x7: op_name="bgeu"; if (original_rs1_val >= original_rs2_val) taken = 1; op_str = ">="; break;
                    default: trigger_trap(2, instruction, current_pc); return;
                }
                if (taken) pc = current_pc + imm_b_sext;
                sprintf(details_buffer, "%-7s%s,%s,0x%03x       (%s(0x%08x)%s%s(0x%08x))=%u->pc=0x%08x", op_name, abi_name[rs1], abi_name[rs2], (uint32_t)imm_b_sext & 0x1FFF, (funct3 >= 6 ? "u" : ""), original_rs1_val, op_str, (funct3 >= 6 ? "u" : ""), original_rs2_val, taken, pc);
            }
            break;
        case 0x03:
            effective_address = original_rs1_val + imm_i_sext;
            const char* op_name_load = "?";
            switch (funct3) {
                case 0x0: op_name_load="lb";  if(rd!=0) regs[rd] = (int32_t)(int8_t)memory_read_byte(effective_address, current_pc); break;
                case 0x1: op_name_load="lh";  if(rd!=0) regs[rd] = (int32_t)(int16_t)memory_read_halfword(effective_address, current_pc); break;
                case 0x2: op_name_load="lw";  if(rd!=0) regs[rd] = memory_read_word(effective_address, current_pc); break;
                case 0x4: op_name_load="lbu"; if(rd!=0) regs[rd] = memory_read_byte(effective_address, current_pc); break;
                case 0x5: op_name_load="lhu"; if(rd!=0) regs[rd] = memory_read_halfword(effective_address, current_pc); break;
                default: trigger_trap(2, instruction, current_pc); return;
            }
            if (!trap_pending_print) {
                sprintf(details_buffer, "%-7s%s,0x%03x(%s)      %s=mem[0x%08x]=0x%08x", op_name_load, abi_name[rd], imm_i_sext & 0xFFF, abi_name[rs1], abi_name[rd], effective_address, rd != 0 ? regs[rd] : 0);
            }
            break;
        case 0x23:
            effective_address = original_rs1_val + imm_s_sext;
            switch (funct3) {
                case 0x0: memory_write_byte(effective_address, (uint8_t)original_rs2_val, current_pc); if(!trap_pending_print) sprintf(details_buffer, "sb     %s,0x%03x(%s)        mem[0x%08x]=0x%02x", abi_name[rs2], imm_s_sext & 0xFFF, abi_name[rs1], effective_address, original_rs2_val & 0xFF); break;
                case 0x1: memory_write_halfword(effective_address, (uint16_t)original_rs2_val, current_pc); if(!trap_pending_print) sprintf(details_buffer, "sh     %s,0x%03x(%s)        mem[0x%08x]=0x%04x", abi_name[rs2], imm_s_sext & 0xFFF, abi_name[rs1], effective_address, original_rs2_val & 0xFFFF); break;
                case 0x2: memory_write_word(effective_address, original_rs2_val, current_pc); if(!trap_pending_print) sprintf(details_buffer, "sw     %s,0x%03x(%s)        mem[0x%08x]=0x%08x", abi_name[rs2], imm_s_sext & 0xFFF, abi_name[rs1], effective_address, original_rs2_val); break;
                default: trigger_trap(2, instruction, current_pc); return;
            }
            break;
        case 0x13:
             {
                uint32_t result_val = 0;
                switch (funct3) {
                    case 0x0: result_val = original_rs1_val + imm_i_sext; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "addi   %s,%s,0x%x       %s=0x%08x+0x%08x=0x%08x", abi_name[rd], abi_name[rs1], (uint32_t)imm_i_sext & 0xFFF, abi_name[rd], original_rs1_val, imm_i_sext, result_val); break;
                    case 0x1: { uint32_t shamt = imm_i_sext & 0x1F; result_val = original_rs1_val << shamt; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "slli   %s,%s,%d          %s=0x%08x<<%d=0x%08x", abi_name[rd], abi_name[rs1], shamt, abi_name[rd], original_rs1_val, shamt, result_val); } break;
                    case 0x2: result_val = ((int32_t)original_rs1_val < imm_i_sext) ? 1 : 0; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "slti   %s,%s,%d       %s=(0x%08x<%d)=%u", abi_name[rd], abi_name[rs1], imm_i_sext, abi_name[rd], original_rs1_val, imm_i_sext, result_val); break;
                    case 0x3: result_val = (original_rs1_val < (uint32_t)imm_i_sext) ? 1 : 0; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "sltiu  %s,%s,%d       %s=(0x%08x<%u)=%u", abi_name[rd], abi_name[rs1], imm_i_sext, abi_name[rd], original_rs1_val, (uint32_t)imm_i_sext, result_val); break;
                    case 0x4: result_val = original_rs1_val ^ imm_i_sext; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "xori   %s,%s,0x%03x       %s=0x%08x^0x%03x=0x%08x", abi_name[rd], abi_name[rs1], imm_i_sext & 0xFFF, abi_name[rd], original_rs1_val, imm_i_sext & 0xFFF, result_val); break;
                    case 0x5:
                        {
                            uint32_t shamt = imm_i_sext & 0x1F;
                            if ((instruction >> 30) == 0x00) { result_val = original_rs1_val >> shamt; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "srli   %s,%s,%d          %s=0x%08x>>%d=0x%08x", abi_name[rd], abi_name[rs1], shamt, abi_name[rd], original_rs1_val, shamt, result_val); }
                            else { result_val = (int32_t)original_rs1_val >> shamt; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "srai   %s,%s,%d          %s=0x%08x>>>%d=0x%08x", abi_name[rd], abi_name[rs1], shamt, abi_name[rd], original_rs1_val, shamt, result_val); }
                        } break;
                    case 0x6: result_val = original_rs1_val | imm_i_sext; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "ori    %s,%s,0x%03x       %s=0x%08x|0x%03x=0x%08x", abi_name[rd], abi_name[rs1], imm_i_sext & 0xFFF, abi_name[rd], original_rs1_val, imm_i_sext & 0xFFF, result_val); break;
                    case 0x7: result_val = original_rs1_val & imm_i_sext; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "andi   %s,%s,0x%03x       %s=0x%08x&0x%03x=0x%08x", abi_name[rd], abi_name[rs1], imm_i_sext & 0xFFF, abi_name[rd], original_rs1_val, imm_i_sext & 0xFFF, result_val); break;
                    default: trigger_trap(2, instruction, current_pc); return;
                }
            }
            break;
        case 0x33:
            {
                uint32_t result_val = 0;
                if (funct7 == 0x01) {
                    switch (funct3) {
                        case 0x0: result_val = (int32_t)original_rs1_val * (int32_t)original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "mul    %s,%s,%s         %s=0x%08x*0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x1: result_val = (uint32_t)(((int64_t)(int32_t)original_rs1_val * (int64_t)(int32_t)original_rs2_val) >> 32); if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "mulh   %s,%s,%s         %s=0x%08x*0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x2: result_val = (uint32_t)(((int64_t)(int32_t)original_rs1_val * (uint64_t)original_rs2_val) >> 32); if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "mulhsu %s,%s,%s         %s=0x%08x*0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x3: result_val = (uint32_t)(((uint64_t)original_rs1_val * (uint64_t)original_rs2_val) >> 32); if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "mulhu  %s,%s,%s         %s=0x%08x*0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x4: if (original_rs2_val == 0) result_val = -1; else if (original_rs1_val == 0x80000000 && original_rs2_val == 0xFFFFFFFF) result_val = 0x80000000; else result_val = (int32_t)original_rs1_val / (int32_t)original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "div    %s,%s,%s         %s=0x%08x/0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x5: if (original_rs2_val == 0) result_val = 0xFFFFFFFF; else result_val = original_rs1_val / original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "divu   %s,%s,%s         %s=0x%08x/0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x6: if (original_rs2_val == 0) result_val = original_rs1_val; else if (original_rs1_val == 0x80000000 && original_rs2_val == 0xFFFFFFFF) result_val = 0; else result_val = (int32_t)original_rs1_val % (int32_t)original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "rem    %s,%s,%s         %s=0x%08x%%0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x7: if (original_rs2_val == 0) result_val = original_rs1_val; else result_val = original_rs1_val % original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "remu   %s,%s,%s         %s=0x%08x%%0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        default: trigger_trap(2, instruction, current_pc); return;
                    }
                } else {
                    uint32_t shamt = original_rs2_val & 0x1F;
                    switch (funct3) {
                        case 0x0: if (funct7 == 0x20) { result_val = original_rs1_val - original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "sub    %s,%s,%s         %s=0x%08x-0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val);} else { result_val = original_rs1_val + original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "add    %s,%s,%s         %s=0x%08x+0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); } break;
                        case 0x1: result_val = original_rs1_val << shamt; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "sll    %s,%s,%s         %s=0x%08x<<%d=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, shamt, result_val); break;
                        case 0x2: result_val = ((int32_t)original_rs1_val < (int32_t)original_rs2_val) ? 1 : 0; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "slt    %s,%s,%s         %s=(0x%08x<0x%08x)=%u", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x3: result_val = (original_rs1_val < original_rs2_val) ? 1 : 0; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "sltu   %s,%s,%s         %s=(0x%08x<0x%08x)=%u", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x4: result_val = original_rs1_val ^ original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "xor    %s,%s,%s         %s=0x%08x^0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x5: if (funct7 == 0x20) { result_val = (int32_t)original_rs1_val >> shamt; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "sra    %s,%s,%s         %s=0x%08x>>>%d=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, shamt, result_val); } else { result_val = original_rs1_val >> shamt; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "srl    %s,%s,%s         %s=0x%08x>>%d=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, shamt, result_val); } break;
                        case 0x6: result_val = original_rs1_val | original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "or     %s,%s,%s         %s=0x%08x|0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        case 0x7: result_val = original_rs1_val & original_rs2_val; if(rd!=0) regs[rd] = result_val; sprintf(details_buffer, "and    %s,%s,%s         %s=0x%08x&0x%08x=0x%08x", abi_name[rd], abi_name[rs1], abi_name[rs2], abi_name[rd], original_rs1_val, original_rs2_val, result_val); break;
                        default: trigger_trap(2, instruction, current_pc); return;
                    }
                }
            }
            break;
        case 0x73:
            {
                uint32_t csr_addr = (uint32_t)imm_i_sext & 0xFFF;
                uint32_t uimm = rs1;
                switch(funct3) {
                    case 0x0:
                        if (imm_i_sext == 0x0) { sprintf(details_buffer, "ecall"); trigger_trap(11, 0, current_pc); }
                        else if (imm_i_sext == 0x1) { sprintf(details_buffer, "ebreak"); halt_flag = 1; mcause = 3; mepc = current_pc; }
                        else if (imm_i_sext == 0x302) {
                             pc = mepc;
                             uint32_t prev_mstatus = mstatus;
                             mstatus &= ~(1 << 7);
                             mstatus |= ((prev_mstatus >> 3) & 1) << 3;
                             sprintf(details_buffer, "mret                       pc=0x%08x", pc);
                        }
                        else { trigger_trap(2, instruction, current_pc); }
                        break;
                    case 0x1:
                         { uint32_t temp = read_csr(csr_addr); if (rd != 0) regs[rd] = temp; write_csr(csr_addr, original_rs1_val); sprintf(details_buffer, "csrrw  %s,%s,%s       %s=%s=0x%08x,%s=0x%08x", abi_name[rd], get_csr_name(csr_addr), abi_name[rs1], abi_name[rd], get_csr_name(csr_addr), temp, get_csr_name(csr_addr), original_rs1_val); } break;
                    case 0x2:
                         { uint32_t temp = read_csr(csr_addr); if (rd != 0) regs[rd] = temp; if(rs1 != 0) write_csr(csr_addr, temp | original_rs1_val); sprintf(details_buffer, "csrrs  %s,%s,%s      %s=%s=0x%08x,%s|=0x%08x=0x%08x", abi_name[rd], get_csr_name(csr_addr), abi_name[rs1], abi_name[rd],get_csr_name(csr_addr), temp, get_csr_name(csr_addr), original_rs1_val, read_csr(csr_addr)); } break;
                    case 0x3:
                         { uint32_t temp = read_csr(csr_addr); if (rd != 0) regs[rd] = temp; if(rs1 != 0) write_csr(csr_addr, temp & ~original_rs1_val); sprintf(details_buffer, "csrrc  %s,%s,%s       %s=%s=0x%08x,%s&=~0x%08x=0x%08x", abi_name[rd], get_csr_name(csr_addr), abi_name[rs1], abi_name[rd],get_csr_name(csr_addr), temp, get_csr_name(csr_addr), original_rs1_val, read_csr(csr_addr)); } break;
                    case 0x5:
                         { uint32_t temp = read_csr(csr_addr); if (rd != 0) regs[rd] = temp; write_csr(csr_addr, uimm); sprintf(details_buffer, "csrrwi %s,%s,%u      %s=%s=0x%08x,%s=%u", abi_name[rd], get_csr_name(csr_addr), uimm, abi_name[rd], get_csr_name(csr_addr), temp, get_csr_name(csr_addr), uimm); } break;
                    case 0x6:
                         { uint32_t temp = read_csr(csr_addr); if (rd != 0) regs[rd] = temp; if(uimm != 0) write_csr(csr_addr, temp | uimm); sprintf(details_buffer, "csrrsi %s,%s,%u      %s=%s=0x%08x,%s|=%u=0x%08x", abi_name[rd], get_csr_name(csr_addr), uimm, abi_name[rd], get_csr_name(csr_addr), temp, get_csr_name(csr_addr), uimm, read_csr(csr_addr)); } break;
                    case 0x7:
                         { uint32_t temp = read_csr(csr_addr); if (rd != 0) regs[rd] = temp; if(uimm != 0) write_csr(csr_addr, temp & ~uimm); sprintf(details_buffer, "csrrci %s,%s,%u      %s=%s=0x%08x,csr&=~%u=0x%08x", abi_name[rd], get_csr_name(csr_addr), uimm, abi_name[rd], get_csr_name(csr_addr), temp, uimm, read_csr(csr_addr)); } break;
                    default: trigger_trap(2, instruction, current_pc); return;
                }
            }
            break;
        case 0x0F:
            if (funct3 == 0x0) sprintf(details_buffer, "fence");
            else if (funct3 == 0x1) sprintf(details_buffer, "fence.i");
            break;
        default:
            trigger_trap(2, instruction, current_pc);
    }
    regs[0] = 0;
}

void load_program_from_hex_string(const char* hex_string) {
    const char *p = hex_string;
    uint32_t address = 0;
    int has_address = 0;

    while (*p != '\0') {
        while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') {
            p++;
        }
        if (*p == '\0') break;

        if (*p == '@') {
            p++;
            address = (uint32_t)strtoul(p, (char**)&p, 16);
            if (!has_address) {
                pc = address;
                has_address = 1;
            }
            continue;
        }

        char hex_byte_str[3] = {p[0], p[1], '\0'};
        uint8_t byte_val = (uint8_t)strtoul(hex_byte_str, NULL, 16);

        if (address >= PC_START_ADDRESS && (address - PC_START_ADDRESS) < MEMORY_SIZE) {
            memory[address - PC_START_ADDRESS] = byte_val;
        }
        address++;
        p += 2;
    }

    if (!has_address) {
        pc = PC_START_ADDRESS;
    }
}

// --- CÓDIGO CORRIGIDO ---
int main(int argc, char *argv[]) {
    if (argc != 5) {
        fprintf(stderr, "Uso: %s <hex_in> <trace_out> <term_in> <term_out>\n", argv[0]);
        return EXIT_FAILURE;
    }

    FILE *infile = fopen(argv[1], "r");
    if (!infile) {
        perror("Erro ao abrir arquivo de entrada hex");
        return EXIT_FAILURE;
    }

    FILE *outfile = fopen(argv[2], "w");
    if (!outfile) {
        perror("Erro ao abrir arquivo de saida trace");
        fclose(infile);
        return EXIT_FAILURE;
    }

    // Abre o arquivo de entrada do terminal (argv[3])
    uart_infile = fopen(argv[3], "r");
    if (!uart_infile) {
        perror("Erro ao abrir arquivo de entrada do terminal");
        fclose(infile);
        fclose(outfile);
        return EXIT_FAILURE;
    }

    uart_outfile = fopen(argv[4], "w");
    if (!uart_outfile) {
        perror("Erro ao criar arquivo de saida do terminal");
        fclose(infile);
        fclose(outfile);
        fclose(uart_infile); // Garante que todos os arquivos abertos sejam fechados
        return EXIT_FAILURE;
    }

    fseek(infile, 0, SEEK_END);
    long file_size = ftell(infile);
    fseek(infile, 0, SEEK_SET);
    char *program_hex_string = (char*)malloc(file_size + 1);
    fread(program_hex_string, 1, file_size, infile);
    program_hex_string[file_size] = '\0';
    fclose(infile);

    memset(regs, 0, sizeof(regs));
    memset(memory, 0, MEMORY_SIZE);
    regs[2] = PC_START_ADDRESS + MEMORY_SIZE;

    load_program_from_hex_string(program_hex_string);
    free(program_hex_string);

    char details_buffer[256];

    uint32_t last_trap_pc = 0xFFFFFFFF;
    uint32_t last_trap_cause = 0xFFFFFFFF;

// --- CÓDIGO CORRIGIDO ---
while (!halt_flag) {
    uint32_t current_instruction_pc = pc;

    // --- Tratamento de Interrupções (pode gerar um trap) ---
    mtime++;
    if (mtimecmp != (uint64_t)-1 && mtime >= mtimecmp) {
        mip |= (1 << 7);
    }
    if ((plic_pending & plic_enable) & (1 << UART_IRQ)) {
        mip |= (1 << 11);
    }

    uint32_t pending_and_enabled = mip & mie;
    if ((mstatus & (1 << 3)) && pending_and_enabled) {
        uint32_t trap_cause = 0;
        if (pending_and_enabled & (1 << 11)) trap_cause = 0x8000000B; // External
        else if (pending_and_enabled & (1 << 3)) trap_cause = 0x80000003; // Software
        else if (pending_and_enabled & (1 << 7)) trap_cause = 0x80000007; // Timer

        if (trap_cause != 0) {
             trigger_trap(trap_cause, 0, current_instruction_pc);
        }
    }
    
    // --- Busca e Execução da Instrução ---
    uint32_t instruction_hex = fetch_instruction_from_pc();

    // Se fetch_instruction_from_pc ou o handler de interrupção causaram um trap,
    // trap_pending_print estará setado.
    if (!trap_pending_print) {
        // Se não há trap pendente, executa a instrução.
        details_buffer[0] = '\0';
        decode_and_execute(instruction_hex, current_instruction_pc, details_buffer);
    }

    // --- Lógica Centralizada de Pós-Execução ---
    if (trap_pending_print) {
        // Um trap ocorreu (seja por interrupção, fetch ou execução).
        if (mepc == last_trap_pc && mcause == last_trap_cause) {
            fprintf(outfile, ">FATAL: Double fault detected. Halting simulation.\n");
            halt_flag = 1;
        } else {
            last_trap_pc = mepc;
            last_trap_cause = mcause;
            fprintf(outfile, ">%s                   cause=0x%08x,epc=0x%08x,tval=0x%08x\n", get_trap_name(mcause), mcause, mepc, mtval);
            
            // Lógica para pular a instrução se não houver handler
            if (mtvec == 0) {
                pc = mepc + 4; 
            }
        }
        trap_pending_print = 0;
    } else {
        // A instrução executou com sucesso.
        if (strlen(details_buffer) > 0) {
            fprintf(outfile, "0x%08x:%s\n", current_instruction_pc, details_buffer);
        }
        
        // Se o PC não foi alterado por um jump/branch, nós o incrementamos.
        if (pc == current_instruction_pc) {
            pc += 4;
        }
    }
}
    // Fecha todos os arquivos abertos
    fclose(outfile);
    if(uart_outfile) fclose(uart_outfile);
    if(uart_infile) fclose(uart_infile); // --- ADICIONADO ---
    
    return EXIT_SUCCESS;
}