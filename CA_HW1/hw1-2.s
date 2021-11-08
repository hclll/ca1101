.globl __start

.rodata
    msg0: .string "This is HW1-2: \n"
    msg1: .string "Plaintext:  "
    msg2: .string "Ciphertext: "
.text

################################################################################
  # print_char function
  # Usage: 
  #     1. Store the beginning address in x20
  #     2. Use "j print_char"
  #     The function will print the string stored from x20 
  #     When finish, the whole program with return value 0

print_char:
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    add a1,x0,x20
    ecall

  # Ends the program with status code 0
    addi a0,x0,10
    ecall
    
################################################################################

__start:
  # Prints msg
    addi a0, x0, 4
    la a1, msg0
    ecall

    la a1, msg1
    ecall
    
    addi a0,x0,8
    li a1, 0x10130
    addi a2,x0,2047
    ecall
    
  # Load address of the input string into a0
    add a0,x0,a1

################################################################################ 
  # Write your main function here. 
  # a0 stores the begining Plaintext
  # Do store 66048(0x10200) into x20 
  # ex. j print_char

  # i: x19
  # addr x[i]: x5
  # addr y[i]: x6
  # x[i]: x7
  # y[i]: x8

  li x20, 0x10200
  j Encryption


  Encryption:
    addi sp, sp, -4
    sw x19, 0(sp) #push x19
    add x19, x0, x0 #i = 0
    add x28, x0, x0  #space_num = 0
    addi x30, x0, 32  #space
    addi x31, x0, 120 #x
    j L1

  L1:
    add x5, x19, a0 #x5 = addr of plain[i]
    lbu x7, 0(x5) #x7 = plain[i]
    beq x7, x0, L2 #if plain[i]== "\0", exit
    beq x7, x30, C3  #x7 = space, go to C3
    blt x7, x31, C1 #x7 < w, go to C1
    bge x7, x31, C2 #x7 >= x, go to C2

  
  C1:
    addi x8, x7, 3  #cipher[i] = plain[i] + 3
    j save_cip

  C2:
    addi x8, x7, -23  #cipher[i] = plain[i] - 23
    j save_cip

  C3:
    addi x8, x7, 16
    add x8, x8, x28  #cipher[i] = plain[i] + 16 + space_num
    addi x28, x28, 1  #space_num += 1
    j save_cip

  save_cip:
    add x6, x20, x19  #x6 = addr of cipher[i]
    sb x8, 0(x6) #store cipher[i] in 
    addi x19, x19, 1  #i += 1
    j L1   

  L2:
    lw x19, 0(sp) 
    addi sp, sp, 4
    j print_char


  
################################################################################

