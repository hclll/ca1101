.globl __start

.rodata
    msg0: .string "This is HW1-1: T(n) = 2T(n/2) + 8n + 5, T(1) = 4\n"
    msg1: .string "Enter a number: "
    msg2: .string "The result is: "

.text
################################################################################
  # You may write function here
  
  
 
################################################################################

__start:
  # Prints msg0
    addi a0, x0, 4
    la a1, msg0
    ecall

  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall

  # Reads an int
    addi a0, x0, 5
    ecall

################################################################################ 
  # Write your main function here. 
  # Input n is in a0. You should store the result T(n) into t0
  # HW1-1 T(n) = 2T(n/2) + 8n + 5, T(1) = 4, round down the result of division
  # ex. addi t0, a0, 1  

  # main:
  #   addi x6, a0, -1  #x6 = n-1
  #   bge x6, x0, T_func  #if(n>=1), go to T_func
  #   jalr x0, 0(x1)  #retun 
  #   ecall
  
  jal x1, T_func
  addi x12, a0, 0 
  
  T_func:
    addi sp, sp, -8
    sw x1, 4(sp)  #save address
    sw a0, 0(sp) #save n
    addi x6, a0, -2  #x6 = n-1
    bge x6, x0, Loop  #if (n>1), go to Loop
    addi t0, x0, 4  #T(1)=4, save result in t0
    addi sp, sp, 8 #pop stack
    jalr x0, 0(x1)  #return

  Loop:
    srli a0, a0, 1 #n=n//2
    jal x1, T_func  #jump to T_func ie. call T(n//2)
    lw a0, 0(sp) #load n
    lw x1, 4(sp)  #load ra
    addi sp, sp, 8 #pop stack
    slli t0, t0, 1  #T(n//2) * 2
    slli x7, a0, 3  #n * 8
    add t0, t0, x7 #t0 = 2T(n//2) + 8n
    addi t0, t0, 5 #t0 = 2T(n//2) + 8n + 5
    beq a0, x12, result
    jalr x0, 0(x1)  #return



################################################################################
result:


  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall

  # Prints the result in t0
    addi a0, x0, 1
    add a1, x0, t0
    ecall
    
  # Ends the program with status code 0
    addi a0, x0, 10
    ecall

