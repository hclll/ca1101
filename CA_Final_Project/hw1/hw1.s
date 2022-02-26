.data
    n: .word 10
    
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
    addi sp,sp,-16
    sw   a0,0(sp)
    sw   x1,8(sp)

                 # a0 = n
                 # a1 = return value
    li  a5,1     # a5 = constant 1

    beq     a0,a5,eq1
    srai    a0,a0,1 # n/2
    jal     x1,FUNCTION
    
    lw      a0,0(sp) # restore n
    slli    a1,a1,1 # 2T(n/2)
    
    slli    a2,a0,3 # 8n
    add     a1,a1,a2
    addi    a1,a1,5
    jal x0,end

eq1:
    li a1,4
    jal x0,end
    
end:
    lw      a0,0(sp)
    lw      x1,8(sp)
    addi    sp,sp,16
    mv      a0,a1 
    jalr    x0,0(x1)
       

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall