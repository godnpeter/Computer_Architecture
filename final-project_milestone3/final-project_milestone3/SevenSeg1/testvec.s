.text
.align 4

	la  $sp, stack    # load address
	nop
	nop
	nop
	nop
	j	SevenSeg
	nop
	nop
	nop
	nop


.data
.align 4
stack:
	.space 1024
