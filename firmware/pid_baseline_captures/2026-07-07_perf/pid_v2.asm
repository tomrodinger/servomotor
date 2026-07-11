08007764 <PID_controller>:
 8007764:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007766:	2900      	cmp	r1, #0
 8007768:	dc0a      	bgt.n	8007780 <PID_controller+0x1c>
 800776a:	d005      	beq.n	8007778 <PID_controller+0x14>
 800776c:	1c4b      	adds	r3, r1, #1
 800776e:	db00      	blt.n	8007772 <PID_controller+0xe>
 8007770:	e0b1      	b.n	80078d6 <PID_controller+0x172>
 8007772:	20e0      	movs	r0, #224	; 0xe0
 8007774:	0600      	lsls	r0, r0, #24
 8007776:	e005      	b.n	8007784 <PID_controller+0x20>
 8007778:	2380      	movs	r3, #128	; 0x80
 800777a:	059b      	lsls	r3, r3, #22
 800777c:	4298      	cmp	r0, r3
 800777e:	d901      	bls.n	8007784 <PID_controller+0x20>
 8007780:	2080      	movs	r0, #128	; 0x80
 8007782:	0580      	lsls	r0, r0, #22
 8007784:	4b5f      	ldr	r3, [pc, #380]	; (8007904 <PID_controller+0x1a0>)
 8007786:	681a      	ldr	r2, [r3, #0]
 8007788:	4282      	cmp	r2, r0
 800778a:	dd00      	ble.n	800778e <PID_controller+0x2a>
 800778c:	6018      	str	r0, [r3, #0]
 800778e:	4b5e      	ldr	r3, [pc, #376]	; (8007908 <PID_controller+0x1a4>)
 8007790:	681a      	ldr	r2, [r3, #0]
 8007792:	4282      	cmp	r2, r0
 8007794:	da00      	bge.n	8007798 <PID_controller+0x34>
 8007796:	6018      	str	r0, [r3, #0]
 8007798:	4b5c      	ldr	r3, [pc, #368]	; (800790c <PID_controller+0x1a8>)
 800779a:	495d      	ldr	r1, [pc, #372]	; (8007910 <PID_controller+0x1ac>)
 800779c:	681d      	ldr	r5, [r3, #0]
 800779e:	2d00      	cmp	r5, #0
 80077a0:	d010      	beq.n	80077c4 <PID_controller+0x60>
 80077a2:	4a5c      	ldr	r2, [pc, #368]	; (8007914 <PID_controller+0x1b0>)
 80077a4:	680b      	ldr	r3, [r1, #0]
 80077a6:	6814      	ldr	r4, [r2, #0]
 80077a8:	1ac3      	subs	r3, r0, r3
 80077aa:	429c      	cmp	r4, r3
 80077ac:	db00      	blt.n	80077b0 <PID_controller+0x4c>
 80077ae:	e086      	b.n	80078be <PID_controller+0x15a>
 80077b0:	4e59      	ldr	r6, [pc, #356]	; (8007918 <PID_controller+0x1b4>)
 80077b2:	6833      	ldr	r3, [r6, #0]
 80077b4:	015a      	lsls	r2, r3, #5
 80077b6:	1ad3      	subs	r3, r2, r3
 80077b8:	d500      	bpl.n	80077bc <PID_controller+0x58>
 80077ba:	e09d      	b.n	80078f8 <PID_controller+0x194>
 80077bc:	115b      	asrs	r3, r3, #5
 80077be:	191b      	adds	r3, r3, r4
 80077c0:	435d      	muls	r5, r3
 80077c2:	6033      	str	r3, [r6, #0]
 80077c4:	4b55      	ldr	r3, [pc, #340]	; (800791c <PID_controller+0x1b8>)
 80077c6:	6008      	str	r0, [r1, #0]
 80077c8:	681b      	ldr	r3, [r3, #0]
 80077ca:	425c      	negs	r4, r3
 80077cc:	4284      	cmp	r4, r0
 80077ce:	dc03      	bgt.n	80077d8 <PID_controller+0x74>
 80077d0:	001c      	movs	r4, r3
 80077d2:	4283      	cmp	r3, r0
 80077d4:	dd00      	ble.n	80077d8 <PID_controller+0x74>
 80077d6:	e087      	b.n	80078e8 <PID_controller+0x184>
 80077d8:	4b51      	ldr	r3, [pc, #324]	; (8007920 <PID_controller+0x1bc>)
 80077da:	6819      	ldr	r1, [r3, #0]
 80077dc:	2900      	cmp	r1, #0
 80077de:	d068      	beq.n	80078b2 <PID_controller+0x14e>
 80077e0:	000b      	movs	r3, r1
 80077e2:	4f50      	ldr	r7, [pc, #320]	; (8007924 <PID_controller+0x1c0>)
 80077e4:	4363      	muls	r3, r4
 80077e6:	683a      	ldr	r2, [r7, #0]
 80077e8:	4694      	mov	ip, r2
 80077ea:	4a4f      	ldr	r2, [pc, #316]	; (8007928 <PID_controller+0x1c4>)
 80077ec:	4463      	add	r3, ip
 80077ee:	6811      	ldr	r1, [r2, #0]
 80077f0:	603b      	str	r3, [r7, #0]
 80077f2:	428b      	cmp	r3, r1
 80077f4:	dc00      	bgt.n	80077f8 <PID_controller+0x94>
 80077f6:	e079      	b.n	80078ec <PID_controller+0x188>
 80077f8:	6039      	str	r1, [r7, #0]
 80077fa:	2280      	movs	r2, #128	; 0x80
 80077fc:	00d2      	lsls	r2, r2, #3
 80077fe:	4694      	mov	ip, r2
 8007800:	4b4a      	ldr	r3, [pc, #296]	; (800792c <PID_controller+0x1c8>)
 8007802:	4a4b      	ldr	r2, [pc, #300]	; (8007930 <PID_controller+0x1cc>)
 8007804:	681e      	ldr	r6, [r3, #0]
 8007806:	6810      	ldr	r0, [r2, #0]
 8007808:	4366      	muls	r6, r4
 800780a:	1873      	adds	r3, r6, r1
 800780c:	195b      	adds	r3, r3, r5
 800780e:	4463      	add	r3, ip
 8007810:	12db      	asrs	r3, r3, #11
 8007812:	4298      	cmp	r0, r3
 8007814:	da0d      	bge.n	8007832 <PID_controller+0xce>
 8007816:	1a1a      	subs	r2, r3, r0
 8007818:	4b43      	ldr	r3, [pc, #268]	; (8007928 <PID_controller+0x1c4>)
 800781a:	02d2      	lsls	r2, r2, #11
 800781c:	1a8a      	subs	r2, r1, r2
 800781e:	6819      	ldr	r1, [r3, #0]
 8007820:	4249      	negs	r1, r1
 8007822:	4291      	cmp	r1, r2
 8007824:	db49      	blt.n	80078ba <PID_controller+0x156>
 8007826:	6039      	str	r1, [r7, #0]
 8007828:	4b42      	ldr	r3, [pc, #264]	; (8007934 <PID_controller+0x1d0>)
 800782a:	781b      	ldrb	r3, [r3, #0]
 800782c:	2b03      	cmp	r3, #3
 800782e:	d010      	beq.n	8007852 <PID_controller+0xee>
 8007830:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007832:	4242      	negs	r2, r0
 8007834:	429a      	cmp	r2, r3
 8007836:	dd3e      	ble.n	80078b6 <PID_controller+0x152>
 8007838:	18c3      	adds	r3, r0, r3
 800783a:	02db      	lsls	r3, r3, #11
 800783c:	1acb      	subs	r3, r1, r3
 800783e:	493a      	ldr	r1, [pc, #232]	; (8007928 <PID_controller+0x1c4>)
 8007840:	6809      	ldr	r1, [r1, #0]
 8007842:	4299      	cmp	r1, r3
 8007844:	dc5a      	bgt.n	80078fc <PID_controller+0x198>
 8007846:	0010      	movs	r0, r2
 8007848:	6039      	str	r1, [r7, #0]
 800784a:	4b3a      	ldr	r3, [pc, #232]	; (8007934 <PID_controller+0x1d0>)
 800784c:	781b      	ldrb	r3, [r3, #0]
 800784e:	2b03      	cmp	r3, #3
 8007850:	d1ee      	bne.n	8007830 <PID_controller+0xcc>
 8007852:	4a39      	ldr	r2, [pc, #228]	; (8007938 <PID_controller+0x1d4>)
 8007854:	7813      	ldrb	r3, [r2, #0]
 8007856:	2b00      	cmp	r3, #0
 8007858:	d1ea      	bne.n	8007830 <PID_controller+0xcc>
 800785a:	4b38      	ldr	r3, [pc, #224]	; (800793c <PID_controller+0x1d8>)
 800785c:	0a27      	lsrs	r7, r4, #8
 800785e:	681b      	ldr	r3, [r3, #0]
 8007860:	701c      	strb	r4, [r3, #0]
 8007862:	705f      	strb	r7, [r3, #1]
 8007864:	0c27      	lsrs	r7, r4, #16
 8007866:	0e24      	lsrs	r4, r4, #24
 8007868:	70dc      	strb	r4, [r3, #3]
 800786a:	0a34      	lsrs	r4, r6, #8
 800786c:	715c      	strb	r4, [r3, #5]
 800786e:	0c34      	lsrs	r4, r6, #16
 8007870:	719c      	strb	r4, [r3, #6]
 8007872:	0a0c      	lsrs	r4, r1, #8
 8007874:	7219      	strb	r1, [r3, #8]
 8007876:	725c      	strb	r4, [r3, #9]
 8007878:	0c0c      	lsrs	r4, r1, #16
 800787a:	0e09      	lsrs	r1, r1, #24
 800787c:	72d9      	strb	r1, [r3, #11]
 800787e:	0a29      	lsrs	r1, r5, #8
 8007880:	7359      	strb	r1, [r3, #13]
 8007882:	0c29      	lsrs	r1, r5, #16
 8007884:	7399      	strb	r1, [r3, #14]
 8007886:	0a01      	lsrs	r1, r0, #8
 8007888:	7459      	strb	r1, [r3, #17]
 800788a:	0201      	lsls	r1, r0, #8
 800788c:	0e09      	lsrs	r1, r1, #24
 800788e:	7499      	strb	r1, [r3, #18]
 8007890:	0e01      	lsrs	r1, r0, #24
 8007892:	74d9      	strb	r1, [r3, #19]
 8007894:	2114      	movs	r1, #20
 8007896:	711e      	strb	r6, [r3, #4]
 8007898:	731d      	strb	r5, [r3, #12]
 800789a:	0e36      	lsrs	r6, r6, #24
 800789c:	0e2d      	lsrs	r5, r5, #24
 800789e:	709f      	strb	r7, [r3, #2]
 80078a0:	71de      	strb	r6, [r3, #7]
 80078a2:	729c      	strb	r4, [r3, #10]
 80078a4:	73dd      	strb	r5, [r3, #15]
 80078a6:	7418      	strb	r0, [r3, #16]
 80078a8:	4b25      	ldr	r3, [pc, #148]	; (8007940 <PID_controller+0x1dc>)
 80078aa:	8019      	strh	r1, [r3, #0]
 80078ac:	2304      	movs	r3, #4
 80078ae:	7013      	strb	r3, [r2, #0]
 80078b0:	e7be      	b.n	8007830 <PID_controller+0xcc>
 80078b2:	4f1c      	ldr	r7, [pc, #112]	; (8007924 <PID_controller+0x1c0>)
 80078b4:	e7a0      	b.n	80077f8 <PID_controller+0x94>
 80078b6:	0018      	movs	r0, r3
 80078b8:	e7b6      	b.n	8007828 <PID_controller+0xc4>
 80078ba:	0011      	movs	r1, r2
 80078bc:	e7b3      	b.n	8007826 <PID_controller+0xc2>
 80078be:	4264      	negs	r4, r4
 80078c0:	429c      	cmp	r4, r3
 80078c2:	db00      	blt.n	80078c6 <PID_controller+0x162>
 80078c4:	e774      	b.n	80077b0 <PID_controller+0x4c>
 80078c6:	4e14      	ldr	r6, [pc, #80]	; (8007918 <PID_controller+0x1b4>)
 80078c8:	001c      	movs	r4, r3
 80078ca:	6833      	ldr	r3, [r6, #0]
 80078cc:	015a      	lsls	r2, r3, #5
 80078ce:	1ad3      	subs	r3, r2, r3
 80078d0:	d400      	bmi.n	80078d4 <PID_controller+0x170>
 80078d2:	e773      	b.n	80077bc <PID_controller+0x58>
 80078d4:	e010      	b.n	80078f8 <PID_controller+0x194>
 80078d6:	1c4b      	adds	r3, r1, #1
 80078d8:	d000      	beq.n	80078dc <PID_controller+0x178>
 80078da:	e753      	b.n	8007784 <PID_controller+0x20>
 80078dc:	23e0      	movs	r3, #224	; 0xe0
 80078de:	061b      	lsls	r3, r3, #24
 80078e0:	4298      	cmp	r0, r3
 80078e2:	d200      	bcs.n	80078e6 <PID_controller+0x182>
 80078e4:	e745      	b.n	8007772 <PID_controller+0xe>
 80078e6:	e74d      	b.n	8007784 <PID_controller+0x20>
 80078e8:	0004      	movs	r4, r0
 80078ea:	e775      	b.n	80077d8 <PID_controller+0x74>
 80078ec:	4249      	negs	r1, r1
 80078ee:	428b      	cmp	r3, r1
 80078f0:	da00      	bge.n	80078f4 <PID_controller+0x190>
 80078f2:	e781      	b.n	80077f8 <PID_controller+0x94>
 80078f4:	0019      	movs	r1, r3
 80078f6:	e780      	b.n	80077fa <PID_controller+0x96>
 80078f8:	331f      	adds	r3, #31
 80078fa:	e75f      	b.n	80077bc <PID_controller+0x58>
 80078fc:	0019      	movs	r1, r3
 80078fe:	0010      	movs	r0, r2
 8007900:	6039      	str	r1, [r7, #0]
 8007902:	e7a2      	b.n	800784a <PID_controller+0xe6>
 8007904:	20000060 	.word	0x20000060
 8007908:	2000003c 	.word	0x2000003c
 800790c:	20000aec 	.word	0x20000aec
 8007910:	20000d8c 	.word	0x20000d8c
 8007914:	20000b5c 	.word	0x20000b5c
 8007918:	20000b54 	.word	0x20000b54
 800791c:	20000b58 	.word	0x20000b58
 8007920:	20000b4c 	.word	0x20000b4c
 8007924:	20000b50 	.word	0x20000b50
 8007928:	20000b60 	.word	0x20000b60
 800792c:	20000d90 	.word	0x20000d90
 8007930:	20000d1c 	.word	0x20000d1c
 8007934:	20000d9c 	.word	0x20000d9c
 8007938:	20000d0e 	.word	0x20000d0e
 800793c:	20000068 	.word	0x20000068
 8007940:	20000d0c 	.word	0x20000d0c

