08007764 <PID_controller>:
 8007764:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007766:	46de      	mov	lr, fp
 8007768:	4657      	mov	r7, sl
 800776a:	464e      	mov	r6, r9
 800776c:	4645      	mov	r5, r8
 800776e:	b5e0      	push	{r5, r6, r7, lr}
 8007770:	2900      	cmp	r1, #0
 8007772:	dc0a      	bgt.n	800778a <PID_controller+0x26>
 8007774:	d005      	beq.n	8007782 <PID_controller+0x1e>
 8007776:	1c4b      	adds	r3, r1, #1
 8007778:	db00      	blt.n	800777c <PID_controller+0x18>
 800777a:	e0e9      	b.n	8007950 <PID_controller+0x1ec>
 800777c:	20e0      	movs	r0, #224	; 0xe0
 800777e:	0600      	lsls	r0, r0, #24
 8007780:	e005      	b.n	800778e <PID_controller+0x2a>
 8007782:	2380      	movs	r3, #128	; 0x80
 8007784:	059b      	lsls	r3, r3, #22
 8007786:	4298      	cmp	r0, r3
 8007788:	d901      	bls.n	800778e <PID_controller+0x2a>
 800778a:	2080      	movs	r0, #128	; 0x80
 800778c:	0580      	lsls	r0, r0, #22
 800778e:	4b79      	ldr	r3, [pc, #484]	; (8007974 <PID_controller+0x210>)
 8007790:	681a      	ldr	r2, [r3, #0]
 8007792:	4282      	cmp	r2, r0
 8007794:	dd00      	ble.n	8007798 <PID_controller+0x34>
 8007796:	6018      	str	r0, [r3, #0]
 8007798:	4b77      	ldr	r3, [pc, #476]	; (8007978 <PID_controller+0x214>)
 800779a:	681a      	ldr	r2, [r3, #0]
 800779c:	4282      	cmp	r2, r0
 800779e:	da00      	bge.n	80077a2 <PID_controller+0x3e>
 80077a0:	6018      	str	r0, [r3, #0]
 80077a2:	4b76      	ldr	r3, [pc, #472]	; (800797c <PID_controller+0x218>)
 80077a4:	4c76      	ldr	r4, [pc, #472]	; (8007980 <PID_controller+0x21c>)
 80077a6:	681a      	ldr	r2, [r3, #0]
 80077a8:	2a00      	cmp	r2, #0
 80077aa:	d010      	beq.n	80077ce <PID_controller+0x6a>
 80077ac:	4975      	ldr	r1, [pc, #468]	; (8007984 <PID_controller+0x220>)
 80077ae:	6823      	ldr	r3, [r4, #0]
 80077b0:	680d      	ldr	r5, [r1, #0]
 80077b2:	1ac3      	subs	r3, r0, r3
 80077b4:	429d      	cmp	r5, r3
 80077b6:	db00      	blt.n	80077ba <PID_controller+0x56>
 80077b8:	e0af      	b.n	800791a <PID_controller+0x1b6>
 80077ba:	4e73      	ldr	r6, [pc, #460]	; (8007988 <PID_controller+0x224>)
 80077bc:	6833      	ldr	r3, [r6, #0]
 80077be:	0159      	lsls	r1, r3, #5
 80077c0:	1acb      	subs	r3, r1, r3
 80077c2:	d500      	bpl.n	80077c6 <PID_controller+0x62>
 80077c4:	e0cf      	b.n	8007966 <PID_controller+0x202>
 80077c6:	115b      	asrs	r3, r3, #5
 80077c8:	195b      	adds	r3, r3, r5
 80077ca:	435a      	muls	r2, r3
 80077cc:	6033      	str	r3, [r6, #0]
 80077ce:	4b6f      	ldr	r3, [pc, #444]	; (800798c <PID_controller+0x228>)
 80077d0:	6020      	str	r0, [r4, #0]
 80077d2:	6819      	ldr	r1, [r3, #0]
 80077d4:	424b      	negs	r3, r1
 80077d6:	4283      	cmp	r3, r0
 80077d8:	dc03      	bgt.n	80077e2 <PID_controller+0x7e>
 80077da:	000b      	movs	r3, r1
 80077dc:	4281      	cmp	r1, r0
 80077de:	dd00      	ble.n	80077e2 <PID_controller+0x7e>
 80077e0:	e0a7      	b.n	8007932 <PID_controller+0x1ce>
 80077e2:	496b      	ldr	r1, [pc, #428]	; (8007990 <PID_controller+0x22c>)
 80077e4:	680e      	ldr	r6, [r1, #0]
 80077e6:	2e00      	cmp	r6, #0
 80077e8:	d010      	beq.n	800780c <PID_controller+0xa8>
 80077ea:	486a      	ldr	r0, [pc, #424]	; (8007994 <PID_controller+0x230>)
 80077ec:	0031      	movs	r1, r6
 80077ee:	4684      	mov	ip, r0
 80077f0:	6800      	ldr	r0, [r0, #0]
 80077f2:	4359      	muls	r1, r3
 80077f4:	4680      	mov	r8, r0
 80077f6:	4660      	mov	r0, ip
 80077f8:	4441      	add	r1, r8
 80077fa:	6001      	str	r1, [r0, #0]
 80077fc:	4866      	ldr	r0, [pc, #408]	; (8007998 <PID_controller+0x234>)
 80077fe:	6806      	ldr	r6, [r0, #0]
 8007800:	42b1      	cmp	r1, r6
 8007802:	dc00      	bgt.n	8007806 <PID_controller+0xa2>
 8007804:	e097      	b.n	8007936 <PID_controller+0x1d2>
 8007806:	4661      	mov	r1, ip
 8007808:	600e      	str	r6, [r1, #0]
 800780a:	e002      	b.n	8007812 <PID_controller+0xae>
 800780c:	4961      	ldr	r1, [pc, #388]	; (8007994 <PID_controller+0x230>)
 800780e:	468c      	mov	ip, r1
 8007810:	600e      	str	r6, [r1, #0]
 8007812:	2080      	movs	r0, #128	; 0x80
 8007814:	00c0      	lsls	r0, r0, #3
 8007816:	4680      	mov	r8, r0
 8007818:	4960      	ldr	r1, [pc, #384]	; (800799c <PID_controller+0x238>)
 800781a:	4861      	ldr	r0, [pc, #388]	; (80079a0 <PID_controller+0x23c>)
 800781c:	6809      	ldr	r1, [r1, #0]
 800781e:	6800      	ldr	r0, [r0, #0]
 8007820:	4359      	muls	r1, r3
 8007822:	198f      	adds	r7, r1, r6
 8007824:	18bf      	adds	r7, r7, r2
 8007826:	4447      	add	r7, r8
 8007828:	468b      	mov	fp, r1
 800782a:	12ff      	asrs	r7, r7, #11
 800782c:	42b8      	cmp	r0, r7
 800782e:	da22      	bge.n	8007876 <PID_controller+0x112>
 8007830:	0034      	movs	r4, r6
 8007832:	17f5      	asrs	r5, r6, #31
 8007834:	1a3e      	subs	r6, r7, r0
 8007836:	46b0      	mov	r8, r6
 8007838:	17f6      	asrs	r6, r6, #31
 800783a:	46b2      	mov	sl, r6
 800783c:	4651      	mov	r1, sl
 800783e:	4646      	mov	r6, r8
 8007840:	02cf      	lsls	r7, r1, #11
 8007842:	4641      	mov	r1, r8
 8007844:	0d76      	lsrs	r6, r6, #21
 8007846:	4337      	orrs	r7, r6
 8007848:	02ce      	lsls	r6, r1, #11
 800784a:	1ba4      	subs	r4, r4, r6
 800784c:	41bd      	sbcs	r5, r7
 800784e:	4e52      	ldr	r6, [pc, #328]	; (8007998 <PID_controller+0x234>)
 8007850:	6836      	ldr	r6, [r6, #0]
 8007852:	4276      	negs	r6, r6
 8007854:	17f7      	asrs	r7, r6, #31
 8007856:	42bd      	cmp	r5, r7
 8007858:	dc00      	bgt.n	800785c <PID_controller+0xf8>
 800785a:	e072      	b.n	8007942 <PID_controller+0x1de>
 800785c:	0026      	movs	r6, r4
 800785e:	4661      	mov	r1, ip
 8007860:	600e      	str	r6, [r1, #0]
 8007862:	4c50      	ldr	r4, [pc, #320]	; (80079a4 <PID_controller+0x240>)
 8007864:	7824      	ldrb	r4, [r4, #0]
 8007866:	2c03      	cmp	r4, #3
 8007868:	d024      	beq.n	80078b4 <PID_controller+0x150>
 800786a:	bcf0      	pop	{r4, r5, r6, r7}
 800786c:	46bb      	mov	fp, r7
 800786e:	46b2      	mov	sl, r6
 8007870:	46a9      	mov	r9, r5
 8007872:	46a0      	mov	r8, r4
 8007874:	bdf0      	pop	{r4, r5, r6, r7, pc}
 8007876:	4241      	negs	r1, r0
 8007878:	4688      	mov	r8, r1
 800787a:	42b9      	cmp	r1, r7
 800787c:	dd71      	ble.n	8007962 <PID_controller+0x1fe>
 800787e:	19c0      	adds	r0, r0, r7
 8007880:	17c1      	asrs	r1, r0, #31
 8007882:	468a      	mov	sl, r1
 8007884:	0d41      	lsrs	r1, r0, #21
 8007886:	4689      	mov	r9, r1
 8007888:	4651      	mov	r1, sl
 800788a:	02cf      	lsls	r7, r1, #11
 800788c:	4649      	mov	r1, r9
 800788e:	0034      	movs	r4, r6
 8007890:	17f5      	asrs	r5, r6, #31
 8007892:	430f      	orrs	r7, r1
 8007894:	02c6      	lsls	r6, r0, #11
 8007896:	1ba4      	subs	r4, r4, r6
 8007898:	41bd      	sbcs	r5, r7
 800789a:	483f      	ldr	r0, [pc, #252]	; (8007998 <PID_controller+0x234>)
 800789c:	6806      	ldr	r6, [r0, #0]
 800789e:	17f0      	asrs	r0, r6, #31
 80078a0:	42a8      	cmp	r0, r5
 80078a2:	dd62      	ble.n	800796a <PID_controller+0x206>
 80078a4:	0026      	movs	r6, r4
 80078a6:	4661      	mov	r1, ip
 80078a8:	4c3e      	ldr	r4, [pc, #248]	; (80079a4 <PID_controller+0x240>)
 80078aa:	4640      	mov	r0, r8
 80078ac:	7824      	ldrb	r4, [r4, #0]
 80078ae:	600e      	str	r6, [r1, #0]
 80078b0:	2c03      	cmp	r4, #3
 80078b2:	d1da      	bne.n	800786a <PID_controller+0x106>
 80078b4:	4d3c      	ldr	r5, [pc, #240]	; (80079a8 <PID_controller+0x244>)
 80078b6:	782c      	ldrb	r4, [r5, #0]
 80078b8:	2c00      	cmp	r4, #0
 80078ba:	d1d6      	bne.n	800786a <PID_controller+0x106>
 80078bc:	4c3b      	ldr	r4, [pc, #236]	; (80079ac <PID_controller+0x248>)
 80078be:	0a1f      	lsrs	r7, r3, #8
 80078c0:	6824      	ldr	r4, [r4, #0]
 80078c2:	7023      	strb	r3, [r4, #0]
 80078c4:	7067      	strb	r7, [r4, #1]
 80078c6:	0c1f      	lsrs	r7, r3, #16
 80078c8:	0e1b      	lsrs	r3, r3, #24
 80078ca:	70e3      	strb	r3, [r4, #3]
 80078cc:	465b      	mov	r3, fp
 80078ce:	7123      	strb	r3, [r4, #4]
 80078d0:	0a1b      	lsrs	r3, r3, #8
 80078d2:	7163      	strb	r3, [r4, #5]
 80078d4:	465b      	mov	r3, fp
 80078d6:	0c1b      	lsrs	r3, r3, #16
 80078d8:	71a3      	strb	r3, [r4, #6]
 80078da:	465b      	mov	r3, fp
 80078dc:	0e19      	lsrs	r1, r3, #24
 80078de:	0a33      	lsrs	r3, r6, #8
 80078e0:	7263      	strb	r3, [r4, #9]
 80078e2:	0c33      	lsrs	r3, r6, #16
 80078e4:	72a3      	strb	r3, [r4, #10]
 80078e6:	0a13      	lsrs	r3, r2, #8
 80078e8:	7322      	strb	r2, [r4, #12]
 80078ea:	7363      	strb	r3, [r4, #13]
 80078ec:	0c13      	lsrs	r3, r2, #16
 80078ee:	0e12      	lsrs	r2, r2, #24
 80078f0:	73e2      	strb	r2, [r4, #15]
 80078f2:	2214      	movs	r2, #20
 80078f4:	73a3      	strb	r3, [r4, #14]
 80078f6:	0a03      	lsrs	r3, r0, #8
 80078f8:	7463      	strb	r3, [r4, #17]
 80078fa:	0203      	lsls	r3, r0, #8
 80078fc:	0e1b      	lsrs	r3, r3, #24
 80078fe:	74a3      	strb	r3, [r4, #18]
 8007900:	0e03      	lsrs	r3, r0, #24
 8007902:	74e3      	strb	r3, [r4, #19]
 8007904:	4b2a      	ldr	r3, [pc, #168]	; (80079b0 <PID_controller+0x24c>)
 8007906:	7226      	strb	r6, [r4, #8]
 8007908:	801a      	strh	r2, [r3, #0]
 800790a:	2304      	movs	r3, #4
 800790c:	0e36      	lsrs	r6, r6, #24
 800790e:	70a7      	strb	r7, [r4, #2]
 8007910:	71e1      	strb	r1, [r4, #7]
 8007912:	72e6      	strb	r6, [r4, #11]
 8007914:	7420      	strb	r0, [r4, #16]
 8007916:	702b      	strb	r3, [r5, #0]
 8007918:	e7a7      	b.n	800786a <PID_controller+0x106>
 800791a:	426d      	negs	r5, r5
 800791c:	429d      	cmp	r5, r3
 800791e:	db00      	blt.n	8007922 <PID_controller+0x1be>
 8007920:	e74b      	b.n	80077ba <PID_controller+0x56>
 8007922:	4e19      	ldr	r6, [pc, #100]	; (8007988 <PID_controller+0x224>)
 8007924:	001d      	movs	r5, r3
 8007926:	6833      	ldr	r3, [r6, #0]
 8007928:	0159      	lsls	r1, r3, #5
 800792a:	1acb      	subs	r3, r1, r3
 800792c:	d400      	bmi.n	8007930 <PID_controller+0x1cc>
 800792e:	e74a      	b.n	80077c6 <PID_controller+0x62>
 8007930:	e019      	b.n	8007966 <PID_controller+0x202>
 8007932:	0003      	movs	r3, r0
 8007934:	e755      	b.n	80077e2 <PID_controller+0x7e>
 8007936:	4276      	negs	r6, r6
 8007938:	42b1      	cmp	r1, r6
 800793a:	da00      	bge.n	800793e <PID_controller+0x1da>
 800793c:	e763      	b.n	8007806 <PID_controller+0xa2>
 800793e:	000e      	movs	r6, r1
 8007940:	e767      	b.n	8007812 <PID_controller+0xae>
 8007942:	42bd      	cmp	r5, r7
 8007944:	d000      	beq.n	8007948 <PID_controller+0x1e4>
 8007946:	e78a      	b.n	800785e <PID_controller+0xfa>
 8007948:	42a6      	cmp	r6, r4
 800794a:	d300      	bcc.n	800794e <PID_controller+0x1ea>
 800794c:	e787      	b.n	800785e <PID_controller+0xfa>
 800794e:	e785      	b.n	800785c <PID_controller+0xf8>
 8007950:	1c4b      	adds	r3, r1, #1
 8007952:	d000      	beq.n	8007956 <PID_controller+0x1f2>
 8007954:	e71b      	b.n	800778e <PID_controller+0x2a>
 8007956:	23e0      	movs	r3, #224	; 0xe0
 8007958:	061b      	lsls	r3, r3, #24
 800795a:	4298      	cmp	r0, r3
 800795c:	d200      	bcs.n	8007960 <PID_controller+0x1fc>
 800795e:	e70d      	b.n	800777c <PID_controller+0x18>
 8007960:	e715      	b.n	800778e <PID_controller+0x2a>
 8007962:	0038      	movs	r0, r7
 8007964:	e77d      	b.n	8007862 <PID_controller+0xfe>
 8007966:	331f      	adds	r3, #31
 8007968:	e72d      	b.n	80077c6 <PID_controller+0x62>
 800796a:	42a8      	cmp	r0, r5
 800796c:	d19b      	bne.n	80078a6 <PID_controller+0x142>
 800796e:	42a6      	cmp	r6, r4
 8007970:	d999      	bls.n	80078a6 <PID_controller+0x142>
 8007972:	e797      	b.n	80078a4 <PID_controller+0x140>
 8007974:	20000060 	.word	0x20000060
 8007978:	2000003c 	.word	0x2000003c
 800797c:	20000aec 	.word	0x20000aec
 8007980:	20000d8c 	.word	0x20000d8c
 8007984:	20000b5c 	.word	0x20000b5c
 8007988:	20000b54 	.word	0x20000b54
 800798c:	20000b58 	.word	0x20000b58
 8007990:	20000b4c 	.word	0x20000b4c
 8007994:	20000b50 	.word	0x20000b50
 8007998:	20000b60 	.word	0x20000b60
 800799c:	20000d90 	.word	0x20000d90
 80079a0:	20000d1c 	.word	0x20000d1c
 80079a4:	20000d9c 	.word	0x20000d9c
 80079a8:	20000d0e 	.word	0x20000d0e
 80079ac:	20000068 	.word	0x20000068
 80079b0:	20000d0c 	.word	0x20000d0c

