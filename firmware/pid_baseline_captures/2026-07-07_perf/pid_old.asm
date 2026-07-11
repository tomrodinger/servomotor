08007444 <PID_controller>:
 8007444:	4b3f      	ldr	r3, [pc, #252]	; (8007544 <PID_controller+0x100>)
 8007446:	b5f0      	push	{r4, r5, r6, r7, lr}
 8007448:	681a      	ldr	r2, [r3, #0]
 800744a:	4282      	cmp	r2, r0
 800744c:	dd00      	ble.n	8007450 <PID_controller+0xc>
 800744e:	6018      	str	r0, [r3, #0]
 8007450:	4b3d      	ldr	r3, [pc, #244]	; (8007548 <PID_controller+0x104>)
 8007452:	681a      	ldr	r2, [r3, #0]
 8007454:	4282      	cmp	r2, r0
 8007456:	da00      	bge.n	800745a <PID_controller+0x16>
 8007458:	6018      	str	r0, [r3, #0]
 800745a:	4b3c      	ldr	r3, [pc, #240]	; (800754c <PID_controller+0x108>)
 800745c:	681c      	ldr	r4, [r3, #0]
 800745e:	4b3c      	ldr	r3, [pc, #240]	; (8007550 <PID_controller+0x10c>)
 8007460:	2c00      	cmp	r4, #0
 8007462:	d00e      	beq.n	8007482 <PID_controller+0x3e>
 8007464:	493b      	ldr	r1, [pc, #236]	; (8007554 <PID_controller+0x110>)
 8007466:	681a      	ldr	r2, [r3, #0]
 8007468:	6809      	ldr	r1, [r1, #0]
 800746a:	1a82      	subs	r2, r0, r2
 800746c:	4291      	cmp	r1, r2
 800746e:	da2f      	bge.n	80074d0 <PID_controller+0x8c>
 8007470:	4d39      	ldr	r5, [pc, #228]	; (8007558 <PID_controller+0x114>)
 8007472:	682a      	ldr	r2, [r5, #0]
 8007474:	682e      	ldr	r6, [r5, #0]
 8007476:	0152      	lsls	r2, r2, #5
 8007478:	1b92      	subs	r2, r2, r6
 800747a:	1152      	asrs	r2, r2, #5
 800747c:	1852      	adds	r2, r2, r1
 800747e:	4354      	muls	r4, r2
 8007480:	602a      	str	r2, [r5, #0]
 8007482:	6018      	str	r0, [r3, #0]
 8007484:	4b35      	ldr	r3, [pc, #212]	; (800755c <PID_controller+0x118>)
 8007486:	681b      	ldr	r3, [r3, #0]
 8007488:	425a      	negs	r2, r3
 800748a:	4282      	cmp	r2, r0
 800748c:	dc02      	bgt.n	8007494 <PID_controller+0x50>
 800748e:	001a      	movs	r2, r3
 8007490:	4283      	cmp	r3, r0
 8007492:	dc1b      	bgt.n	80074cc <PID_controller+0x88>
 8007494:	4b32      	ldr	r3, [pc, #200]	; (8007560 <PID_controller+0x11c>)
 8007496:	681b      	ldr	r3, [r3, #0]
 8007498:	2b00      	cmp	r3, #0
 800749a:	d00a      	beq.n	80074b2 <PID_controller+0x6e>
 800749c:	4353      	muls	r3, r2
 800749e:	4831      	ldr	r0, [pc, #196]	; (8007564 <PID_controller+0x120>)
 80074a0:	6801      	ldr	r1, [r0, #0]
 80074a2:	1859      	adds	r1, r3, r1
 80074a4:	4b30      	ldr	r3, [pc, #192]	; (8007568 <PID_controller+0x124>)
 80074a6:	6001      	str	r1, [r0, #0]
 80074a8:	681b      	ldr	r3, [r3, #0]
 80074aa:	4299      	cmp	r1, r3
 80074ac:	dd15      	ble.n	80074da <PID_controller+0x96>
 80074ae:	6003      	str	r3, [r0, #0]
 80074b0:	e001      	b.n	80074b6 <PID_controller+0x72>
 80074b2:	492c      	ldr	r1, [pc, #176]	; (8007564 <PID_controller+0x120>)
 80074b4:	600b      	str	r3, [r1, #0]
 80074b6:	492d      	ldr	r1, [pc, #180]	; (800756c <PID_controller+0x128>)
 80074b8:	4d2d      	ldr	r5, [pc, #180]	; (8007570 <PID_controller+0x12c>)
 80074ba:	6809      	ldr	r1, [r1, #0]
 80074bc:	782d      	ldrb	r5, [r5, #0]
 80074be:	4351      	muls	r1, r2
 80074c0:	18c8      	adds	r0, r1, r3
 80074c2:	1900      	adds	r0, r0, r4
 80074c4:	12c0      	asrs	r0, r0, #11
 80074c6:	2d03      	cmp	r5, #3
 80074c8:	d00c      	beq.n	80074e4 <PID_controller+0xa0>
 80074ca:	bdf0      	pop	{r4, r5, r6, r7, pc}
 80074cc:	0002      	movs	r2, r0
 80074ce:	e7e1      	b.n	8007494 <PID_controller+0x50>
 80074d0:	4249      	negs	r1, r1
 80074d2:	4291      	cmp	r1, r2
 80074d4:	dacc      	bge.n	8007470 <PID_controller+0x2c>
 80074d6:	0011      	movs	r1, r2
 80074d8:	e7ca      	b.n	8007470 <PID_controller+0x2c>
 80074da:	425b      	negs	r3, r3
 80074dc:	4299      	cmp	r1, r3
 80074de:	dbe6      	blt.n	80074ae <PID_controller+0x6a>
 80074e0:	000b      	movs	r3, r1
 80074e2:	e7e8      	b.n	80074b6 <PID_controller+0x72>
 80074e4:	4e23      	ldr	r6, [pc, #140]	; (8007574 <PID_controller+0x130>)
 80074e6:	7835      	ldrb	r5, [r6, #0]
 80074e8:	2d00      	cmp	r5, #0
 80074ea:	d1ee      	bne.n	80074ca <PID_controller+0x86>
 80074ec:	4d22      	ldr	r5, [pc, #136]	; (8007578 <PID_controller+0x134>)
 80074ee:	0a17      	lsrs	r7, r2, #8
 80074f0:	682d      	ldr	r5, [r5, #0]
 80074f2:	702a      	strb	r2, [r5, #0]
 80074f4:	706f      	strb	r7, [r5, #1]
 80074f6:	0c17      	lsrs	r7, r2, #16
 80074f8:	0e12      	lsrs	r2, r2, #24
 80074fa:	70ea      	strb	r2, [r5, #3]
 80074fc:	0a0a      	lsrs	r2, r1, #8
 80074fe:	716a      	strb	r2, [r5, #5]
 8007500:	0c0a      	lsrs	r2, r1, #16
 8007502:	71aa      	strb	r2, [r5, #6]
 8007504:	0a1a      	lsrs	r2, r3, #8
 8007506:	726a      	strb	r2, [r5, #9]
 8007508:	0c1a      	lsrs	r2, r3, #16
 800750a:	72aa      	strb	r2, [r5, #10]
 800750c:	2214      	movs	r2, #20
 800750e:	722b      	strb	r3, [r5, #8]
 8007510:	0e1b      	lsrs	r3, r3, #24
 8007512:	72eb      	strb	r3, [r5, #11]
 8007514:	0a23      	lsrs	r3, r4, #8
 8007516:	736b      	strb	r3, [r5, #13]
 8007518:	0c23      	lsrs	r3, r4, #16
 800751a:	73ab      	strb	r3, [r5, #14]
 800751c:	0a03      	lsrs	r3, r0, #8
 800751e:	746b      	strb	r3, [r5, #17]
 8007520:	0203      	lsls	r3, r0, #8
 8007522:	0e1b      	lsrs	r3, r3, #24
 8007524:	74ab      	strb	r3, [r5, #18]
 8007526:	0e03      	lsrs	r3, r0, #24
 8007528:	74eb      	strb	r3, [r5, #19]
 800752a:	4b14      	ldr	r3, [pc, #80]	; (800757c <PID_controller+0x138>)
 800752c:	7129      	strb	r1, [r5, #4]
 800752e:	801a      	strh	r2, [r3, #0]
 8007530:	2304      	movs	r3, #4
 8007532:	0e09      	lsrs	r1, r1, #24
 8007534:	732c      	strb	r4, [r5, #12]
 8007536:	0e24      	lsrs	r4, r4, #24
 8007538:	70af      	strb	r7, [r5, #2]
 800753a:	71e9      	strb	r1, [r5, #7]
 800753c:	73ec      	strb	r4, [r5, #15]
 800753e:	7428      	strb	r0, [r5, #16]
 8007540:	7033      	strb	r3, [r6, #0]
 8007542:	e7c2      	b.n	80074ca <PID_controller+0x86>
 8007544:	20000060 	.word	0x20000060
 8007548:	2000003c 	.word	0x2000003c
 800754c:	20000aec 	.word	0x20000aec
 8007550:	20000d84 	.word	0x20000d84
 8007554:	20000b5c 	.word	0x20000b5c
 8007558:	20000b54 	.word	0x20000b54
 800755c:	20000b58 	.word	0x20000b58
 8007560:	20000b4c 	.word	0x20000b4c
 8007564:	20000b50 	.word	0x20000b50
 8007568:	20000b60 	.word	0x20000b60
 800756c:	20000d88 	.word	0x20000d88
 8007570:	20000d94 	.word	0x20000d94
 8007574:	20000d0e 	.word	0x20000d0e
 8007578:	20000068 	.word	0x20000068
 800757c:	20000d0c 	.word	0x20000d0c

