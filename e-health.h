/*
 * Copyright (c) 2016, Zolertia - http://www.zolertia.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/**
 * \author Antonio Lignan <alinan@zolertia.com>
 */

#ifndef EHEALTH_
#define EHEALTH_
/*---------------------------------------------------------------------------*/
/* This is the UDP port used to send and receive data */
#define UDP_CLIENT_PORT   8765
#define UDP_SERVER_PORT   5678
/*---------------------------------------------------------------------------*/
#define SLS_USING_HW	1

#if (SLS_USING_HW==0)
#define SLS_USING_CC2650
#endif

#if (SLS_USING_HW==1)
#define SLS_USING_CC2538DK
#endif

#define	SFD 	0x7E		/* Start of E-HEALTH frame Delimitter */
/*---------------------------------------------------------------------------*/
/* This data structure is used to store the packet content (payload) */
struct my_msg_t {
  uint16_t  sfd;
  uint16_t counter;
  uint16_t type;
  uint16_t temper1;
  uint16_t temper2;
  uint16_t spo2bpm;
  uint16_t systolic;
  uint16_t diastolic;
  uint16_t spo2;
  
};
/*---------------------------------------------------------------------------*/
#endif /* __EHEALTH__ */

