/*Main.c */
int main (void) 
{ 
/* SystemClockUpdate () updates the SystemFrequency variable */ 
SystemClockUpdate (); 
UARTInit (192000); /* baud rate setting */ 
SysTick_Config (SystemFrequency/1000000); 

LPC_PINCON->PINSEL4 &= ~ (0xFFFF); // 1111l112111l11111 
LPC_GPI02->FIODIRO |= 0xFF; // 11111111 | Init LCD pin 

LPC_PINCON->PINMODE0 &= ~(0xFF); // 11111111 
LPC_PINCON->PINSELO &= ~(0xFFFFFF); // 111111111l11l111l111l111 
LPC_GPIO0->FIODIRL |= OxFF0; // 11111l1l0000 

LPC_PINCON->PINMODE1 &= ~(OxFF); // 11111111 
LPC_PINCON->PINSEL1 &= ~ (0xFFFFFFFF); // 11111111111111111111111111111111 
LPC_GPIO0->FIODIRH &= ~(0x1F80); // 1111110000000 

lcd_init (); 
lcd_print ("I love you!!!"); 

unsigned char right, left, forword, backword; 
unsigned char pht_right, pht_left, pht_forword, pht_backword; 

LED_RED_OFF; 
LED_BLUE_OFF; 

delay us (1000000); 

MOTORIEN1; 
MOTOR2EN1; 

//init neural network 
int i; 
£loat a=0.1; 
for (i=0; i<4; i++) { 
  neo[i].w[0]=0; 
  neo[i].w[1] =0; 
  neo[i].w[2] =0; 
  neo[i].w[3]=0; 
  neo[i].ch=l; 
  neo[i].y=0;
}

unsigned int slowdownDelay = 1000000; 
int counter = 0; 

while (1) { 
if (UART3Count==0) {
  LPC_UART3->IER = IER THRE | IER RLS; 
  /* Disable RBR */ 
  if (LPC_UART3->RBR=='L') 
    left=l; 
  else if (LPC_UART3->RBR== 'F') 
    forword=1; 
  else if (LPC_UART3->RBR=='R') 
    right = 'R' 
  else if (LPC_UART3->RBR=='B')
    backword=1;
  LPC_UART3->IER = IER THRE | IER RLS | IER RBR; 
  /* Re-enable RBR */ 
} 

right = 'R' ? 1 : 0; 
left = 'L' ? 1: 0; 
forword = 'F' ? 1 : 0; 
backword = 'B' ? 1 : 0; 

pht_right = ( (LPC_GPIO0->FIOPIN & (1 << 25)) >> 25) ? 1: 0;
pht_left = ( (LPC_GPIO0->FIOPIN & (1 << 27) ) >> 27) ? 1: 0;
pht_forword = ((LPC_GPIO0->FIOPIN & (1 << 26)) >> 26) ? 1: 0;
pht_backword = ((LPC_GPIO0->FIOPIN & (1 << 28)) >>> 28) ? 1: 0;
if (pht_right == 0 || pht_left == 0 || pht_forword == 0 || pht_backword == 0){
  LED_RED_ON; 
}
else
  LED_RED_OFF; 

// Slowdown 
if (slowdownDelay == 0){
slowdownDelay = 1000000; 
int x [4], c[4]; 
x[0] = ~pht_right; 
x[1] = ~pht_left; 
x[2] = ~pht_forword; 
x[3] = ~pht_backword; 
for (i=0; i<4;i++)
Neuron (x, i);  
if (learning == 1) 
{
int j, d[4]: 
d[0] = right; 
d[1] = left; 
d[2] = forword; 
d[3) = backword; 
for (i=0; i<4;i++) 
  for (j=0;j<4;j++)
    neo[i].w[j]=neo[i].w[j] + a * (d[i] - neo[i].y) * x[j];
Counter++; 
int count; 
float E=0, El=l, Err; 
Count=counter; 
for (i=3; i>=0; i--){ 
  c[i]=count%10+0x30; 
  Count=count/10;
  lcd_clr(); 
  lcd_print(c);
  El=0;  
  for (i=0;i<4;it+) 
  {  
     E1 += pow(d[i]-neo[i].y,2); 
  }
  E+=E1;// /4 
  Err=sqrt(E); 
  if (Err<0.5){
     learning=0;
     lcd_clr();
     lcd_print("All OK!!!");
  }
  else 
    learning=1;
}
}
else
  slowdonDelay--;
if (neo[0].y == 1)
{
MOTORR_F1; 
MOTORR_BO; 
} 
else if (neo[1].y == 1)
{
MOTORL_Fl; 
MOTORR_Fl; 
MOTORL_BO; 
MOTORR_BO; 
}
else if (neo[2].y == 1) 
{
MOTORL_F1;
MOTORL_BO; 
}
else if (neo[3].y == 1) 
{
MOTORL_B1;
MOTORR_B1; 
MOTORL_FO; 
MOTORR_FO; 
LED_RED_ON;
}
else
{
MOTORL_F0; 
MOTORR_FO; 
MOTORL_B0; 
MOTORR_BO; 
}
}
return 0;
}

/*Robot.h */
#ifndef ROBOT _H_
#define ROBOT _H_
  
#define LED_RED_ON LPC_GPIO0->>FIOSETL |= (1 << 8); 
#define LED_RED_OFF LPC_GPIO0->FIOCLRL |= (1 << 8); 
#define LED_BLUE_ON LPC_GPIO0->FIOSETL |= (1 << 9); 
#define LED_BLUE_OFF LPC_GPIO0->FIOCLRL |= (1 << 9);
  
#define MOTORL_F1 LPC_GPIO0->FIOSETL |= (1 << 7); 
#define MOTORL_B1 LPC_GPIO0->FIOSETL |= (1 << 6): 
#define MOTORR_F1 LPC_GPIO0->FIOSETL |= (1 << 5); 
#define MOTORR_B1 LPC_GPIO0->FIOSETL |= (1 << 4); 
#define MOTORL_FO LPC_GPIO0->FIOCLRL |= (1 << 7); 
#define MOTORL_BO LPC_GPIO0->FIOCLRL |= (1 << 6); 
#define MOTORR_FO LPC_GPIO0->FIOCLRL |= (1 << 5); 
#define MOTORR_BO LPC_GPIO0->FIOCLRL |= (1 << 4); 
#define MOTOR1EN1 LPC_GPIO2->FIOSETL |= (1 << 6); 
#define MOTOR1ENO LPC_GPIO2->FIOCLRL |= (1 << 6); 
#define MOTOR2EN1 LPC_GPIO2->FIOSETL |= (1 << 7) ; 
#define MOTOR2ENO LPC_GPIO2->FIOCLRL |= (1 << 7); 


typedef struct {
float w[4]; 
int ch; 
int yi; 
}neuron; 
#endif /* ROBOT H*/ 

/*LCD.c */

#ifdef __USE_CMSIS
#include "LPC17xx.h" 
#endi£ 
 
#include "LCD.h" 

void SysTick_Handler (void); 
void delay_us (uint32_t); 

static volatile uint32_t usTicks = 0; 

void SysTick_Handler (void){ 
usTicks++; }

void delay_us (uint32_t us){ 
uint32_t startTicks; 
startTicks = usTicks; 
while ((usTicks - startTicks) < us); 

void delay (unsigned int i){
while (i--);
}

void lcd_clr (void){
lcd_command (0x01); 
delay_us (5000); 
}

void lcd_init(void){
delay_us (40000); 
lcd_init_write (0x3); 
delay_us (5000);
lcd_command (0x30); 
delay_us (5000); 
lcd_command (0x30); 
delay_us (5000); 
lcd_command (0x20); 
delay_us (5000);
lcd_command (0x28); 
delay_us (1000); 
lcd_command (0x06); 
delay_us (1000); 
lcd_command (0x0F); 
delay_us (1000); 
lcd_command (0x01); 
delay_us (5000); 
}

void lcd_init_write(unsigned char a){
RS0; 
LPC_GPIO2->FIOCLR0 = 0xF; 
LPC GPIO2->FIOSET0 = a; 
EN1; 
delay_us (100);
EN0;
}

void lcd_write4 (unsigned char value){
LPC_GPIO2->FIOCLR0 = 0xF; 
LPC_GPIO2->FIOSET0 = (value & 0xF); 
EN1; 
delay_us (100);
EN0;
}

void lcd_write (unsigned char value){
lcd_write4 (value >> 4); 
lcd_write4 (value);
}

void lcd_comnand (unsigned char value){
RS0; 
lcd_write (value); 
}

void lcd_print (char chap[]){ 
int count=0; 
while (chap[count] !='\0') 
{
RS1; 
lcd_write (chap[count]); 
}
}

/*LCD.h */

#ifndef LCD_H_
#define LCD_H_

#define RS0 LPC_GPIO2->FIOCLR0 |= 0x10; 
#define RS1 LPC_GPIO2->FIOSET0 |= 0x10; 
#define EN0 LPC_GPIO2->FIOCLR0 |= 0x20; 
#define EN1 LPC_GPIO2->FIOSET0 |= 0x20; 

void delay (unsigned int); 
void lcd_init (void); 
void lcd_write4 (unsigned char); 
void lcd_write (unsigned char); 
void lcd_init_write (unsigned char); 
void lcd_command (unsigned char) ; 
void lcd_print (char 
void lcd_clr (void); 

#endif /* LCD-H- */ 

/*Uart.c */

#include "lpc17xx.h" 
#include "type.h" 
#include "uart.h" 

volatile uint32_t UART3Status; 
volatile uint8_t UART3T×Empty = 1; 
volatile uint8_t UART3Buffer[BUFSIZE]; 
volatile uint32_t UART3Count = 0; 

/****************************************************************************************
********
**Function name:      UART0_IRQHandler
**
**Descritions:        UART0 interrupt handler
**
**parameters:         NONE
**Returned value:     NONE
*****************************************************************************************
*********/

void UART3_IRQHandler (void){
uint8_t IIRValue, LSRValue; 
uint8_t Dummy = Dummy; 
IIRValue = LPC_UART0->IIR; 
IIRValue >>= 1;     /* skip pending bit in IIR */
IIRValue &= 0x07;   /* check bit 1~3, interrupt identificati0n  */
if ( IIRValue ==IIR_RLS)   /* Receive Line Status */
{ 
LSRValue = LPC_UART0->LSR; 
if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI)) 
{
/* There are errors or break interrupt */ 
/* Read LSR will clear the interrupt */ 
UART3Status = LSRValue; 
Dummy = LPC_UART0->RBR;    /*Dummy read on RX to clear interrupt, then bail out  */
return;
}
if ( LSRValue & LSR_RDR )  /* Receive Data Ready */
{
/* If no error on RLS, normal ready, save into the data buffer.  */
/* Note: read RBR will clear the interrupt */
UART3Buffer[UART3Count] = LPC_UART3->RBR;
UART3Count++; 
if (UART3Count == BUESIZE) 
  UART3Count = 0;  /* buffer overflow */
}
}
else if ( IIRValue == IIR_RDA ) /* Receive Data Available */
{
UART3Buffer[UART3Count] = LPC_UART3->RBR; 
UART3Count++; 
if (UART3Count == BUFSIZE) 
  UART3Count = 0;  /* buffer overflow */
}
else if ( IIRValue == IIR_CTI ) /* Character timeout indicator */ 
  UART3Status |= 0x100;  /*Bit 9 as the CTI error */
else if ( IIRValue == IIR_THRE )   /* THRE, transmit holding register empty */
{
  /*  THRE interrupt */
  LSRValue = LPC_UART3->LSR;    /* Check status in the LSR to see if valid data in UART3 or not */
if ( LSRValue & LSR_THRE ) 
  UART3TxEmpty = 1;
else 
  UART3TxEmpty = 0;
}
}


/***************************************************************************************
**********
**Function name:        UARTInit
**
**Descriptions:         Initialize UART port, setup pin select, clock, parity, stop bits, FIFO, etc.
**
**parameters:           UARRT baudrate
**Returned value:       true 
installed to the VIC table
**
****************************************************************************************
**********/

uint32_t UARTInit(uint32_t baudrate) {
uint32_t Fdiv;
uint32_t pclkdiv, pclk; 

  LPC_PINCON->PINSEL0 &= ~0x0000000F;
  LPC_PINCON->PINSEL0 |= 0x0000000A; /* Enable RxD1 PO.1, TxDi P0.0 */ 
  /* By default, the PCLKSELX value is zero, thus, the PCLK for all the peripherals is 1/4 of the SystemFrequency. */ 
  /* Bit 8,9 are for UART3 */
  pclkdiv = LPC_SC->PCLKSEL0 >> 8) & 0x03; 
  switch ( pclkdiv ) 
    {
      case 0x00: 
      default: 
        pclk = SystemFrequency/4; 
        break; 
      case 0x01: 
        pclk = SystemFrequency; 
        break; 
      case 0x02: 
        pclk = SystemFrequency/2; 
        break; 
      case 0x03: 
        pclk = SystemFrequency/8; 
        break; 
    }
  LPC_UART3->LCR = 0x83; /* 8 bits, no Parity, 1 Stop bit */
  Fdiv = ( pclk / 16 ) / baudrate ; /*baud rate */ 
  LPC_UART3->DLM = Fdiv / 256; 
  LPC_UART3->DLL = Fdiv % 256; 
  LPC_UART3->LCR = 0x03; /* DLAB = 0 */ 
  LPC_UART3->FCR = 0x07; /* Enable and reset TX and RX FIFO. */
  NVIC_EnableIRQ(UART3_IRQn); 
  LPC_UART3->IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART3 interrupt */ 
  return (0); 
}
  
/*Uart.h */
  
#ifndef __UART_H 
#define __UART_H 

#define IER_RBR 0x01
#define IER_THRE 0x02 
#define IER_RLS 0x04

#define IIR_PEND 0x01 
#define IIR_RLS 0x03
#define IIR_RDA 0x02
#define IIR_CTI 0x06
#define IIR_THRE 0x01

#define LSR_RDR 0x01
#define LSR_OE 0x02
#define LSR_PE 0x04
#define LSR_FE 0x08
#define LSR_BI 0x10
#define LSR_THRE 0x20 
#define LSR_TEMT 0x40 
#define LSR_RXFE 0x80 
  
#define BUESIZE 0x40 

uint32_t UARTInit( uint32_t Baudrate ); 
void UART3_IRQHandler( void ) ; 
#endif /* end __UART_H */ 


/* Remote*/  

/*Main.c */
  
#include <cr_section_macros.h> 
#include <NXP/crp.h> 
// Variable to store CRP value in. Will be placed automatically 
// by the linker when "Enable Code Read Protect" selected. 
// See crp.h header for more information 
__CRP const unsigned int CRP_WORD = CRP_NO_CRP; 

#include "lpc17xx. h" 
#include "type.h" 
#include "uart.h" 

extern volatile uint32_t UART3Count; 
extern volatile uint8_t UART3Buffer [BUFSIZE]; 

int main (void) {
/* SystemClockUpdate() updates the SystemFrequency variable */ 
SystemClockUpdate () ; 
LPC_PINCON->PINSEL4 &=~ (0xF) ; 
LPC_GPIO2->FIODIR0 &=~(0xF); 

UARTInit (192000); /* baud rate setting */

while (1) 
{ /* Loop forever */
if ( UART3Count != 0 ) 
{ 
if (LPC_GPIO2->FIOPIN0 & (1 << 0)>>0) 
{
  LPC_UART3->IER = IER_THRE| IER_RLS;
  /* Disable RBR */ 
   UARTSend( 'L' ); 
   UART3Count = 0; 
   LED_ON;
   LPC_UART3->IER = IER_THRE | IER_RLS | IER_RBR; /*  Re-enable RBR */ 
}
else if (LPC GPIO2->FIOPIN0 & (1 << 1) >>1) 
{
LPC_UART3->IER = IER_THRE | IER_RLS; /* Disable RBR */
UARTSend( 'F' ) ;
UART3Count = 0; 
LED_ON; 
LPC_UART3->IER = IER_THRE | IER_RLS | IER_RBR; /*  Re-enable RBR */
}
else if (LPC GPIO2->FIOPIN0 & (1 << 2) >>2) 
{
LPC_UART3->IER = IER_THRE | IER_RLS ; /* Disable RBR */ 
UARTSend( 'R' ); 
UART3Count = 0; 
LED_ON;
LPC_UART3->IER = IER_THRE | IER_RLS | IER_RBR; /*Re-enable RBR */  
else if (LPC_GPIO2->FIOPIN0 & (1 << 3) >>3) 
{
LPC_UART3->IER =IER_THRE | IER_RLS;  /* Disable RBR */ 
UARTSend ( 'B'); 
UART3Count = 0; 
LED_ON;
LPC_UART3->IER = IER_THRE | IER_RLS | IER_RBR; /*Re-enable RBR */ 
}
else 
  LED_OFF;
}
}
return 0; 
}

/*Uart.c */
#include "lpc17xx.h" 
#include "type.h" 
#include "uart.h" 

volatile uint32_t UART3Status; 
volatile uint8_t UART3TxEmpty = 1; 
volatile uint8_t UART3Buffer [BUFSIZE]; 
volatile uint32_t UART3Count = 0;

/**************************************************************************** 
******* 
**Function name:           UART3_IROHandler
**
**Descriptions:            UART3 interrupt handler
**
**pararneters:             NONE
**Returrned value:         NONE
*****************************************************************************
******/
 
 
 
 
void UART3_IRQHandler (void) {
uint8_t IIRValue, LSRValue; 
uint8_t Dummy = Dummy;

IIRValue = LPC_UART0->IIR; 
IIRValue >>= 1; /* skip pending bit in IIR */ 
IIRValue &= 0x07; /* check bit 1~3, interrupt identification */
if ( IIRValue == IIR_RLS )  /* Receive Line Status */
{
LSRValue = LPC_UART3->LSR; 
/* Receive Line Status */ 
if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE| LSR_RXFE| LSR_BI)) 
{ 
/* There are errors or break interrupt */ 
/* Read LSR will clear the interrupt */ 
UART3Status = LSRValue; 
Dummy = LPC_UART3->RBR;  /* Dummy read on RX to clear interrupt, then bail out */
return;
}
if ( LSRValue & LSR_RDR)  /* Receive Data Ready  */
{
/* If no error on RLS, normal ready, save into the data buffer. */ 
/* Note: read RBR will clear the interrupt */
UART3Buffer [UART3Count] = LPC_UART0->RBR;
UART3Countt+; 
if (UART3Count == BUFSIZE )
  UART3Count = 0; /* buffer overfloW */
}
else if ( IIRValue == IIR_CTI ) /* Character Time-out indicator */
{
/* Character Time-out indicator */
UART3Status |= 0x100; /* Bit 9 as the CTI error */ 
}
else if ( IIRValue == IIR_THRE ) /* THRE, t:ansmit holding register empty */  
{
/* THRE interrupt */ 
LSRValue = LPC_UART3->LSR; /* Check status in the LSR to see if valid data in U0THR or not */
if ( LSRValue & LSR_THRE )
{
UART3TxEmpty = 1;
}
else
{
YART3txEmpty = 0;
}
}
}

/***********************************************************************
********
**Function name:       UARTInit 
**
**Descriptions:        Initialize UART port, setup pin select, clock, parity, stop bits, FIFO, etc. 
**
**parameters: 
** Returned value: 
insta1led to the 
**
**
*************************************************************************
********/


uint32_t UARTInit( 
uint32_t Fdiv; 
uint32_t pclkdiv, pclk; 

LPC_PINCON->PINSEL0 &= ~0x0000000F; 
LPC_PINCON->PINSEL0 |= 0x0000000A; /* Enable RxD1 P0.l, TxD1 P0.0 */ 

/* By default, the PCLKSELx value is zero, thus, the PCLK for all the peripherals is 1/4 of the SystemFrequency. */
/* Bit 8,9 are for UART1 */ 
pclkdiv = (LPC SC->PCLKSEL0 >> 8) & 0x03;
switch ( pclkdiv ) 
{
case 0x00: 
  default: 
    pclk = SystemFrequency/4; 
    break; 
case 0x01: 
     pclk = SystemFrequency; 
     break;
case 0x02: 
  pclk = SystemFrequency/2; 
  break; 
case 0x03:
  pclk = SystemFrequency/8; 
  break;
}

LPC_UART3->LCR = 0x83; /* ß bits, no Parity, 1 Stop bit */
Fdiv = ( pclk / 16 ) / baudrate ;   /*baud rate */
LPC_UART3->DLM = Fdiv / 256; 
LPC_UART3-> DLL = Fdiv % 256; 
LPC_UART3->LCR = 0x03; /* DLAB = 0 */
LPC_UART3->FCR = 0x07; /*Enable and reset TX and RX FIFO. */

NVIC_EnableIRQ( UART3_IRQn);

LPC_UART3->IER = IER_RBR | IER_THRE | IER_RLS;  /* Enable UART3 interrupt */ 
return (TRUE);
}

/****************************************************************************
********
**Function name:         UARTSend
**
**Descriptions:          Send data to the UART 3 port based
**
**parameters:            buffer
**Returned value:        None
**
*****************************************************************************
********/

void UARTSend( uint8_t BufferPtr )
{
/* THRE status, contain valid data */
while (! (UART3TxEmpty & 0x01) ); 
LPC_UART3->THR = BufferPtr; 
UART3TxEmpty = 0;  /* not empty in the THR until it shifts out  */ 
 return; 
}


/*Uart.h */

#ifndef __UART_H 
#define __UART_H 
 
 
#define IER_RBR 0x01 
#define IER_THRE 0x02 
#define IER_RLS 0x04

#define IIR_PEND 0x01 
#define IIR_RLS 0x03
#define IIR_RDA 0x02
#define IIR_CTI 0x06
#define IIR_THRE 0x01

#define LSR_RDR 0x01
#define LSR_OE 0x02
#define LSR_PE 0x04
#define LSR_FE 0x08
#define LSR_BI 0x10
#define LSR_THRE 0x20
#define LSR_TEMT 0x40
#define LSR_RXFE 0x80

#define BUFSIZE 0x40

#define LED_ON LPC_GPIO2 ->FIOSET0 |=0x10; 
#define LED_OFF LPC_GPIO2->FIOCLR0|=0x10; 

uint32 _t UARIInit( uint32_t Baudrate ); 
void UART0_IRQHandler( void ); 
void UART1_IRQHandler( void ); 
void UARTSend ( uint8_t BufferPtr ); 
#endif /* end __UART_H */ 
