/**HEADER********************************************************************
* 
* Copyright (c) 2010 Freescale Semiconductor;
* All Rights Reserved
*
*************************************************************************** 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
*
* Comments:  This file includes operation of serial communication interface.
*
*
*END************************************************************************/
#include "sci.h"
#include "psptypes.h"
#include "usb_bsp.h"
#include "common.h"

char   buff[200];
uint_32 buff_index;

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : TERMIO_PutChar
* Returned Value   :
* Comments         :
*                     This function sends a char via SCI.
*
*END*----------------------------------------------------------------------*/
void TERMIO_PutChar (char ch)
{
    /* Wait until space is available in the FIFO */
    while(!(UART_S1_REG(TERM_PORT) & UART_S1_TDRE_MASK)){};
    
    /* Send the character */
    UART_D_REG(TERM_PORT) = (uint_8)ch;
 }

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : TERMIO_GetChar
* Returned Value   : the char get via SCI
* Comments         : This function gets a char via SCI.
*                     
*
*END*----------------------------------------------------------------------*/
char TERMIO_GetChar (void)
{
    /* Wait until character has been received */
    while (!(UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK));
    
    /* Return the 8-bit data from the receiver */
    return UART_D_REG(TERM_PORT);
}

/********************************************************************/
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : uart_getcharNB
* Returned Value   : the char get via SCI
* Comments         : This function gets a char via SCI. non blocking
*                     
*
*END*----------------------------------------------------------------------*/
char TERMIO_GetCharNB (void)
{
    char dummy;
      if((UART_S1_REG(TERM_PORT) & UART_S1_RDRF_MASK))
      {    
       dummy = (char)UART_S1_REG(TERM_PORT);
       return (char)UART_D_REG(TERM_PORT);
      }
      else
      {    
       return 0; 
      }
}

/********************************************************************/
void
out_char (char ch)
{
	TERMIO_PutChar(ch);
}

void sci2_init(void) 
{

}


/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : sci2_PutChar 
* Returned Value   :
* Comments         : send a charactor through UART2
*    
*
*END*----------------------------------------------------------------------*/
void sci2_PutChar(char send)
{

}
