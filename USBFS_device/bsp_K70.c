#include "bsp_K70.h"
#include "Settings.h"

void vfnfll_init (void);
void fll_init (void);

void vfnInitUSBClock (uint8 u8ClkOption)
{

  switch (u8ClkOption)
  {
  case MCGPLL0:
  {
    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL(1)       /** PLL0 reference */   
              |  SIM_SOPT2_USBFSRC(0)         /** MCGPLLCLK as CLKC source */
              |  SIM_SOPT2_USBF_CLKSEL_MASK;  /** USB fractional divider like USB reference clock */  
    SIM_CLKDIV2 = USB_FRAC | USB_DIV;         /** Divide reference clock to obtain 48MHz */
    break;     
  }
  
  case MCGPLL1:
  {
      SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL(2)     /** PLL1 reference */   
              |  SIM_SOPT2_USBFSRC(0)         /** MCGPLLCLK as CLKC source */
              |  SIM_SOPT2_USBF_CLKSEL_MASK;  /** USB fractional divider like USB reference clock */ 
 
     SIM_CLKDIV2 = USB_FRAC | USB_DIV;         /** Divide reference clock to obtain 48MHz */
     break;
  }

  case MCGFLL:
  {
      /** Configure FLL to generate a 48MHz core clock */
     fll_init();                            
        
      SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL(0)       /** FLL reference */   
                |  SIM_SOPT2_USBFSRC(0)         /** MCGPLLCLK as CLKC source */
                |  SIM_SOPT2_USBF_CLKSEL_MASK;  /** USB fractional divider like USB reference clock */ 
      /** Divide reference clock by one to obtain 48MHz */
      SIM_CLKDIV2 &= ~(SIM_CLKDIV2_USBFSDIV_MASK |SIM_CLKDIV2_USBFSFRAC_MASK);      
      break;
  }
  
  case PLL1:
  {
     SIM_SOPT2 |= SIM_SOPT2_USBFSRC(2)         /** PLL1 selected as USBFS CLK source */
               |  SIM_SOPT2_USBF_CLKSEL_MASK;  /** USB fractional divider like USB reference clock */      
     SIM_CLKDIV2 = USB_FRAC | USB_DIV;         /** Divide reference clock to obtain 48MHz */   
     break;
  }
    
  case CLKIN:
  {
    SIM_SOPT2 &= (uint32)(~SIM_SOPT2_USBF_CLKSEL_MASK);    /** PTE26 selected as USBFS CLK source */ 
    FLAG_SET(SIM_SCGC5_PORTE_SHIFT,SIM_SCGC5); 
    PORTE_PCR26=(0|PORT_PCR_MUX(7));                      // Enabling PTE26 as CLK input    
  }
  }
  
  SIM_SCGC4|=(SIM_SCGC4_USBFS_MASK);             // USB Clock Gating
}



void fll_init(void)
{
    pee_pbe(CLK0_FREQ_HZ);
    pbe_fbe(CLK0_FREQ_HZ);
    
    MCG_C2 &= ~MCG_C2_IRCS_MASK;          // select slow IRC by clearing IRCS
    while ((MCG_S & MCG_S_IRCST_MASK)){}; // wait until internal reference switches to slow clock.
    
    MCG_C1 = MCG_C1_CLKS(0) | MCG_C1_IREFS_MASK;
    while (!(MCG_S & MCG_S_IREFST_MASK));
    
    /**Fixed DCO multiplier of 1464, MCGOUT = 32.768 kHz * 1464 / 1 = 48 MHz*/
    MCG_C4 = MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(1);	

}

/**FLL output like bus clock source, divide reference by 256 (8MHz /256 = 31.25kHz)
	MCG_C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(3);
	
	while (!(MCG_S & MCG_S_OSCINIT_MASK)){};  // wait for oscillator to initialize
	while (MCG_S & MCG_S_IREFST_MASK){}; 	  // wait for Reference clock Status bit to clear  */

        /**Fixed DCO multiplier of 1464, MCGOUT = 32.768 kHz * 1464 / 1 = 48 MHz
        MCG_C4 = MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(1);		       
        while (MCG_S & MCG_S_CLKST_MASK){};    	  // Wait for output of FLL */
	