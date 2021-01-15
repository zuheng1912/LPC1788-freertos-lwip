#line 1 "..\\Lwip\\core\\ipv4\\dhcp.c"



























 



































 

#line 1 "..\\Lwip\\include\\lwip/opt.h"




 































 




 







 
#line 1 "..\\User\\arch\\lwipopts.h"



























 








 





 





 


 


 



 




 


 


 


 


 


 




 
 

 



 




 


 


 



 



 



 



 


 



 




 



 


 





 





 




   

   

   

   

   

   

   
#line 184 "..\\User\\arch\\lwipopts.h"






 


 






 


 






 
























 




 

































 


































































































































































































































#line 52 "..\\Lwip\\include\\lwip/opt.h"
#line 1 "..\\Lwip\\include\\lwip/debug.h"



 































 



#line 1 "..\\Lwip\\include\\lwip/arch.h"



 































 











#line 1 "..\\User\\arch/cc.h"






























 



#line 1 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 36 "..\\User\\arch/cc.h"

#line 1 "..\\User\\main.h"



#line 1 "..\\..\\Core\\CM3\\DeviceSupport\\NXP\\LPC177x_8x\\LPC177x_8x.h"


 





















 








 

typedef enum IRQn
{
 
  NonMaskableInt_IRQn           = -14,       
  MemoryManagement_IRQn         = -12,       
  BusFault_IRQn                 = -11,       
  UsageFault_IRQn               = -10,       
  SVCall_IRQn                   = -5,        
  DebugMonitor_IRQn             = -4,        
  PendSV_IRQn                   = -2,        
  SysTick_IRQn                  = -1,        

 
  WDT_IRQn                      = 0,         
  TIMER0_IRQn                   = 1,         
  TIMER1_IRQn                   = 2,         
  TIMER2_IRQn                   = 3,         
  TIMER3_IRQn                   = 4,         
  UART0_IRQn                    = 5,         
  UART1_IRQn                    = 6,         
  UART2_IRQn                    = 7,         
  UART3_IRQn                    = 8,         
  PWM1_IRQn                     = 9,         
  I2C0_IRQn                     = 10,        
  I2C1_IRQn                     = 11,        
  I2C2_IRQn                     = 12,        
  Reserved0_IRQn                = 13,        
  SSP0_IRQn                     = 14,        
  SSP1_IRQn                     = 15,        
  PLL0_IRQn                     = 16,        
  RTC_IRQn                      = 17,        
  EINT0_IRQn                    = 18,        
  EINT1_IRQn                    = 19,        
  EINT2_IRQn                    = 20,        
  EINT3_IRQn                    = 21,        
  ADC_IRQn                      = 22,        
  BOD_IRQn                      = 23,        
  USB_IRQn                      = 24,        
  CAN_IRQn                      = 25,        
  DMA_IRQn                      = 26,        
  I2S_IRQn                      = 27,        
  ENET_IRQn                     = 28,        
  MCI_IRQn                      = 29,        
  MCPWM_IRQn                    = 30,        
  QEI_IRQn                      = 31,        
  PLL1_IRQn                     = 32,        
  USBActivity_IRQn              = 33,        
  CANActivity_IRQn              = 34,        
  UART4_IRQn                    = 35,        
  SSP2_IRQn                     = 36,        
  LCD_IRQn                      = 37,        
  GPIO_IRQn                     = 38,        
  PWM0_IRQn                     = 39,        
  EEPROM_IRQn                   = 40,        
} IRQn_Type;






 

 





#line 1 "..\\..\\Core\\CM3\\CoreSupport\\core_cm3.h"
 




















 
























 









 




 







 

 











#line 97 "..\\..\\Core\\CM3\\CoreSupport\\core_cm3.h"

#line 1 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 99 "..\\..\\Core\\CM3\\CoreSupport\\core_cm3.h"
#line 1 "..\\..\\Core\\CM3\\CoreSupport\\core_cmInstr.h"
 




















 





 




 


 




 







 







 






 








 







 







 









 









 



static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}









 



static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}











 









 









 









 











 











 











 







 














 










 









 






#line 782 "..\\..\\Core\\CM3\\CoreSupport\\core_cmInstr.h"

   

#line 100 "..\\..\\Core\\CM3\\CoreSupport\\core_cm3.h"
#line 1 "..\\..\\Core\\CM3\\CoreSupport\\core_cmFunc.h"
 




















 




 



 


 

 
 






 



static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}








 



static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}








 



static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}








 



static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}








 



static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}








 



static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}








 



static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}








 



static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}








 



static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}








 



static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}








 



static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}









 







 








 



static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}








 



static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}








 



static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}








 



static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}





#line 348 "..\\..\\Core\\CM3\\CoreSupport\\core_cmFunc.h"


#line 848 "..\\..\\Core\\CM3\\CoreSupport\\core_cmFunc.h"

 


#line 101 "..\\..\\Core\\CM3\\CoreSupport\\core_cm3.h"









 
#line 118 "..\\..\\Core\\CM3\\CoreSupport\\core_cm3.h"

 





 









 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
} SCB_Type;

 












 






























 






 





















 









 


















 










































 









 









 















 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t RELOAD;                     
  volatile uint32_t CURR;                      
  volatile  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];
  volatile uint32_t IWR;                      
  volatile uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];
  volatile uint32_t LAR;                      
  volatile uint32_t LSR;                      
       uint32_t RESERVED5[6];
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;

 



 
























 



 



 



 









   






 


 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                     



       uint32_t RESERVED1;

} InterruptType_Type;

 



 









   







 


 
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;

 









 









 



 









 



























 







 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 




 

 
#line 853 "..\\..\\Core\\CM3\\CoreSupport\\core_cm3.h"

#line 860 "..\\..\\Core\\CM3\\CoreSupport\\core_cm3.h"






 





 







 



 



 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR & (7UL << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 5)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 5)) & 0xff);    }         
}













 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 5)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL))->IP[(uint32_t)(IRQn)]           >> (8 - 5)));  }  
}















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL))->AIRCR & (7UL << 8)) |
                 (1UL << 2));                    
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 



 











 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->RELOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<5) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->CURR   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL))->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 



 



 

extern volatile int32_t ITM_RxBuffer;                     











 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0UL))->DEMCR & (1UL << 24))  &&       
      (((ITM_Type *) (0xE0000000UL))->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL))->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL))->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}










 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}









 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 









 
#line 106 "..\\..\\Core\\CM3\\DeviceSupport\\NXP\\LPC177x_8x\\LPC177x_8x.h"



 
 
 


#pragma anon_unions


 
typedef struct
{
  volatile uint32_t FLASHCFG;                    
       uint32_t RESERVED0[31];
  volatile uint32_t PLL0CON;                     
  volatile uint32_t PLL0CFG;                     
  volatile const  uint32_t PLL0STAT;                    
  volatile  uint32_t PLL0FEED;                    
       uint32_t RESERVED1[4];
  volatile uint32_t PLL1CON;                     
  volatile uint32_t PLL1CFG;                     
  volatile const  uint32_t PLL1STAT;                    
  volatile  uint32_t PLL1FEED;                    
       uint32_t RESERVED2[4];
  volatile uint32_t PCON;                        
  volatile uint32_t PCONP;                       
       uint32_t RESERVED3[14];
  volatile uint32_t EMCCLKSEL;                   
  volatile uint32_t CCLKSEL;                     
  volatile uint32_t USBCLKSEL;                   
  volatile uint32_t CLKSRCSEL;                   
  volatile uint32_t	CANSLEEPCLR;                 
  volatile uint32_t	CANWAKEFLAGS;                
       uint32_t RESERVED4[10];
  volatile uint32_t EXTINT;                      
       uint32_t RESERVED5[1];
  volatile uint32_t EXTMODE;                     
  volatile uint32_t EXTPOLAR;                    
       uint32_t RESERVED6[12];
  volatile uint32_t RSID;                        
       uint32_t RESERVED7[7];
  volatile uint32_t SCS;                         
  volatile uint32_t IRCTRIM;                     
  volatile uint32_t PCLKSEL;                     
       uint32_t RESERVED8[3];
  volatile uint32_t LCD_CFG;                     
       uint32_t RESERVED9[1];
  volatile uint32_t USBIntSt;                    
  volatile uint32_t DMAREQSEL;                   
  volatile uint32_t CLKOUTCFG;                   
  volatile uint32_t RSTCON0;                     
  volatile uint32_t RSTCON1;                     
       uint32_t RESERVED10[2];
  volatile uint32_t EMCDLYCTL;                   
  volatile uint32_t EMCCAL;                      
 } LPC_SC_TypeDef;

 
typedef struct
{
  volatile uint32_t P0_0;				 
  volatile uint32_t P0_1;
  volatile uint32_t P0_2;
  volatile uint32_t P0_3;
  volatile uint32_t P0_4;
  volatile uint32_t P0_5;
  volatile uint32_t P0_6;
  volatile uint32_t P0_7;

  volatile uint32_t P0_8;				 
  volatile uint32_t P0_9;
  volatile uint32_t P0_10;
  volatile uint32_t P0_11;
  volatile uint32_t P0_12;
  volatile uint32_t P0_13;
  volatile uint32_t P0_14;
  volatile uint32_t P0_15;

  volatile uint32_t P0_16;				 
  volatile uint32_t P0_17;
  volatile uint32_t P0_18;
  volatile uint32_t P0_19;
  volatile uint32_t P0_20;
  volatile uint32_t P0_21;
  volatile uint32_t P0_22;
  volatile uint32_t P0_23;

  volatile uint32_t P0_24;				 
  volatile uint32_t P0_25;
  volatile uint32_t P0_26;
  volatile uint32_t P0_27;
  volatile uint32_t P0_28;
  volatile uint32_t P0_29;
  volatile uint32_t P0_30;
  volatile uint32_t P0_31;

  volatile uint32_t P1_0;				 
  volatile uint32_t P1_1;
  volatile uint32_t P1_2;
  volatile uint32_t P1_3;
  volatile uint32_t P1_4;
  volatile uint32_t P1_5;
  volatile uint32_t P1_6;
  volatile uint32_t P1_7;

  volatile uint32_t P1_8;				 
  volatile uint32_t P1_9;
  volatile uint32_t P1_10;
  volatile uint32_t P1_11;
  volatile uint32_t P1_12;
  volatile uint32_t P1_13;
  volatile uint32_t P1_14;
  volatile uint32_t P1_15;

  volatile uint32_t P1_16;				 
  volatile uint32_t P1_17;
  volatile uint32_t P1_18;
  volatile uint32_t P1_19;
  volatile uint32_t P1_20;
  volatile uint32_t P1_21;
  volatile uint32_t P1_22;
  volatile uint32_t P1_23;

  volatile uint32_t P1_24;				 
  volatile uint32_t P1_25;
  volatile uint32_t P1_26;
  volatile uint32_t P1_27;
  volatile uint32_t P1_28;
  volatile uint32_t P1_29;
  volatile uint32_t P1_30;
  volatile uint32_t P1_31;

  volatile uint32_t P2_0;				 
  volatile uint32_t P2_1;
  volatile uint32_t P2_2;
  volatile uint32_t P2_3;
  volatile uint32_t P2_4;
  volatile uint32_t P2_5;
  volatile uint32_t P2_6;
  volatile uint32_t P2_7;

  volatile uint32_t P2_8;				 
  volatile uint32_t P2_9;
  volatile uint32_t P2_10;
  volatile uint32_t P2_11;
  volatile uint32_t P2_12;
  volatile uint32_t P2_13;
  volatile uint32_t P2_14;
  volatile uint32_t P2_15;

  volatile uint32_t P2_16;				 
  volatile uint32_t P2_17;
  volatile uint32_t P2_18;
  volatile uint32_t P2_19;
  volatile uint32_t P2_20;
  volatile uint32_t P2_21;
  volatile uint32_t P2_22;
  volatile uint32_t P2_23;

  volatile uint32_t P2_24;				 
  volatile uint32_t P2_25;
  volatile uint32_t P2_26;
  volatile uint32_t P2_27;
  volatile uint32_t P2_28;
  volatile uint32_t P2_29;
  volatile uint32_t P2_30;
  volatile uint32_t P2_31;

  volatile uint32_t P3_0;				 
  volatile uint32_t P3_1;
  volatile uint32_t P3_2;
  volatile uint32_t P3_3;
  volatile uint32_t P3_4;
  volatile uint32_t P3_5;
  volatile uint32_t P3_6;
  volatile uint32_t P3_7;

  volatile uint32_t P3_8;				 
  volatile uint32_t P3_9;
  volatile uint32_t P3_10;
  volatile uint32_t P3_11;
  volatile uint32_t P3_12;
  volatile uint32_t P3_13;
  volatile uint32_t P3_14;
  volatile uint32_t P3_15;

  volatile uint32_t P3_16;				 
  volatile uint32_t P3_17;
  volatile uint32_t P3_18;
  volatile uint32_t P3_19;
  volatile uint32_t P3_20;
  volatile uint32_t P3_21;
  volatile uint32_t P3_22;
  volatile uint32_t P3_23;

  volatile uint32_t P3_24;				 
  volatile uint32_t P3_25;
  volatile uint32_t P3_26;
  volatile uint32_t P3_27;
  volatile uint32_t P3_28;
  volatile uint32_t P3_29;
  volatile uint32_t P3_30;
  volatile uint32_t P3_31;

  volatile uint32_t P4_0;				 
  volatile uint32_t P4_1;
  volatile uint32_t P4_2;
  volatile uint32_t P4_3;
  volatile uint32_t P4_4;
  volatile uint32_t P4_5;
  volatile uint32_t P4_6;
  volatile uint32_t P4_7;

  volatile uint32_t P4_8;				 
  volatile uint32_t P4_9;
  volatile uint32_t P4_10;
  volatile uint32_t P4_11;
  volatile uint32_t P4_12;
  volatile uint32_t P4_13;
  volatile uint32_t P4_14;
  volatile uint32_t P4_15;

  volatile uint32_t P4_16;				 
  volatile uint32_t P4_17;
  volatile uint32_t P4_18;
  volatile uint32_t P4_19;
  volatile uint32_t P4_20;
  volatile uint32_t P4_21;
  volatile uint32_t P4_22;
  volatile uint32_t P4_23;

  volatile uint32_t P4_24;				 
  volatile uint32_t P4_25;
  volatile uint32_t P4_26;
  volatile uint32_t P4_27;
  volatile uint32_t P4_28;
  volatile uint32_t P4_29;
  volatile uint32_t P4_30;
  volatile uint32_t P4_31;

  volatile uint32_t P5_0;				 
  volatile uint32_t P5_1;
  volatile uint32_t P5_2;
  volatile uint32_t P5_3;
  volatile uint32_t P5_4;				 
} LPC_IOCON_TypeDef;

 
typedef struct
{
  volatile uint32_t DIR;
       uint32_t RESERVED0[3];
  volatile uint32_t MASK;
  volatile uint32_t PIN;
  volatile uint32_t SET;
  volatile  uint32_t CLR;
} LPC_GPIO_TypeDef;

typedef struct
{
  volatile const  uint32_t IntStatus;
  volatile const  uint32_t IO0IntStatR;
  volatile const  uint32_t IO0IntStatF;
  volatile  uint32_t IO0IntClr;
  volatile uint32_t IO0IntEnR;
  volatile uint32_t IO0IntEnF;
       uint32_t RESERVED0[3];
  volatile const  uint32_t IO2IntStatR;
  volatile const  uint32_t IO2IntStatF;
  volatile  uint32_t IO2IntClr;
  volatile uint32_t IO2IntEnR;
  volatile uint32_t IO2IntEnF;
} LPC_GPIOINT_TypeDef;

 
typedef struct
{
  volatile uint32_t IR;                      
  volatile uint32_t TCR;                     
  volatile uint32_t TC;                      
  volatile uint32_t PR;                      
  volatile uint32_t PC;                      
  volatile uint32_t MCR;                     
  volatile uint32_t MR0;                     
  volatile uint32_t MR1;                     
  volatile uint32_t MR2;                     
  volatile uint32_t MR3;                     
  volatile uint32_t CCR;                     
  volatile const  uint32_t CR0;                     
  volatile const  uint32_t CR1;					 
       uint32_t RESERVED0[2];
  volatile uint32_t EMR;                     
       uint32_t RESERVED1[12];
  volatile uint32_t CTCR;                    
} LPC_TIM_TypeDef;

 
typedef struct
{
  volatile uint32_t IR;                      
  volatile uint32_t TCR;                     
  volatile uint32_t TC;                      
  volatile uint32_t PR;                      
  volatile uint32_t PC;                      
  volatile uint32_t MCR;                     
  volatile uint32_t MR0;                     
  volatile uint32_t MR1;                     
  volatile uint32_t MR2;                     
  volatile uint32_t MR3;                     
  volatile uint32_t CCR;                     
  volatile const  uint32_t CR0;                     
  volatile const  uint32_t CR1;					 
  volatile const  uint32_t CR2;					 
  volatile const  uint32_t CR3;					 
       uint32_t RESERVED0;
  volatile uint32_t MR4;					 
  volatile uint32_t MR5;					 
  volatile uint32_t MR6;					 
  volatile uint32_t PCR;					 
  volatile uint32_t LER;					 
       uint32_t RESERVED1[7];
  volatile uint32_t CTCR;					 
} LPC_PWM_TypeDef;

 




 

#line 472 "..\\..\\Core\\CM3\\DeviceSupport\\NXP\\LPC177x_8x\\LPC177x_8x.h"
typedef struct
{
	union
	{
		volatile const  uint8_t  RBR;
		volatile  uint8_t  THR;
		volatile uint8_t  DLL;
		uint32_t RESERVED0;
	};
	union
	{
		volatile uint8_t  DLM;
		volatile uint32_t IER;
	};
	union
	{
		volatile const  uint32_t IIR;
		volatile  uint8_t  FCR;
	};
	volatile uint8_t  LCR;
	uint8_t  RESERVED1[7];
	volatile const  uint8_t  LSR;
	uint8_t  RESERVED2[7];
	volatile uint8_t  SCR;
	uint8_t  RESERVED3[3];
	volatile uint32_t ACR;
	volatile uint8_t  ICR;
	uint8_t  RESERVED4[3];
	volatile uint8_t  FDR;
	uint8_t  RESERVED5[7];
	volatile uint8_t  TER;
	uint8_t  RESERVED8[27];
	volatile uint8_t  RS485CTRL;
	uint8_t  RESERVED9[3];
	volatile uint8_t  ADRMATCH;
	uint8_t  RESERVED10[3];
	volatile uint8_t  RS485DLY;
	uint8_t  RESERVED11[3];
	volatile const  uint8_t  FIFOLVL;
}LPC_UART_TypeDef;



typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  MCR;
       uint8_t  RESERVED2[3];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED3[3];
  volatile const  uint8_t  MSR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  SCR;
       uint8_t  RESERVED5[3];
  volatile uint32_t ACR;
       uint32_t RESERVED6;
  volatile uint32_t FDR;
       uint32_t RESERVED7;
  volatile uint8_t  TER;
       uint8_t  RESERVED8[27];
  volatile uint8_t  RS485CTRL;
       uint8_t  RESERVED9[3];
  volatile uint8_t  ADRMATCH;
       uint8_t  RESERVED10[3];
  volatile uint8_t  RS485DLY;
       uint8_t  RESERVED11[3];
  volatile const  uint8_t  FIFOLVL;
} LPC_UART1_TypeDef;

typedef struct
{
  union {
  volatile const  uint32_t  RBR;                    
  volatile  uint32_t  THR;                    
  volatile uint32_t  DLL;                    
  };
  union {
  volatile uint32_t  DLM;                    
  volatile uint32_t  IER;                    
  };
  union {
  volatile const  uint32_t  IIR;                    
  volatile  uint32_t  FCR;                    
  };
  volatile uint32_t  LCR;                    
  volatile uint32_t  MCR;                    
  volatile const  uint32_t  LSR;                    
  volatile const  uint32_t  MSR;                    
  volatile uint32_t  SCR;                    
  volatile uint32_t  ACR;                    
  volatile uint32_t  ICR;                    
  volatile uint32_t  FDR;                    
  volatile uint32_t  OSR;                    
  volatile  uint32_t  POP;                    
  volatile uint32_t  MODE;                   
       uint32_t  RESERVED0[2];
  volatile uint32_t  HDEN;                   
       uint32_t  RESERVED1;
  volatile uint32_t  SCI_CTRL;				 
  volatile uint32_t  RS485CTRL;              
  volatile uint32_t  ADRMATCH;               
  volatile uint32_t  RS485DLY;               
  volatile uint32_t  SYNCCTRL;               
  volatile uint32_t  TER;                    
       uint32_t  RESERVED2[989];
  volatile const  uint32_t  CFG;                    
  volatile  uint32_t  INTCE;                  
  volatile  uint32_t  INTSE;                  
  volatile const  uint32_t  INTS;                   
  volatile const  uint32_t  INTE;                   
  volatile  uint32_t  INTCS;                  
  volatile  uint32_t  INTSS;                  
       uint32_t  RESERVED3[3];
  volatile const  uint32_t  MID;                    
} LPC_UART4_TypeDef;

 
typedef struct
{
  volatile uint32_t CR0;                     
  volatile uint32_t CR1;                     
  volatile uint32_t DR;                      
  volatile const  uint32_t SR;                      
  volatile uint32_t CPSR;                    
  volatile uint32_t IMSC;                    
  volatile uint32_t RIS;                     
  volatile uint32_t MIS;                     
  volatile uint32_t ICR;                     
  volatile uint32_t DMACR;
} LPC_SSP_TypeDef;

 
typedef struct
{
  volatile uint32_t CONSET;                  
  volatile const  uint32_t STAT;                    
  volatile uint32_t DAT;                     
  volatile uint32_t ADR0;                    
  volatile uint32_t SCLH;                    
  volatile uint32_t SCLL;                    
  volatile  uint32_t CONCLR;                  
  volatile uint32_t MMCTRL;                  
  volatile uint32_t ADR1;                    
  volatile uint32_t ADR2;                    
  volatile uint32_t ADR3;                    
  volatile const  uint32_t DATA_BUFFER;             
  volatile uint32_t MASK0;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t MASK3;                   
} LPC_I2C_TypeDef;

 
typedef struct
{
  volatile uint32_t DAO;
  volatile uint32_t DAI;
  volatile  uint32_t TXFIFO;
  volatile const  uint32_t RXFIFO;
  volatile const  uint32_t STATE;
  volatile uint32_t DMA1;
  volatile uint32_t DMA2;
  volatile uint32_t IRQ;
  volatile uint32_t TXRATE;
  volatile uint32_t RXRATE;
  volatile uint32_t TXBITRATE;
  volatile uint32_t RXBITRATE;
  volatile uint32_t TXMODE;
  volatile uint32_t RXMODE;
} LPC_I2S_TypeDef;

 
typedef struct
{
  volatile uint8_t  ILR;
       uint8_t  RESERVED0[7];
  volatile uint8_t  CCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  CIIR;
       uint8_t  RESERVED2[3];
  volatile uint8_t  AMR;
       uint8_t  RESERVED3[3];
  volatile const  uint32_t CTIME0;
  volatile const  uint32_t CTIME1;
  volatile const  uint32_t CTIME2;
  volatile uint8_t  SEC;
       uint8_t  RESERVED4[3];
  volatile uint8_t  MIN;
       uint8_t  RESERVED5[3];
  volatile uint8_t  HOUR;
       uint8_t  RESERVED6[3];
  volatile uint8_t  DOM;
       uint8_t  RESERVED7[3];
  volatile uint8_t  DOW;
       uint8_t  RESERVED8[3];
  volatile uint16_t DOY;
       uint16_t RESERVED9;
  volatile uint8_t  MONTH;
       uint8_t  RESERVED10[3];
  volatile uint16_t YEAR;
       uint16_t RESERVED11;
  volatile uint32_t CALIBRATION;
  volatile uint32_t GPREG0;
  volatile uint32_t GPREG1;
  volatile uint32_t GPREG2;
  volatile uint32_t GPREG3;
  volatile uint32_t GPREG4;
  volatile uint8_t  RTC_AUXEN;
       uint8_t  RESERVED12[3];
  volatile uint8_t  RTC_AUX;
       uint8_t  RESERVED13[3];
  volatile uint8_t  ALSEC;
       uint8_t  RESERVED14[3];
  volatile uint8_t  ALMIN;
       uint8_t  RESERVED15[3];
  volatile uint8_t  ALHOUR;
       uint8_t  RESERVED16[3];
  volatile uint8_t  ALDOM;
       uint8_t  RESERVED17[3];
  volatile uint8_t  ALDOW;
       uint8_t  RESERVED18[3];
  volatile uint16_t ALDOY;
       uint16_t RESERVED19;
  volatile uint8_t  ALMON;
       uint8_t  RESERVED20[3];
  volatile uint16_t ALYEAR;
       uint16_t RESERVED21;
  volatile uint32_t ERSTATUS;
  volatile uint32_t ERCONTROL;
  volatile uint32_t ERCOUNTERS;
       uint32_t RESERVED22;
  volatile uint32_t ERFIRSTSTAMP0;
  volatile uint32_t ERFIRSTSTAMP1;
  volatile uint32_t ERFIRSTSTAMP2;
       uint32_t RESERVED23;
  volatile uint32_t ERLASTSTAMP0;
  volatile uint32_t ERLASTSTAMP1;
  volatile uint32_t ERLASTSTAMP2;
} LPC_RTC_TypeDef;

 
typedef struct
{
  volatile uint8_t  MOD;
       uint8_t  RESERVED0[3];
  volatile uint32_t TC;
  volatile  uint8_t  FEED;
       uint8_t  RESERVED1[3];
  volatile const  uint32_t TV;
       uint32_t RESERVED2;
  volatile uint32_t WARNINT;
  volatile uint32_t WINDOW;
} LPC_WDT_TypeDef;

 
typedef struct
{
  volatile uint32_t CR;                      
  volatile uint32_t GDR;                     
       uint32_t RESERVED0;
  volatile uint32_t INTEN;                   
  volatile uint32_t DR[8];                   
  volatile const  uint32_t STAT;                    
  volatile uint32_t ADTRM;
} LPC_ADC_TypeDef;

 
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CTRL;
  volatile uint32_t CNTVAL;
} LPC_DAC_TypeDef;

 
typedef struct
{
  volatile const  uint32_t CON;
  volatile  uint32_t CON_SET;
  volatile  uint32_t CON_CLR;
  volatile const  uint32_t CAPCON;
  volatile  uint32_t CAPCON_SET;
  volatile  uint32_t CAPCON_CLR;
  volatile uint32_t TC0;
  volatile uint32_t TC1;
  volatile uint32_t TC2;
  volatile uint32_t LIM0;
  volatile uint32_t LIM1;
  volatile uint32_t LIM2;
  volatile uint32_t MAT0;
  volatile uint32_t MAT1;
  volatile uint32_t MAT2;
  volatile uint32_t DT;
  volatile uint32_t CP;
  volatile uint32_t CAP0;
  volatile uint32_t CAP1;
  volatile uint32_t CAP2;
  volatile const  uint32_t INTEN;
  volatile  uint32_t INTEN_SET;
  volatile  uint32_t INTEN_CLR;
  volatile const  uint32_t CNTCON;
  volatile  uint32_t CNTCON_SET;
  volatile  uint32_t CNTCON_CLR;
  volatile const  uint32_t INTF;
  volatile  uint32_t INTF_SET;
  volatile  uint32_t INTF_CLR;
  volatile  uint32_t CAP_CLR;
} LPC_MCPWM_TypeDef;

 
typedef struct
{
  volatile  uint32_t CON;
  volatile const  uint32_t STAT;
  volatile uint32_t CONF;
  volatile const  uint32_t POS;
  volatile uint32_t MAXPOS;
  volatile uint32_t CMPOS0;
  volatile uint32_t CMPOS1;
  volatile uint32_t CMPOS2;
  volatile const  uint32_t INXCNT;
  volatile uint32_t INXCMP0;
  volatile uint32_t LOAD;
  volatile const  uint32_t TIME;
  volatile const  uint32_t VEL;
  volatile const  uint32_t CAP;
  volatile uint32_t VELCOMP;
  volatile uint32_t FILTERPHA;
  volatile uint32_t FILTERPHB;
  volatile uint32_t FILTERINX;
  volatile uint32_t WINDOW;
  volatile uint32_t INXCMP1;
  volatile uint32_t INXCMP2;
       uint32_t RESERVED0[993];
  volatile  uint32_t IEC;
  volatile  uint32_t IES;
  volatile const  uint32_t INTSTAT;
  volatile const  uint32_t IE;
  volatile  uint32_t CLR;
  volatile  uint32_t SET;
} LPC_QEI_TypeDef;

 
typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLOCK;
  volatile uint32_t ARGUMENT;
  volatile uint32_t COMMAND;
  volatile const  uint32_t RESP_CMD;
  volatile const  uint32_t RESP0;
  volatile const  uint32_t RESP1;
  volatile const  uint32_t RESP2;
  volatile const  uint32_t RESP3;
  volatile uint32_t DATATMR;
  volatile uint32_t DATALEN;
  volatile uint32_t DATACTRL;
  volatile const  uint32_t DATACNT;
  volatile const  uint32_t STATUS;
  volatile  uint32_t CLEAR;
  volatile uint32_t MASK0;
       uint32_t RESERVED0[2];
  volatile const  uint32_t FIFOCNT;
       uint32_t RESERVED1[13];
  volatile uint32_t FIFO;
} LPC_MCI_TypeDef;

 
typedef struct
{
  volatile uint32_t mask[512];               
} LPC_CANAF_RAM_TypeDef;

typedef struct                           
{
	
	volatile uint32_t AFMR;

	
	volatile uint32_t SFF_sa;

	
	volatile uint32_t SFF_GRP_sa;

	
	volatile uint32_t EFF_sa;

	
	volatile uint32_t EFF_GRP_sa;

	
	volatile uint32_t ENDofTable;

	
	volatile const  uint32_t LUTerrAd;

	
	volatile const  uint32_t LUTerr;

	
	volatile uint32_t FCANIE;

	
	volatile uint32_t FCANIC0;

	
	volatile uint32_t FCANIC1;
} LPC_CANAF_TypeDef;

typedef struct                           
{
  volatile const  uint32_t TxSR;
  volatile const  uint32_t RxSR;
  volatile const  uint32_t MSR;
} LPC_CANCR_TypeDef;

typedef struct                           
{
	
	volatile uint32_t MOD;

	
	volatile  uint32_t CMR;

	
	volatile uint32_t GSR;

	
	volatile const  uint32_t ICR;

	
	volatile uint32_t IER;

	
	volatile uint32_t BTR;

	
	volatile uint32_t EWL;

	
	volatile const  uint32_t SR;

	
	volatile uint32_t RFS;

	
	volatile uint32_t RID;

	
	volatile uint32_t RDA;

	
	volatile uint32_t RDB;

	
	volatile uint32_t TFI1;

	
	volatile uint32_t TID1;

	
	volatile uint32_t TDA1;

	
	volatile uint32_t TDB1;

	
	volatile uint32_t TFI2;

	
	volatile uint32_t TID2;

	
	volatile uint32_t TDA2;

	
	volatile uint32_t TDB2;

	
	volatile uint32_t TFI3;

	
	volatile uint32_t TID3;

	
	volatile uint32_t TDA3;

	
	volatile uint32_t TDB3;
} LPC_CAN_TypeDef;

 
typedef struct                           
{
  volatile const  uint32_t IntStat;
  volatile const  uint32_t IntTCStat;
  volatile  uint32_t IntTCClear;
  volatile const  uint32_t IntErrStat;
  volatile  uint32_t IntErrClr;
  volatile const  uint32_t RawIntTCStat;
  volatile const  uint32_t RawIntErrStat;
  volatile const  uint32_t EnbldChns;
  volatile uint32_t SoftBReq;
  volatile uint32_t SoftSReq;
  volatile uint32_t SoftLBReq;
  volatile uint32_t SoftLSReq;
  volatile uint32_t Config;
  volatile uint32_t Sync;
} LPC_GPDMA_TypeDef;

typedef struct                           
{
  volatile uint32_t CSrcAddr;
  volatile uint32_t CDestAddr;
  volatile uint32_t CLLI;
  volatile uint32_t CControl;
  volatile uint32_t CConfig;
} LPC_GPDMACH_TypeDef;

 
typedef struct
{
  volatile const  uint32_t Revision;              
  volatile uint32_t Control;
  volatile uint32_t CommandStatus;
  volatile uint32_t InterruptStatus;
  volatile uint32_t InterruptEnable;
  volatile uint32_t InterruptDisable;
  volatile uint32_t HCCA;
  volatile const  uint32_t PeriodCurrentED;
  volatile uint32_t ControlHeadED;
  volatile uint32_t ControlCurrentED;
  volatile uint32_t BulkHeadED;
  volatile uint32_t BulkCurrentED;
  volatile const  uint32_t DoneHead;
  volatile uint32_t FmInterval;
  volatile const  uint32_t FmRemaining;
  volatile const  uint32_t FmNumber;
  volatile uint32_t PeriodicStart;
  volatile uint32_t LSTreshold;
  volatile uint32_t RhDescriptorA;
  volatile uint32_t RhDescriptorB;
  volatile uint32_t RhStatus;
  volatile uint32_t RhPortStatus1;
  volatile uint32_t RhPortStatus2;
       uint32_t RESERVED0[40];
  volatile const  uint32_t Module_ID;

  volatile const  uint32_t IntSt;                
  volatile uint32_t IntEn;
  volatile  uint32_t IntSet;
  volatile  uint32_t IntClr;
  volatile uint32_t StCtrl;
  volatile uint32_t Tmr;
       uint32_t RESERVED1[58];

  volatile const  uint32_t DevIntSt;             
  volatile uint32_t DevIntEn;
  volatile  uint32_t DevIntClr;
  volatile  uint32_t DevIntSet;

  volatile  uint32_t CmdCode;              
  volatile const  uint32_t CmdData;

  volatile const  uint32_t RxData;               
  volatile  uint32_t TxData;
  volatile const  uint32_t RxPLen;
  volatile  uint32_t TxPLen;
  volatile uint32_t Ctrl;
  volatile  uint32_t DevIntPri;

  volatile const  uint32_t EpIntSt;              
  volatile uint32_t EpIntEn;
  volatile  uint32_t EpIntClr;
  volatile  uint32_t EpIntSet;
  volatile  uint32_t EpIntPri;

  volatile uint32_t ReEp;                 
  volatile  uint32_t EpInd;
  volatile uint32_t MaxPSize;

  volatile const  uint32_t DMARSt;               
  volatile  uint32_t DMARClr;
  volatile  uint32_t DMARSet;
       uint32_t RESERVED2[9];
  volatile uint32_t UDCAH;
  volatile const  uint32_t EpDMASt;
  volatile  uint32_t EpDMAEn;
  volatile  uint32_t EpDMADis;
  volatile const  uint32_t DMAIntSt;
  volatile uint32_t DMAIntEn;
       uint32_t RESERVED3[2];
  volatile const  uint32_t EoTIntSt;
  volatile  uint32_t EoTIntClr;
  volatile  uint32_t EoTIntSet;
  volatile const  uint32_t NDDRIntSt;
  volatile  uint32_t NDDRIntClr;
  volatile  uint32_t NDDRIntSet;
  volatile const  uint32_t SysErrIntSt;
  volatile  uint32_t SysErrIntClr;
  volatile  uint32_t SysErrIntSet;
       uint32_t RESERVED4[15];

  union {
  volatile const  uint32_t I2C_RX;                  
  volatile  uint32_t I2C_TX;
  };
  volatile  uint32_t I2C_STS;
  volatile uint32_t I2C_CTL;
  volatile uint32_t I2C_CLKHI;
  volatile  uint32_t I2C_CLKLO;
       uint32_t RESERVED5[824];

  union {
  volatile uint32_t USBClkCtrl;              
  volatile uint32_t OTGClkCtrl;
  };
  union {
  volatile const  uint32_t USBClkSt;
  volatile const  uint32_t OTGClkSt;
  };
} LPC_USB_TypeDef;

 
typedef struct
{
  volatile uint32_t MAC1;                    
  volatile uint32_t MAC2;
  volatile uint32_t IPGT;
  volatile uint32_t IPGR;
  volatile uint32_t CLRT;
  volatile uint32_t MAXF;
  volatile uint32_t SUPP;
  volatile uint32_t TEST;
  volatile uint32_t MCFG;
  volatile uint32_t MCMD;
  volatile uint32_t MADR;
  volatile  uint32_t MWTD;
  volatile const  uint32_t MRDD;
  volatile const  uint32_t MIND;
       uint32_t RESERVED0[2];
  volatile uint32_t SA0;
  volatile uint32_t SA1;
  volatile uint32_t SA2;
       uint32_t RESERVED1[45];
  volatile uint32_t Command;                 
  volatile const  uint32_t Status;
  volatile uint32_t RxDescriptor;
  volatile uint32_t RxStatus;
  volatile uint32_t RxDescriptorNumber;
  volatile const  uint32_t RxProduceIndex;
  volatile uint32_t RxConsumeIndex;
  volatile uint32_t TxDescriptor;
  volatile uint32_t TxStatus;
  volatile uint32_t TxDescriptorNumber;
  volatile uint32_t TxProduceIndex;
  volatile const  uint32_t TxConsumeIndex;
       uint32_t RESERVED2[10];
  volatile const  uint32_t TSV0;
  volatile const  uint32_t TSV1;
  volatile const  uint32_t RSV;
       uint32_t RESERVED3[3];
  volatile uint32_t FlowControlCounter;
  volatile const  uint32_t FlowControlStatus;
       uint32_t RESERVED4[34];
  volatile uint32_t RxFilterCtrl;            
  volatile const  uint32_t RxFilterWoLStatus;
  volatile  uint32_t RxFilterWoLClear;
       uint32_t RESERVED5;
  volatile uint32_t HashFilterL;
  volatile uint32_t HashFilterH;
       uint32_t RESERVED6[882];
  volatile const  uint32_t IntStatus;               
  volatile uint32_t IntEnable;
  volatile  uint32_t IntClear;
  volatile  uint32_t IntSet;
       uint32_t RESERVED7;
  volatile uint32_t PowerDown;
       uint32_t RESERVED8;
  volatile uint32_t Module_ID;
} LPC_EMAC_TypeDef;

 
typedef struct
{
  volatile uint32_t TIMH;                    
  volatile uint32_t TIMV;
  volatile uint32_t POL;
  volatile uint32_t LE;
  volatile uint32_t UPBASE;
  volatile uint32_t LPBASE;
  volatile uint32_t CTRL;
  volatile uint32_t INTMSK;
  volatile const  uint32_t INTRAW;
  volatile const  uint32_t INTSTAT;
  volatile  uint32_t INTCLR;
  volatile const  uint32_t UPCURR;
  volatile const  uint32_t LPCURR;
       uint32_t RESERVED0[115];
  volatile uint32_t PAL[128];
       uint32_t RESERVED1[256];
  volatile uint32_t CRSR_IMG[256];
  volatile uint32_t CRSR_CTRL;
  volatile uint32_t CRSR_CFG;
  volatile uint32_t CRSR_PAL0;
  volatile uint32_t CRSR_PAL1;
  volatile uint32_t CRSR_XY;
  volatile uint32_t CRSR_CLIP;
       uint32_t RESERVED2[2];
  volatile uint32_t CRSR_INTMSK;
  volatile  uint32_t CRSR_INTCLR;
  volatile const  uint32_t CRSR_INTRAW;
  volatile const  uint32_t CRSR_INTSTAT;
} LPC_LCD_TypeDef;

 
typedef struct
{
  volatile uint32_t Control;
  volatile const  uint32_t Status;
  volatile uint32_t Config;
       uint32_t RESERVED0[5];
  volatile uint32_t DynamicControl;
  volatile uint32_t DynamicRefresh;
  volatile uint32_t DynamicReadConfig;
       uint32_t RESERVED1[1];
  volatile uint32_t DynamicRP;
  volatile uint32_t DynamicRAS;
  volatile uint32_t DynamicSREX;
  volatile uint32_t DynamicAPR;
  volatile uint32_t DynamicDAL;
  volatile uint32_t DynamicWR;
  volatile uint32_t DynamicRC;
  volatile uint32_t DynamicRFC;
  volatile uint32_t DynamicXSR;
  volatile uint32_t DynamicRRD;
  volatile uint32_t DynamicMRD;
       uint32_t RESERVED2[9];
  volatile uint32_t StaticExtendedWait;
       uint32_t RESERVED3[31];
  volatile uint32_t DynamicConfig0;
  volatile uint32_t DynamicRasCas0;
       uint32_t RESERVED4[6];
  volatile uint32_t DynamicConfig1;
  volatile uint32_t DynamicRasCas1;
       uint32_t RESERVED5[6];
  volatile uint32_t DynamicConfig2;
  volatile uint32_t DynamicRasCas2;
       uint32_t RESERVED6[6];
  volatile uint32_t DynamicConfig3;
  volatile uint32_t DynamicRasCas3;
       uint32_t RESERVED7[38];
  volatile uint32_t StaticConfig0;
  volatile uint32_t StaticWaitWen0;
  volatile uint32_t StaticWaitOen0;
  volatile uint32_t StaticWaitRd0;
  volatile uint32_t StaticWaitPage0;
  volatile uint32_t StaticWaitWr0;
  volatile uint32_t StaticWaitTurn0;
       uint32_t RESERVED8[1];
  volatile uint32_t StaticConfig1;
  volatile uint32_t StaticWaitWen1;
  volatile uint32_t StaticWaitOen1;
  volatile uint32_t StaticWaitRd1;
  volatile uint32_t StaticWaitPage1;
  volatile uint32_t StaticWaitWr1;
  volatile uint32_t StaticWaitTurn1;
       uint32_t RESERVED9[1];
  volatile uint32_t StaticConfig2;
  volatile uint32_t StaticWaitWen2;
  volatile uint32_t StaticWaitOen2;
  volatile uint32_t StaticWaitRd2;
  volatile uint32_t StaticWaitPage2;
  volatile uint32_t StaticWaitWr2;
  volatile uint32_t StaticWaitTurn2;
       uint32_t RESERVED10[1];
  volatile uint32_t StaticConfig3;
  volatile uint32_t StaticWaitWen3;
  volatile uint32_t StaticWaitOen3;
  volatile uint32_t StaticWaitRd3;
  volatile uint32_t StaticWaitPage3;
  volatile uint32_t StaticWaitWr3;
  volatile uint32_t StaticWaitTurn3;
} LPC_EMC_TypeDef;

 
typedef struct
{
  volatile uint32_t MODE;
  volatile uint32_t SEED;
  union {
  volatile const  uint32_t SUM;
  volatile  uint32_t WR_DATA_DWORD;
  volatile  uint16_t WR_DATA_WORD;
       uint16_t RESERVED_WORD;
  volatile  uint8_t WR_DATA_BYTE;
       uint8_t RESERVED_BYTE[3];
  };
} LPC_CRC_TypeDef;

 
typedef struct
{
  volatile uint32_t CMD;			 
  volatile uint32_t ADDR;
  volatile uint32_t WDATA;
  volatile uint32_t RDATA;
  volatile uint32_t WSTATE;			 
  volatile uint32_t CLKDIV;
  volatile uint32_t PWRDWN;			 
       uint32_t RESERVED0[975];
  volatile uint32_t INT_CLR_ENABLE;	 
  volatile uint32_t INT_SET_ENABLE;
  volatile uint32_t INT_STATUS;		 
  volatile uint32_t INT_ENABLE;
  volatile uint32_t INT_CLR_STATUS;
  volatile uint32_t INT_SET_STATUS;
} LPC_EEPROM_TypeDef;


#pragma no_anon_unions


 
 
 
 
#line 1322 "..\\..\\Core\\CM3\\DeviceSupport\\NXP\\LPC177x_8x\\LPC177x_8x.h"

 
#line 1343 "..\\..\\Core\\CM3\\DeviceSupport\\NXP\\LPC177x_8x\\LPC177x_8x.h"

 
#line 1359 "..\\..\\Core\\CM3\\DeviceSupport\\NXP\\LPC177x_8x\\LPC177x_8x.h"

 
#line 1381 "..\\..\\Core\\CM3\\DeviceSupport\\NXP\\LPC177x_8x\\LPC177x_8x.h"




 
 
 
#line 1442 "..\\..\\Core\\CM3\\DeviceSupport\\NXP\\LPC177x_8x\\LPC177x_8x.h"

#line 5 "..\\User\\main.h"
#line 1 "..\\User\\UART2.h"



int fputc(int , FILE *);  







#line 6 "..\\User\\main.h"
#line 1 "..\\..\\BoardSupport\\bsp.h"


 
























 


 



 






 

 

 

 


 










#line 108 "..\\..\\BoardSupport\\bsp.h"



#line 1 "..\\..\\BoardSupport\\phylan_lan8720.h"


 





















 


 



 
 













 




 







 
#line 70 "..\\..\\BoardSupport\\phylan_lan8720.h"


 
#line 82 "..\\..\\BoardSupport\\phylan_lan8720.h"

 
#line 95 "..\\..\\BoardSupport\\phylan_lan8720.h"















#line 117 "..\\..\\BoardSupport\\phylan_lan8720.h"
















 








#line 112 "..\\..\\BoardSupport\\bsp.h"



































































#line 220 "..\\..\\BoardSupport\\bsp.h"






 
#line 7 "..\\User\\main.h"



#line 38 "..\\User\\arch/cc.h"











#line 56 "..\\User\\arch/cc.h"





 
#line 71 "..\\User\\arch/cc.h"






#line 92 "..\\User\\arch/cc.h"




#line 49 "..\\Lwip\\include\\lwip/arch.h"











 




 




 








 
#line 83 "..\\Lwip\\include\\lwip/arch.h"
#line 1 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
 
 
 




 
 



 






   














  


 








#line 54 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


  



    typedef unsigned int size_t;    
#line 70 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"






    



    typedef unsigned short wchar_t;  
#line 91 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { long long quot, rem; } lldiv_t;
    


#line 112 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   



 

   




 
#line 131 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) long long atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) long long strtoll(const char * __restrict  ,
                                  char ** __restrict  , int  )
                          __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned long long strtoull(const char * __restrict  ,
                                            char ** __restrict  , int  )
                                   __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 436 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 524 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 553 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"

extern __declspec(__nothrow) __attribute__((const)) int abs(int  );
   



 

extern __declspec(__nothrow) __attribute__((const)) div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __attribute__((const)) long int labs(long int  );
   



 




extern __declspec(__nothrow) __attribute__((const)) ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __attribute__((const)) long long llabs(long long  );
   



 




extern __declspec(__nothrow) __attribute__((const)) lldiv_t lldiv(long long  , long long  );
   











 
#line 634 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"




 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __attribute__((const)) __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 




 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 892 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stdlib.h"





 
#line 84 "..\\Lwip\\include\\lwip/arch.h"






 
#line 97 "..\\Lwip\\include\\lwip/arch.h"




 




#line 1 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 108 "..\\Lwip\\include\\lwip/arch.h"





 




 
#line 121 "..\\Lwip\\include\\lwip/arch.h"
 



typedef uint8_t   u8_t;
typedef int8_t    s8_t;
typedef uint16_t  u16_t;
typedef int16_t   s16_t;
typedef uint32_t  u32_t;
typedef int32_t   s32_t;

typedef uint64_t  u64_t;
typedef int64_t   s64_t;

typedef uintptr_t mem_ptr_t;





 




 
#line 1 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"
 
 





 

 








#line 20 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"



#line 33 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"


     

#line 51 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 66 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 81 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 96 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 111 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 126 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 141 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 156 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 171 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 186 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

#line 201 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"





    



      typedef unsigned short wchar_t;  
#line 221 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\inttypes.h"

typedef struct imaxdiv_t { intmax_t quot, rem; } imaxdiv_t;
    





__declspec(__nothrow) intmax_t strtoimax(const char * __restrict  ,
                   char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
     
__declspec(__nothrow) uintmax_t strtoumax(const char * __restrict  ,
                    char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
     

__declspec(__nothrow) intmax_t wcstoimax(const wchar_t * __restrict  ,
                   wchar_t ** __restrict  , int  ) __attribute__((__nonnull__(1)));
__declspec(__nothrow) uintmax_t wcstoumax(const wchar_t * __restrict  ,
                    wchar_t ** __restrict  , int  ) __attribute__((__nonnull__(1)));

extern __declspec(__nothrow) __attribute__((const)) intmax_t imaxabs(intmax_t  );
   



 
extern __declspec(__nothrow) __attribute__((const)) imaxdiv_t imaxdiv(intmax_t  , intmax_t  );
   











 







 

#line 149 "..\\Lwip\\include\\lwip/arch.h"
#line 174 "..\\Lwip\\include\\lwip/arch.h"




 




 
#line 1 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
 
 
 





 






     

     

     

     
#line 30 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
       

       






#line 45 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\limits.h"
     
     


     

     

     

     

     

     





     





     





     


       

       

       




 

#line 186 "..\\Lwip\\include\\lwip/arch.h"






 
#line 202 "..\\Lwip\\include\\lwip/arch.h"
typedef int ssize_t;



 






 




#line 1 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\ctype.h"
 
 
 
 





 






 








#line 35 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\ctype.h"






#line 49 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\ctype.h"

 
#line 59 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\ctype.h"

 
 









 
#line 81 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\ctype.h"





#line 133 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\ctype.h"

extern __declspec(__nothrow) __attribute__((const)) unsigned char **__rt_ctype_table(void);







    extern int (isalnum)(int  );

     





    extern int (isalpha)(int  );

     





    extern int (iscntrl)(int  );

     
     

 




    extern int (isdigit)(int  );

     

    extern int (isblank)(int  );
     
     
     





    extern int (isgraph)(int  );

     





    extern int (islower)(int  );

     





    extern int (isprint)(int  );

     
     





    extern int (ispunct)(int  );

     





    extern int (isspace)(int  );

     





    extern int (isupper)(int  );

     

 
 

static inline int __isxdigit_helper(int __t) { return (__t ^ (__t << 2)); }




    extern int (isxdigit)(int  );

     



extern int tolower(int  );
     
     

extern int toupper(int  );
     
     







#line 272 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\ctype.h"



 

#line 229 "..\\Lwip\\include\\lwip/arch.h"
#line 237 "..\\Lwip\\include\\lwip/arch.h"

 




 






 




 













 







 







 






 












 








 








 
#line 331 "..\\Lwip\\include\\lwip/arch.h"





 








 








 











 




 








 






 





#line 41 "..\\Lwip\\include\\lwip/debug.h"
#line 1 "..\\Lwip\\include\\lwip/opt.h"




 































 




 

#line 42 "..\\Lwip\\include\\lwip/debug.h"





 



 
 

 

 

 



 


 




 
 

 



 



 
 

 

 

 



 



 





 



 






 

#line 121 "..\\Lwip\\include\\lwip/debug.h"

#line 130 "..\\Lwip\\include\\lwip/debug.h"

 






 





#line 160 "..\\Lwip\\include\\lwip/debug.h"

#line 53 "..\\Lwip\\include\\lwip/opt.h"
















 

 



 




 






 





 





 




 
#line 112 "..\\Lwip\\include\\lwip/opt.h"







 





 





 



 







 








 





 





 




 






 











 











 










 













 








 





 





 




 




 












 








 








 







 












 







 












 







 









 







 









 





















 





 





 




 




 







 








 







 







 







 









 







 










 









 









 







 






 







 







 








 








 








 







 







 






 






 





 






 






 





 





 




 


 






 







 










 







 













 





 








 






 







 





 





 




 


 








 








 








 




#line 766 "..\\Lwip\\include\\lwip/opt.h"





 








 











 






 








 







 










 





 





 




 



 






 






 






 





 





 




 


 






 





 





 




 


 
#line 925 "..\\Lwip\\include\\lwip/opt.h"



 






 








 






 








 





 





 




 


 
#line 989 "..\\Lwip\\include\\lwip/opt.h"




 










 





 





 




 




 





 





 




 




 





 





 




 


 
#line 1076 "..\\Lwip\\include\\lwip/opt.h"


 





 




 



 




 




 







 




 




 







 




 











 





 





 





 





 




 


 






 






 






 





 





 




 


 






 










 






 






 







 






 













 










 











 








 







 








 








 








 









 
#line 1365 "..\\Lwip\\include\\lwip/opt.h"





 









 
#line 1388 "..\\Lwip\\include\\lwip/opt.h"



 








 

















 









 







 










 
#line 1459 "..\\Lwip\\include\\lwip/opt.h"








 









 















 








 






 





 




 




 
#line 1533 "..\\Lwip\\include\\lwip/opt.h"




 








 







 





 





 




 



 







 






 







 








 







 







 










 





















 







 





 





 




 




 






 







 







 
















 





 





 




 


 








 








 








 







 






 








 








 






 








 








 








 








 








 








 





 





 




 


 






 













 











 





 





 




 


 









 








 










 








 







 







 







 






 






 






 






 






 








 











 









 







 





 





 




 


 




#line 2234 "..\\Lwip\\include\\lwip/opt.h"

#line 2253 "..\\Lwip\\include\\lwip/opt.h"




 





 




 




 






 






 






 






 






 






 






 






 






 






 







 





 





 




 


 








 




















 









 






 






 






 






 







 






 










 






 





 





 


 







 






 





 





 




 









 





 





 



 






 






 






 






 






 







 







 






 






 







 






 







 







 








 








 





 





 


 







 






 








 






 








 





 





 






 





 
























 





























 





















 























 


















 















 
















 




















 





















 


















 
















 





















 

















 























 










 


















 




























 



























 



























 






















 






















 






















 





 





 




 





 








 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 







 






 






 






 






 






 






 






 






 






 






 






 






 






 





 



 








 




 



 





 

#line 67 "..\\Lwip\\core\\ipv4\\dhcp.c"



#line 1 "..\\Lwip\\include\\lwip/stats.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/stats.h"

#line 1 "..\\Lwip\\include\\lwip/mem.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/mem.h"





#line 59 "..\\Lwip\\include\\lwip/mem.h"



 




typedef u16_t mem_size_t;




void  mem_init(void);
void *mem_trim(void *mem, mem_size_t size);
void *mem_malloc(mem_size_t size);
void *mem_calloc(mem_size_t count, mem_size_t size);
void  mem_free(void *mem);





#line 43 "..\\Lwip\\include\\lwip/stats.h"
#line 1 "..\\Lwip\\include\\lwip/memp.h"



 































 




#line 42 "..\\Lwip\\include\\lwip/memp.h"





 
#line 1 "..\\Lwip\\include\\lwip/priv/memp_std.h"






 











 


 







 









 
































#line 89 "..\\Lwip\\include\\lwip/priv/memp_std.h"
















#line 111 "..\\Lwip\\include\\lwip/priv/memp_std.h"





















 







 







 
#line 50 "..\\Lwip\\include\\lwip/memp.h"

 
typedef enum {
#line 1 "..\\Lwip\\include\\lwip/priv/memp_std.h"






 











 


 







 









 





MEMP_UDP_PCB,



MEMP_TCP_PCB,
MEMP_TCP_PCB_LISTEN,
MEMP_TCP_SEG,







MEMP_REASSDATA,


MEMP_FRAG_PBUF,



MEMP_NETBUF,
MEMP_NETCONN,



MEMP_TCPIP_MSG_API,
#line 89 "..\\Lwip\\include\\lwip/priv/memp_std.h"
MEMP_TCPIP_MSG_INPKT,












MEMP_SYS_TIMEOUT,


#line 111 "..\\Lwip\\include\\lwip/priv/memp_std.h"





















 
MEMP_PBUF,
MEMP_PBUF_POOL,





 







 
#line 55 "..\\Lwip\\include\\lwip/memp.h"
  MEMP_MAX
} memp_t;

#line 1 "..\\Lwip\\include\\lwip/priv/memp_priv.h"



 































 




#line 42 "..\\Lwip\\include\\lwip/priv/memp_priv.h"





#line 48 "..\\Lwip\\include\\lwip/priv/memp_priv.h"
#line 1 "..\\Lwip\\include\\lwip/priv/mem_priv.h"



 































 




#line 42 "..\\Lwip\\include\\lwip/priv/mem_priv.h"





#line 48 "..\\Lwip\\include\\lwip/priv/mem_priv.h"

#line 79 "..\\Lwip\\include\\lwip/priv/mem_priv.h"





#line 49 "..\\Lwip\\include\\lwip/priv/memp_priv.h"

#line 58 "..\\Lwip\\include\\lwip/priv/memp_priv.h"




 






struct memp {
  struct memp *next;




};


#line 106 "..\\Lwip\\include\\lwip/priv/memp_priv.h"

 
struct memp_desc {
#line 117 "..\\Lwip\\include\\lwip/priv/memp_priv.h"

   
  u16_t size;


   
  u16_t num;

   
  u8_t *base;

   
  struct memp **tab;

};







#line 146 "..\\Lwip\\include\\lwip/priv/memp_priv.h"

void memp_init_pool(const struct memp_desc *desc);





void *memp_malloc_pool(const struct memp_desc *desc);

void  memp_free_pool(const struct memp_desc* desc, void *mem);





#line 59 "..\\Lwip\\include\\lwip/memp.h"
#line 1 "..\\Lwip\\include\\lwip/stats.h"



 































 
#line 60 "..\\Lwip\\include\\lwip/memp.h"

extern const struct memp_desc* const memp_pools[MEMP_MAX];




 


#line 80 "..\\Lwip\\include\\lwip/memp.h"














 
#line 110 "..\\Lwip\\include\\lwip/memp.h"






 




 




 


#line 140 "..\\Lwip\\include\\lwip/memp.h"

void  memp_init(void);





void *memp_malloc(memp_t type);

void  memp_free(memp_t type, void *mem);





#line 44 "..\\Lwip\\include\\lwip/stats.h"





#line 323 "..\\Lwip\\include\\lwip/stats.h"

#line 331 "..\\Lwip\\include\\lwip/stats.h"

#line 339 "..\\Lwip\\include\\lwip/stats.h"

#line 347 "..\\Lwip\\include\\lwip/stats.h"

#line 355 "..\\Lwip\\include\\lwip/stats.h"

#line 363 "..\\Lwip\\include\\lwip/stats.h"

#line 371 "..\\Lwip\\include\\lwip/stats.h"

#line 379 "..\\Lwip\\include\\lwip/stats.h"

#line 387 "..\\Lwip\\include\\lwip/stats.h"

#line 401 "..\\Lwip\\include\\lwip/stats.h"

#line 411 "..\\Lwip\\include\\lwip/stats.h"

#line 423 "..\\Lwip\\include\\lwip/stats.h"

#line 431 "..\\Lwip\\include\\lwip/stats.h"

#line 439 "..\\Lwip\\include\\lwip/stats.h"

#line 447 "..\\Lwip\\include\\lwip/stats.h"

#line 455 "..\\Lwip\\include\\lwip/stats.h"

#line 463 "..\\Lwip\\include\\lwip/stats.h"







 
#line 486 "..\\Lwip\\include\\lwip/stats.h"





#line 71 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 72 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 1 "..\\Lwip\\include\\lwip/udp.h"




 































 



#line 42 "..\\Lwip\\include\\lwip/udp.h"



#line 1 "..\\Lwip\\include\\lwip/pbuf.h"



 































 




#line 42 "..\\Lwip\\include\\lwip/pbuf.h"
#line 1 "..\\Lwip\\include\\lwip/err.h"



 






























 



#line 40 "..\\Lwip\\include\\lwip/err.h"
#line 41 "..\\Lwip\\include\\lwip/err.h"









 

 
typedef enum {
 
  ERR_OK         = 0,
 
  ERR_MEM        = -1,
 
  ERR_BUF        = -2,
 
  ERR_TIMEOUT    = -3,
 
  ERR_RTE        = -4,
 
  ERR_INPROGRESS = -5,
 
  ERR_VAL        = -6,
 
  ERR_WOULDBLOCK = -7,
 
  ERR_USE        = -8,
 
  ERR_ALREADY    = -9,
 
  ERR_ISCONN     = -10,
 
  ERR_CONN       = -11,
 
  ERR_IF         = -12,

 
  ERR_ABRT       = -13,
 
  ERR_RST        = -14,
 
  ERR_CLSD       = -15,
 
  ERR_ARG        = -16
} err_enum_t;


 



typedef s8_t err_t;




 








int err_to_errno(err_t err);






#line 43 "..\\Lwip\\include\\lwip/pbuf.h"










 
















 





 

#line 84 "..\\Lwip\\include\\lwip/pbuf.h"




 
typedef enum {
  

 
  PBUF_TRANSPORT = 0 + (14 + 0) + 20 + 20,
  

 
  PBUF_IP = 0 + (14 + 0) + 20,
  


 
  PBUF_LINK = 0 + (14 + 0),
  



 
  PBUF_RAW_TX = 0,
  
 
  PBUF_RAW = 0
} pbuf_layer;


 


 


 


 



 

 





 

 





 
typedef enum {
  




 
  PBUF_RAM = (0x0200 | 0x80 | 0x00),
  

 
  PBUF_ROM = 0x01,
  

 
  PBUF_REF = (0x40 | 0x01),
  




 
  PBUF_POOL = (0x0100 | 0x80 | 0x02)
} pbuf_type;


 


 

 

 

 

 


 
struct pbuf {
   
  struct pbuf *next;

   
  void *payload;

  





 
  u16_t tot_len;

   
  u16_t len;

  

 
  u8_t type_internal;

   
  u8_t flags;

  



 
  u8_t ref;

   
  u8_t if_idx;
};





 
struct pbuf_rom {
   
  struct pbuf *next;

   
  const void *payload;
};


 
typedef void (*pbuf_free_custom_fn)(struct pbuf *p);

 
struct pbuf_custom {
   
  struct pbuf pbuf;
   
  pbuf_free_custom_fn custom_free_function;
};


 
#line 265 "..\\Lwip\\include\\lwip/pbuf.h"
   



 


struct pbuf *pbuf_alloc(pbuf_layer l, u16_t length, pbuf_type type);
struct pbuf *pbuf_alloc_reference(void *payload, u16_t length, pbuf_type type);

struct pbuf *pbuf_alloced_custom(pbuf_layer l, u16_t length, pbuf_type type,
                                 struct pbuf_custom *p, void *payload_mem,
                                 u16_t payload_mem_len);

void pbuf_realloc(struct pbuf *p, u16_t size);



u8_t pbuf_header(struct pbuf *p, s16_t header_size);
u8_t pbuf_header_force(struct pbuf *p, s16_t header_size);
u8_t pbuf_add_header(struct pbuf *p, size_t header_size_increment);
u8_t pbuf_add_header_force(struct pbuf *p, size_t header_size_increment);
u8_t pbuf_remove_header(struct pbuf *p, size_t header_size);
struct pbuf *pbuf_free_header(struct pbuf *q, u16_t size);
void pbuf_ref(struct pbuf *p);
u8_t pbuf_free(struct pbuf *p);
u16_t pbuf_clen(const struct pbuf *p);
void pbuf_cat(struct pbuf *head, struct pbuf *tail);
void pbuf_chain(struct pbuf *head, struct pbuf *tail);
struct pbuf *pbuf_dechain(struct pbuf *p);
err_t pbuf_copy(struct pbuf *p_to, const struct pbuf *p_from);
u16_t pbuf_copy_partial(const struct pbuf *p, void *dataptr, u16_t len, u16_t offset);
void *pbuf_get_contiguous(const struct pbuf *p, void *buffer, size_t bufsize, u16_t len, u16_t offset);
err_t pbuf_take(struct pbuf *buf, const void *dataptr, u16_t len);
err_t pbuf_take_at(struct pbuf *buf, const void *dataptr, u16_t len, u16_t offset);
struct pbuf *pbuf_skip(struct pbuf* in, u16_t in_offset, u16_t* out_offset);
struct pbuf *pbuf_coalesce(struct pbuf *p, pbuf_layer layer);
struct pbuf *pbuf_clone(pbuf_layer l, pbuf_type type, struct pbuf *p);
#line 310 "..\\Lwip\\include\\lwip/pbuf.h"

u8_t pbuf_get_at(const struct pbuf* p, u16_t offset);
int pbuf_try_get_at(const struct pbuf* p, u16_t offset);
void pbuf_put_at(struct pbuf* p, u16_t offset, u8_t data);
u16_t pbuf_memcmp(const struct pbuf* p, u16_t offset, const void* s2, u16_t n);
u16_t pbuf_memfind(const struct pbuf* p, const void* mem, u16_t mem_len, u16_t start_offset);
u16_t pbuf_strstr(const struct pbuf* p, const char* substr);





#line 46 "..\\Lwip\\include\\lwip/udp.h"
#line 1 "..\\Lwip\\include\\lwip/netif.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/netif.h"



#line 45 "..\\Lwip\\include\\lwip/netif.h"

#line 1 "..\\Lwip\\include\\lwip/ip_addr.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/ip_addr.h"
#line 1 "..\\Lwip\\include\\lwip/def.h"



 































 









 




 
#line 53 "..\\Lwip\\include\\lwip/def.h"
#line 54 "..\\Lwip\\include\\lwip/def.h"
#line 60 "..\\Lwip\\include\\lwip/def.h"








 


 





#line 84 "..\\Lwip\\include\\lwip/def.h"

#line 96 "..\\Lwip\\include\\lwip/def.h"
u16_t lwip_htons(u16_t x);




u32_t lwip_htonl(u32_t x);





 
#line 116 "..\\Lwip\\include\\lwip/def.h"

 
#line 124 "..\\Lwip\\include\\lwip/def.h"





 


 
void  lwip_itoa(char* result, size_t bufsize, int number);


 
int   lwip_strnicmp(const char* str1, const char* str2, size_t len);


 
int   lwip_stricmp(const char* str1, const char* str2);


 
char* lwip_strnstr(const char* buffer, const char* token, size_t n);






#line 42 "..\\Lwip\\include\\lwip/ip_addr.h"

#line 1 "..\\Lwip\\include\\lwip/ip4_addr.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/ip4_addr.h"
#line 42 "..\\Lwip\\include\\lwip/ip4_addr.h"








 
struct ip4_addr {
  u32_t addr;
};


 
typedef struct ip4_addr ip4_addr_t;

 
struct netif;

 

 

 

 





 




























 


 

 



 

 

 

 


 



 

 


 









 










u8_t ip4_addr_isbroadcast_u32(u32_t addr, const struct netif *netif);


u8_t ip4_addr_netmask_valid(u32_t netmask);





#line 174 "..\\Lwip\\include\\lwip/ip4_addr.h"

 






 






 
#line 198 "..\\Lwip\\include\\lwip/ip4_addr.h"



 


u32_t ipaddr_addr(const char *cp);
int ip4addr_aton(const char *cp, ip4_addr_t *addr);
 
char *ip4addr_ntoa(const ip4_addr_t *addr);
char *ip4addr_ntoa_r(const ip4_addr_t *addr, char *buf, int buflen);







#line 44 "..\\Lwip\\include\\lwip/ip_addr.h"
#line 1 "..\\Lwip\\include\\lwip/ip6_addr.h"




 



































 



#line 46 "..\\Lwip\\include\\lwip/ip6_addr.h"
#line 47 "..\\Lwip\\include\\lwip/ip6_addr.h"

#line 351 "..\\Lwip\\include\\lwip/ip6_addr.h"

#line 45 "..\\Lwip\\include\\lwip/ip_addr.h"








 
enum lwip_ip_addr_type {
   
  IPADDR_TYPE_V4 =   0U,
   
  IPADDR_TYPE_V6 =   6U,
   
  IPADDR_TYPE_ANY = 46U
};

#line 261 "..\\Lwip\\include\\lwip/ip_addr.h"









typedef ip4_addr_t ip_addr_t;
#line 284 "..\\Lwip\\include\\lwip/ip_addr.h"

#line 311 "..\\Lwip\\include\\lwip/ip_addr.h"





#line 366 "..\\Lwip\\include\\lwip/ip_addr.h"



extern const ip_addr_t ip_addr_any;
extern const ip_addr_t ip_addr_broadcast;










 





 





 


 

 




#line 426 "..\\Lwip\\include\\lwip/ip_addr.h"

#line 433 "..\\Lwip\\include\\lwip/ip_addr.h"





#line 47 "..\\Lwip\\include\\lwip/netif.h"

#line 49 "..\\Lwip\\include\\lwip/netif.h"
#line 50 "..\\Lwip\\include\\lwip/netif.h"
#line 51 "..\\Lwip\\include\\lwip/netif.h"






 



 







 






 






 


 





 



 



 


 


 




 

enum lwip_internal_netif_client_data_index
{


   LWIP_NETIF_CLIENT_DATA_INDEX_DHCP,
#line 134 "..\\Lwip\\include\\lwip/netif.h"
   LWIP_NETIF_CLIENT_DATA_INDEX_MAX
};

#line 151 "..\\Lwip\\include\\lwip/netif.h"

struct netif;


 
enum netif_mac_filter_action {
   
  NETIF_DEL_MAC_FILTER = 0,
   
  NETIF_ADD_MAC_FILTER = 1
};





 
typedef err_t (*netif_init_fn)(struct netif *netif);








 
typedef err_t (*netif_input_fn)(struct pbuf *p, struct netif *inp);









 
typedef err_t (*netif_output_fn)(struct netif *netif, struct pbuf *p,
       const ip4_addr_t *ipaddr);


#line 205 "..\\Lwip\\include\\lwip/netif.h"






 
typedef err_t (*netif_linkoutput_fn)(struct netif *netif, struct pbuf *p);
 
typedef void (*netif_status_callback_fn)(struct netif *netif);
#line 225 "..\\Lwip\\include\\lwip/netif.h"







 



 







typedef u8_t netif_addr_idx_t;



#line 256 "..\\Lwip\\include\\lwip/netif.h"



 
struct netif {

   
  struct netif *next;



   
  ip_addr_t ip_addr;
  ip_addr_t netmask;
  ip_addr_t gw;
#line 286 "..\\Lwip\\include\\lwip/netif.h"
  
 
  netif_input_fn input;

  


 
  netif_output_fn output;

  

 
  netif_linkoutput_fn linkoutput;
#line 321 "..\\Lwip\\include\\lwip/netif.h"
  
 
  void *state;

  void* client_data[LWIP_NETIF_CLIENT_DATA_INDEX_MAX + 0];
#line 334 "..\\Lwip\\include\\lwip/netif.h"
   
  u16_t mtu;




   
  u8_t hwaddr[6U];
   
  u8_t hwaddr_len;
   
  u8_t flags;
   
  char name[2];
  
 
  u8_t num;





   
  u8_t rs_count;
#line 390 "..\\Lwip\\include\\lwip/netif.h"
};

#line 400 "..\\Lwip\\include\\lwip/netif.h"




 
extern struct netif *netif_list;


 
extern struct netif *netif_default;

void netif_init(void);

struct netif *netif_add_noaddr(struct netif *netif, void *state, netif_init_fn init, netif_input_fn input);


struct netif *netif_add(struct netif *netif,
                            const ip4_addr_t *ipaddr, const ip4_addr_t *netmask, const ip4_addr_t *gw,
                            void *state, netif_init_fn init, netif_input_fn input);
void netif_set_addr(struct netif *netif, const ip4_addr_t *ipaddr, const ip4_addr_t *netmask,
                    const ip4_addr_t *gw);



void netif_remove(struct netif * netif);




 
struct netif *netif_find(const char *name);

void netif_set_default(struct netif *netif);


void netif_set_ipaddr(struct netif *netif, const ip4_addr_t *ipaddr);
void netif_set_netmask(struct netif *netif, const ip4_addr_t *netmask);
void netif_set_gw(struct netif *netif, const ip4_addr_t *gw);
 

 

 

 

 

 







void netif_set_up(struct netif *netif);
void netif_set_down(struct netif *netif);


 


#line 469 "..\\Lwip\\include\\lwip/netif.h"

void netif_set_link_up(struct netif *netif);
void netif_set_link_down(struct netif *netif);
 






#line 485 "..\\Lwip\\include\\lwip/netif.h"







#line 498 "..\\Lwip\\include\\lwip/netif.h"

#line 506 "..\\Lwip\\include\\lwip/netif.h"

err_t netif_input(struct pbuf *p, struct netif *inp);

#line 542 "..\\Lwip\\include\\lwip/netif.h"

#line 550 "..\\Lwip\\include\\lwip/netif.h"

u8_t netif_name_to_index(const char *name);
char * netif_index_to_name(u8_t idx, char *name);
struct netif* netif_get_by_index(u8_t idx);

 







 
typedef u16_t netif_nsc_reason_t;

 

 

 

 



 

 

 

 

 

 

 




 
typedef union
{
   
  struct link_changed_s
  {
     
    u8_t state;
  } link_changed;
   
  struct status_changed_s
  {
     
    u8_t state;
  } status_changed;
   
  struct ipv4_changed_s
  {
     
    const ip_addr_t* old_address;
    const ip_addr_t* old_netmask;
    const ip_addr_t* old_gw;
  } ipv4_changed;
   
  struct ipv6_set_s
  {
     
    s8_t addr_index;
     
    const ip_addr_t* old_address;
  } ipv6_set;
   
  struct ipv6_addr_state_changed_s
  {
     
    s8_t addr_index;
     
    u8_t old_state;
     
    const ip_addr_t* address;
  } ipv6_addr_state_changed;
} netif_ext_callback_args_t;








 
typedef void (*netif_ext_callback_fn)(struct netif* netif, netif_nsc_reason_t reason, const netif_ext_callback_args_t* args);

#line 664 "..\\Lwip\\include\\lwip/netif.h"





#line 47 "..\\Lwip\\include\\lwip/udp.h"
#line 48 "..\\Lwip\\include\\lwip/udp.h"
#line 1 "..\\Lwip\\include\\lwip/ip.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/ip.h"

#line 43 "..\\Lwip\\include\\lwip/ip.h"
#line 44 "..\\Lwip\\include\\lwip/ip.h"
#line 45 "..\\Lwip\\include\\lwip/ip.h"
#line 46 "..\\Lwip\\include\\lwip/ip.h"
#line 47 "..\\Lwip\\include\\lwip/ip.h"
#line 1 "..\\Lwip\\include\\lwip/ip4.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/ip4.h"



#line 45 "..\\Lwip\\include\\lwip/ip4.h"
#line 46 "..\\Lwip\\include\\lwip/ip4.h"
#line 47 "..\\Lwip\\include\\lwip/ip4.h"
#line 48 "..\\Lwip\\include\\lwip/ip4.h"
#line 49 "..\\Lwip\\include\\lwip/ip4.h"
#line 1 "..\\Lwip\\include\\lwip/prot/ip4.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/prot/ip4.h"
#line 42 "..\\Lwip\\include\\lwip/prot/ip4.h"






 



__packed
struct ip4_addr_packed {
  u32_t addr;
} ;





typedef struct ip4_addr_packed ip4_addr_p_t;

 

 





__packed
 
struct ip_hdr {
   
  u8_t _v_hl;
   
  u8_t _tos;
   
  u16_t _len;
   
  u16_t _id;
   
  u16_t _offset;




   
  u8_t _ttl;
   
  u8_t _proto;
   
  u16_t _chksum;
   
  ip4_addr_p_t src;
  ip4_addr_p_t dest;
} ;





 
#line 115 "..\\Lwip\\include\\lwip/prot/ip4.h"

 
#line 125 "..\\Lwip\\include\\lwip/prot/ip4.h"






#line 50 "..\\Lwip\\include\\lwip/ip4.h"











 



struct netif *ip4_route(const ip4_addr_t *dest);





err_t ip4_input(struct pbuf *p, struct netif *inp);
err_t ip4_output(struct pbuf *p, const ip4_addr_t *src, const ip4_addr_t *dest,
       u8_t ttl, u8_t tos, u8_t proto);
err_t ip4_output_if(struct pbuf *p, const ip4_addr_t *src, const ip4_addr_t *dest,
       u8_t ttl, u8_t tos, u8_t proto, struct netif *netif);
err_t ip4_output_if_src(struct pbuf *p, const ip4_addr_t *src, const ip4_addr_t *dest,
       u8_t ttl, u8_t tos, u8_t proto, struct netif *netif);
#line 90 "..\\Lwip\\include\\lwip/ip4.h"






















#line 48 "..\\Lwip\\include\\lwip/ip.h"
#line 1 "..\\Lwip\\include\\lwip/ip6.h"




 


































 



#line 45 "..\\Lwip\\include\\lwip/ip6.h"

#line 92 "..\\Lwip\\include\\lwip/ip6.h"

#line 49 "..\\Lwip\\include\\lwip/ip.h"
#line 1 "..\\Lwip\\include\\lwip/prot/ip.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/prot/ip.h"











 






#line 50 "..\\Lwip\\include\\lwip/ip.h"







 



 













 
#line 90 "..\\Lwip\\include\\lwip/ip.h"

struct ip_pcb {
   
  ip_addr_t local_ip; ip_addr_t remote_ip; u8_t netif_idx; u8_t so_options; u8_t tos; u8_t ttl ;
};



 




 


 
struct ip_globals
{
   
  struct netif *current_netif;
   
  struct netif *current_input_netif;

   
  const struct ip_hdr *current_ip4_header;





   
  u16_t current_ip_header_tot_len;
   
  ip_addr_t current_iphdr_src;
   
  ip_addr_t current_iphdr_dest;
};
extern struct ip_globals ip_data;





 



 

 

 

 


#line 176 "..\\Lwip\\include\\lwip/ip.h"



 

 

 

 

 

 


#line 210 "..\\Lwip\\include\\lwip/ip.h"

 

 


 

 

 


#line 278 "..\\Lwip\\include\\lwip/ip.h"

#line 294 "..\\Lwip\\include\\lwip/ip.h"



#line 318 "..\\Lwip\\include\\lwip/ip.h"













#line 49 "..\\Lwip\\include\\lwip/udp.h"
#line 50 "..\\Lwip\\include\\lwip/udp.h"
#line 1 "..\\Lwip\\include\\lwip/prot/udp.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/prot/udp.h"







 



__packed
struct udp_hdr {
  u16_t src;
  u16_t dest;   
  u16_t len;
  u16_t chksum;
} ;









#line 51 "..\\Lwip\\include\\lwip/udp.h"










struct udp_pcb;














 
typedef void (*udp_recv_fn)(void *arg, struct udp_pcb *pcb, struct pbuf *p,
    const ip_addr_t *addr, u16_t port);

 
struct udp_pcb {
 
  ip_addr_t local_ip; ip_addr_t remote_ip; u8_t netif_idx; u8_t so_options; u8_t tos; u8_t ttl ;

 

  struct udp_pcb *next;

  u8_t flags;
   
  u16_t local_port, remote_port;

#line 103 "..\\Lwip\\include\\lwip/udp.h"






   
  udp_recv_fn recv;
   
  void *recv_arg;
};
 
extern struct udp_pcb *udp_pcbs;


 
struct udp_pcb * udp_new        (void);
struct udp_pcb * udp_new_ip_type(u8_t type);
void             udp_remove     (struct udp_pcb *pcb);
err_t            udp_bind       (struct udp_pcb *pcb, const ip_addr_t *ipaddr,
                                 u16_t port);
void             udp_bind_netif (struct udp_pcb *pcb, const struct netif* netif);
err_t            udp_connect    (struct udp_pcb *pcb, const ip_addr_t *ipaddr,
                                 u16_t port);
void             udp_disconnect (struct udp_pcb *pcb);
void             udp_recv       (struct udp_pcb *pcb, udp_recv_fn recv,
                                 void *recv_arg);
err_t            udp_sendto_if  (struct udp_pcb *pcb, struct pbuf *p,
                                 const ip_addr_t *dst_ip, u16_t dst_port,
                                 struct netif *netif);
err_t            udp_sendto_if_src(struct udp_pcb *pcb, struct pbuf *p,
                                 const ip_addr_t *dst_ip, u16_t dst_port,
                                 struct netif *netif, const ip_addr_t *src_ip);
err_t            udp_sendto     (struct udp_pcb *pcb, struct pbuf *p,
                                 const ip_addr_t *dst_ip, u16_t dst_port);
err_t            udp_send       (struct udp_pcb *pcb, struct pbuf *p);

#line 154 "..\\Lwip\\include\\lwip/udp.h"








 
void             udp_input      (struct pbuf *p, struct netif *inp);

void             udp_init       (void);

 


#line 180 "..\\Lwip\\include\\lwip/udp.h"







void udp_netif_ip_addr_changed(const ip_addr_t* old_addr, const ip_addr_t* new_addr);







#line 73 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 74 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 75 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 76 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 1 "..\\Lwip\\include\\lwip/dhcp.h"



 
































 



#line 42 "..\\Lwip\\include\\lwip/dhcp.h"



#line 46 "..\\Lwip\\include\\lwip/dhcp.h"
#line 47 "..\\Lwip\\include\\lwip/dhcp.h"





 

 

 




 
typedef enum {
  DHCP_AUTOIP_COOP_STATE_OFF  = 0,
  DHCP_AUTOIP_COOP_STATE_ON   = 1
} dhcp_autoip_coop_state_enum_t;

struct dhcp
{
   
  u32_t xid;
   
  u8_t pcb_allocated;
   
  u8_t state;
   
  u8_t tries;



  u8_t subnet_mask_given;

  u16_t request_timeout;  
  u16_t t1_timeout;   
  u16_t t2_timeout;   
  u16_t t1_renew_time;   
  u16_t t2_rebind_time;  
  u16_t lease_used;  
  u16_t t0_timeout;  
  ip_addr_t server_ip_addr;  
  ip4_addr_t offered_ip_addr;
  ip4_addr_t offered_sn_mask;
  ip4_addr_t offered_gw_addr;

  u32_t offered_t0_lease;  
  u32_t offered_t1_renew;  
  u32_t offered_t2_rebind;  




};


void dhcp_set_struct(struct netif *netif, struct dhcp *dhcp);
 

void dhcp_cleanup(struct netif *netif);
err_t dhcp_start(struct netif *netif);
err_t dhcp_renew(struct netif *netif);
err_t dhcp_release(struct netif *netif);
void dhcp_stop(struct netif *netif);
void dhcp_release_and_stop(struct netif *netif);
void dhcp_inform(struct netif *netif);
void dhcp_network_changed(struct netif *netif);

void dhcp_arp_reply(struct netif *netif, const ip4_addr_t *addr);

u8_t dhcp_supplied_address(const struct netif *netif);
 
void dhcp_coarse_tmr(void);
 
void dhcp_fine_tmr(void);

#line 130 "..\\Lwip\\include\\lwip/dhcp.h"









#line 77 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 1 "..\\Lwip\\include\\lwip/autoip.h"




 

































 




#line 45 "..\\Lwip\\include\\lwip/autoip.h"

#line 98 "..\\Lwip\\include\\lwip/autoip.h"

#line 78 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 1 "..\\Lwip\\include\\lwip/dns.h"



 
































 




#line 43 "..\\Lwip\\include\\lwip/dns.h"

#line 130 "..\\Lwip\\include\\lwip/dns.h"

#line 79 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 1 "..\\Lwip\\include\\lwip/etharp.h"





 

































 




#line 46 "..\\Lwip\\include\\lwip/etharp.h"



#line 50 "..\\Lwip\\include\\lwip/etharp.h"
#line 51 "..\\Lwip\\include\\lwip/etharp.h"
#line 52 "..\\Lwip\\include\\lwip/etharp.h"
#line 53 "..\\Lwip\\include\\lwip/etharp.h"
#line 1 "..\\Lwip\\include\\lwip/prot/ethernet.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/prot/ethernet.h"
#line 1 "..\\Lwip\\include\\lwip/prot/ieee.h"






 































 











 
enum lwip_ieee_eth_type {
   
  ETHTYPE_IP        = 0x0800U,
   
  ETHTYPE_ARP       = 0x0806U, 
   
  ETHTYPE_WOL       = 0x0842U,
   
  ETHTYPE_RARP      = 0x8035U,
   
  ETHTYPE_VLAN      = 0x8100U,
   
  ETHTYPE_IPV6      = 0x86DDU,
   
  ETHTYPE_PPPOEDISC = 0x8863U,
   
  ETHTYPE_PPPOE     = 0x8864U,
   
  ETHTYPE_JUMBO     = 0x8870U,
   
  ETHTYPE_PROFINET  = 0x8892U,
   
  ETHTYPE_ETHERCAT  = 0x88A4U,
   
  ETHTYPE_LLDP      = 0x88CCU,
   
  ETHTYPE_SERCOS    = 0x88CDU,
   
  ETHTYPE_MRP       = 0x88E3U,
   
  ETHTYPE_PTP       = 0x88F7U,
   
  ETHTYPE_QINQ      = 0x9100U
};





#line 42 "..\\Lwip\\include\\lwip/prot/ethernet.h"





#line 54 "..\\Lwip\\include\\lwip/prot/ethernet.h"




__packed
 
struct eth_addr {
  u8_t addr[6];
} ;





 





__packed
 
struct eth_hdr {



  struct eth_addr dest;
  struct eth_addr src;
  u16_t type;
} ;










__packed


 
struct eth_vlan_hdr {
  u16_t prio_vid;
  u16_t tpid;
} ;








 




 









#line 54 "..\\Lwip\\include\\lwip/etharp.h"



#line 1 "..\\Lwip\\include\\lwip/prot/etharp.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/prot/etharp.h"
#line 42 "..\\Lwip\\include\\lwip/prot/etharp.h"












 



__packed
struct ip4_addr_wordaligned {
  u16_t addrw[2];
} ;







 




 

 







__packed
 
struct etharp_hdr {
  u16_t hwtype;
  u16_t proto;
  u8_t hwlen;
  u8_t protolen;
  u16_t opcode;
  struct eth_addr shwaddr;
  struct ip4_addr_wordaligned sipaddr;
  struct eth_addr dhwaddr;
  struct ip4_addr_wordaligned dipaddr;
} ;







 
enum etharp_opcode {
  ARP_REQUEST = 1,
  ARP_REPLY   = 2
};





#line 58 "..\\Lwip\\include\\lwip/etharp.h"





 


#line 75 "..\\Lwip\\include\\lwip/etharp.h"


void etharp_tmr(void);
ssize_t etharp_find_addr(struct netif *netif, const ip4_addr_t *ipaddr,
         struct eth_addr **eth_ret, const ip4_addr_t **ip_ret);
int etharp_get_entry(size_t i, ip4_addr_t **ipaddr, struct netif **netif, struct eth_addr **eth_ret);
err_t etharp_output(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr);
err_t etharp_query(struct netif *netif, const ip4_addr_t *ipaddr, struct pbuf *q);
err_t etharp_request(struct netif *netif, const ip4_addr_t *ipaddr);



 

void etharp_cleanup_netif(struct netif *netif);






void etharp_input(struct pbuf *p, struct netif *netif);








#line 80 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 1 "..\\Lwip\\include\\lwip/prot/dhcp.h"



 
































 



#line 42 "..\\Lwip\\include\\lwip/prot/dhcp.h"
#line 43 "..\\Lwip\\include\\lwip/prot/dhcp.h"
#line 44 "..\\Lwip\\include\\lwip/prot/dhcp.h"





  
#line 57 "..\\Lwip\\include\\lwip/prot/dhcp.h"




__packed
 
struct dhcp_msg
{
  u8_t op;
  u8_t htype;
  u8_t hlen;
  u8_t hops;
  u32_t xid;
  u16_t secs;
  u16_t flags;
  ip4_addr_p_t ciaddr;
  ip4_addr_p_t yiaddr;
  ip4_addr_p_t siaddr;
  ip4_addr_p_t giaddr;
  u8_t chaddr[16U];
  u8_t sname[64U];
  u8_t file[128U];
  u32_t cookie;

 



 

 


  u8_t options[68U];
} ;






 
typedef enum {
  DHCP_STATE_OFF             = 0,
  DHCP_STATE_REQUESTING      = 1,
  DHCP_STATE_INIT            = 2,
  DHCP_STATE_REBOOTING       = 3,
  DHCP_STATE_REBINDING       = 4,
  DHCP_STATE_RENEWING        = 5,
  DHCP_STATE_SELECTING       = 6,
  DHCP_STATE_INFORMING       = 7,
  DHCP_STATE_CHECKING        = 8,
  DHCP_STATE_PERMANENT       = 9,   
  DHCP_STATE_BOUND           = 10,
  DHCP_STATE_RELEASING       = 11,  
  DHCP_STATE_BACKING_OFF     = 12
} dhcp_state_enum_t;

 



 
#line 128 "..\\Lwip\\include\\lwip/prot/dhcp.h"



 

 
#line 145 "..\\Lwip\\include\\lwip/prot/dhcp.h"

 













#line 166 "..\\Lwip\\include\\lwip/prot/dhcp.h"

 










#line 81 "..\\Lwip\\core\\ipv4\\dhcp.c"
#line 1 "..\\Lwip\\include\\lwip/prot/iana.h"






 































 











 
enum lwip_iana_hwtype {
   
  LWIP_IANA_HWTYPE_ETHERNET = 1
};





 
enum lwip_iana_port_number {
   
  LWIP_IANA_PORT_SMTP        = 25,
   
  LWIP_IANA_PORT_DHCP_SERVER = 67,
   
  LWIP_IANA_PORT_DHCP_CLIENT = 68,
   
  LWIP_IANA_PORT_TFTP        = 69,
   
  LWIP_IANA_PORT_HTTP        = 80,
   
  LWIP_IANA_PORT_SNTP        = 123,
   
  LWIP_IANA_PORT_NETBIOS     = 137,
   
  LWIP_IANA_PORT_SNMP        = 161,
   
  LWIP_IANA_PORT_SNMP_TRAP   = 162,
   
  LWIP_IANA_PORT_HTTPS       = 443,
   
  LWIP_IANA_PORT_SMTPS       = 465,
   
  LWIP_IANA_PORT_MQTT        = 1883,
   
  LWIP_IANA_PORT_MDNS        = 5353,
   
  LWIP_IANA_PORT_SECURE_MQTT = 8883
};





#line 82 "..\\Lwip\\core\\ipv4\\dhcp.c"

#line 1 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const unsigned char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "E:\\Keil\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 84 "..\\Lwip\\core\\ipv4\\dhcp.c"

#line 94 "..\\Lwip\\core\\ipv4\\dhcp.c"



 








 





 


 




#line 129 "..\\Lwip\\core\\ipv4\\dhcp.c"





 
enum dhcp_option_idx {
  DHCP_OPTION_IDX_OVERLOAD = 0,
  DHCP_OPTION_IDX_MSG_TYPE,
  DHCP_OPTION_IDX_SERVER_ID,
  DHCP_OPTION_IDX_LEASE_TIME,
  DHCP_OPTION_IDX_T1,
  DHCP_OPTION_IDX_T2,
  DHCP_OPTION_IDX_SUBNET_MASK,
  DHCP_OPTION_IDX_ROUTER,
#line 152 "..\\Lwip\\core\\ipv4\\dhcp.c"
  DHCP_OPTION_IDX_MAX
};


 
u32_t dhcp_rx_options_val[DHCP_OPTION_IDX_MAX];


 
u8_t  dhcp_rx_options_given[DHCP_OPTION_IDX_MAX];

static u8_t dhcp_discover_request_options[] = {
  1,
  3,
  28
#line 173 "..\\Lwip\\core\\ipv4\\dhcp.c"
};






#line 186 "..\\Lwip\\core\\ipv4\\dhcp.c"

static struct udp_pcb *dhcp_pcb;
static u8_t dhcp_pcb_refcount;

 
static err_t dhcp_discover(struct netif *netif);
static err_t dhcp_select(struct netif *netif);
static void dhcp_bind(struct netif *netif);

static err_t dhcp_decline(struct netif *netif);

static err_t dhcp_rebind(struct netif *netif);
static err_t dhcp_reboot(struct netif *netif);
static void dhcp_set_state(struct dhcp *dhcp, u8_t new_state);

 
static void dhcp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

 
static void dhcp_timeout(struct netif *netif);
static void dhcp_t1_timeout(struct netif *netif);
static void dhcp_t2_timeout(struct netif *netif);

 
 
static struct pbuf *dhcp_create_msg(struct netif *netif, struct dhcp *dhcp, u8_t message_type, u16_t *options_out_len);
 
static u16_t dhcp_option(u16_t options_out_len, u8_t *options, u8_t option_type, u8_t option_len);
 
static u16_t dhcp_option_byte(u16_t options_out_len, u8_t *options, u8_t value);
static u16_t dhcp_option_short(u16_t options_out_len, u8_t *options, u16_t value);
static u16_t dhcp_option_long(u16_t options_out_len, u8_t *options, u32_t value);



 
static void dhcp_option_trailer(u16_t options_out_len, u8_t *options, struct pbuf *p_out);

 
static err_t
dhcp_inc_pcb_refcount(void)
{
  if (dhcp_pcb_refcount == 0) {
    do { if (!(dhcp_pcb == 0)) { do {printf("dhcp_inc_pcb_refcount(): memory leak");}while(0); }} while(0);

     
    dhcp_pcb = udp_new();

    if (dhcp_pcb == 0) {
      return ERR_MEM;
    }

    ((dhcp_pcb)->so_options = (u8_t)((dhcp_pcb)->so_options | (0x20U)));

     
    udp_bind(dhcp_pcb, (&ip_addr_any), LWIP_IANA_PORT_DHCP_CLIENT);
    udp_connect(dhcp_pcb, (&ip_addr_any), LWIP_IANA_PORT_DHCP_SERVER);
    udp_recv(dhcp_pcb, dhcp_recv, 0);
  }

  dhcp_pcb_refcount++;

  return ERR_OK;
}

 
static void
dhcp_dec_pcb_refcount(void)
{
  do { if (!((dhcp_pcb_refcount > 0))) { do {printf("dhcp_pcb_refcount(): refcount error");}while(0); }} while(0);
  dhcp_pcb_refcount--;

  if (dhcp_pcb_refcount == 0) {
    udp_remove(dhcp_pcb);
    dhcp_pcb = 0;
  }
}












 
static void
dhcp_handle_nak(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);

  ;

  
 
  dhcp_set_state(dhcp, DHCP_STATE_BACKING_OFF);
   
  netif_set_addr(netif, ((&ip_addr_any)), ((&ip_addr_any)), ((&ip_addr_any)));
   
  dhcp_discover(netif);
}










 
static void
dhcp_check(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  err_t result;
  u16_t msecs;
  ;

  dhcp_set_state(dhcp, DHCP_STATE_CHECKING);
  
 
  result = etharp_query(netif, &dhcp->offered_ip_addr, 0);
  if (result != ERR_OK) {
    ;
  }
  if (dhcp->tries < 255) {
    dhcp->tries++;
  }
  msecs = 500;
  dhcp->request_timeout = (u16_t)((msecs + 500 - 1) / 500);
  ;
}






 
static void
dhcp_handle_offer(struct netif *netif, struct dhcp_msg *msg_in)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);

  ;

   
  if ((dhcp_rx_options_given[DHCP_OPTION_IDX_SERVER_ID] != 0)) {
    dhcp->request_timeout = 0;  

    (((&dhcp->server_ip_addr))->addr = (lwip_htonl((dhcp_rx_options_val[DHCP_OPTION_IDX_SERVER_ID]))));
    ;

     
    ((dhcp->offered_ip_addr). addr = (msg_in->yiaddr). addr);
    ;


    dhcp_select(netif);
  } else {
    ;

  }
}








 
static err_t
dhcp_select(struct netif *netif)
{
  struct dhcp *dhcp;
  err_t result;
  u16_t msecs;
  u8_t i;
  struct pbuf *p_out;
  u16_t options_out_len;

  do { if (!((netif != 0))) { do {printf("dhcp_select: netif != NULL");}while(0); return ERR_ARG;;}} while(0);
  dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  do { if (!((dhcp != 0))) { do {printf("dhcp_select: dhcp != NULL");}while(0); return ERR_VAL;;}} while(0);

  ;
  dhcp_set_state(dhcp, DHCP_STATE_REQUESTING);

   
  p_out = dhcp_create_msg(netif, dhcp, 3, &options_out_len);
  if (p_out != 0) {
    struct dhcp_msg *msg_out = (struct dhcp_msg *)p_out->payload;
    options_out_len = dhcp_option(options_out_len, msg_out->options, 57, 2);
    options_out_len = dhcp_option_short(options_out_len, msg_out->options, (netif->mtu));

     
    options_out_len = dhcp_option(options_out_len, msg_out->options, 50, 4);
    options_out_len = dhcp_option_long(options_out_len, msg_out->options, lwip_htonl(((&dhcp->offered_ip_addr)->addr)));

    options_out_len = dhcp_option(options_out_len, msg_out->options, 54, 4);
    options_out_len = dhcp_option_long(options_out_len, msg_out->options, lwip_htonl((((&dhcp->server_ip_addr))->addr)));

    options_out_len = dhcp_option(options_out_len, msg_out->options, 55, (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])));
    for (i = 0; i < (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])); i++) {
      options_out_len = dhcp_option_byte(options_out_len, msg_out->options, dhcp_discover_request_options[i]);
    }





    ;
    dhcp_option_trailer(options_out_len, msg_out->options, p_out);

     
    result = udp_sendto_if_src(dhcp_pcb, p_out, (&ip_addr_broadcast), LWIP_IANA_PORT_DHCP_SERVER, netif, (&ip_addr_any));
    pbuf_free(p_out);
    ;
  } else {
    ;
    result = ERR_MEM;
  }
  if (dhcp->tries < 255) {
    dhcp->tries++;
  }
  msecs = (u16_t)((dhcp->tries < 6 ? 1 << dhcp->tries : 60) * 1000);
  dhcp->request_timeout = (u16_t)((msecs + 500 - 1) / 500);
  ;
  return result;
}




 
void
dhcp_coarse_tmr(void)
{
  struct netif *netif;
  ;
   
  for ((netif) = netif_list; (netif) != 0; (netif) = (netif)->next) {
     
    struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
    if ((dhcp != 0) && (dhcp->state != DHCP_STATE_OFF)) {
       
      if (dhcp->t0_timeout && (++dhcp->lease_used == dhcp->t0_timeout)) {
        ;
         
        dhcp_release_and_stop(netif);
        dhcp_start(netif);
         
      } else if (dhcp->t2_rebind_time && (dhcp->t2_rebind_time-- == 1)) {
        ;
         
        dhcp_t2_timeout(netif);
         
      } else if (dhcp->t1_renew_time && (dhcp->t1_renew_time-- == 1)) {
        ;
         
        dhcp_t1_timeout(netif);
      }
    }
  }
}







 
void
dhcp_fine_tmr(void)
{
  struct netif *netif;
   
  for ((netif) = netif_list; (netif) != 0; (netif) = (netif)->next) {
    struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
     
    if (dhcp != 0) {
       
      if (dhcp->request_timeout > 1) {
        dhcp->request_timeout--;
      } else if (dhcp->request_timeout == 1) {
        dhcp->request_timeout--;
         
        ;
         
        dhcp_timeout(netif);
      }
    }
  }
}








 
static void
dhcp_timeout(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);

  ;
   
  if ((dhcp->state == DHCP_STATE_BACKING_OFF) || (dhcp->state == DHCP_STATE_SELECTING)) {
    ;
    dhcp_discover(netif);
     
  } else if (dhcp->state == DHCP_STATE_REQUESTING) {
    ;
    if (dhcp->tries <= 5) {
      dhcp_select(netif);
    } else {
      ;
      dhcp_release_and_stop(netif);
      dhcp_start(netif);
    }

     
  } else if (dhcp->state == DHCP_STATE_CHECKING) {
    ;
    if (dhcp->tries <= 1) {
      dhcp_check(netif);
      
 
    } else {
       
      dhcp_bind(netif);
    }

  } else if (dhcp->state == DHCP_STATE_REBOOTING) {
    if (dhcp->tries < 2) {
      dhcp_reboot(netif);
    } else {
      dhcp_discover(netif);
    }
  }
}





 
static void
dhcp_t1_timeout(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);

  ;
  if ((dhcp->state == DHCP_STATE_REQUESTING) || (dhcp->state == DHCP_STATE_BOUND) ||
      (dhcp->state == DHCP_STATE_RENEWING)) {
    
 
    ;

    
 
    dhcp_renew(netif);
     
    if (((dhcp->t2_timeout - dhcp->lease_used) / 2) >= ((60 + 60 / 2) / 60)) {
      dhcp->t1_renew_time = (u16_t)((dhcp->t2_timeout - dhcp->lease_used) / 2);
    }
  }
}





 
static void
dhcp_t2_timeout(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);

  ;
  if ((dhcp->state == DHCP_STATE_REQUESTING) || (dhcp->state == DHCP_STATE_BOUND) ||
      (dhcp->state == DHCP_STATE_RENEWING) || (dhcp->state == DHCP_STATE_REBINDING)) {
     
    ;

    
 
    dhcp_rebind(netif);
     
    if (((dhcp->t0_timeout - dhcp->lease_used) / 2) >= ((60 + 60 / 2) / 60)) {
      dhcp->t2_rebind_time = (u16_t)((dhcp->t0_timeout - dhcp->lease_used) / 2);
    }
  }
}





 
static void
dhcp_handle_ack(struct netif *netif, struct dhcp_msg *msg_in)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);

#line 609 "..\\Lwip\\core\\ipv4\\dhcp.c"

   
  ((&dhcp->offered_sn_mask)->addr = 0);
  ((&dhcp->offered_gw_addr)->addr = 0);




   
  if ((dhcp_rx_options_given[DHCP_OPTION_IDX_LEASE_TIME] != 0)) {
     
    dhcp->offered_t0_lease = (dhcp_rx_options_val[DHCP_OPTION_IDX_LEASE_TIME]);
  }
   
  if ((dhcp_rx_options_given[DHCP_OPTION_IDX_T1] != 0)) {
     
    dhcp->offered_t1_renew = (dhcp_rx_options_val[DHCP_OPTION_IDX_T1]);
  } else {
     
    dhcp->offered_t1_renew = dhcp->offered_t0_lease / 2;
  }

   
  if ((dhcp_rx_options_given[DHCP_OPTION_IDX_T2] != 0)) {
     
    dhcp->offered_t2_rebind = (dhcp_rx_options_val[DHCP_OPTION_IDX_T2]);
  } else {
     
    dhcp->offered_t2_rebind = (dhcp->offered_t0_lease * 7U) / 8U;
  }

   
  ((dhcp->offered_ip_addr). addr = (msg_in->yiaddr). addr);







   
  if ((dhcp_rx_options_given[DHCP_OPTION_IDX_SUBNET_MASK] != 0)) {
     
    ((&dhcp->offered_sn_mask)->addr = (lwip_htonl((dhcp_rx_options_val[DHCP_OPTION_IDX_SUBNET_MASK]))));
    dhcp->subnet_mask_given = 1;
  } else {
    dhcp->subnet_mask_given = 0;
  }

   
  if ((dhcp_rx_options_given[DHCP_OPTION_IDX_ROUTER] != 0)) {
    ((&dhcp->offered_gw_addr)->addr = (lwip_htonl((dhcp_rx_options_val[DHCP_OPTION_IDX_ROUTER]))));
  }

#line 670 "..\\Lwip\\core\\ipv4\\dhcp.c"

#line 679 "..\\Lwip\\core\\ipv4\\dhcp.c"
}








 
void
dhcp_set_struct(struct netif *netif, struct dhcp *dhcp)
{
  ;
  do { if (!(netif != 0)) { do {printf("netif != NULL");}while(0); }} while(0);
  do { if (!(dhcp != 0)) { do {printf("dhcp != NULL");}while(0); }} while(0);
  do { if (!(((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]) == 0)) { do {printf("netif already has a struct dhcp set");}while(0); }} while(0);

   
  memset(dhcp, 0, sizeof(struct dhcp));
   
  (netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)] = (dhcp);
}









 
void dhcp_cleanup(struct netif *netif)
{
  ;
  do { if (!(netif != 0)) { do {printf("netif != NULL");}while(0); }} while(0);

  if (((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]) != 0) {
    mem_free(((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]));
    (netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)] = (0);
  }
}













 
err_t
dhcp_start(struct netif *netif)
{
  struct dhcp *dhcp;
  err_t result;

  ;
  do { if (!((netif != 0))) { do {printf("netif != NULL");}while(0); return ERR_ARG;;}} while(0);
  do { if (!((((netif)->flags & 0x01U) ? (u8_t)1 : (u8_t)0))) { do {printf("netif is not up, old style port?");}while(0); return ERR_ARG;;}} while(0);
  dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  ;

   
  if (netif->mtu < 576) {
    ;
    return ERR_MEM;
  }

   
  if (dhcp == 0) {
    ;
    dhcp = (struct dhcp *)mem_malloc(sizeof(struct dhcp));
    if (dhcp == 0) {
      ;
      return ERR_MEM;
    }

     
    (netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)] = (dhcp);
    ;
     
  } else {
    ;

    if (dhcp->pcb_allocated != 0) {
      dhcp_dec_pcb_refcount();  
    }
     
  }

   
  memset(dhcp, 0, sizeof(struct dhcp));
   

  ;

  if (dhcp_inc_pcb_refcount() != ERR_OK) {  
    return ERR_MEM;
  }
  dhcp->pcb_allocated = 1;

  if (!(((netif)->flags & 0x04U) ? (u8_t)1 : (u8_t)0)) {
     
    dhcp_set_state(dhcp, DHCP_STATE_INIT);
    return ERR_OK;
  }

   
  result = dhcp_discover(netif);
  if (result != ERR_OK) {
     
    dhcp_release_and_stop(netif);
    return ERR_MEM;
  }
  return result;
}










 
void
dhcp_inform(struct netif *netif)
{
  struct dhcp dhcp;
  struct pbuf *p_out;
  u16_t options_out_len;

  ;
  do { if (!((netif != 0))) { do {printf("netif != NULL");}while(0); return;;}} while(0);

  if (dhcp_inc_pcb_refcount() != ERR_OK) {  
    return;
  }

  memset(&dhcp, 0, sizeof(struct dhcp));
  dhcp_set_state(&dhcp, DHCP_STATE_INFORMING);

   
  p_out = dhcp_create_msg(netif, &dhcp, 8, &options_out_len);
  if (p_out != 0) {
    struct dhcp_msg *msg_out = (struct dhcp_msg *)p_out->payload;
    options_out_len = dhcp_option(options_out_len, msg_out->options, 57, 2);
    options_out_len = dhcp_option_short(options_out_len, msg_out->options, (netif->mtu));

    ;
    dhcp_option_trailer(options_out_len, msg_out->options, p_out);

    ;

    udp_sendto_if(dhcp_pcb, p_out, (&ip_addr_broadcast), LWIP_IANA_PORT_DHCP_SERVER, netif);

    pbuf_free(p_out);
  } else {
    ;
  }

  dhcp_dec_pcb_refcount();  
}





 
void
dhcp_network_changed(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);

  if (!dhcp) {
    return;
  }
  switch (dhcp->state) {
    case DHCP_STATE_REBINDING:
    case DHCP_STATE_RENEWING:
    case DHCP_STATE_BOUND:
    case DHCP_STATE_REBOOTING:
      dhcp->tries = 0;
      dhcp_reboot(netif);
      break;
    case DHCP_STATE_OFF:
       
      break;
    default:
      do { if (!(dhcp->state <= DHCP_STATE_BACKING_OFF)) { do {printf("invalid dhcp->state");}while(0); }} while(0);
      

 
#line 887 "..\\Lwip\\core\\ipv4\\dhcp.c"
       
      dhcp->tries = 0;
      dhcp_discover(netif);
      break;
  }
}








 
void
dhcp_arp_reply(struct netif *netif, const ip4_addr_t *addr)
{
  struct dhcp *dhcp;

  do { if (!((netif != 0))) { do {printf("netif != NULL");}while(0); return;;}} while(0);
  dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  ;
   
  if ((dhcp != 0) && (dhcp->state == DHCP_STATE_CHECKING)) {
    ;

    
 
    if (((addr)->addr == (&dhcp->offered_ip_addr)->addr)) {
       
      ;

      dhcp_decline(netif);
    }
  }
}









 
static err_t
dhcp_decline(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  err_t result;
  u16_t msecs;
  struct pbuf *p_out;
  u16_t options_out_len;

  ;
  dhcp_set_state(dhcp, DHCP_STATE_BACKING_OFF);
   
  p_out = dhcp_create_msg(netif, dhcp, 4, &options_out_len);
  if (p_out != 0) {
    struct dhcp_msg *msg_out = (struct dhcp_msg *)p_out->payload;
    options_out_len = dhcp_option(options_out_len, msg_out->options, 50, 4);
    options_out_len = dhcp_option_long(options_out_len, msg_out->options, lwip_htonl(((&dhcp->offered_ip_addr)->addr)));

    ;
    dhcp_option_trailer(options_out_len, msg_out->options, p_out);

     
    result = udp_sendto_if_src(dhcp_pcb, p_out, (&ip_addr_broadcast), LWIP_IANA_PORT_DHCP_SERVER, netif, (&ip_addr_any));
    pbuf_free(p_out);
    ;
  } else {
    ;

    result = ERR_MEM;
  }
  if (dhcp->tries < 255) {
    dhcp->tries++;
  }
  msecs = 10 * 1000;
  dhcp->request_timeout = (u16_t)((msecs + 500 - 1) / 500);
  ;
  return result;
}







 
static err_t
dhcp_discover(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  err_t result = ERR_OK;
  u16_t msecs;
  u8_t i;
  struct pbuf *p_out;
  u16_t options_out_len;

  ;

  ((&dhcp->offered_ip_addr)->addr = ((u32_t)0x00000000UL));
  dhcp_set_state(dhcp, DHCP_STATE_SELECTING);
   
  p_out = dhcp_create_msg(netif, dhcp, 1, &options_out_len);
  if (p_out != 0) {
    struct dhcp_msg *msg_out = (struct dhcp_msg *)p_out->payload;
    ;

    options_out_len = dhcp_option(options_out_len, msg_out->options, 57, 2);
    options_out_len = dhcp_option_short(options_out_len, msg_out->options, (netif->mtu));

    options_out_len = dhcp_option(options_out_len, msg_out->options, 55, (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])));
    for (i = 0; i < (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])); i++) {
      options_out_len = dhcp_option_byte(options_out_len, msg_out->options, dhcp_discover_request_options[i]);
    }
    ;
    dhcp_option_trailer(options_out_len, msg_out->options, p_out);

    ;
    udp_sendto_if_src(dhcp_pcb, p_out, (&ip_addr_broadcast), LWIP_IANA_PORT_DHCP_SERVER, netif, (&ip_addr_any));
    ;
    pbuf_free(p_out);
    ;
  } else {
    ;
  }
  if (dhcp->tries < 255) {
    dhcp->tries++;
  }
#line 1027 "..\\Lwip\\core\\ipv4\\dhcp.c"
  msecs = (u16_t)((dhcp->tries < 6 ? 1 << dhcp->tries : 60) * 1000);
  dhcp->request_timeout = (u16_t)((msecs + 500 - 1) / 500);
  ;
  return result;
}






 
static void
dhcp_bind(struct netif *netif)
{
  u32_t timeout;
  struct dhcp *dhcp;
  ip4_addr_t sn_mask, gw_addr;
  do { if (!((netif != 0))) { do {printf("dhcp_bind: netif != NULL");}while(0); return;;}} while(0);
  dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  do { if (!((dhcp != 0))) { do {printf("dhcp_bind: dhcp != NULL");}while(0); return;;}} while(0);
  ;

   
  dhcp->lease_used = 0;

  if (dhcp->offered_t0_lease != 0xffffffffUL) {
     
    ;
    timeout = (dhcp->offered_t0_lease + 60 / 2) / 60;
    if (timeout > 0xffff) {
      timeout = 0xffff;
    }
    dhcp->t0_timeout = (u16_t)timeout;
    if (dhcp->t0_timeout == 0) {
      dhcp->t0_timeout = 1;
    }
    ;
  }

   
  if (dhcp->offered_t1_renew != 0xffffffffUL) {
     
    ;
    timeout = (dhcp->offered_t1_renew + 60 / 2) / 60;
    if (timeout > 0xffff) {
      timeout = 0xffff;
    }
    dhcp->t1_timeout = (u16_t)timeout;
    if (dhcp->t1_timeout == 0) {
      dhcp->t1_timeout = 1;
    }
    ;
    dhcp->t1_renew_time = dhcp->t1_timeout;
  }
   
  if (dhcp->offered_t2_rebind != 0xffffffffUL) {
    ;
    timeout = (dhcp->offered_t2_rebind + 60 / 2) / 60;
    if (timeout > 0xffff) {
      timeout = 0xffff;
    }
    dhcp->t2_timeout = (u16_t)timeout;
    if (dhcp->t2_timeout == 0) {
      dhcp->t2_timeout = 1;
    }
    ;
    dhcp->t2_rebind_time = dhcp->t2_timeout;
  }

   
  if ((dhcp->t1_timeout >= dhcp->t2_timeout) && (dhcp->t2_timeout > 0)) {
    dhcp->t1_timeout = 0;
  }

  if (dhcp->subnet_mask_given) {
     
    ((sn_mask). addr = (dhcp->offered_sn_mask). addr);
  } else {
     
    u8_t first_octet = (((const u8_t*)(&(&dhcp->offered_ip_addr)->addr))[0]);
    if (first_octet <= 127) {
      ((&sn_mask)->addr = (((((0xff000000UL) & (u32_t)0x000000ffUL) << 24) | (((0xff000000UL) & (u32_t)0x0000ff00UL) << 8) | (((0xff000000UL) & (u32_t)0x00ff0000UL) >> 8) | (((0xff000000UL) & (u32_t)0xff000000UL) >> 24))));
    } else if (first_octet >= 192) {
      ((&sn_mask)->addr = (((((0xffffff00UL) & (u32_t)0x000000ffUL) << 24) | (((0xffffff00UL) & (u32_t)0x0000ff00UL) << 8) | (((0xffffff00UL) & (u32_t)0x00ff0000UL) >> 8) | (((0xffffff00UL) & (u32_t)0xff000000UL) >> 24))));
    } else {
      ((&sn_mask)->addr = (((((0xffff0000UL) & (u32_t)0x000000ffUL) << 24) | (((0xffff0000UL) & (u32_t)0x0000ff00UL) << 8) | (((0xffff0000UL) & (u32_t)0x00ff0000UL) >> 8) | (((0xffff0000UL) & (u32_t)0xff000000UL) >> 24))));
    }
  }

  ((gw_addr). addr = (dhcp->offered_gw_addr). addr);
   
  if (((gw_addr). addr == ((u32_t)0x00000000UL))) {
     
    do { ((&gw_addr)->addr = ((&dhcp->offered_ip_addr)->addr) & ((&sn_mask)->addr)); } while(0);
     
    ((&gw_addr)->addr = (((&gw_addr)->addr) | ((((0x00000001UL) & (u32_t)0x000000ffUL) << 24) | (((0x00000001UL) & (u32_t)0x0000ff00UL) << 8) | (((0x00000001UL) & (u32_t)0x00ff0000UL) >> 8) | (((0x00000001UL) & (u32_t)0xff000000UL) >> 24))));
  }

#line 1132 "..\\Lwip\\core\\ipv4\\dhcp.c"

  ;

  
 
  dhcp_set_state(dhcp, DHCP_STATE_BOUND);

  netif_set_addr(netif, &dhcp->offered_ip_addr, &sn_mask, &gw_addr);
   
}






 
err_t
dhcp_renew(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  err_t result;
  u16_t msecs;
  u8_t i;
  struct pbuf *p_out;
  u16_t options_out_len;

  ;
  ;
  dhcp_set_state(dhcp, DHCP_STATE_RENEWING);

   
  p_out = dhcp_create_msg(netif, dhcp, 3, &options_out_len);
  if (p_out != 0) {
    struct dhcp_msg *msg_out = (struct dhcp_msg *)p_out->payload;
    options_out_len = dhcp_option(options_out_len, msg_out->options, 57, 2);
    options_out_len = dhcp_option_short(options_out_len, msg_out->options, (netif->mtu));

    options_out_len = dhcp_option(options_out_len, msg_out->options, 55, (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])));
    for (i = 0; i < (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])); i++) {
      options_out_len = dhcp_option_byte(options_out_len, msg_out->options, dhcp_discover_request_options[i]);
    }





    ;
    dhcp_option_trailer(options_out_len, msg_out->options, p_out);

    result = udp_sendto_if(dhcp_pcb, p_out, &dhcp->server_ip_addr, LWIP_IANA_PORT_DHCP_SERVER, netif);
    pbuf_free(p_out);

    ;
  } else {
    ;
    result = ERR_MEM;
  }
  if (dhcp->tries < 255) {
    dhcp->tries++;
  }
   
  msecs = (u16_t)(dhcp->tries < 10 ? dhcp->tries * 2000 : 20 * 1000);
  dhcp->request_timeout = (u16_t)((msecs + 500 - 1) / 500);
  ;
  return result;
}





 
static err_t
dhcp_rebind(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  err_t result;
  u16_t msecs;
  u8_t i;
  struct pbuf *p_out;
  u16_t options_out_len;

  ;
  dhcp_set_state(dhcp, DHCP_STATE_REBINDING);

   
  p_out = dhcp_create_msg(netif, dhcp, 3, &options_out_len);
  if (p_out != 0) {
    struct dhcp_msg *msg_out = (struct dhcp_msg *)p_out->payload;
    options_out_len = dhcp_option(options_out_len, msg_out->options, 57, 2);
    options_out_len = dhcp_option_short(options_out_len, msg_out->options, (netif->mtu));

    options_out_len = dhcp_option(options_out_len, msg_out->options, 55, (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])));
    for (i = 0; i < (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])); i++) {
      options_out_len = dhcp_option_byte(options_out_len, msg_out->options, dhcp_discover_request_options[i]);
    }





    ;
    dhcp_option_trailer(options_out_len, msg_out->options, p_out);

     
    result = udp_sendto_if(dhcp_pcb, p_out, (&ip_addr_broadcast), LWIP_IANA_PORT_DHCP_SERVER, netif);
    pbuf_free(p_out);
    ;
  } else {
    ;
    result = ERR_MEM;
  }
  if (dhcp->tries < 255) {
    dhcp->tries++;
  }
  msecs = (u16_t)(dhcp->tries < 10 ? dhcp->tries * 1000 : 10 * 1000);
  dhcp->request_timeout = (u16_t)((msecs + 500 - 1) / 500);
  ;
  return result;
}





 
static err_t
dhcp_reboot(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  err_t result;
  u16_t msecs;
  u8_t i;
  struct pbuf *p_out;
  u16_t options_out_len;

  ;
  dhcp_set_state(dhcp, DHCP_STATE_REBOOTING);

   
  p_out = dhcp_create_msg(netif, dhcp, 3, &options_out_len);
  if (p_out != 0) {
    struct dhcp_msg *msg_out = (struct dhcp_msg *)p_out->payload;
    options_out_len = dhcp_option(options_out_len, msg_out->options, 57, 2);
    options_out_len = dhcp_option_short(options_out_len, msg_out->options, 576);

    options_out_len = dhcp_option(options_out_len, msg_out->options, 50, 4);
    options_out_len = dhcp_option_long(options_out_len, msg_out->options, lwip_htonl(((&dhcp->offered_ip_addr)->addr)));

    options_out_len = dhcp_option(options_out_len, msg_out->options, 55, (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])));
    for (i = 0; i < (sizeof(dhcp_discover_request_options)/sizeof((dhcp_discover_request_options)[0])); i++) {
      options_out_len = dhcp_option_byte(options_out_len, msg_out->options, dhcp_discover_request_options[i]);
    }





    ;
    dhcp_option_trailer(options_out_len, msg_out->options, p_out);

     
    result = udp_sendto_if(dhcp_pcb, p_out, (&ip_addr_broadcast), LWIP_IANA_PORT_DHCP_SERVER, netif);
    pbuf_free(p_out);
    ;
  } else {
    ;
    result = ERR_MEM;
  }
  if (dhcp->tries < 255) {
    dhcp->tries++;
  }
  msecs = (u16_t)(dhcp->tries < 10 ? dhcp->tries * 1000 : 10 * 1000);
  dhcp->request_timeout = (u16_t)((msecs + 500 - 1) / 500);
  ;
  return result;
}






 
void
dhcp_release_and_stop(struct netif *netif)
{
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  ip_addr_t server_ip_addr;

  ;
  ;
  if (dhcp == 0) {
    return;
  }

   
  if (dhcp->state == DHCP_STATE_OFF) {
    return;
  }

  ((server_ip_addr). addr = (dhcp->server_ip_addr). addr);

   
  ((&dhcp->server_ip_addr)->addr = 0);
  ((&dhcp->offered_ip_addr)->addr = 0);
  ((&dhcp->offered_sn_mask)->addr = 0);
  ((&dhcp->offered_gw_addr)->addr = 0);



  dhcp->offered_t0_lease = dhcp->offered_t1_renew = dhcp->offered_t2_rebind = 0;
  dhcp->t1_renew_time = dhcp->t2_rebind_time = dhcp->lease_used = dhcp->t0_timeout = 0;

   
  if (dhcp_supplied_address(netif)) {
     
    struct pbuf *p_out;
    u16_t options_out_len;
    p_out = dhcp_create_msg(netif, dhcp, 7, &options_out_len);
    if (p_out != 0) {
      struct dhcp_msg *msg_out = (struct dhcp_msg *)p_out->payload;
      options_out_len = dhcp_option(options_out_len, msg_out->options, 54, 4);
      options_out_len = dhcp_option_long(options_out_len, msg_out->options, lwip_htonl((((&server_ip_addr))->addr)));

      ;
      dhcp_option_trailer(options_out_len, msg_out->options, p_out);

      udp_sendto_if(dhcp_pcb, p_out, &server_ip_addr, LWIP_IANA_PORT_DHCP_SERVER, netif);
      pbuf_free(p_out);
      ;
    } else {
       
      ;
    }
  }

   
  netif_set_addr(netif, ((&ip_addr_any)), ((&ip_addr_any)), ((&ip_addr_any)));

#line 1379 "..\\Lwip\\core\\ipv4\\dhcp.c"

  dhcp_set_state(dhcp, DHCP_STATE_OFF);

  if (dhcp->pcb_allocated != 0) {
    dhcp_dec_pcb_refcount();  
    dhcp->pcb_allocated = 0;
  }
}





 
err_t
dhcp_release(struct netif *netif)
{
  dhcp_release_and_stop(netif);
  return ERR_OK;
}





 
void
dhcp_stop(struct netif *netif)
{
  dhcp_release_and_stop(netif);
}





 
static void
dhcp_set_state(struct dhcp *dhcp, u8_t new_state)
{
  if (new_state != dhcp->state) {
    dhcp->state = new_state;
    dhcp->tries = 0;
    dhcp->request_timeout = 0;
  }
}





 
static u16_t
dhcp_option(u16_t options_out_len, u8_t *options, u8_t option_type, u8_t option_len)
{
  do { if (!(options_out_len + 2U + option_len <= 68U)) { do {printf("dhcp_option: options_out_len + 2 + option_len <= DHCP_OPTIONS_LEN");}while(0); }} while(0);
  options[options_out_len++] = option_type;
  options[options_out_len++] = option_len;
  return options_out_len;
}



 
static u16_t
dhcp_option_byte(u16_t options_out_len, u8_t *options, u8_t value)
{
  do { if (!(options_out_len < 68U)) { do {printf("dhcp_option_byte: options_out_len < DHCP_OPTIONS_LEN");}while(0); }} while(0);
  options[options_out_len++] = value;
  return options_out_len;
}

static u16_t
dhcp_option_short(u16_t options_out_len, u8_t *options, u16_t value)
{
  do { if (!(options_out_len + 2U <= 68U)) { do {printf("dhcp_option_short: options_out_len + 2 <= DHCP_OPTIONS_LEN");}while(0); }} while(0);
  options[options_out_len++] = (u8_t)((value & 0xff00U) >> 8);
  options[options_out_len++] = (u8_t) (value & 0x00ffU);
  return options_out_len;
}

static u16_t
dhcp_option_long(u16_t options_out_len, u8_t *options, u32_t value)
{
  do { if (!(options_out_len + 4U <= 68U)) { do {printf("dhcp_option_long: options_out_len + 4 <= DHCP_OPTIONS_LEN");}while(0); }} while(0);
  options[options_out_len++] = (u8_t)((value & 0xff000000UL) >> 24);
  options[options_out_len++] = (u8_t)((value & 0x00ff0000UL) >> 16);
  options[options_out_len++] = (u8_t)((value & 0x0000ff00UL) >> 8);
  options[options_out_len++] = (u8_t)((value & 0x000000ffUL));
  return options_out_len;
}

#line 1495 "..\\Lwip\\core\\ipv4\\dhcp.c"










 
static err_t
dhcp_parse_reply(struct pbuf *p, struct dhcp *dhcp)
{
  u8_t *options;
  u16_t offset;
  u16_t offset_max;
  u16_t options_idx;
  u16_t options_idx_max;
  struct pbuf *q;
  int parse_file_as_options = 0;
  int parse_sname_as_options = 0;
  struct dhcp_msg *msg_in;




  (void)dhcp;

   
  (memset(dhcp_rx_options_given, 0, sizeof(dhcp_rx_options_given)));
   
  if (p->len < 44U) {
    return ERR_BUF;
  }
  msg_in = (struct dhcp_msg *)p->payload;





   

   
  options_idx = (236U + 4U);
   
  options_idx_max = p->tot_len;
again:
  q = p;
  while ((q != 0) && (options_idx >= q->len)) {
    options_idx = (u16_t)(options_idx - q->len);
    options_idx_max = (u16_t)(options_idx_max - q->len);
    q = q->next;
  }
  if (q == 0) {
    return ERR_BUF;
  }
  offset = options_idx;
  offset_max = options_idx_max;
  options = (u8_t *)q->payload;
   
  while ((q != 0) && (offset < offset_max) && (options[offset] != 255)) {
    u8_t op = options[offset];
    u8_t len;
    u8_t decode_len = 0;
    int decode_idx = -1;
    u16_t val_offset = (u16_t)(offset + 2);
    if (val_offset < offset) {
       
      return ERR_BUF;
    }
     
    if ((offset + 1) < q->len) {
      len = options[offset + 1];
    } else {
      len = (q->next != 0 ? ((u8_t *)q->next->payload)[0] : 0);
    }
     
    decode_len = len;
    switch (op) {
       
      case (0):
         
        decode_len = len = 0;
         
        break;
      case (1):
        do { if (!(len == 4)) { do {printf("len == 4");}while(0); return ERR_VAL;;}} while(0);
        decode_idx = DHCP_OPTION_IDX_SUBNET_MASK;
        break;
      case (3):
        decode_len = 4;  
        do { if (!(len >= decode_len)) { do {printf("len >= decode_len");}while(0); return ERR_VAL;;}} while(0);
        decode_idx = DHCP_OPTION_IDX_ROUTER;
        break;
#line 1600 "..\\Lwip\\core\\ipv4\\dhcp.c"
      case (51):
        do { if (!(len == 4)) { do {printf("len == 4");}while(0); return ERR_VAL;;}} while(0);
        decode_idx = DHCP_OPTION_IDX_LEASE_TIME;
        break;
#line 1614 "..\\Lwip\\core\\ipv4\\dhcp.c"
      case (52):
        do { if (!(len == 1)) { do {printf("len == 1");}while(0); return ERR_VAL;;}} while(0);
         
        do { if (!(options_idx == (236U + 4U))) { do {printf("overload in file/sname");}while(0); return ERR_VAL;;}} while(0);
        decode_idx = DHCP_OPTION_IDX_OVERLOAD;
        break;
      case (53):
        do { if (!(len == 1)) { do {printf("len == 1");}while(0); return ERR_VAL;;}} while(0);
        decode_idx = DHCP_OPTION_IDX_MSG_TYPE;
        break;
      case (54):
        do { if (!(len == 4)) { do {printf("len == 4");}while(0); return ERR_VAL;;}} while(0);
        decode_idx = DHCP_OPTION_IDX_SERVER_ID;
        break;
      case (58):
        do { if (!(len == 4)) { do {printf("len == 4");}while(0); return ERR_VAL;;}} while(0);
        decode_idx = DHCP_OPTION_IDX_T1;
        break;
      case (59):
        do { if (!(len == 4)) { do {printf("len == 4");}while(0); return ERR_VAL;;}} while(0);
        decode_idx = DHCP_OPTION_IDX_T2;
        break;
      default:
        decode_len = 0;
        ;
        do { (void)msg_in; } while(0);


        break;
    }
    if (op == 0) {
      offset++;
    } else {
      if (offset + len + 2 > 0xFFFF) {
         
        return ERR_BUF;
      }
      offset = (u16_t)(offset + len + 2);
      if (decode_len > 0) {
        u32_t value = 0;
        u16_t copy_len;
decode_next:
        do { if (!(decode_idx >= 0 && decode_idx < DHCP_OPTION_IDX_MAX)) { do {printf("check decode_idx");}while(0); }} while(0);
        if (!(dhcp_rx_options_given[decode_idx] != 0)) {
          copy_len = (((decode_len) < (4)) ? (decode_len) : (4));
          if (pbuf_copy_partial(q, &value, copy_len, val_offset) != copy_len) {
            return ERR_BUF;
          }
          if (decode_len > 4) {
             
            u16_t next_val_offset;
            do { if (!(decode_len % 4 == 0)) { do {printf("decode_len %% 4 == 0");}while(0); return ERR_VAL;;}} while(0);
            (dhcp_rx_options_given[decode_idx] = 1);
            (dhcp_rx_options_val[decode_idx] = (lwip_htonl(value)));
            decode_len = (u8_t)(decode_len - 4);
            next_val_offset = (u16_t)(val_offset + 4);
            if (next_val_offset < val_offset) {
               
              return ERR_BUF;
            }
            val_offset = next_val_offset;
            decode_idx++;
            goto decode_next;
          } else if (decode_len == 4) {
            value = lwip_htonl(value);
          } else {
            do { if (!(decode_len == 1)) { do {printf("invalid decode_len");}while(0); return ERR_VAL;;}} while(0);
            value = ((u8_t *)&value)[0];
          }
          (dhcp_rx_options_given[decode_idx] = 1);
          (dhcp_rx_options_val[decode_idx] = (value));
        }
      }
    }
    if (offset >= q->len) {
      offset = (u16_t)(offset - q->len);
      offset_max = (u16_t)(offset_max - q->len);
      if (offset < offset_max) {
        q = q->next;
        do { if (!(q != 0)) { do {printf("next pbuf was null");}while(0); return ERR_VAL;;}} while(0);
        options = (u8_t *)q->payload;
      } else {
         
        return ERR_BUF;
      }
    }
  }
   
  if ((dhcp_rx_options_given[DHCP_OPTION_IDX_OVERLOAD] != 0)) {
    u32_t overload = (dhcp_rx_options_val[DHCP_OPTION_IDX_OVERLOAD]);
    (dhcp_rx_options_given[DHCP_OPTION_IDX_OVERLOAD] = 0);
    if (overload == 1) {
      parse_file_as_options = 1;
      ;
    } else if (overload == 2) {
      parse_sname_as_options = 1;
      ;
    } else if (overload == 3) {
      parse_sname_as_options = 1;
      parse_file_as_options = 1;
      ;
    } else {
      ;
    }
  }
  if (parse_file_as_options) {
     
    parse_file_as_options = 0;
    options_idx = 108U;
    options_idx_max = 108U + 128U;



    goto again;
  } else if (parse_sname_as_options) {
    parse_sname_as_options = 0;
    options_idx = 44U;
    options_idx_max = 44U + 64U;
    goto again;
  }
#line 1747 "..\\Lwip\\core\\ipv4\\dhcp.c"
  return ERR_OK;
}



 
static void
dhcp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  struct netif *netif = (ip_data . current_input_netif);
  struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
  struct dhcp_msg *reply_msg = (struct dhcp_msg *)p->payload;
  u8_t msg_type;
  u8_t i;
  struct dhcp_msg *msg_in;

  (void)arg;

   
  if ((dhcp == 0) || (dhcp->pcb_allocated == 0)) {
    goto free_pbuf_and_return;
  }

  do { if (!(1)) { do {printf("invalid server address type");}while(0); }} while(0);

  ;

  ;
  ;
   
  (void)pcb;
  (void)addr;
  (void)port;

  if (p->len < 44) {
    ;
    goto free_pbuf_and_return;
  }

  if (reply_msg->op != 2) {
    ;
    goto free_pbuf_and_return;
  }
   
  for (i = 0; i < netif->hwaddr_len && i < (((16U) < (6U)) ? (16U) : (6U)); i++) {
    if (netif->hwaddr[i] != reply_msg->chaddr[i]) {
      ;


      goto free_pbuf_and_return;
    }
  }
   
  if (lwip_htonl(reply_msg->xid) != dhcp->xid) {
    ;

    goto free_pbuf_and_return;
  }
   
  if (dhcp_parse_reply(p, dhcp) != ERR_OK) {
    ;

    goto free_pbuf_and_return;
  }

  ;
   
  if (!(dhcp_rx_options_given[DHCP_OPTION_IDX_MSG_TYPE] != 0)) {
    ;
    goto free_pbuf_and_return;
  }

  msg_in = (struct dhcp_msg *)p->payload;
   
  msg_type = (u8_t)(dhcp_rx_options_val[DHCP_OPTION_IDX_MSG_TYPE]);
   
  if (msg_type == 5) {
    ;
     
    if (dhcp->state == DHCP_STATE_REQUESTING) {
      dhcp_handle_ack(netif, msg_in);

      if ((netif->flags & 0x08U) != 0) {
         
        dhcp_check(netif);
      } else {
         
        dhcp_bind(netif);
      }




    }
     
    else if ((dhcp->state == DHCP_STATE_REBOOTING) || (dhcp->state == DHCP_STATE_REBINDING) ||
             (dhcp->state == DHCP_STATE_RENEWING)) {
      dhcp_handle_ack(netif, msg_in);
      dhcp_bind(netif);
    }
  }
   
  else if ((msg_type == 6) &&
           ((dhcp->state == DHCP_STATE_REBOOTING) || (dhcp->state == DHCP_STATE_REQUESTING) ||
            (dhcp->state == DHCP_STATE_REBINDING) || (dhcp->state == DHCP_STATE_RENEWING  ))) {
    ;
    dhcp_handle_nak(netif);
  }
   
  else if ((msg_type == 2) && (dhcp->state == DHCP_STATE_SELECTING)) {
    ;
     
    dhcp_handle_offer(netif, msg_in);
  }

free_pbuf_and_return:
  pbuf_free(p);
}







 
static struct pbuf *
dhcp_create_msg(struct netif *netif, struct dhcp *dhcp, u8_t message_type, u16_t *options_out_len)
{
  u16_t i;
  struct pbuf *p_out;
  struct dhcp_msg *msg_out;
  u16_t options_out_len_loc;


  


 



  static u32_t xid = 0xABCD0000;
#line 1897 "..\\Lwip\\core\\ipv4\\dhcp.c"
  do { if (!((netif != 0))) { do {printf("dhcp_create_msg: netif != NULL");}while(0); return 0;;}} while(0);
  do { if (!((dhcp != 0))) { do {printf("dhcp_create_msg: dhcp != NULL");}while(0); return 0;;}} while(0);
  p_out = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct dhcp_msg), PBUF_RAM);
  if (p_out == 0) {
    ;

    return 0;
  }
  do { if (!((p_out->len >= sizeof(struct dhcp_msg)))) { do {printf("dhcp_create_msg: check that first pbuf can hold struct dhcp_msg");}while(0); }} while(0);


   
  if ((message_type != 3) || (dhcp->state == DHCP_STATE_REBOOTING)) {
     
    if (dhcp->tries == 0) {



      xid++;

    }
    dhcp->xid = xid;
  }
  ;


  msg_out = (struct dhcp_msg *)p_out->payload;
  memset(msg_out, 0, sizeof(struct dhcp_msg));

  msg_out->op = 1;
   
  msg_out->htype = LWIP_IANA_HWTYPE_ETHERNET;
  msg_out->hlen = netif->hwaddr_len;
  msg_out->xid = lwip_htonl(dhcp->xid);
  
 
   
  if ((message_type == 8) || (message_type == 4) || (message_type == 7) ||
      ((message_type == 3) &&  
       ((dhcp->state == DHCP_STATE_RENEWING) || dhcp->state == DHCP_STATE_REBINDING))) {
    ((msg_out->ciaddr). addr = (*((const ip4_addr_t*)(&((netif)->ip_addr)))). addr);
  }
  for (i = 0; i < (((16U) < (6U)) ? (16U) : (6U)); i++) {
     
    msg_out->chaddr[i] = netif->hwaddr[i];
  }
  msg_out->cookie = ((((0x63825363UL) & (u32_t)0x000000ffUL) << 24) | (((0x63825363UL) & (u32_t)0x0000ff00UL) << 8) | (((0x63825363UL) & (u32_t)0x00ff0000UL) >> 8) | (((0x63825363UL) & (u32_t)0xff000000UL) >> 24));
   
  options_out_len_loc = dhcp_option(0, msg_out->options, 53, 1);
  options_out_len_loc = dhcp_option_byte(options_out_len_loc, msg_out->options, message_type);
  if (options_out_len) {
    *options_out_len = options_out_len_loc;
  }
  return p_out;
}






 
static void
dhcp_option_trailer(u16_t options_out_len, u8_t *options, struct pbuf *p_out)
{
  options[options_out_len++] = 255;
   
  while (((options_out_len < 68U) || (options_out_len & 3)) &&
         (options_out_len < 68U)) {
     
    options[options_out_len++] = 0;
  }
   
  pbuf_realloc(p_out, (u16_t)(sizeof(struct dhcp_msg) - 68U + options_out_len));
}






 
u8_t
dhcp_supplied_address(const struct netif *netif)
{
  if ((netif != 0) && (((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]) != 0)) {
    struct dhcp *dhcp = ((struct dhcp*)(netif)->client_data[(LWIP_NETIF_CLIENT_DATA_INDEX_DHCP)]);
    return (dhcp->state == DHCP_STATE_BOUND) || (dhcp->state == DHCP_STATE_RENEWING) ||
           (dhcp->state == DHCP_STATE_REBINDING);
  }
  return 0;
}

