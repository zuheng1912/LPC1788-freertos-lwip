#line 1 "..\\Lwip\\api\\api_msg.c"




 































 

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




 





 




 




 






 






 






 






 






 






 






 






 






 






 







 





 





 




 


 








 




















 









 






 






 






 






 







 






 










 






 





 





 


 







 






 





 





 




 









 





 





 



 






 






 






 






 






 







 







 






 






 







 






 







 







 








 








 





 





 


 







 






 








 






 








 





 





 






 





 
























 





























 





















 























 


















 















 
















 




















 





















 


















 
















 





















 

















 























 










 


















 




























 



























 



























 






















 






















 






















 





 





 




 





 








 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 






 







 






 






 






 






 






 






 






 






 






 






 






 






 






 





 



 








 




 



 





 

#line 40 "..\\Lwip\\api\\api_msg.c"



#line 1 "..\\Lwip\\include\\lwip/priv/api_msg.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/priv/api_msg.h"

#line 43 "..\\Lwip\\include\\lwip/priv/api_msg.h"
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





#line 44 "..\\Lwip\\include\\lwip/priv/api_msg.h"
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






#line 45 "..\\Lwip\\include\\lwip/priv/api_msg.h"
#line 1 "..\\Lwip\\include\\lwip/sys.h"



 






























 




#line 41 "..\\Lwip\\include\\lwip/sys.h"





#line 85 "..\\Lwip\\include\\lwip/sys.h"

 




 


#line 95 "..\\Lwip\\include\\lwip/sys.h"
#line 1 "..\\User\\arch/sys_arch.h"






























 
 
#line 34 "..\\User\\arch/sys_arch.h"
#line 35 "..\\User\\arch/sys_arch.h"
#line 1 "..\\Lwip\\include\\lwip\\tcpip.h"



 































 



#line 41 "..\\Lwip\\include\\lwip\\tcpip.h"



#line 45 "..\\Lwip\\include\\lwip\\tcpip.h"
#line 1 "..\\Lwip\\include\\lwip/timeouts.h"



 
































 



#line 42 "..\\Lwip\\include\\lwip/timeouts.h"
#line 43 "..\\Lwip\\include\\lwip/timeouts.h"
#line 1 "..\\Lwip\\include\\lwip/sys.h"



 






























 

#line 45 "..\\Lwip\\include\\lwip/timeouts.h"






#line 58 "..\\Lwip\\include\\lwip/timeouts.h"



 



 
typedef void (* lwip_cyclic_timer_handler)(void);


 
struct lwip_cyclic_timer {
  u32_t interval_ms;
  lwip_cyclic_timer_handler handler;



};


 
extern const struct lwip_cyclic_timer lwip_cyclic_timers[];
 
extern const int lwip_num_cyclic_timers;







 
typedef void (* sys_timeout_handler)(void *arg);

struct sys_timeo {
  struct sys_timeo *next;
  u32_t time;
  sys_timeout_handler h;
  void *arg;



};

void sys_timeouts_init(void);





void sys_timeout(u32_t msecs, sys_timeout_handler handler, void *arg);


void sys_untimeout(sys_timeout_handler handler, void *arg);
void sys_restart_timeouts(void);
void sys_check_timeouts(void);
u32_t sys_timeouts_sleeptime(void);












#line 46 "..\\Lwip\\include\\lwip\\tcpip.h"
#line 1 "..\\Lwip\\include\\lwip/netif.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/netif.h"



#line 45 "..\\Lwip\\include\\lwip/netif.h"

#line 47 "..\\Lwip\\include\\lwip/netif.h"

#line 49 "..\\Lwip\\include\\lwip/netif.h"
#line 1 "..\\Lwip\\include\\lwip/pbuf.h"



 































 




#line 42 "..\\Lwip\\include\\lwip/pbuf.h"
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





#line 50 "..\\Lwip\\include\\lwip/netif.h"
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





#line 47 "..\\Lwip\\include\\lwip\\tcpip.h"
#line 1 "..\\User\\arch\\sys_arch.h"






























 
 
#line 34 "..\\User\\arch\\sys_arch.h"
#line 35 "..\\User\\arch\\sys_arch.h"
#line 1 "..\\Lwip\\include\\lwip\\tcpip.h"



 































 
#line 36 "..\\User\\arch\\sys_arch.h"

  
#line 1 ".\\FreeRTOS\\include\\FreeRTOS.h"
























 






 
#line 34 ".\\FreeRTOS\\include\\FreeRTOS.h"













 
#line 49 ".\\FreeRTOS\\include\\FreeRTOS.h"

 



 

 
#line 1 ".\\FreeRTOS\\FreeRTOSConfig.h"


































































 





#line 74 ".\\FreeRTOS\\FreeRTOSConfig.h"
#line 75 ".\\FreeRTOS\\FreeRTOSConfig.h"
#line 76 ".\\FreeRTOS\\FreeRTOSConfig.h"




#line 82 ".\\FreeRTOS\\FreeRTOSConfig.h"
    extern uint32_t SystemCoreClock;








 





 



















 

                                                                        












 






 










  



 




















 

                                                                       

                      



 










 









 










 
  








 





 


 





 

                                                                        
                                                                        


 








 
 










 
#line 278 ".\\FreeRTOS\\FreeRTOSConfig.h"







 


















 




 








#line 58 ".\\FreeRTOS\\include\\FreeRTOS.h"

 
#line 1 ".\\FreeRTOS\\include\\projdefs.h"
























 







 
typedef void (* TaskFunction_t)( void * );



 












 




 











 
#line 109 ".\\FreeRTOS\\include\\projdefs.h"


 



 




#line 61 ".\\FreeRTOS\\include\\FreeRTOS.h"

 
#line 1 ".\\FreeRTOS\\include\\portable.h"
























 



 













 
#line 1 ".\\FreeRTOS\\include\\deprecated_definitions.h"
























 












 











































































































































































#line 218 ".\\FreeRTOS\\include\\deprecated_definitions.h"

#line 227 ".\\FreeRTOS\\include\\deprecated_definitions.h"







#line 241 ".\\FreeRTOS\\include\\deprecated_definitions.h"






































#line 45 ".\\FreeRTOS\\include\\portable.h"




 
#line 1 ".\\FreeRTOS\\portable\\RVDS\\ARM_CM3\\portmacro.h"
























 





 



 









 

 
#line 55 ".\\FreeRTOS\\portable\\RVDS\\ARM_CM3\\portmacro.h"

    typedef uint32_t   StackType_t;
    typedef long             BaseType_t;
    typedef unsigned long    UBaseType_t;





        typedef uint32_t     TickType_t;



 


 

 




 


 

 
#line 94 ".\\FreeRTOS\\portable\\RVDS\\ARM_CM3\\portmacro.h"
 





 

 
    extern void vPortEnterCritical( void );
    extern void vPortExitCritical( void );

#line 112 ".\\FreeRTOS\\portable\\RVDS\\ARM_CM3\\portmacro.h"

 

 

        extern void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime );


 

 






 




 



 




 



 


 


        void vPortValidateInterruptPriority( void );



 








 

    static __forceinline void vPortSetBASEPRI( uint32_t ulBASEPRI )
    {
        __asm
        {
            
 
 
            msr basepri, ulBASEPRI
 
        }
    }
 

    static __forceinline void vPortRaiseBASEPRI( void )
    {
        uint32_t ulNewBASEPRI = ( 5 << (8 - 5) );

        __asm
        {
            
 
 
            msr basepri, ulNewBASEPRI
            dsb
            isb
 
        }
    }
 

    static __forceinline void vPortClearBASEPRIFromISR( void )
    {
        __asm
        {
            

 
 
            msr basepri, # 0
 
        }
    }
 

    static __forceinline uint32_t ulPortRaiseBASEPRI( void )
    {
        uint32_t ulReturn, ulNewBASEPRI = ( 5 << (8 - 5) );

        __asm
        {
            
 
 
            mrs ulReturn, basepri
            msr basepri, ulNewBASEPRI
            dsb
            isb
 
        }

        return ulReturn;
    }
 

    static __forceinline BaseType_t xPortIsInsideInterrupt( void )
    {
        uint32_t ulCurrentInterrupt;
        BaseType_t xReturn;

         
        __asm
        {
 
            mrs ulCurrentInterrupt, ipsr
 
        }

        if( ulCurrentInterrupt == 0 )
        {
            xReturn = ( ( BaseType_t ) 0 );
        }
        else
        {
            xReturn = ( ( BaseType_t ) 1 );
        }

        return xReturn;
    }


 



 

#line 52 ".\\FreeRTOS\\include\\portable.h"










































 



 

#line 1 ".\\FreeRTOS\\include\\mpu_wrappers.h"
























 





 
#line 176 ".\\FreeRTOS\\include\\mpu_wrappers.h"









#line 101 ".\\FreeRTOS\\include\\portable.h"






 
#line 128 ".\\FreeRTOS\\include\\portable.h"
        StackType_t * pxPortInitialiseStack( StackType_t * pxTopOfStack,
                                             TaskFunction_t pxCode,
                                             void * pvParameters ) ;




 
typedef struct HeapRegion
{
    uint8_t * pucStartAddress;
    size_t xSizeInBytes;
} HeapRegion_t;

 
typedef struct xHeapStats
{
    size_t xAvailableHeapSpaceInBytes;           
    size_t xSizeOfLargestFreeBlockInBytes;       
    size_t xSizeOfSmallestFreeBlockInBytes;      
    size_t xNumberOfFreeBlocks;                  
    size_t xMinimumEverFreeBytesRemaining;       
    size_t xNumberOfSuccessfulAllocations;       
    size_t xNumberOfSuccessfulFrees;             
} HeapStats_t;











 
void vPortDefineHeapRegions( const HeapRegion_t * const pxHeapRegions ) ;




 
void vPortGetHeapStats( HeapStats_t * pxHeapStats );



 
void * pvPortMalloc( size_t xSize ) ;
void vPortFree( void * pv ) ;
void vPortInitialiseBlocks( void ) ;
size_t xPortGetFreeHeapSize( void ) ;
size_t xPortGetMinimumEverFreeHeapSize( void ) ;




 
BaseType_t xPortStartScheduler( void ) ;





 
void vPortEndScheduler( void ) ;







 
#line 209 ".\\FreeRTOS\\include\\portable.h"

 



 

#line 64 ".\\FreeRTOS\\include\\FreeRTOS.h"

 




 








 

















































#line 137 ".\\FreeRTOS\\include\\FreeRTOS.h"



        



 


















































































































#line 265 ".\\FreeRTOS\\include\\FreeRTOS.h"





 
#line 277 ".\\FreeRTOS\\include\\FreeRTOS.h"









 
#line 302 ".\\FreeRTOS\\include\\FreeRTOS.h"



































 



 






 






 






 




     




     






 









 








 








 








 








 















 

























































































































































































































































































































#line 768 ".\\FreeRTOS\\include\\FreeRTOS.h"






































































































































 







 



 


















#line 943 ".\\FreeRTOS\\include\\FreeRTOS.h"


 







 















 






 






 



#line 1008 ".\\FreeRTOS\\include\\FreeRTOS.h"


 






 










 





 





 





 





 













































 













 
struct xSTATIC_LIST_ITEM
{



    TickType_t xDummy2;
    void * pvDummy3[ 4 ];



};
typedef struct xSTATIC_LIST_ITEM StaticListItem_t;

 
struct xSTATIC_MINI_LIST_ITEM
{



    TickType_t xDummy2;
    void * pvDummy3[ 2 ];
};
typedef struct xSTATIC_MINI_LIST_ITEM StaticMiniListItem_t;

 
typedef struct xSTATIC_LIST
{



    UBaseType_t uxDummy2;
    void * pvDummy3;
    StaticMiniListItem_t xDummy4;



} StaticList_t;













 
typedef struct xSTATIC_TCB
{
    void * pxDummy1;



    StaticListItem_t xDummy3[ 2 ];
    UBaseType_t uxDummy5;
    void * pxDummy6;
    uint8_t ucDummy7[ (16) ];
#line 1184 ".\\FreeRTOS\\include\\FreeRTOS.h"
        UBaseType_t uxDummy12[ 2 ];
#line 1199 ".\\FreeRTOS\\include\\FreeRTOS.h"
        uint32_t ulDummy18[ 1 ];
        uint8_t ucDummy19[ 1 ];





#line 1212 ".\\FreeRTOS\\include\\FreeRTOS.h"
} StaticTask_t;














 
typedef struct xSTATIC_QUEUE
{
    void * pvDummy1[ 3 ];

    union
    {
        void * pvDummy2;
        UBaseType_t uxDummy2;
    } u;

    StaticList_t xDummy3[ 2 ];
    UBaseType_t uxDummy4[ 3 ];
    uint8_t ucDummy5[ 2 ];






        void * pvDummy7;






} StaticQueue_t;
typedef StaticQueue_t StaticSemaphore_t;














 
typedef struct xSTATIC_EVENT_GROUP
{
    TickType_t xDummy1;
    StaticList_t xDummy2;








} StaticEventGroup_t;














 
typedef struct xSTATIC_TIMER
{
    void * pvDummy1;
    StaticListItem_t xDummy2;
    TickType_t xDummy3;
    void * pvDummy5;
    TaskFunction_t pvDummy6;



    uint8_t ucDummy8;
} StaticTimer_t;














 
typedef struct xSTATIC_STREAM_BUFFER
{
    size_t uxDummy1[ 4 ];
    void * pvDummy2[ 3 ];
    uint8_t ucDummy3;



} StaticStreamBuffer_t;

 
typedef StaticStreamBuffer_t StaticMessageBuffer_t;

 



 

#line 39 "..\\User\\arch\\sys_arch.h"
#line 1 ".\\FreeRTOS\\include\\task.h"
























 









#line 1 ".\\FreeRTOS\\include\\list.h"
























 



























 




































 




 



 





 

     
#line 136 ".\\FreeRTOS\\include\\list.h"




 
struct xLIST;
struct xLIST_ITEM
{
                    
     TickType_t xItemValue;               
    struct xLIST_ITEM *  pxNext;          
    struct xLIST_ITEM *  pxPrevious;      
    void * pvOwner;                                          
    struct xLIST *  pvContainer;          
                   
};
typedef struct xLIST_ITEM ListItem_t;                        

struct xMINI_LIST_ITEM
{
          
     TickType_t xItemValue;
    struct xLIST_ITEM *  pxNext;
    struct xLIST_ITEM *  pxPrevious;
};
typedef struct xMINI_LIST_ITEM MiniListItem_t;



 
typedef struct xLIST
{
               
    volatile UBaseType_t uxNumberOfItems;
    ListItem_t *  pxIndex;      
    MiniListItem_t xListEnd;                       
              
} List_t;







 








 








 









 








 







 







 







 








 




 





















 
#line 291 ".\\FreeRTOS\\include\\list.h"

















 










 







 






 











 
void vListInitialise( List_t * const pxList ) ;









 
void vListInitialiseItem( ListItem_t * const pxItem ) ;











 
void vListInsert( List_t * const pxList,
                  ListItem_t * const pxNewListItem ) ;



















 
void vListInsertEnd( List_t * const pxList,
                     ListItem_t * const pxNewListItem ) ;













 
UBaseType_t uxListRemove( ListItem_t * const pxItemToRemove ) ;

 



 

#line 36 ".\\FreeRTOS\\include\\task.h"

 



 



 







 










 











 
struct tskTaskControlBlock;      
typedef struct tskTaskControlBlock * TaskHandle_t;




 
typedef BaseType_t (* TaskHookFunction_t)( void * );

 
typedef enum
{
    eRunning = 0,      
    eReady,            
    eBlocked,          
    eSuspended,        
    eDeleted,          
    eInvalid           
} eTaskState;

 
typedef enum
{
    eNoAction = 0,                 
    eSetBits,                      
    eIncrement,                    
    eSetValueWithOverwrite,        
    eSetValueWithoutOverwrite      
} eNotifyAction;



 
typedef struct xTIME_OUT
{
    BaseType_t xOverflowCount;
    TickType_t xTimeOnEntering;
} TimeOut_t;



 
typedef struct xMEMORY_REGION
{
    void * pvBaseAddress;
    uint32_t ulLengthInBytes;
    uint32_t ulParameters;
} MemoryRegion_t;



 
typedef struct xTASK_PARAMETERS
{
    TaskFunction_t pvTaskCode;
    const char * pcName;      
    uint16_t usStackDepth;
    void * pvParameters;
    UBaseType_t uxPriority;
    StackType_t * puxStackBuffer;
    MemoryRegion_t xRegions[ 1 ];



} TaskParameters_t;


 
typedef struct xTASK_STATUS
{
    TaskHandle_t xHandle;                             
    const char * pcTaskName;                            
    UBaseType_t xTaskNumber;                          
    eTaskState eCurrentState;                         
    UBaseType_t uxCurrentPriority;                    
    UBaseType_t uxBasePriority;                       
    uint32_t ulRunTimeCounter;                        
    StackType_t * pxStackBase;                        
    uint16_t usStackHighWaterMark;      
} TaskStatus_t;

 
typedef enum
{
    eAbortSleep = 0,            
    eStandardSleep,             
    eNoTasksWaitingTimeout      
} eSleepModeStatus;





 









 













 














 










 









 




 







 






























































































 

    BaseType_t xTaskCreate( TaskFunction_t pxTaskCode,
                            const char * const pcName,      
                            const uint16_t usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pxCreatedTask ) ;













































































































 
#line 465 ".\\FreeRTOS\\include\\task.h"









































































 

























































































 



















































 
void vTaskAllocateMPURegions( TaskHandle_t xTask,
                              const MemoryRegion_t * const pxRegions ) ;









































 
void vTaskDelete( TaskHandle_t xTaskToDelete ) ;



 
















































 
void vTaskDelay( const TickType_t xTicksToDelay ) ;

































































 
BaseType_t xTaskDelayUntil( TickType_t * const pxPreviousWakeTime,
                            const TickType_t xTimeIncrement ) ;




 



































 
BaseType_t xTaskAbortDelay( TaskHandle_t xTask ) ;















































 
UBaseType_t uxTaskPriorityGet( const TaskHandle_t xTask ) ;








 
UBaseType_t uxTaskPriorityGetFromISR( const TaskHandle_t xTask ) ;


















 
eTaskState eTaskGetState( TaskHandle_t xTask ) ;
























































 
void vTaskGetInfo( TaskHandle_t xTask,
                   TaskStatus_t * pxTaskStatus,
                   BaseType_t xGetFreeStackSpace,
                   eTaskState eState ) ;










































 
void vTaskPrioritySet( TaskHandle_t xTask,
                       UBaseType_t uxNewPriority ) ;



















































 
void vTaskSuspend( TaskHandle_t xTaskToSuspend ) ;

















































 
void vTaskResume( TaskHandle_t xTaskToResume ) ;





























 
BaseType_t xTaskResumeFromISR( TaskHandle_t xTaskToResume ) ;



 





























 
void vTaskStartScheduler( void ) ;
























































 
void vTaskEndScheduler( void ) ;



















































 
void vTaskSuspendAll( void ) ;






















































 
BaseType_t xTaskResumeAll( void ) ;



 









 
TickType_t xTaskGetTickCount( void ) ;














 
TickType_t xTaskGetTickCountFromISR( void ) ;












 
UBaseType_t uxTaskGetNumberOfTasks( void ) ;











 
char * pcTaskGetName( TaskHandle_t xTaskToQuery ) ;      














 
TaskHandle_t xTaskGetHandle( const char * pcNameToQuery ) ;      

























 
UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask ) ;

























 
uint16_t uxTaskGetStackHighWaterMark2( TaskHandle_t xTask ) ;






 
#line 1587 ".\\FreeRTOS\\include\\task.h"

#line 1602 ".\\FreeRTOS\\include\\task.h"

#line 1620 ".\\FreeRTOS\\include\\task.h"

#line 1631 ".\\FreeRTOS\\include\\task.h"

#line 1648 ".\\FreeRTOS\\include\\task.h"













 
BaseType_t xTaskCallApplicationTaskHook( TaskHandle_t xTask,
                                         void * pvParameter ) ;







 
TaskHandle_t xTaskGetIdleTaskHandle( void ) ;

































































































 
UBaseType_t uxTaskGetSystemState( TaskStatus_t * const pxTaskStatusArray,
                                  const UBaseType_t uxArraySize,
                                  uint32_t * const pulTotalRunTime ) ;













































 
void vTaskList( char * pcWriteBuffer ) ;      




















































 
void vTaskGetRunTimeStats( char * pcWriteBuffer ) ;      




























 
uint32_t ulTaskGetIdleRunTimeCounter( void ) ;











































































































 
BaseType_t xTaskGenericNotify( TaskHandle_t xTaskToNotify,
                               UBaseType_t uxIndexToNotify,
                               uint32_t ulValue,
                               eNotifyAction eAction,
                               uint32_t * pulPreviousNotificationValue ) ;


























 



















































































































 
BaseType_t xTaskGenericNotifyFromISR( TaskHandle_t xTaskToNotify,
                                      UBaseType_t uxIndexToNotify,
                                      uint32_t ulValue,
                                      eNotifyAction eAction,
                                      uint32_t * pulPreviousNotificationValue,
                                      BaseType_t * pxHigherPriorityTaskWoken ) ;


























 












































































































 
BaseType_t xTaskGenericNotifyWait( UBaseType_t uxIndexToWaitOn,
                                   uint32_t ulBitsToClearOnEntry,
                                   uint32_t ulBitsToClearOnExit,
                                   uint32_t * pulNotificationValue,
                                   TickType_t xTicksToWait ) ;










































































 

















































































 
void vTaskGenericNotifyGiveFromISR( TaskHandle_t xTaskToNotify,
                                    UBaseType_t uxIndexToNotify,
                                    BaseType_t * pxHigherPriorityTaskWoken ) ;






































































































 
uint32_t ulTaskGenericNotifyTake( UBaseType_t uxIndexToWaitOn,
                                  BaseType_t xClearCountOnExit,
                                  TickType_t xTicksToWait ) ;





























































 
BaseType_t xTaskGenericNotifyStateClear( TaskHandle_t xTask,
                                         UBaseType_t uxIndexToClear ) ;






























































 
uint32_t ulTaskGenericNotifyValueClear( TaskHandle_t xTask,
                                        UBaseType_t uxIndexToClear,
                                        uint32_t ulBitsToClear ) ;


















 
void vTaskSetTimeOutState( TimeOut_t * const pxTimeOut ) ;



















































































 
BaseType_t xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut,
                                 TickType_t * const pxTicksToWait ) ;


























 
BaseType_t xTaskCatchUpTicks( TickType_t xTicksToCatchUp ) ;




 















 
BaseType_t xTaskIncrementTick( void ) ;































 
void vTaskPlaceOnEventList( List_t * const pxEventList,
                            const TickType_t xTicksToWait ) ;
void vTaskPlaceOnUnorderedEventList( List_t * pxEventList,
                                     const TickType_t xItemValue,
                                     const TickType_t xTicksToWait ) ;











 
void vTaskPlaceOnEventListRestricted( List_t * const pxEventList,
                                      TickType_t xTicksToWait,
                                      const BaseType_t xWaitIndefinitely ) ;
























 
BaseType_t xTaskRemoveFromEventList( const List_t * const pxEventList ) ;
void vTaskRemoveFromUnorderedEventList( ListItem_t * pxEventListItem,
                                        const TickType_t xItemValue ) ;








 
 void vTaskSwitchContext( void ) ;




 
TickType_t uxTaskResetEventItemValue( void ) ;



 
TaskHandle_t xTaskGetCurrentTaskHandle( void ) ;




 
void vTaskMissedYield( void ) ;




 
BaseType_t xTaskGetSchedulerState( void ) ;




 
BaseType_t xTaskPriorityInherit( TaskHandle_t const pxMutexHolder ) ;




 
BaseType_t xTaskPriorityDisinherit( TaskHandle_t const pxMutexHolder ) ;








 
void vTaskPriorityDisinheritAfterTimeout( TaskHandle_t const pxMutexHolder,
                                          UBaseType_t uxHighestPriorityWaitingTask ) ;



 
UBaseType_t uxTaskGetTaskNumber( TaskHandle_t xTask ) ;




 
void vTaskSetTaskNumber( TaskHandle_t xTask,
                         const UBaseType_t uxHandle ) ;








 
void vTaskStepTick( const TickType_t xTicksToJump ) ;














 
eSleepModeStatus eTaskConfirmSleepModeStatus( void ) ;




 
TaskHandle_t pvTaskIncrementMutexHeldCount( void ) ;




 
void vTaskInternalSetTimeOutState( TimeOut_t * const pxTimeOut ) ;


 



 
#line 40 "..\\User\\arch\\sys_arch.h"
#line 1 ".\\FreeRTOS\\include\\queue.h"
























 









 



 

#line 42 ".\\FreeRTOS\\include\\queue.h"





 
struct QueueDefinition;  
typedef struct QueueDefinition   * QueueHandle_t;





 
typedef struct QueueDefinition   * QueueSetHandle_t;





 
typedef struct QueueDefinition   * QueueSetMemberHandle_t;

 




 
#line 77 ".\\FreeRTOS\\include\\queue.h"




































































 





















































































 

















































































 


















































































 




















































































 



















































































 
























































































 
BaseType_t xQueueGenericSend( QueueHandle_t xQueue,
                              const void * const pvItemToQueue,
                              TickType_t xTicksToWait,
                              const BaseType_t xCopyPosition ) ;





























































































 
BaseType_t xQueuePeek( QueueHandle_t xQueue,
                       void * const pvBuffer,
                       TickType_t xTicksToWait ) ;
































 
BaseType_t xQueuePeekFromISR( QueueHandle_t xQueue,
                              void * const pvBuffer ) ;


























































































 
BaseType_t xQueueReceive( QueueHandle_t xQueue,
                          void * const pvBuffer,
                          TickType_t xTicksToWait ) ;















 
UBaseType_t uxQueueMessagesWaiting( const QueueHandle_t xQueue ) ;

















 
UBaseType_t uxQueueSpacesAvailable( const QueueHandle_t xQueue ) ;














 
void vQueueDelete( QueueHandle_t xQueue ) ;




































































 







































































 























































































 










































































 















































































 
BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue,
                                     const void * const pvItemToQueue,
                                     BaseType_t * const pxHigherPriorityTaskWoken,
                                     const BaseType_t xCopyPosition ) ;
BaseType_t xQueueGiveFromISR( QueueHandle_t xQueue,
                              BaseType_t * const pxHigherPriorityTaskWoken ) ;























































































 
BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue,
                                 void * const pvBuffer,
                                 BaseType_t * const pxHigherPriorityTaskWoken ) ;




 
BaseType_t xQueueIsQueueEmptyFromISR( const QueueHandle_t xQueue ) ;
BaseType_t xQueueIsQueueFullFromISR( const QueueHandle_t xQueue ) ;
UBaseType_t uxQueueMessagesWaitingFromISR( const QueueHandle_t xQueue ) ;









 
BaseType_t xQueueCRSendFromISR( QueueHandle_t xQueue,
                                const void * pvItemToQueue,
                                BaseType_t xCoRoutinePreviouslyWoken );
BaseType_t xQueueCRReceiveFromISR( QueueHandle_t xQueue,
                                   void * pvBuffer,
                                   BaseType_t * pxTaskWoken );
BaseType_t xQueueCRSend( QueueHandle_t xQueue,
                         const void * pvItemToQueue,
                         TickType_t xTicksToWait );
BaseType_t xQueueCRReceive( QueueHandle_t xQueue,
                            void * pvBuffer,
                            TickType_t xTicksToWait );





 
QueueHandle_t xQueueCreateMutex( const uint8_t ucQueueType ) ;
QueueHandle_t xQueueCreateMutexStatic( const uint8_t ucQueueType,
                                       StaticQueue_t * pxStaticQueue ) ;
QueueHandle_t xQueueCreateCountingSemaphore( const UBaseType_t uxMaxCount,
                                             const UBaseType_t uxInitialCount ) ;
QueueHandle_t xQueueCreateCountingSemaphoreStatic( const UBaseType_t uxMaxCount,
                                                   const UBaseType_t uxInitialCount,
                                                   StaticQueue_t * pxStaticQueue ) ;
BaseType_t xQueueSemaphoreTake( QueueHandle_t xQueue,
                                TickType_t xTicksToWait ) ;
TaskHandle_t xQueueGetMutexHolder( QueueHandle_t xSemaphore ) ;
TaskHandle_t xQueueGetMutexHolderFromISR( QueueHandle_t xSemaphore ) ;




 
BaseType_t xQueueTakeMutexRecursive( QueueHandle_t xMutex,
                                     TickType_t xTicksToWait ) ;
BaseType_t xQueueGiveMutexRecursive( QueueHandle_t xMutex ) ;




 























 

    void vQueueAddToRegistry( QueueHandle_t xQueue,
                              const char * pcQueueName ) ;  











 

    void vQueueUnregisterQueue( QueueHandle_t xQueue ) ;












 

    const char * pcQueueGetName( QueueHandle_t xQueue ) ;  






 

    QueueHandle_t xQueueGenericCreate( const UBaseType_t uxQueueLength,
                                       const UBaseType_t uxItemSize,
                                       const uint8_t ucQueueType ) ;






 
#line 1560 ".\\FreeRTOS\\include\\queue.h"
















































 
QueueSetHandle_t xQueueCreateSet( const UBaseType_t uxEventQueueLength ) ;






















 
BaseType_t xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                           QueueSetHandle_t xQueueSet ) ;

















 
BaseType_t xQueueRemoveFromSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                                QueueSetHandle_t xQueueSet ) ;


































 
QueueSetMemberHandle_t xQueueSelectFromSet( QueueSetHandle_t xQueueSet,
                                            const TickType_t xTicksToWait ) ;



 
QueueSetMemberHandle_t xQueueSelectFromSetFromISR( QueueSetHandle_t xQueueSet ) ;

 
void vQueueWaitForMessageRestricted( QueueHandle_t xQueue,
                                     TickType_t xTicksToWait,
                                     const BaseType_t xWaitIndefinitely ) ;
BaseType_t xQueueGenericReset( QueueHandle_t xQueue,
                               BaseType_t xNewQueue ) ;
void vQueueSetQueueNumber( QueueHandle_t xQueue,
                           UBaseType_t uxQueueNumber ) ;
UBaseType_t uxQueueGetQueueNumber( QueueHandle_t xQueue ) ;
uint8_t ucQueueGetQueueType( QueueHandle_t xQueue ) ;


 



 

#line 41 "..\\User\\arch\\sys_arch.h"
#line 1 ".\\FreeRTOS\\include\\semphr.h"
























 








#line 35 ".\\FreeRTOS\\include\\semphr.h"

typedef QueueHandle_t SemaphoreHandle_t;
























































 
#line 104 ".\\FreeRTOS\\include\\semphr.h"



























































 




























































 






































































 






























































































 


































































 





















































































 





























































































 


































 


























































 






























































 





































































 










































































 

















































































 






















































































 

















 
















 












 













 


#line 42 "..\\User\\arch\\sys_arch.h"

 












 





 





 




 






typedef SemaphoreHandle_t sys_sem_t;
typedef SemaphoreHandle_t sys_mutex_t;
typedef QueueHandle_t sys_mbox_t;
typedef TaskHandle_t sys_thread_t;

typedef int sys_prot_t;



void TCPIP_Init(void);

#line 48 "..\\Lwip\\include\\lwip\\tcpip.h"






 
extern sys_mutex_t lock_tcpip_core;

 

 
#line 66 "..\\Lwip\\include\\lwip\\tcpip.h"

struct pbuf;
struct netif;

 
typedef void (*tcpip_init_done_fn)(void *arg);
 
typedef void (*tcpip_callback_fn)(void *ctx);

 
struct tcpip_callback_msg;

void   tcpip_init(tcpip_init_done_fn tcpip_init_done, void *arg);

err_t  tcpip_inpkt(struct pbuf *p, struct netif *inp, netif_input_fn input_fn);
err_t  tcpip_input(struct pbuf *p, struct netif *inp);

err_t  tcpip_try_callback(tcpip_callback_fn function, void *ctx);
err_t  tcpip_callback(tcpip_callback_fn function, void *ctx);


 


struct tcpip_callback_msg* tcpip_callbackmsg_new(tcpip_callback_fn function, void *ctx);
void   tcpip_callbackmsg_delete(struct tcpip_callback_msg* msg);
err_t  tcpip_callbackmsg_trycallback(struct tcpip_callback_msg* msg);
err_t  tcpip_callbackmsg_trycallback_fromisr(struct tcpip_callback_msg* msg);

 
err_t  pbuf_free_callback(struct pbuf *p);
err_t  mem_free_callback(void *m);
















#line 36 "..\\User\\arch/sys_arch.h"

  
#line 39 "..\\User\\arch/sys_arch.h"
#line 40 "..\\User\\arch/sys_arch.h"
#line 41 "..\\User\\arch/sys_arch.h"
#line 42 "..\\User\\arch/sys_arch.h"

 












 





 





 




 






typedef SemaphoreHandle_t sys_sem_t;
typedef SemaphoreHandle_t sys_mutex_t;
typedef QueueHandle_t sys_mbox_t;
typedef TaskHandle_t sys_thread_t;

typedef int sys_prot_t;



void TCPIP_Init(void);

#line 96 "..\\Lwip\\include\\lwip/sys.h"

 
typedef void (*lwip_thread_fn)(void *arg);


 

 


 




#line 122 "..\\Lwip\\include\\lwip/sys.h"














 
err_t sys_mutex_new(sys_mutex_t *mutex);




 
void sys_mutex_lock(sys_mutex_t *mutex);




 
void sys_mutex_unlock(sys_mutex_t *mutex);




 
void sys_mutex_free(sys_mutex_t *mutex);







 
int sys_mutex_valid(sys_mutex_t *mutex);








 
void sys_mutex_set_invalid(sys_mutex_t *mutex);



 















 
err_t sys_sem_new(sys_sem_t *sem, u8_t count);




 
void sys_sem_signal(sys_sem_t *sem);
















 
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout);




 
void sys_sem_free(sys_sem_t *sem);
 








 
int sys_sem_valid(sys_sem_t *sem);








 
void sys_sem_set_invalid(sys_sem_t *sem);




 





 







 
void sys_msleep(u32_t ms);  


 














 
err_t sys_mbox_new(sys_mbox_t *mbox, int size);







 
void sys_mbox_post(sys_mbox_t *mbox, void *msg);








 
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg);








 
err_t sys_mbox_trypost_fromisr(sys_mbox_t *mbox, void *msg);




















 
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout);
 
















 
u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg);



 








 
void sys_mbox_free(sys_mbox_t *mbox);








 
int sys_mbox_valid(sys_mbox_t *mbox);








 
void sys_mbox_set_invalid(sys_mbox_t *mbox);




 





 


















 
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio);







 
void sys_init(void);




 
u32_t sys_jiffies(void);









 
u32_t sys_now(void);

 



 





 








 











 










 

sys_prot_t sys_arch_protect(void);
void sys_arch_unprotect(sys_prot_t pval);

#line 502 "..\\Lwip\\include\\lwip/sys.h"






 

#line 518 "..\\Lwip\\include\\lwip/sys.h"

#line 527 "..\\Lwip\\include\\lwip/sys.h"

#line 536 "..\\Lwip\\include\\lwip/sys.h"

#line 545 "..\\Lwip\\include\\lwip/sys.h"

#line 554 "..\\Lwip\\include\\lwip/sys.h"






#line 46 "..\\Lwip\\include\\lwip/priv/api_msg.h"
#line 1 "..\\Lwip\\include\\lwip/igmp.h"



 

































 




#line 44 "..\\Lwip\\include\\lwip/igmp.h"
#line 45 "..\\Lwip\\include\\lwip/igmp.h"
#line 46 "..\\Lwip\\include\\lwip/igmp.h"
#line 47 "..\\Lwip\\include\\lwip/igmp.h"

#line 114 "..\\Lwip\\include\\lwip/igmp.h"

#line 47 "..\\Lwip\\include\\lwip/priv/api_msg.h"
#line 1 "..\\Lwip\\include\\lwip/api.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/api.h"



 

#line 47 "..\\Lwip\\include\\lwip/api.h"
#line 1 "..\\Lwip\\include\\lwip/netbuf.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/netbuf.h"



 

#line 47 "..\\Lwip\\include\\lwip/netbuf.h"
#line 48 "..\\Lwip\\include\\lwip/netbuf.h"
#line 49 "..\\Lwip\\include\\lwip/netbuf.h"





 

 


 
struct netbuf {
  struct pbuf *p, *ptr;
  ip_addr_t addr;
  u16_t port;
#line 71 "..\\Lwip\\include\\lwip/netbuf.h"
};

 
struct netbuf *   netbuf_new      (void);
void              netbuf_delete   (struct netbuf *buf);
void *            netbuf_alloc    (struct netbuf *buf, u16_t size);
void              netbuf_free     (struct netbuf *buf);
err_t             netbuf_ref      (struct netbuf *buf,
                                   const void *dataptr, u16_t size);
void              netbuf_chain    (struct netbuf *head, struct netbuf *tail);

err_t             netbuf_data     (struct netbuf *buf,
                                   void **dataptr, u16_t *len);
s8_t              netbuf_next     (struct netbuf *buf);
void              netbuf_first    (struct netbuf *buf);


#line 109 "..\\Lwip\\include\\lwip/netbuf.h"







#line 48 "..\\Lwip\\include\\lwip/api.h"
#line 49 "..\\Lwip\\include\\lwip/api.h"
#line 50 "..\\Lwip\\include\\lwip/api.h"
#line 51 "..\\Lwip\\include\\lwip/api.h"







 

 
#line 68 "..\\Lwip\\include\\lwip/api.h"

 
 

 

 






 
#line 93 "..\\Lwip\\include\\lwip/api.h"
 


 
#line 109 "..\\Lwip\\include\\lwip/api.h"



 
enum netconn_type {
  NETCONN_INVALID     = 0,
   
  NETCONN_TCP         = 0x10,




   
  NETCONN_UDP         = 0x20,
   
  NETCONN_UDPLITE     = 0x21,
   
  NETCONN_UDPNOCHKSUM = 0x22,

#line 136 "..\\Lwip\\include\\lwip/api.h"

   
  NETCONN_RAW         = 0x40




};


 
enum netconn_state {
  NETCONN_NONE,
  NETCONN_WRITE,
  NETCONN_LISTEN,
  NETCONN_CONNECT,
  NETCONN_CLOSE
};


























 
enum netconn_evt {
  NETCONN_EVT_RCVPLUS,
  NETCONN_EVT_RCVMINUS,
  NETCONN_EVT_SENDPLUS,
  NETCONN_EVT_SENDMINUS,
  NETCONN_EVT_ERROR
};

#line 196 "..\\Lwip\\include\\lwip/api.h"

#line 205 "..\\Lwip\\include\\lwip/api.h"

 
struct ip_pcb;
struct tcp_pcb;
struct udp_pcb;
struct raw_pcb;
struct netconn;
struct api_msg;

 
typedef void (* netconn_callback)(struct netconn *, enum netconn_evt, u16_t len);

 
struct netconn {
   
  enum netconn_type type;
   
  enum netconn_state state;
   
  union {
    struct ip_pcb  *ip;
    struct tcp_pcb *tcp;
    struct udp_pcb *udp;
    struct raw_pcb *raw;
  } pcb;
   
  err_t pending_err;

   
  sys_sem_t op_completed;

  
 
  sys_mbox_t recvmbox;

  
 
  sys_mbox_t acceptmbox;
#line 249 "..\\Lwip\\include\\lwip/api.h"
   

  int socket;
#line 276 "..\\Lwip\\include\\lwip/api.h"
   
  u8_t flags;

  

 
  struct api_msg *current_msg;

   
  netconn_callback callback;
};





 
struct netvector {
   
  const void *ptr;
   
  size_t len;
};

 




 



 


struct netconn *netconn_new_with_proto_and_callback(enum netconn_type t, u8_t proto,
                                             netconn_callback callback);
err_t   netconn_prepare_delete(struct netconn *conn);
err_t   netconn_delete(struct netconn *conn);
 


err_t   netconn_getaddr(struct netconn *conn, ip_addr_t *addr,
                        u16_t *port, u8_t local);
 

 


err_t   netconn_bind(struct netconn *conn, const ip_addr_t *addr, u16_t port);
err_t   netconn_bind_if(struct netconn *conn, u8_t if_idx);
err_t   netconn_connect(struct netconn *conn, const ip_addr_t *addr, u16_t port);
err_t   netconn_disconnect (struct netconn *conn);
err_t   netconn_listen_with_backlog(struct netconn *conn, u8_t backlog);
 

err_t   netconn_accept(struct netconn *conn, struct netconn **new_conn);
err_t   netconn_recv(struct netconn *conn, struct netbuf **new_buf);
err_t   netconn_recv_udp_raw_netbuf(struct netconn *conn, struct netbuf **new_buf);
err_t   netconn_recv_udp_raw_netbuf_flags(struct netconn *conn, struct netbuf **new_buf, u8_t apiflags);
err_t   netconn_recv_tcp_pbuf(struct netconn *conn, struct pbuf **new_buf);
err_t   netconn_recv_tcp_pbuf_flags(struct netconn *conn, struct pbuf **new_buf, u8_t apiflags);
err_t   netconn_tcp_recvd(struct netconn *conn, size_t len);
err_t   netconn_sendto(struct netconn *conn, struct netbuf *buf,
                             const ip_addr_t *addr, u16_t port);
err_t   netconn_send(struct netconn *conn, struct netbuf *buf);
err_t   netconn_write_partly(struct netconn *conn, const void *dataptr, size_t size,
                             u8_t apiflags, size_t *bytes_written);
err_t   netconn_write_vectors_partly(struct netconn *conn, struct netvector *vectors, u16_t vectorcnt,
                                     u8_t apiflags, size_t *bytes_written);
 


err_t   netconn_close(struct netconn *conn);
err_t   netconn_shutdown(struct netconn *conn, u8_t shut_rx, u8_t shut_tx);

#line 368 "..\\Lwip\\include\\lwip/api.h"

err_t   netconn_err(struct netconn *conn);






 




 


#line 397 "..\\Lwip\\include\\lwip/api.h"

#line 416 "..\\Lwip\\include\\lwip/api.h"

#line 424 "..\\Lwip\\include\\lwip/api.h"







#line 48 "..\\Lwip\\include\\lwip/priv/api_msg.h"
#line 1 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"



#line 45 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"
#line 46 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"
#line 47 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"





struct pbuf;
struct netif;

#line 96 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"

err_t tcpip_send_msg_wait_sem(tcpip_callback_fn fn, void *apimsg, sys_sem_t* sem);

struct tcpip_api_call_data
{
#line 107 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"
  u8_t dummy;  

};
typedef err_t (*tcpip_api_call_fn)(struct tcpip_api_call_data* call);
err_t tcpip_api_call(tcpip_api_call_fn fn, struct tcpip_api_call_data *call);

enum tcpip_msg_type {





  TCPIP_MSG_INPKT,





  TCPIP_MSG_CALLBACK,
  TCPIP_MSG_CALLBACK_STATIC
};

struct tcpip_msg {
  enum tcpip_msg_type type;
  union {
#line 144 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"
    struct {
      struct pbuf *p;
      struct netif *netif;
      netif_input_fn input_fn;
    } inp;

    struct {
      tcpip_callback_fn function;
      void *ctx;
    } cb;
#line 161 "..\\Lwip\\include\\lwip/priv/tcpip_priv.h"
  } msg;
};







#line 49 "..\\Lwip\\include\\lwip/priv/api_msg.h"







 

#line 67 "..\\Lwip\\include\\lwip/priv/api_msg.h"

 






 


 
struct api_msg {
  
 
  struct netconn *conn;
   
  err_t err;
   
  union {
     
    struct netbuf *b;
     
    struct {
      u8_t proto;
    } n;
     
    struct {
      const ip_addr_t * ipaddr;
      u16_t port;
      u8_t if_idx;
    } bc;
     
    struct {
      ip_addr_t *ipaddr;
      u16_t *port;
      u8_t local;
    } ad;
     
    struct {
       
      const struct netvector *vector;
       
      u16_t vector_cnt;
       
      size_t vector_off;
       
      size_t len;
       
      size_t offset;
      u8_t apiflags;



    } w;
     
    struct {
      size_t len;
    } r;

     
    struct {
      u8_t shut;



      u8_t polls_left;

    } sd;
#line 151 "..\\Lwip\\include\\lwip/priv/api_msg.h"
  } msg;



};








#line 189 "..\\Lwip\\include\\lwip/priv/api_msg.h"




int lwip_netconn_is_err_msg(void *msg, err_t *err);
void lwip_netconn_do_newconn         (void *m);
void lwip_netconn_do_delconn         (void *m);
void lwip_netconn_do_bind            (void *m);
void lwip_netconn_do_bind_if         (void *m);
void lwip_netconn_do_connect         (void *m);
void lwip_netconn_do_disconnect      (void *m);
void lwip_netconn_do_listen          (void *m);
void lwip_netconn_do_send            (void *m);
void lwip_netconn_do_recv            (void *m);



void lwip_netconn_do_write           (void *m);
void lwip_netconn_do_getaddr         (void *m);
void lwip_netconn_do_close           (void *m);
void lwip_netconn_do_shutdown        (void *m);









struct netconn* netconn_alloc(enum netconn_type t, netconn_callback callback);
void netconn_free(struct netconn *conn);



#line 267 "..\\Lwip\\include\\lwip/priv/api_msg.h"





#line 44 "..\\Lwip\\api\\api_msg.c"

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













#line 46 "..\\Lwip\\api\\api_msg.c"
#line 47 "..\\Lwip\\api\\api_msg.c"
#line 1 "..\\Lwip\\include\\lwip/udp.h"




 































 



#line 42 "..\\Lwip\\include\\lwip/udp.h"



#line 46 "..\\Lwip\\include\\lwip/udp.h"
#line 47 "..\\Lwip\\include\\lwip/udp.h"
#line 48 "..\\Lwip\\include\\lwip/udp.h"
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







#line 48 "..\\Lwip\\api\\api_msg.c"
#line 1 "..\\Lwip\\include\\lwip/tcp.h"




 































 



#line 42 "..\\Lwip\\include\\lwip/tcp.h"



#line 1 "..\\Lwip\\include\\lwip/tcpbase.h"




 































 



#line 42 "..\\Lwip\\include\\lwip/tcpbase.h"











typedef u16_t tcpwnd_size_t;


enum tcp_state {
  CLOSED      = 0,
  LISTEN      = 1,
  SYN_SENT    = 2,
  SYN_RCVD    = 3,
  ESTABLISHED = 4,
  FIN_WAIT_1  = 5,
  FIN_WAIT_2  = 6,
  CLOSE_WAIT  = 7,
  CLOSING     = 8,
  LAST_ACK    = 9,
  TIME_WAIT   = 10
};
 


 







const char* tcp_debug_state_str(enum tcp_state s);







#line 46 "..\\Lwip\\include\\lwip/tcp.h"
#line 47 "..\\Lwip\\include\\lwip/tcp.h"
#line 48 "..\\Lwip\\include\\lwip/tcp.h"
#line 49 "..\\Lwip\\include\\lwip/tcp.h"
#line 1 "..\\Lwip\\include\\lwip/icmp.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/icmp.h"
#line 42 "..\\Lwip\\include\\lwip/icmp.h"
#line 43 "..\\Lwip\\include\\lwip/icmp.h"
#line 44 "..\\Lwip\\include\\lwip/icmp.h"
#line 1 "..\\Lwip\\include\\lwip/prot/icmp.h"



 































 



#line 41 "..\\Lwip\\include\\lwip/prot/icmp.h"





#line 59 "..\\Lwip\\include\\lwip/prot/icmp.h"








 
__packed
struct icmp_echo_hdr {
  u8_t type;
  u8_t code;
  u16_t chksum;
  u16_t id;
  u16_t seqno;
} ;





 









#line 45 "..\\Lwip\\include\\lwip/icmp.h"









 
enum icmp_dur_type {
   
  ICMP_DUR_NET   = 0,
   
  ICMP_DUR_HOST  = 1,
   
  ICMP_DUR_PROTO = 2,
   
  ICMP_DUR_PORT  = 3,
   
  ICMP_DUR_FRAG  = 4,
   
  ICMP_DUR_SR    = 5
};

 
enum icmp_te_type {
   
  ICMP_TE_TTL  = 0,
   
  ICMP_TE_FRAG = 1
};



void icmp_input(struct pbuf *p, struct netif *inp);
void icmp_dest_unreach(struct pbuf *p, enum icmp_dur_type t);
void icmp_time_exceeded(struct pbuf *p, enum icmp_te_type t);



#line 105 "..\\Lwip\\include\\lwip/icmp.h"





#line 50 "..\\Lwip\\include\\lwip/tcp.h"
#line 51 "..\\Lwip\\include\\lwip/tcp.h"
#line 52 "..\\Lwip\\include\\lwip/tcp.h"
#line 53 "..\\Lwip\\include\\lwip/tcp.h"





struct tcp_pcb;
struct tcp_pcb_listen;









 
typedef err_t (*tcp_accept_fn)(void *arg, struct tcp_pcb *newpcb, err_t err);










 
typedef err_t (*tcp_recv_fn)(void *arg, struct tcp_pcb *tpcb,
                             struct pbuf *p, err_t err);











 
typedef err_t (*tcp_sent_fn)(void *arg, struct tcp_pcb *tpcb,
                              u16_t len);









 
typedef err_t (*tcp_poll_fn)(void *arg, struct tcp_pcb *tpcb);










 
typedef void  (*tcp_err_fn)(void *arg, err_t err);












 
typedef err_t (*tcp_connected_fn)(void *arg, struct tcp_pcb *tpcb, err_t err);

#line 147 "..\\Lwip\\include\\lwip/tcp.h"
 
#line 155 "..\\Lwip\\include\\lwip/tcp.h"

#line 166 "..\\Lwip\\include\\lwip/tcp.h"






 
typedef void (*tcp_extarg_callback_pcb_destroyed_fn)(u8_t id, void *data);







 
typedef err_t (*tcp_extarg_callback_passive_open_fn)(u8_t id, struct tcp_pcb_listen *lpcb, struct tcp_pcb *cpcb);

 
struct tcp_ext_arg_callbacks {
   
  tcp_extarg_callback_pcb_destroyed_fn destroy;
   
  tcp_extarg_callback_passive_open_fn passive_open;
};



#line 205 "..\\Lwip\\include\\lwip/tcp.h"

typedef u16_t tcpflags_t;




 
#line 220 "..\\Lwip\\include\\lwip/tcp.h"


 
struct tcp_pcb_listen {
 
  ip_addr_t local_ip; ip_addr_t remote_ip; u8_t netif_idx; u8_t so_options; u8_t tos; u8_t ttl ;
 
  struct tcp_pcb_listen *next; void *callback_arg;  enum tcp_state state; u8_t prio; u16_t local_port;


   
  tcp_accept_fn accept;






};


 
struct tcp_pcb {
 
  ip_addr_t local_ip; ip_addr_t remote_ip; u8_t netif_idx; u8_t so_options; u8_t tos; u8_t ttl ;
 
  struct tcp_pcb *next; void *callback_arg;  enum tcp_state state; u8_t prio; u16_t local_port;

   
  u16_t remote_port;

  tcpflags_t flags;
#line 273 "..\\Lwip\\include\\lwip/tcp.h"

  
 

   
  u8_t polltmr, pollinterval;
  u8_t last_timer;
  u32_t tmr;

   
  u32_t rcv_nxt;    
  tcpwnd_size_t rcv_wnd;    
  tcpwnd_size_t rcv_ann_wnd;  
  u32_t rcv_ann_right_edge;  







   
  s16_t rtime;

  u16_t mss;    

   
  u32_t rttest;  
  u32_t rtseq;   
  s16_t sa, sv;  

  s16_t rto;     
  u8_t nrtx;     

   
  u8_t dupacks;
  u32_t lastack;  

   
  tcpwnd_size_t cwnd;
  tcpwnd_size_t ssthresh;

   
  u32_t rto_end;

   
  u32_t snd_nxt;    
  u32_t snd_wl1, snd_wl2; 
 
  u32_t snd_lbb;        
  tcpwnd_size_t snd_wnd;    
  tcpwnd_size_t snd_wnd_max;  

  tcpwnd_size_t snd_buf;    

  u16_t snd_queuelen;  


   
  u16_t unsent_oversize;


  tcpwnd_size_t bytes_acked;

   
  struct tcp_seg *unsent;    
  struct tcp_seg *unacked;   




  struct pbuf *refused_data;  


  struct tcp_pcb_listen* listener;



   
  tcp_sent_fn sent;
   
  tcp_recv_fn recv;
   
  tcp_connected_fn connected;
   
  tcp_poll_fn poll;
   
  tcp_err_fn errf;







   
  u32_t keep_idle;





   
  u8_t persist_cnt;
   
  u8_t persist_backoff;
   
  u8_t persist_probe;

   
  u8_t keep_cnt_sent;





};

#line 409 "..\\Lwip\\include\\lwip/tcp.h"

 
struct tcp_pcb * tcp_new     (void);
struct tcp_pcb * tcp_new_ip_type (u8_t type);

void             tcp_arg     (struct tcp_pcb *pcb, void *arg);

void             tcp_recv    (struct tcp_pcb *pcb, tcp_recv_fn recv);
void             tcp_sent    (struct tcp_pcb *pcb, tcp_sent_fn sent);
void             tcp_err     (struct tcp_pcb *pcb, tcp_err_fn err);
void             tcp_accept  (struct tcp_pcb *pcb, tcp_accept_fn accept);

void             tcp_poll    (struct tcp_pcb *pcb, tcp_poll_fn poll, u8_t interval);








 


 

 

 

 

 


#line 456 "..\\Lwip\\include\\lwip/tcp.h"

void             tcp_recved  (struct tcp_pcb *pcb, u16_t len);
err_t            tcp_bind    (struct tcp_pcb *pcb, const ip_addr_t *ipaddr,
                              u16_t port);
void             tcp_bind_netif(struct tcp_pcb *pcb, const struct netif *netif);
err_t            tcp_connect (struct tcp_pcb *pcb, const ip_addr_t *ipaddr,
                              u16_t port, tcp_connected_fn connected);

struct tcp_pcb * tcp_listen_with_backlog_and_err(struct tcp_pcb *pcb, u8_t backlog, err_t *err);
struct tcp_pcb * tcp_listen_with_backlog(struct tcp_pcb *pcb, u8_t backlog);
 


void             tcp_abort (struct tcp_pcb *pcb);
err_t            tcp_close   (struct tcp_pcb *pcb);
err_t            tcp_shutdown(struct tcp_pcb *pcb, int shut_rx, int shut_tx);

err_t            tcp_write   (struct tcp_pcb *pcb, const void *dataptr, u16_t len,
                              u8_t apiflags);

void             tcp_setprio (struct tcp_pcb *pcb, u8_t prio);

err_t            tcp_output  (struct tcp_pcb *pcb);

err_t            tcp_tcp_get_tcp_addrinfo(struct tcp_pcb *pcb, int local, ip_addr_t *addr, u16_t *port);



 


#line 493 "..\\Lwip\\include\\lwip/tcp.h"







#line 49 "..\\Lwip\\api\\api_msg.c"
#line 1 "..\\Lwip\\include\\lwip/raw.h"




 































 



#line 42 "..\\Lwip\\include\\lwip/raw.h"

#line 142 "..\\Lwip\\include\\lwip/raw.h"

#line 50 "..\\Lwip\\api\\api_msg.c"

#line 52 "..\\Lwip\\api\\api_msg.c"
#line 53 "..\\Lwip\\api\\api_msg.c"
#line 1 "..\\Lwip\\include\\lwip/dns.h"



 
































 




#line 43 "..\\Lwip\\include\\lwip/dns.h"

#line 130 "..\\Lwip\\include\\lwip/dns.h"

#line 54 "..\\Lwip\\api\\api_msg.c"
#line 1 "..\\Lwip\\include\\lwip/mld6.h"





 


































 




#line 47 "..\\Lwip\\include\\lwip/mld6.h"

#line 98 "..\\Lwip\\include\\lwip/mld6.h"

#line 55 "..\\Lwip\\api\\api_msg.c"
#line 56 "..\\Lwip\\api\\api_msg.c"

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



 

#line 58 "..\\Lwip\\api\\api_msg.c"

 














 
#line 83 "..\\Lwip\\api\\api_msg.c"
static err_t lwip_netconn_do_writemore(struct netconn *conn  , u8_t delayed);
static err_t lwip_netconn_do_close_internal(struct netconn *conn  , u8_t delayed);


static void netconn_drain(struct netconn *conn);







#line 107 "..\\Lwip\\api\\api_msg.c"


const u8_t netconn_aborted = 0;
const u8_t netconn_reset = 0;
const u8_t netconn_closed = 0;

 
static void *
lwip_netconn_err_to_msg(err_t err)
{
  switch (err) {
    case ERR_ABRT:
      return ((void *)((ptrdiff_t)&netconn_aborted));
    case ERR_RST:
      return ((void *)((ptrdiff_t)&netconn_reset));
    case ERR_CLSD:
      return ((void *)((ptrdiff_t)&netconn_closed));
    default:
      do { if (!(err == ERR_OK)) { do {printf("unhandled error");}while(0); }} while(0);
      return 0;
  }
}

int
lwip_netconn_is_err_msg(void *msg, err_t *err)
{
  do { if (!(err != 0)) { do {printf("err != NULL");}while(0); }} while(0);

  if (msg == &netconn_aborted) {
    *err = ERR_ABRT;
    return 1;
  } else if (msg == &netconn_reset) {
    *err = ERR_RST;
    return 1;
  } else if (msg == &netconn_closed) {
    *err = ERR_CLSD;
    return 1;
  }
  return 0;
}



#line 209 "..\\Lwip\\api\\api_msg.c"







 
static void
recv_udp(void *arg, struct udp_pcb *pcb, struct pbuf *p,
         const ip_addr_t *addr, u16_t port)
{
  struct netbuf *buf;
  struct netconn *conn;
  u16_t len;




  (void)pcb;  
  do { if (!(pcb != 0)) { do {printf("recv_udp must have a pcb argument");}while(0); }} while(0);
  do { if (!(arg != 0)) { do {printf("recv_udp must have an argument");}while(0); }} while(0);
  conn = (struct netconn *)arg;

  if (conn == 0) {
    pbuf_free(p);
    return;
  }

  do { if (!(conn->pcb . udp == pcb)) { do {printf("recv_udp: recv for wrong pcb!");}while(0); }} while(0);






  if (!sys_mbox_valid(&conn->recvmbox)) {

    pbuf_free(p);
    return;
  }

  buf = (struct netbuf *)memp_malloc(MEMP_NETBUF);
  if (buf == 0) {
    pbuf_free(p);
    return;
  } else {
    buf->p = p;
    buf->ptr = p;
    ((&buf->addr)->addr = ((addr) == 0 ? 0 : (addr)->addr));
    buf->port = port;
#line 269 "..\\Lwip\\api\\api_msg.c"
  }

  len = p->tot_len;
  if (sys_mbox_trypost(&conn->recvmbox, buf) != ERR_OK) {
    netbuf_delete(buf);
    return;
  } else {



     
    if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_RCVPLUS, len); };
  }
}








 
static err_t
recv_tcp(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  struct netconn *conn;
  u16_t len;
  void *msg;

  (void)pcb;
  do { if (!(pcb != 0)) { do {printf("recv_tcp must have a pcb argument");}while(0); }} while(0);
  do { if (!(arg != 0)) { do {printf("recv_tcp must have an argument");}while(0); }} while(0);
  do { if (!(err == ERR_OK)) { do {printf("err != ERR_OK unhandled");}while(0); }} while(0);
  (void)err;  
  conn = (struct netconn *)arg;

  if (conn == 0) {
    return ERR_VAL;
  }
  do { if (!(conn->pcb . tcp == pcb)) { do {printf("recv_tcp: recv for wrong pcb!");}while(0); }} while(0);

  if (!sys_mbox_valid(&conn->recvmbox)) {
     
    if (p != 0) {
      tcp_recved(pcb, p->tot_len);
      pbuf_free(p);
    }
    return ERR_OK;
  }
  

 

  if (p != 0) {
    msg = p;
    len = p->tot_len;
  } else {
    msg = ((void *)((ptrdiff_t)&netconn_closed));
    len = 0;
  }

  if (sys_mbox_trypost(&conn->recvmbox, msg) != ERR_OK) {
     
    return ERR_MEM;
  } else {



     
    if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_RCVPLUS, len); };
  }

  return ERR_OK;
}











 
static err_t
poll_tcp(void *arg, struct tcp_pcb *pcb)
{
  struct netconn *conn = (struct netconn *)arg;

  (void)pcb;
  do { if (!((conn != 0))) { do {printf("conn != NULL");}while(0); }} while(0);

  if (conn->state == NETCONN_WRITE) {
    lwip_netconn_do_writemore(conn  , 1);
  } else if (conn->state == NETCONN_CLOSE) {

    if (conn->current_msg && conn->current_msg->msg.sd.polls_left) {
      conn->current_msg->msg.sd.polls_left--;
    }

    lwip_netconn_do_close_internal(conn  , 1);
  }
   

   
  if (conn->flags & 0x10) {
    
 
    if ((conn->pcb.tcp != 0) && ((((conn->pcb . tcp)->snd_buf)) > (((((((((13*(1500 - 40)))/2)) > ((2 * (1500 - 40)) + 1)) ? ((((13*(1500 - 40)))/2)) : ((2 * (1500 - 40)) + 1))) < (((13*(1500 - 40))) - 1)) ? (((((((13*(1500 - 40)))/2)) > ((2 * (1500 - 40)) + 1)) ? ((((13*(1500 - 40)))/2)) : ((2 * (1500 - 40)) + 1))) : (((13*(1500 - 40))) - 1))) &&
        (((conn->pcb . tcp)->snd_queuelen) < ((((((8* (13*(1500 - 40))/(1500 - 40)))/2)) > (5)) ? ((((8* (13*(1500 - 40))/(1500 - 40)))/2)) : (5)))) {
      do { (conn)->flags = (u8_t)((conn)->flags & (u8_t)(~(0x10) & 0xff)); } while(0);
      if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_SENDPLUS, 0); };
    }
  }

  return ERR_OK;
}







 
static err_t
sent_tcp(void *arg, struct tcp_pcb *pcb, u16_t len)
{
  struct netconn *conn = (struct netconn *)arg;

  (void)pcb;
  do { if (!((conn != 0))) { do {printf("conn != NULL");}while(0); }} while(0);

  if (conn) {
    if (conn->state == NETCONN_WRITE) {
      lwip_netconn_do_writemore(conn  , 1);
    } else if (conn->state == NETCONN_CLOSE) {
      lwip_netconn_do_close_internal(conn  , 1);
    }

    
 
    if ((conn->pcb.tcp != 0) && ((((conn->pcb . tcp)->snd_buf)) > (((((((((13*(1500 - 40)))/2)) > ((2 * (1500 - 40)) + 1)) ? ((((13*(1500 - 40)))/2)) : ((2 * (1500 - 40)) + 1))) < (((13*(1500 - 40))) - 1)) ? (((((((13*(1500 - 40)))/2)) > ((2 * (1500 - 40)) + 1)) ? ((((13*(1500 - 40)))/2)) : ((2 * (1500 - 40)) + 1))) : (((13*(1500 - 40))) - 1))) &&
        (((conn->pcb . tcp)->snd_queuelen) < ((((((8* (13*(1500 - 40))/(1500 - 40)))/2)) > (5)) ? ((((8* (13*(1500 - 40))/(1500 - 40)))/2)) : (5)))) {
      do { (conn)->flags = (u8_t)((conn)->flags & (u8_t)(~(0x10) & 0xff)); } while(0);
      if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_SENDPLUS, len); };
    }
  }

  return ERR_OK;
}







 
static void
err_tcp(void *arg, err_t err)
{
  struct netconn *conn;
  enum netconn_state old_state;
  void *mbox_msg;
  sys_prot_t lev;

  conn = (struct netconn *)arg;
  do { if (!((conn != 0))) { do {printf("conn != NULL");}while(0); }} while(0);

  lev = sys_arch_protect();

   
  conn->pcb.tcp = 0;
   
  conn->pending_err = err;
   
  conn->flags |= 0x01;

   
  old_state = conn->state;
  conn->state = NETCONN_NONE;

  sys_arch_unprotect(lev);

   
  if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_ERROR, 0); };
  
 
  if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_RCVPLUS, 0); };
  if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_SENDPLUS, 0); };

  mbox_msg = lwip_netconn_err_to_msg(err);
   
  if (sys_mbox_valid(&conn->recvmbox)) {
     
    sys_mbox_trypost(&conn->recvmbox, mbox_msg);
  }
   
  if (sys_mbox_valid(&conn->acceptmbox)) {
     
    sys_mbox_trypost(&conn->acceptmbox, mbox_msg);
  }

  if ((old_state == NETCONN_WRITE) || (old_state == NETCONN_CLOSE) ||
      (old_state == NETCONN_CONNECT)) {
    
 
    int was_nonblocking_connect = (((conn)->flags & (0x04)) != 0);
    do { if (0) { do { (conn)->flags = (u8_t)((conn)->flags | (0x04)); } while(0); } else { do { (conn)->flags = (u8_t)((conn)->flags & (u8_t)(~(0x04) & 0xff)); } while(0); }} while(0);

    if (!was_nonblocking_connect) {
      sys_sem_t *op_completed_sem;
       
      do { if (!(conn->current_msg != 0)) { do {printf("conn->current_msg != NULL");}while(0); }} while(0);
      if (old_state == NETCONN_CLOSE) {
         
        conn->current_msg->err = ERR_OK;
      } else {
         
        conn->current_msg->err = err;
      }
      op_completed_sem = (&(conn->current_msg)->conn->op_completed);
      do { if (!(sys_sem_valid(op_completed_sem))) { do {printf("inavlid op_completed_sem");}while(0); }} while(0);
      conn->current_msg = 0;
       
      sys_sem_signal(op_completed_sem);
    } else {
       
    }
  } else {
    do { if (!(conn->current_msg == 0)) { do {printf("conn->current_msg == NULL");}while(0); }} while(0);
  }
}






 
static void
setup_tcp(struct netconn *conn)
{
  struct tcp_pcb *pcb;

  pcb = conn->pcb.tcp;
  tcp_arg(pcb, conn);
  tcp_recv(pcb, recv_tcp);
  tcp_sent(pcb, sent_tcp);
  tcp_poll(pcb, poll_tcp, 2);
  tcp_err(pcb, err_tcp);
}






 
static err_t
accept_function(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  struct netconn *newconn;
  struct netconn *conn = (struct netconn *)arg;

  if (conn == 0) {
    return ERR_VAL;
  }
  if (!sys_mbox_valid(&conn->acceptmbox)) {
    ;
    return ERR_VAL;
  }

  if (newpcb == 0) {
     
    if (sys_mbox_trypost(&conn->acceptmbox, lwip_netconn_err_to_msg(ERR_ABRT)) == ERR_OK) {
       
      if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_RCVPLUS, 0); };
    }
    return ERR_VAL;
  }
  do { if (!(err == ERR_OK)) { do {printf("expect newpcb == NULL or err == ERR_OK");}while(0); }} while(0);
  (void)err;  

  ;

  
 
  newconn = netconn_alloc(conn->type, conn->callback);
  if (newconn == 0) {
     
    if (sys_mbox_trypost(&conn->acceptmbox, lwip_netconn_err_to_msg(ERR_ABRT)) == ERR_OK) {
       
      if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_RCVPLUS, 0); };
    }
    return ERR_MEM;
  }
  newconn->pcb.tcp = newpcb;
  setup_tcp(newconn);

   
  ;

  if (sys_mbox_trypost(&conn->acceptmbox, newconn) != ERR_OK) {
    
 
     
    struct tcp_pcb *pcb = newconn->pcb.tcp;
    tcp_arg(pcb, 0);
    tcp_recv(pcb, 0);
    tcp_sent(pcb, 0);
    tcp_poll(pcb, 0, 0);
    tcp_err(pcb, 0);
     
    newconn->pcb.tcp = 0;
     
    sys_mbox_free(&newconn->recvmbox);
    sys_mbox_set_invalid(&newconn->recvmbox);
    netconn_free(newconn);
    return ERR_MEM;
  } else {
     
    if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_RCVPLUS, 0); };
  }

  return ERR_OK;
}







 
static void
pcb_new(struct api_msg *msg)
{
  enum lwip_ip_addr_type iptype = IPADDR_TYPE_V4;

  do { if (!(msg->conn->pcb . tcp == 0)) { do {printf("pcb_new: pcb already allocated");}while(0); }} while(0);

#line 621 "..\\Lwip\\api\\api_msg.c"

   
  switch (((msg->conn->type)&0xF0)) {
#line 640 "..\\Lwip\\api\\api_msg.c"
    case NETCONN_UDP:
      msg->conn->pcb.udp = udp_new_ip_type(iptype);
      if (msg->conn->pcb.udp != 0) {





        if (((msg->conn->type) == NETCONN_UDPNOCHKSUM)) {
          ((msg->conn->pcb . udp)->flags = (0x01U));
        }
        udp_recv(msg->conn->pcb.udp, recv_udp, msg->conn);
      }
      break;


    case NETCONN_TCP:
      msg->conn->pcb.tcp = tcp_new_ip_type(iptype);
      if (msg->conn->pcb.tcp != 0) {
        setup_tcp(msg->conn);
      }
      break;

    default:
       
      msg->err = ERR_VAL;
      return;
  }
  if (msg->conn->pcb.ip == 0) {
    msg->err = ERR_MEM;
  }
}






 
void
lwip_netconn_do_newconn(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;

  msg->err = ERR_OK;
  if (msg->conn->pcb.tcp == 0) {
    pcb_new(msg);
  }
   
   
   

  ;
}









 
struct netconn *
netconn_alloc(enum netconn_type t, netconn_callback callback)
{
  struct netconn *conn;
  int size;
  u8_t init_flags = 0;

  conn = (struct netconn *)memp_malloc(MEMP_NETCONN);
  if (conn == 0) {
    return 0;
  }

  conn->pending_err = ERR_OK;
  conn->type = t;
  conn->pcb.tcp = 0;

   
  switch (((t)&0xF0)) {
#line 728 "..\\Lwip\\api\\api_msg.c"
    case NETCONN_UDP:
      size = 10;



      break;


    case NETCONN_TCP:
      size = 10;
      break;

    default:
      do { if (!(0)) { do {printf("netconn_alloc: undefined netconn_type");}while(0); }} while(0);
      goto free_and_return;
  }

  if (sys_mbox_new(&conn->recvmbox, size) != ERR_OK) {
    goto free_and_return;
  }

  if (sys_sem_new(&conn->op_completed, 0) != ERR_OK) {
    sys_mbox_free(&conn->recvmbox);
    goto free_and_return;
  }



  sys_mbox_set_invalid(&conn->acceptmbox);

  conn->state        = NETCONN_NONE;

   
  conn->socket       = -1;

  conn->callback     = callback;

  conn->current_msg  = 0;
#line 780 "..\\Lwip\\api\\api_msg.c"
  conn->flags = init_flags;
  return conn;
free_and_return:
  memp_free(MEMP_NETCONN, conn);
  return 0;
}






 
void
netconn_free(struct netconn *conn)
{
  do { if (!(conn->pcb . tcp == 0)) { do {printf("PCB must be deallocated outside this function");}while(0); }} while(0);






  do { if (!(!sys_mbox_valid(&conn->recvmbox))) { do {printf("recvmbox must be deallocated before calling this function");}while(0); }} while(0);


  do { if (!(!sys_mbox_valid(&conn->acceptmbox))) { do {printf("acceptmbox must be deallocated before calling this function");}while(0); }} while(0);




  sys_sem_free(&conn->op_completed);
  sys_sem_set_invalid(&conn->op_completed);


  memp_free(MEMP_NETCONN, conn);
}








 
static void
netconn_drain(struct netconn *conn)
{
  void *mem;

  
 




   
  if (sys_mbox_valid(&conn->recvmbox)) {
    while (sys_arch_mbox_tryfetch(&conn->recvmbox, &mem) != 0xffffffffUL) {



      {

        if (((conn->type)&0xF0) == NETCONN_TCP) {
          err_t err;
          if (!lwip_netconn_is_err_msg(mem, &err)) {
            pbuf_free((struct pbuf *)mem);
          }
        } else

        {
          netbuf_delete((struct netbuf *)mem);
        }
      }
    }
    sys_mbox_free(&conn->recvmbox);
    sys_mbox_set_invalid(&conn->recvmbox);
  }

   

  if (sys_mbox_valid(&conn->acceptmbox)) {
    while (sys_arch_mbox_tryfetch(&conn->acceptmbox, &mem) != 0xffffffffUL) {



      {
        err_t err;
        if (!lwip_netconn_is_err_msg(mem, &err)) {
          struct netconn *newconn = (struct netconn *)mem;
           
           
           
          netconn_drain(newconn);
          if (newconn->pcb.tcp != 0) {
            tcp_abort(newconn->pcb.tcp);
            newconn->pcb.tcp = 0;
          }
          netconn_free(newconn);
        }
      }
    }
    sys_mbox_free(&conn->acceptmbox);
    sys_mbox_set_invalid(&conn->acceptmbox);
  }

}

#line 910 "..\\Lwip\\api\\api_msg.c"








 
static err_t
lwip_netconn_do_close_internal(struct netconn *conn  , u8_t delayed)
{
  err_t err;
  u8_t shut, shut_rx, shut_tx, shut_close;
  u8_t close_finished = 0;
  struct tcp_pcb *tpcb;




  do { if (!((conn != 0))) { do {printf("invalid conn");}while(0); }} while(0);
  do { if (!((((conn->type)&0xF0) == NETCONN_TCP))) { do {printf("this is for tcp netconns only");}while(0); }} while(0);
  do { if (!((conn->state == NETCONN_CLOSE))) { do {printf("conn must be in state NETCONN_CLOSE");}while(0); }} while(0);
  do { if (!((conn->pcb . tcp != 0))) { do {printf("pcb already closed");}while(0); }} while(0);
  do { if (!(conn->current_msg != 0)) { do {printf("conn->current_msg != NULL");}while(0); }} while(0);

  tpcb = conn->pcb.tcp;
  shut = conn->current_msg->msg.sd.shut;
  shut_rx = shut & 1;
  shut_tx = shut & 2;
  
 
  if (shut == (1 | 2)) {
    shut_close = 1;
  } else if (shut_rx &&
             ((tpcb->state == FIN_WAIT_1) ||
              (tpcb->state == FIN_WAIT_2) ||
              (tpcb->state == CLOSING))) {
    shut_close = 1;
  } else if (shut_tx && ((tpcb->flags & 0x10U) != 0)) {
    shut_close = 1;
  } else {
    shut_close = 0;
  }

   
  if (shut_close) {
    tcp_arg(tpcb, 0);
  }
  if (tpcb->state == LISTEN) {
    tcp_accept(tpcb, 0);
  } else {
     
    if (shut_rx) {
      tcp_recv(tpcb, 0);
      tcp_accept(tpcb, 0);
    }
    if (shut_tx) {
      tcp_sent(tpcb, 0);
    }
    if (shut_close) {
      tcp_poll(tpcb, 0, 0);
      tcp_err(tpcb, 0);
    }
  }
   
  if (shut_close) {
#line 1005 "..\\Lwip\\api\\api_msg.c"
    {
      err = tcp_close(tpcb);
    }
  } else {
    err = tcp_shutdown(tpcb, shut_rx, shut_tx);
  }
  if (err == ERR_OK) {
    close_finished = 1;
#line 1020 "..\\Lwip\\api\\api_msg.c"
  } else {
    if (err == ERR_MEM) {
      



 
#line 1042 "..\\Lwip\\api\\api_msg.c"
      if (conn->current_msg->msg.sd.polls_left == 0) {

        close_finished = 1;
        if (shut_close) {
           
          tcp_abort(tpcb);
          err = ERR_OK;
        }
      }
    } else {
       
      close_finished = 1;
    }
  }
  if (close_finished) {
     
    sys_sem_t *op_completed_sem = (&(conn->current_msg)->conn->op_completed);
    conn->current_msg->err = err;
    conn->current_msg = 0;
    conn->state = NETCONN_NONE;
    if (err == ERR_OK) {
      if (shut_close) {
         
        conn->pcb.tcp = 0;
        
 
        if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_ERROR, 0); };
      }
      if (shut_rx) {
        if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_RCVPLUS, 0); };
      }
      if (shut_tx) {
        if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_SENDPLUS, 0); };
      }
    }

    if (delayed)

    {
       
      sys_sem_signal(op_completed_sem);
    }
    return ERR_OK;
  }
  if (!close_finished) {
     
     
    do { if (!((tpcb->state != LISTEN))) { do {printf("Closing a listen pcb may not fail!");}while(0); }} while(0);
    if (shut_tx) {
      tcp_sent(tpcb, sent_tcp);
    }
     
    tcp_poll(tpcb, poll_tcp, 1);
    tcp_err(tpcb, err_tcp);
    tcp_arg(tpcb, conn);
     
  }
  
 
  do { if (!(err != ERR_OK)) { do {printf("err != ERR_OK");}while(0); }} while(0);
  return err;
}







 
void
lwip_netconn_do_delconn(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;

  enum netconn_state state = msg->conn->state;
  do { if (!((state == NETCONN_NONE) || (((msg->conn->type)&0xF0) == NETCONN_TCP))) { do {printf("netconn state error");}while(0); }} while(0);
#line 1136 "..\\Lwip\\api\\api_msg.c"
  if (((state != NETCONN_NONE) &&
       (state != NETCONN_LISTEN) &&
       (state != NETCONN_CONNECT)) ||
      ((state == NETCONN_CONNECT) && !(((msg->conn)->flags & (0x04)) != 0))) {
    
 
    msg->err = ERR_INPROGRESS;
  } else

  {
    do { if (!((state != NETCONN_CONNECT) || (((msg->conn)->flags & (0x04)) != 0))) { do {printf("blocking connect in progress");}while(0); }} while(0);

    msg->err = ERR_OK;




    netconn_drain(msg->conn);


    if (msg->conn->pcb.tcp != 0) {

      switch (((msg->conn->type)&0xF0)) {
#line 1165 "..\\Lwip\\api\\api_msg.c"
        case NETCONN_UDP:
          msg->conn->pcb.udp->recv_arg = 0;
          udp_remove(msg->conn->pcb.udp);
          break;


        case NETCONN_TCP:
          do { if (!(msg->conn->current_msg == 0)) { do {printf("already writing or closing");}while(0); }} while(0);
          msg->conn->state = NETCONN_CLOSE;
          msg->msg.sd.shut = (1 | 2);
          msg->conn->current_msg = msg;

          if (lwip_netconn_do_close_internal(msg->conn, 0) != ERR_OK) {
            do { if (!(msg->conn->state == NETCONN_CLOSE)) { do {printf("state!");}while(0); }} while(0);
            sys_mutex_unlock(&lock_tcpip_core);
            sys_arch_sem_wait((&(msg)->conn->op_completed), 0);
            sys_mutex_lock(&lock_tcpip_core);
            do { if (!(msg->conn->state == NETCONN_NONE)) { do {printf("state!");}while(0); }} while(0);
          }



          
 
          return;

        default:
          break;
      }
      msg->conn->pcb.tcp = 0;
    }
     

    
 
    if (msg->conn->callback) { (*msg->conn->callback)(msg->conn, NETCONN_EVT_RCVPLUS, 0); };
    if (msg->conn->callback) { (*msg->conn->callback)(msg->conn, NETCONN_EVT_SENDPLUS, 0); };
  }
  if (sys_sem_valid((&(msg)->conn->op_completed))) {
    ;
  }
}







 
void
lwip_netconn_do_bind(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;
  err_t err;

  if (msg->conn->pcb.tcp != 0) {
    switch (((msg->conn->type)&0xF0)) {
#line 1229 "..\\Lwip\\api\\api_msg.c"
      case NETCONN_UDP:
        err = udp_bind(msg->conn->pcb.udp, msg->msg . bc . ipaddr, msg->msg.bc.port);
        break;


      case NETCONN_TCP:
        err = tcp_bind(msg->conn->pcb.tcp, msg->msg . bc . ipaddr, msg->msg.bc.port);
        break;

      default:
        err = ERR_VAL;
        break;
    }
  } else {
    err = ERR_VAL;
  }
  msg->err = err;
  ;
}






 
void
lwip_netconn_do_bind_if(void *m)
{
  struct netif *netif;
  struct api_msg *msg = (struct api_msg *)m;
  err_t err;

  netif = netif_get_by_index(msg->msg.bc.if_idx);

  if ((netif != 0) && (msg->conn->pcb.tcp != 0)) {
    err = ERR_OK;
    switch (((msg->conn->type)&0xF0)) {
#line 1273 "..\\Lwip\\api\\api_msg.c"
      case NETCONN_UDP:
        udp_bind_netif(msg->conn->pcb.udp, netif);
        break;


      case NETCONN_TCP:
        tcp_bind_netif(msg->conn->pcb.tcp, netif);
        break;

      default:
        err = ERR_VAL;
        break;
    }
  } else {
    err = ERR_VAL;
  }
  msg->err = err;
  ;
}







 
static err_t
lwip_netconn_do_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
  struct netconn *conn;
  int was_blocking;
  sys_sem_t *op_completed_sem = 0;

  (void)pcb;

  conn = (struct netconn *)arg;

  if (conn == 0) {
    return ERR_VAL;
  }

  do { if (!(conn->state == NETCONN_CONNECT)) { do {printf("conn->state == NETCONN_CONNECT");}while(0); }} while(0);
  do { if (!((conn->current_msg != 0) || (((conn)->flags & (0x04)) != 0))) { do {printf("(conn->current_msg != NULL) || conn->in_non_blocking_connect");}while(0); }} while(0);


  if (conn->current_msg != 0) {
    conn->current_msg->err = err;
    op_completed_sem = (&(conn->current_msg)->conn->op_completed);
  }
  if ((((conn->type)&0xF0) == NETCONN_TCP) && (err == ERR_OK)) {
    setup_tcp(conn);
  }
  was_blocking = !(((conn)->flags & (0x04)) != 0);
  do { if (0) { do { (conn)->flags = (u8_t)((conn)->flags | (0x04)); } while(0); } else { do { (conn)->flags = (u8_t)((conn)->flags & (u8_t)(~(0x04) & 0xff)); } while(0); }} while(0);
  do { if (!((was_blocking && op_completed_sem != 0) || (!was_blocking && op_completed_sem == 0))) { do {printf("blocking connect state error");}while(0); }} while(0);


  conn->current_msg = 0;
  conn->state = NETCONN_NONE;
  if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_SENDPLUS, 0); };

  if (was_blocking) {
    sys_sem_signal(op_completed_sem);
  }
  return ERR_OK;
}








 
void
lwip_netconn_do_connect(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;
  err_t err;

  if (msg->conn->pcb.tcp == 0) {
     
    err = ERR_CLSD;
  } else {
    switch (((msg->conn->type)&0xF0)) {
#line 1366 "..\\Lwip\\api\\api_msg.c"
      case NETCONN_UDP:
        err = udp_connect(msg->conn->pcb.udp, msg->msg . bc . ipaddr, msg->msg.bc.port);
        break;


      case NETCONN_TCP:
         
        if (msg->conn->state == NETCONN_CONNECT) {
          err = ERR_ALREADY;
        } else if (msg->conn->state != NETCONN_NONE) {
          err = ERR_ISCONN;
        } else {
          setup_tcp(msg->conn);
          err = tcp_connect(msg->conn->pcb.tcp, msg->msg . bc . ipaddr,
                            msg->msg.bc.port, lwip_netconn_do_connected);
          if (err == ERR_OK) {
            u8_t non_blocking = (((msg->conn)->flags & 0x02) != 0);
            msg->conn->state = NETCONN_CONNECT;
            do { if (non_blocking) { do { (msg->conn)->flags = (u8_t)((msg->conn)->flags | (0x04)); } while(0); } else { do { (msg->conn)->flags = (u8_t)((msg->conn)->flags & (u8_t)(~(0x04) & 0xff)); } while(0); }} while(0);
            if (non_blocking) {
              err = ERR_INPROGRESS;
            } else {
              msg->conn->current_msg = msg;
              
 

              do { if (!(msg->conn->state == NETCONN_CONNECT)) { do {printf("state!");}while(0); }} while(0);
              sys_mutex_unlock(&lock_tcpip_core);
              sys_arch_sem_wait((&(msg)->conn->op_completed), 0);
              sys_mutex_lock(&lock_tcpip_core);
              do { if (!(msg->conn->state != NETCONN_CONNECT)) { do {printf("state!");}while(0); }} while(0);

              return;
            }
          }
        }
        break;

      default:
        do { if (!(0)) { do {printf("Invalid netconn type");}while(0); do { err = ERR_VAL; } while (0);}} while(0);


        break;
    }
  }
  msg->err = err;
  
 
  ;
}







 
void
lwip_netconn_do_disconnect(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;


  if (((msg->conn->type)&0xF0) == NETCONN_UDP) {
    udp_disconnect(msg->conn->pcb.udp);
    msg->err = ERR_OK;
  } else

  {
    msg->err = ERR_VAL;
  }
  ;
}







 
void
lwip_netconn_do_listen(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;
  err_t err;

  if (msg->conn->pcb.tcp != 0) {
    if (((msg->conn->type)&0xF0) == NETCONN_TCP) {
      if (msg->conn->state == NETCONN_NONE) {
        struct tcp_pcb *lpcb;
        if (msg->conn->pcb.tcp->state != CLOSED) {
           
          err = ERR_VAL;
        } else {
          u8_t backlog;



          backlog = 0xff;
#line 1479 "..\\Lwip\\api\\api_msg.c"

          lpcb = tcp_listen_with_backlog_and_err(msg->conn->pcb.tcp, backlog, &err);

          if (lpcb == 0) {
             
          } else {
             
            if (sys_mbox_valid(&msg->conn->recvmbox)) {
               
              sys_mbox_free(&msg->conn->recvmbox);
              sys_mbox_set_invalid(&msg->conn->recvmbox);
            }
            err = ERR_OK;
            if (!sys_mbox_valid(&msg->conn->acceptmbox)) {
              err = sys_mbox_new(&msg->conn->acceptmbox, 10);
            }
            if (err == ERR_OK) {
              msg->conn->state = NETCONN_LISTEN;
              msg->conn->pcb.tcp = lpcb;
              tcp_arg(msg->conn->pcb.tcp, msg->conn);
              tcp_accept(msg->conn->pcb.tcp, accept_function);
            } else {
               
              tcp_close(lpcb);
              msg->conn->pcb.tcp = 0;
            }
          }
        }
      } else if (msg->conn->state == NETCONN_LISTEN) {
         
        err = ERR_OK;
        ;
      } else {
        err = ERR_CONN;
      }
    } else {
      err = ERR_ARG;
    }
  } else {
    err = ERR_CONN;
  }
  msg->err = err;
  ;
}







 
void
lwip_netconn_do_send(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;

  err_t err = netconn_err(msg->conn);
  if (err == ERR_OK) {
    if (msg->conn->pcb.tcp != 0) {
      switch (((msg->conn->type)&0xF0)) {
#line 1550 "..\\Lwip\\api\\api_msg.c"
        case NETCONN_UDP:
#line 1561 "..\\Lwip\\api\\api_msg.c"
          if (((msg->msg . b->addr). addr == ((u32_t)0x00000000UL)) || 0) {
            err = udp_send(msg->conn->pcb.udp, msg->msg.b->p);
          } else {
            err = udp_sendto(msg->conn->pcb.udp, msg->msg.b->p, &msg->msg.b->addr, msg->msg.b->port);
          }

          break;

        default:
          err = ERR_CONN;
          break;
      }
    } else {
      err = ERR_CONN;
    }
  }
  msg->err = err;
  ;
}







 
void
lwip_netconn_do_recv(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;

  msg->err = ERR_OK;
  if (msg->conn->pcb.tcp != 0) {
    if (((msg->conn->type)&0xF0) == NETCONN_TCP) {
      size_t remaining = msg->msg.r.len;
      do {
        u16_t recved = (u16_t)((remaining > 0xffff) ? 0xffff : remaining);
        tcp_recved(msg->conn->pcb.tcp, recved);
        remaining -= recved;
      } while (remaining != 0);
    }
  }
  ;
}

#line 1627 "..\\Lwip\\api\\api_msg.c"











 
static err_t
lwip_netconn_do_writemore(struct netconn *conn  , u8_t delayed)
{
  err_t err;
  const void *dataptr;
  u16_t len, available;
  u8_t write_finished = 0;
  size_t diff;
  u8_t dontblock;
  u8_t apiflags;
  u8_t write_more;

  do { if (!(conn != 0)) { do {printf("conn != NULL");}while(0); }} while(0);
  do { if (!((conn->state == NETCONN_WRITE))) { do {printf("conn->state == NETCONN_WRITE");}while(0); }} while(0);
  do { if (!(conn->current_msg != 0)) { do {printf("conn->current_msg != NULL");}while(0); }} while(0);
  do { if (!(conn->pcb . tcp != 0)) { do {printf("conn->pcb.tcp != NULL");}while(0); }} while(0);
  do { if (!(conn->current_msg->msg . w . offset < conn->current_msg->msg . w . len)) { do {printf("conn->current_msg->msg.w.offset < conn->current_msg->msg.w.len");}while(0); }} while(0);

  do { if (!(conn->current_msg->msg . w . vector_cnt > 0)) { do {printf("conn->current_msg->msg.w.vector_cnt > 0");}while(0); }} while(0);

  apiflags = conn->current_msg->msg.w.apiflags;
  dontblock = (((conn)->flags & 0x02) != 0) || (apiflags & 0x04);

#line 1675 "..\\Lwip\\api\\api_msg.c"
  {
    do {
      dataptr = (const u8_t *)conn->current_msg->msg.w.vector->ptr + conn->current_msg->msg.w.vector_off;
      diff = conn->current_msg->msg.w.vector->len - conn->current_msg->msg.w.vector_off;
      if (diff > 0xffffUL) {  
        len = 0xffff;
        apiflags |= 0x02;
      } else {
        len = (u16_t)diff;
      }
      available = (((conn->pcb . tcp)->snd_buf));
      if (available < len) {
         
        len = available;
        if (dontblock) {
          if (!len) {
             
            err = (conn->current_msg->msg.w.offset == 0) ? ERR_WOULDBLOCK : ERR_OK;
            goto err_mem;
          }
        } else {
          apiflags |= 0x02;
        }
      }
      do { if (!(((conn->current_msg->msg . w . vector_off + len) <= conn->current_msg->msg . w . vector->len))) { do {printf("lwip_netconn_do_writemore: invalid length!");}while(0); }} while(0);

      


 
      if ((len == 0xffff && diff > 0xffffUL) ||
          (len == (u16_t)diff && conn->current_msg->msg.w.vector_cnt > 1)) {
        write_more = 1;
        apiflags |= 0x02;
      } else {
        write_more = 0;
      }
      err = tcp_write(conn->pcb.tcp, dataptr, len, apiflags);
      if (err == ERR_OK) {
        conn->current_msg->msg.w.offset += len;
        conn->current_msg->msg.w.vector_off += len;
         
        if (conn->current_msg->msg.w.vector_off == conn->current_msg->msg.w.vector->len) {
          conn->current_msg->msg.w.vector_cnt--;
           
          if (conn->current_msg->msg.w.vector_cnt > 0) {
            conn->current_msg->msg.w.vector++;
            conn->current_msg->msg.w.vector_off = 0;
          }
        }
      }
    } while (write_more && err == ERR_OK);
     
    if ((err == ERR_OK) || (err == ERR_MEM)) {
err_mem:
      if (dontblock && (conn->current_msg->msg.w.offset < conn->current_msg->msg.w.len)) {
        
 
        if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_SENDMINUS, 0); };
        conn->flags |= 0x10;
      } else if (((((conn->pcb . tcp)->snd_buf)) <= (((((((((13*(1500 - 40)))/2)) > ((2 * (1500 - 40)) + 1)) ? ((((13*(1500 - 40)))/2)) : ((2 * (1500 - 40)) + 1))) < (((13*(1500 - 40))) - 1)) ? (((((((13*(1500 - 40)))/2)) > ((2 * (1500 - 40)) + 1)) ? ((((13*(1500 - 40)))/2)) : ((2 * (1500 - 40)) + 1))) : (((13*(1500 - 40))) - 1))) ||
                 (((conn->pcb . tcp)->snd_queuelen) >= ((((((8* (13*(1500 - 40))/(1500 - 40)))/2)) > (5)) ? ((((8* (13*(1500 - 40))/(1500 - 40)))/2)) : (5)))) {
        
 
        if (conn->callback) { (*conn->callback)(conn, NETCONN_EVT_SENDMINUS, 0); };
      }
    }

    if (err == ERR_OK) {
      err_t out_err;
      if ((conn->current_msg->msg.w.offset == conn->current_msg->msg.w.len) || dontblock) {
         
        write_finished = 1;
      }
      out_err = tcp_output(conn->pcb.tcp);
      if (out_err == ERR_RTE) {
        

 
        err = out_err;
        write_finished = 1;
      }
    } else if (err == ERR_MEM) {
      


 

       
      err_t out_err = tcp_output(conn->pcb.tcp);
      if (out_err == ERR_RTE) {
        

 
        err = out_err;
        write_finished = 1;
      } else if (dontblock) {
        
 
        err = (conn->current_msg->msg.w.offset == 0) ? ERR_WOULDBLOCK : ERR_OK;
        write_finished = 1;
      }
    } else {
      
 
      write_finished = 1;
    }
  }
  if (write_finished) {
    
 
    sys_sem_t *op_completed_sem = (&(conn->current_msg)->conn->op_completed);
    conn->current_msg->err = err;
    conn->current_msg = 0;
    conn->state = NETCONN_NONE;

    if (delayed)

    {
      sys_sem_signal(op_completed_sem);
    }
  }

  else {
    return ERR_MEM;
  }

  return ERR_OK;
}







 
void
lwip_netconn_do_write(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;

  err_t err = netconn_err(msg->conn);
  if (err == ERR_OK) {
    if (((msg->conn->type)&0xF0) == NETCONN_TCP) {

      if (msg->conn->state != NETCONN_NONE) {
         
        err = ERR_INPROGRESS;
      } else if (msg->conn->pcb.tcp != 0) {
        msg->conn->state = NETCONN_WRITE;
         
        do { if (!(msg->conn->current_msg == 0)) { do {printf("already writing or closing");}while(0); }} while(0);
        do { if (!(msg->msg . w . len != 0)) { do {printf("msg->msg.w.len != 0");}while(0); }} while(0);
        msg->conn->current_msg = msg;

        if (lwip_netconn_do_writemore(msg->conn, 0) != ERR_OK) {
          do { if (!(msg->conn->state == NETCONN_WRITE)) { do {printf("state!");}while(0); }} while(0);
          sys_mutex_unlock(&lock_tcpip_core);
          sys_arch_sem_wait((&(msg)->conn->op_completed), 0);
          sys_mutex_lock(&lock_tcpip_core);
          do { if (!(msg->conn->state != NETCONN_WRITE)) { do {printf("state!");}while(0); }} while(0);
        }



        
 
        return;
      } else {
        err = ERR_CONN;
      }




    } else {
      err = ERR_VAL;

    }
  }
  msg->err = err;
  ;
}






 
void
lwip_netconn_do_getaddr(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;

  if (msg->conn->pcb.ip != 0) {
    if (msg->msg.ad.local) {
      (((*(msg->msg . ad . ipaddr))). addr = (msg->conn->pcb . ip->local_ip). addr);

    } else {
      (((*(msg->msg . ad . ipaddr))). addr = (msg->conn->pcb . ip->remote_ip). addr);

    }

    msg->err = ERR_OK;
    switch (((msg->conn->type)&0xF0)) {
#line 1893 "..\\Lwip\\api\\api_msg.c"
      case NETCONN_UDP:
        if (msg->msg.ad.local) {
          (*(msg->msg . ad . port)) = msg->conn->pcb.udp->local_port;
        } else {
          if ((msg->conn->pcb.udp->flags & 0x04U) == 0) {
            msg->err = ERR_CONN;
          } else {
            (*(msg->msg . ad . port)) = msg->conn->pcb.udp->remote_port;
          }
        }
        break;


      case NETCONN_TCP:
        if ((msg->msg.ad.local == 0) &&
            ((msg->conn->pcb.tcp->state == CLOSED) || (msg->conn->pcb.tcp->state == LISTEN))) {
           
          msg->err = ERR_CONN;
        } else {
          (*(msg->msg . ad . port)) = (msg->msg.ad.local ? msg->conn->pcb.tcp->local_port : msg->conn->pcb.tcp->remote_port);
        }
        break;

      default:
        do { if (!(0)) { do {printf("invalid netconn_type");}while(0); }} while(0);
        break;
    }
  } else {
    msg->err = ERR_CONN;
  }
  ;
}







 
void
lwip_netconn_do_close(void *m)
{
  struct api_msg *msg = (struct api_msg *)m;


  enum netconn_state state = msg->conn->state;
  
 
  if ((msg->conn->pcb.tcp != 0) &&
      (((msg->conn->type)&0xF0) == NETCONN_TCP) &&
      ((msg->msg.sd.shut == (1 | 2)) || (state != NETCONN_LISTEN))) {
     
    if (state == NETCONN_CONNECT) {
       
      msg->err = ERR_CONN;
    } else if (state == NETCONN_WRITE) {
#line 1970 "..\\Lwip\\api\\api_msg.c"
      msg->err = ERR_INPROGRESS;
    } else {

      if (msg->msg.sd.shut & 1) {




        netconn_drain(msg->conn);

      }
      do { if (!(msg->conn->current_msg == 0)) { do {printf("already writing or closing");}while(0); }} while(0);
      msg->conn->state = NETCONN_CLOSE;
      msg->conn->current_msg = msg;

      if (lwip_netconn_do_close_internal(msg->conn, 0) != ERR_OK) {
        do { if (!(msg->conn->state == NETCONN_CLOSE)) { do {printf("state!");}while(0); }} while(0);
        sys_mutex_unlock(&lock_tcpip_core);
        sys_arch_sem_wait((&(msg)->conn->op_completed), 0);
        sys_mutex_lock(&lock_tcpip_core);
        do { if (!(msg->conn->state == NETCONN_NONE)) { do {printf("state!");}while(0); }} while(0);
      }



       
      return;
    }
  } else

  {
    msg->err = ERR_CONN;
  }
  ;
}

#line 2109 "..\\Lwip\\api\\api_msg.c"

#line 2172 "..\\Lwip\\api\\api_msg.c"

