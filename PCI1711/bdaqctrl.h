#if defined(__cplusplus) && !defined(_BDAQ_NO_NAMESPACE) && !defined(_BDAQ_C_INTERFACE)
#  ifndef BEGIN_NAMEAPCE_AUTOMATION_BDAQ
#     define BEGIN_NAMEAPCE_AUTOMATION_BDAQ namespace Automation { namespace BDaq {
#     define END_NAMEAPCE_AUTOMATION_BDAQ  } /*BDaq*/ } /*Automation*/
#  endif
#else
#  ifndef BEGIN_NAMEAPCE_AUTOMATION_BDAQ
#     define BEGIN_NAMEAPCE_AUTOMATION_BDAQ
#     define END_NAMEAPCE_AUTOMATION_BDAQ
#  endif
#endif

/**********************************************************/
/*Bionic DAQ types                                        */
/**********************************************************/
#ifndef _BDAQ_TYPES_DEFINED
#define _BDAQ_TYPES_DEFINED

BEGIN_NAMEAPCE_AUTOMATION_BDAQ

#define DAQ_NAVI_VER          0x400

#define MAX_DEVICE_DESC_LEN   64
#define MAX_VRG_DESC_LEN      256
#define MAX_SIG_DROP_DESC_LEN 256

#define MAX_AI_CH_COUNT       128
#define MAX_AO_CH_COUNT       128
#define MAX_DIO_PORT_COUNT    32
#define MAX_CNTR_CH_COUNT     8

typedef signed char    int8;
typedef signed short   int16;

typedef unsigned char  uint8;
typedef unsigned short uint16;

#if defined(_WIN32) || defined(WIN32)
#  define BDAQCALL       WINAPI
#  ifndef _WIN64
      typedef signed   int  int32;
      typedef unsigned int  uint32;
#  else
      typedef signed   long int32;
      typedef unsigned long uint32;
#  endif
   typedef signed __int64   int64;
   typedef unsigned __int64 uint64;
#else
#  define BDAQCALL
   typedef signed int         int32;
   typedef unsigned int       uint32;
   typedef signed long long   int64;
   typedef unsigned long long uint64;
   typedef void *             HANDLE;
#endif

typedef enum TerminalBoard {
   WiringBoard = 0,
   PCLD8710,
   PCLD789,
   PCLD8115,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   BoardUnknown = 0xffffffff,
} TerminalBoard;

typedef enum ModuleType {
   DaqGroup = 1,
   DaqDevice,
   DaqAi,
   DaqAo,
   DaqDio,
   DaqCounter,
   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   DaqAny = 0xffffffff,
} ModuleType;

typedef enum AccessMode {
   ModeRead = 0,
   ModeWrite,
   ModeWriteWithReset,
   ModeWriteShared,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   ModeUnknown = 0xffffffff,
} AccessMode;

typedef enum Depository {
   DepositoryNone = 0,
   DepositoryOnSystem,
   DepositoryOnDevice,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   DepositoryUnknown = 0xffffffff,
} Depository;

typedef enum MathIntervalType {
   /* Right boundary definition, define the maximum value state, use the bit 0,1 */
   RightOpenSet        = 0x0, /* No maximum value limitation.  */
   RightClosedBoundary = 0x1,   /* The maximum value is included. */
   RightOpenBoundary   = 0x2, /* The maximum value is not included. */

   /* Left boundary definition, define the minimum value state, used the bit 2, 3 */
   LeftOpenSet        = 0x0,   /* No minimum value limitation. */
   LeftClosedBoundary = 0x4,    /* The minimum value is included. */
   LeftOpenBoundary   = 0x8,   /* The minimum value is not included */

   /* The signality expression */
   Boundless          = 0x0,  /* Boundless set. (LeftOpenSet | RightOpenSet) */

   /* The combination notation */
   LOSROS = 0x0,    /* (LeftOpenSet | RightOpenSet), algebra notation: (un-limit, max) */
   LOSRCB = 0x1,    /* (LeftOpenSet | RightClosedBoundary), algebra notation: (un-limit, max ] */
   LOSROB = 0x2,    /* (LeftOpenSet | RightOpenBoundary), algebra notation: (un-limit, max) */

   LCBROS = 0x4,    /* (LeftClosedBoundary | RightOpenSet), algebra notation: [min, un-limit) */
   LCBRCB = 0x5,    /* (LeftClosedBoundary | RightClosedBoundary), algebra notation: [ min, right ] */
   LCBROB = 0x6,    /* (LeftClosedBoundary | RightOpenBoundary), algebra notation: [ min, right) */

   LOBROS = 0x8,    /* (LeftOpenBoundary | RightOpenSet), algebra notation: (min, un-limit) */
   LOBRCB = 0x9,    /* (LeftOpenBoundary | RightClosedBoundary), algebra notation: (min, right ] */
   LOBROB = 0xA,    /* (LeftOpenBoundary | RightOpenBoundary), algebra notation: (min, right) */

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   IntervalUnknown = 0xffffffff,
} MathIntervalType;

typedef enum AiChannelType {
   AllSingleEnded = 0,
   AllDifferential,
   AllSeDiffAdj,
   MixedSeDiffAdj,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   ChannelUnknown = 0xffffffff,
} AiChannelType;

typedef enum AiSignalType {
   SingleEnded = 0,
   Differential,
   PseudoDifferential,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   AiSignalUnknown = 0xffffffff,
} AiSignalType;

typedef enum CouplingType {
   DCCoupling = 0,
   ACCoupling,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   CouplingUnknown = 0xffffffff,
} CouplingType;

typedef enum ImpedanceType  {
   Ipd1Momh = 0,
   Ipd50omh,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   ImpedanceUnknown = 0xffffffff,
} ImpedanceType;

typedef enum IepeType  {
   IEPENone = 0,
   IEPE4mA = 1,
   IEPE10mA = 2,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   IepeUnknown = 0xffffffff,
} IepeType;

typedef enum MeasureType  {
   VoltageType = 0,
   CurrentType = 1,
   FullBridge = 2,
   HalfBridge = 3,
   QuadBridge = 4,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   MeasureUnknown = 0xffffffff,
} MeasureType;

typedef enum SensorType  {
   StrainGage = 0,
   ForceCell = 1,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   SensorUnknown = 0xffffffff,
} SensorType;

typedef enum FilterType {
   FilterNone = 0,
   LowPass,
   HighPass,
   BandPass,
   BandStop,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   FilterUnknown = 0xffffffff,
} FilterType;

typedef enum DioPortType {
   PortDi = 0,        /*the port number references to a DI port*/
   PortDo,            /*the port number references to a DO port*/
   PortDio,           /*the port number references to a DI port and a DO port*/
   Port8255A,         /*the port number references to a PPI port A mode DIO port.*/
   Port8255C,         /*the port number references to a PPI port C mode DIO port.*/
   PortIndvdlDio,     /*the port number references to a port whose each channel can be configured as in or out.*/

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   PortUnknown = 0xffffffff,
} DioPortType;

typedef enum DioPortDir {
   Input   = 0x00,
   LoutHin = 0x0F,
   LinHout = 0xF0,
   Output  = 0xFF,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   DirUnknown = 0xffffffff,
} DioPortDir;

typedef enum DiOpenState {
   pullHighAllPort = 0x00,
   pullLowAllPort = 0x11,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   OpenStateUnknown = 0xffffffff,
} DiOpenState;

typedef enum SamplingMethod {
   EqualTimeSwitch = 0,
   Simultaneous,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   SamplingUnknown = 0xffffffff,
} SamplingMethod;

typedef enum TemperatureDegree {
   Celsius = 0,
   Fahrenheit,
   Rankine,
   Kelvin,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   DegreeUnknown = 0xffffffff,
} TemperatureDegree;

typedef enum BurnoutRetType {
   Current = 0,
   ParticularValue,
   UpLimit,
   LowLimit,
   LastCorrectValue,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   ReturnUnknown = 0xffffffff,
} BurnoutRetType;

typedef enum ValueUnit {
   Kilovolt,      /* KV */
   Volt,          /* V  */
   Millivolt,     /* mV */
   Microvolt,     /* uV */
   Kiloampere,    /* KA */
   Ampere,        /* A  */
   Milliampere,   /* mA */
   Microampere,   /* uA */
   CelsiusUnit,   /* Celsius */

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   UnitUnknown = 0xffffffff,
} ValueUnit;

typedef enum ValueRange {
   V_Neg15To15 = 0,        /* +/- 15 V  */
   V_Neg10To10,            /* +/- 10 V  */
   V_Neg5To5,              /* +/- 5 V */
   V_Neg2pt5To2pt5,        /* +/- 2.5 V */
   V_Neg1pt25To1pt25,      /* +/- 1.25 V */
   V_Neg1To1,              /* +/- 1 V */

   V_0To15,                /* 0~15 V  */
   V_0To10,                /* 0~10 V  */
   V_0To5,                 /* 0~5 V */
   V_0To2pt5,              /* 0~2.5 V */
   V_0To1pt25,             /* 0~1.25 V */
   V_0To1,                 /* 0~1 V */

   mV_Neg625To625,         /* +/- 625mV */
   mV_Neg500To500,         /* +/- 500 mV */
   mV_Neg312pt5To312pt5,   /* +/- 312.5 mV */
   mV_Neg200To200,         /* +/- 200 mV */
   mV_Neg150To150,         /* +/- 150 mV */
   mV_Neg100To100,         /* +/- 100 mV */
   mV_Neg50To50,           /* +/- 50 mV */
   mV_Neg30To30,           /* +/- 30 mV */
   mV_Neg20To20,           /* +/- 20 mV */
   mV_Neg15To15,           /* +/- 15 mV  */
   mV_Neg10To10,           /* +/- 10 mV */
   mV_Neg5To5,             /* +/- 5 mV */

   mV_0To625,              /* 0 ~ 625 mV */
   mV_0To500,              /* 0 ~ 500 mV */
   mV_0To150,              /* 0 ~ 150 mV */
   mV_0To100,              /* 0 ~ 100 mV */
   mV_0To50,               /* 0 ~ 50 mV */
   mV_0To20,               /* 0 ~ 20 mV */
   mV_0To15,               /* 0 ~ 15 mV */
   mV_0To10,               /* 0 ~ 10 mV */

   mA_Neg20To20,           /* +/- 20mA */
   mA_0To20,               /* 0 ~ 20 mA */
   mA_4To20,               /* 4 ~ 20 mA */
   mA_0To24,               /* 0 ~ 24 mA */

   /* For USB4702_4704 */
   V_Neg2To2,              /* +/- 2 V */
   V_Neg4To4,              /* +/- 4 V */
   V_Neg20To20,            /* +/- 20 V */

   Jtype_0To760C = 0x8000, /* T/C J type 0~760 'C */
   Ktype_0To1370C,         /* T/C K type 0~1370 'C */
   Ttype_Neg100To400C,     /* T/C T type -100~400 'C */
   Etype_0To1000C,         /* T/C E type 0~1000 'C */
   Rtype_500To1750C,       /* T/C R type 500~1750 'C */
   Stype_500To1750C,       /* T/C S type 500~1750 'C */
   Btype_500To1800C,       /* T/C B type 500~1800 'C */

   Pt392_Neg50To150,       /* Pt392 -50~150 'C  */
   Pt385_Neg200To200,      /* Pt385 -200~200 'C */
   Pt385_0To400,           /* Pt385 0~400 'C */
   Pt385_Neg50To150,       /* Pt385 -50~150 'C */
   Pt385_Neg100To100,      /* Pt385 -100~100 'C */
   Pt385_0To100,           /* Pt385 0~100 'C  */
   Pt385_0To200,           /* Pt385 0~200 'C */
   Pt385_0To600,           /* Pt385 0~600 'C */
   Pt392_Neg100To100,      /* Pt392 -100~100 'C  */
   Pt392_0To100,           /* Pt392 0~100 'C */
   Pt392_0To200,           /* Pt392 0~200 'C */
   Pt392_0To600,           /* Pt392 0~600 'C */
   Pt392_0To400,           /* Pt392 0~400 'C */
   Pt392_Neg200To200,      /* Pt392 -200~200 'C  */
   Pt1000_Neg40To160,      /* Pt1000 -40~160 'C  */

   Balcon500_Neg30To120,   /* Balcon500 -30~120 'C  */

   Ni518_Neg80To100,       /* Ni518 -80~100 'C */
   Ni518_0To100,           /* Ni518 0~100 'C */
   Ni508_0To100,           /* Ni508 0~100 'C */
   Ni508_Neg50To200,       /* Ni508 -50~200 'C */

   Thermistor_3K_0To100,   /* Thermistor 3K 0~100 'C */
   Thermistor_10K_0To100,  /* Thermistor 10K 0~100 'C */

   Jtype_Neg210To1200C,    /* T/C J type -210~1200 'C */
   Ktype_Neg270To1372C,    /* T/C K type -270~1372 'C */
   Ttype_Neg270To400C,     /* T/C T type -270~400 'C */
   Etype_Neg270To1000C,    /* T/C E type -270~1000 'C */
   Rtype_Neg50To1768C,     /* T/C R type -50~1768 'C */
   Stype_Neg50To1768C,     /* T/C S type -50~1768 'C */
   Btype_40To1820C,        /* T/C B type 40~1820 'C */

   Jtype_Neg210To870C,     /* T/C J type -210~870 'C */
   Rtype_0To1768C,         /* T/C R type 0~1768 'C */
   Stype_0To1768C,         /* T/C S type 0~1768 'C */
   Ttype_Neg20To135C,      /* T/C T type -20~135 'C */

   /* 0xC000 ~ 0xF000 : user customized value range type */
   UserCustomizedVrgStart = 0xC000,
   UserCustomizedVrgEnd = 0xF000,

   /* AO external reference type */
   V_ExternalRefBipolar = 0xF001, /* External reference voltage unipolar */
   V_ExternalRefUnipolar = 0xF002, /* External reference voltage bipolar */

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   V_OMIT = 0xffffffff,            /* Unknown when get, ignored when set */
} ValueRange;

typedef enum SignalPolarity {
   Negative = 0,
   Positive,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   PolarityUnknown = 0xffffffff,
} SignalPolarity;

typedef enum CountingType {
   CountingNone = 0,
   DownCount,      /* counter value decreases on each clock */
   UpCount,        /* counter value increases on each clock */
   PulseDirection, /* counting direction is determined by two signals, one is clock, the other is direction signal */
   TwoPulse,       /* counting direction is determined by two signals, one is up-counting signal, the other is down-counting signal */
   AbPhaseX1,      /* AB phase, 1x rate up/down counting */
   AbPhaseX2,      /* AB phase, 2x rate up/down counting */
   AbPhaseX4,      /* AB phase, 4x rate up/down counting */

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   CountingUnknown = 0xffffffff,
} CountingType;
/*for compatible*/
typedef CountingType SignalCountingType; 

typedef enum OutSignalType{
   SignalOutNone = 0,  /* no output or output is 'disabled' */
   ChipDefined,        /* hardware chip defined */
   NegChipDefined,     /* hardware chip defined, negative logical */
   PositivePulse,      /* a low-to-high pulse */
   NegativePulse,      /* a high-to-low pulse */
   ToggledFromLow,     /* the level toggled from low to high */
   ToggledFromHigh,    /* the level toggled from high to low */

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   OutSignalUnknown = 0xffffffff,
} OutSignalType;

typedef enum CounterCapability {
   Primary = 1,
   InstantEventCount,
   OneShot,
   TimerPulse,
   InstantFreqMeter,
   InstantPwmIn,
   InstantPwmOut,
   UpDownCount,

   BufferedEventCount,
   BufferedPwmIn,
   BufferedPwmOut,
   BufferedUpDownCount,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   CapabilityUnknown = 0xffffffff,
} CounterCapability;

typedef enum CounterOperationMode {
   C8254_M0 = 0, /*8254 mode 0, interrupt on terminal count */
   C8254_M1,     /*8254 mode 1, hardware retriggerable one-shot */
   C8254_M2,     /*8254 mode 2, rate generator */
   C8254_M3,     /*8254 mode 3, square save mode */
   C8254_M4,     /*8254 mode 4, software triggered strobe */
   C8254_M5,     /*8254 mode 5, hardware triggered strobe */

   C1780_MA,   /* Mode A level & pulse out, Software-Triggered without Hardware Gating */
   C1780_MB,   /* Mode B level & pulse out, Software-Triggered with Level Gating, = 8254_M0 */
   C1780_MC,   /* Mode C level & pulse out, Hardware-triggered strobe level */
   C1780_MD,   /* Mode D level & Pulse out, Rate generate with no hardware gating */
   C1780_ME,   /* Mode E level & pulse out, Rate generator with level Gating */
   C1780_MF,   /* Mode F level & pulse out, Non-retriggerable One-shot (Pulse type = 8254_M1) */
   C1780_MG,   /* Mode G level & pulse out, Software-triggered delayed pulse one-shot */
   C1780_MH,   /* Mode H level & pulse out, Software-triggered delayed pulse one-shot with hardware gating */
   C1780_MI,   /* Mode I level & pulse out, Hardware-triggered delay pulse strobe */
   C1780_MJ,   /* Mode J level & pulse out, Variable Duty Cycle Rate Generator with No Hardware Gating */
   C1780_MK,   /* Mode K level & pulse out, Variable Duty Cycle Rate Generator with Level Gating */
   C1780_ML,   /* Mode L level & pulse out, Hardware-Triggered Delayed Pulse One-Shot */
   C1780_MO,   /* Mode O level & pulse out, Hardware-Triggered Strobe with Edge Disarm */
   C1780_MR,   /* Mode R level & pulse out, Non-Retriggerbale One-Shot with Edge Disarm */
   C1780_MU,   /* Mode U level & pulse out, Hardware-Triggered Delayed Pulse Strobe with Edge Disarm */
   C1780_MX,   /* Mode X level & pulse out, Hardware-Triggered Delayed Pulse One-Shot with Edge Disarm */

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   OpModeUnknown = 0xffffffff,
} CounterOperationMode;

typedef enum CounterValueRegister {
   CntLoad,
   CntPreset = CntLoad,
   CntHold,
   CntOverCompare,
   CntUnderCompare,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
  RegisterUnknown = 0xffffffff,
} CounterValueRegister;

typedef enum CounterCascadeGroup {
   GroupNone = 0,    /* no cascade*/
   Cnt0Cnt1,         /* Counter 0 as first, counter 1 as second. */
   Cnt2Cnt3,         /* Counter 2 as first, counter 3 as second */
   Cnt4Cnt5,         /* Counter 4 as first, counter 5 as second */
   Cnt6Cnt7,         /* Counter 6 as first, counter 7 as second */

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   CascadeUnknown = 0xffffffff,
} CounterCascadeGroup;

typedef enum FreqMeasureMethod {
   AutoAdaptive = 0,          /* Intelligently select the measurement method according to the input signal. */
   CountingPulseBySysTime,    /* Using system timing clock to calculate the frequency */
   CountingPulseByDevTime,      /* Using the device timing clock to calculate the frequency */
   PeriodInverse,               /* Calculate the frequency from the period of the signal */

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   MethodUnknown = 0xffffffff,
} FreqMeasureMethod;

typedef enum ActiveSignal {
   ActiveNone = 0,
   RisingEdge,
   FallingEdge,
   BothEdge,
   HighLevel,
   LowLevel,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   ActSignalUnknown = 0xffffffff,
} ActiveSignal;

typedef enum TriggerAction {
   ActionNone = 0,   /* No action to take even if the trigger condition is satisfied */
   DelayToStart,     /* Begin to start after the specified time is elapsed if the trigger condition is satisfied */
   DelayToStop,      /* Stop execution after the specified time is elapsed if the trigger condition is satisfied */
   Mark,             /* Generate a mark data*/

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   ActionUnknown = 0xffffffff,
} TriggerAction;

typedef enum SignalPosition {
   InternalSig = 0,
   OnConnector,
   OnAmsi,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   PositionUnknown = 0xffffffff,
} SignalPosition;

typedef enum SignalDrop {
   SignalNone = 0,          /* No connection */

   /*Internal signal connector*/
   SigInternalClock,        /* Device built-in clock, If the device has several built-in clock, this represent the highest freq one. */
   SigInternal1KHz,         /* Device built-in clock, 1KHz */
   SigInternal10KHz,        /* Device built-in clock, 10KHz */
   SigInternal100KHz,       /* Device built-in clock, 100KHz */
   SigInternal1MHz,         /* Device built-in clock, 1MHz */
   SigInternal10MHz,        /* Device built-in clock, 10MHz */
   SigInternal20MHz,        /* Device built-in clock, 20MHz */
   SigInternal30MHz,        /* Device built-in clock, 30MHz */
   SigInternal40MHz,        /* Device built-in clock, 40MHz */
   SigInternal50MHz,        /* Device built-in clock, 50MHz */
   SigInternal60MHz,        /* Device built-in clock, 60MHz */

   SigDiPatternMatch,       /* When DI pattern match occurred */
   SigDiStatusChange,       /* When DI status change occurred */

   /*Function pin on connector*/
   SigExtAnaClock,          /* Analog clock pin of connector */
   SigExtAnaScanClock,      /* scan clock pin of connector */
   SigExtAnaTrigger,        /* external analog trigger pin of connector */
   SigExtAnaTrigger0 = SigExtAnaTrigger, /* external analog trigger pin of connector 0*/
   SigExtDigClock,          /* digital clock pin of connector */
   SigExtDigTrigger0,       /* external digital trigger 0 pin(or DI start trigger pin) of connector */
   SigExtDigTrigger1,       /* external digital trigger 1 pin(or DI stop trigger pin) of connector  */
   SigExtDigTrigger2,       /* external digital trigger 2 pin(or DO start trigger pin) of connector */
   SigExtDigTrigger3,       /* external digital trigger 3 pin(or DO stop trigger pin) of connector  */
   SigCHFrzDo,              /* Channel freeze DO ports pin */

   /*Signal source or target on the connector*/
   /*AI channel pins*/
   SigAi0,  SigAi1,  SigAi2,  SigAi3,  SigAi4,  SigAi5,  SigAi6,  SigAi7, 
   SigAi8,  SigAi9,  SigAi10, SigAi11, SigAi12, SigAi13, SigAi14, SigAi15,
   SigAi16, SigAi17, SigAi18, SigAi19, SigAi20, SigAi21, SigAi22, SigAi23,
   SigAi24, SigAi25, SigAi26, SigAi27, SigAi28, SigAi29, SigAi30, SigAi31, 
   SigAi32, SigAi33, SigAi34, SigAi35, SigAi36, SigAi37, SigAi38, SigAi39,
   SigAi40, SigAi41, SigAi42, SigAi43, SigAi44, SigAi45, SigAi46, SigAi47,
   SigAi48, SigAi49, SigAi50, SigAi51, SigAi52, SigAi53, SigAi54, SigAi55, 
   SigAi56, SigAi57, SigAi58, SigAi59, SigAi60, SigAi61, SigAi62, SigAi63,

   /*AO channel pins*/
   SigAo0,  SigAo1,  SigAo2,  SigAo3,  SigAo4,  SigAo5,  SigAo6,  SigAo7,
   SigAo8,  SigAo9,  SigAo10, SigAo11, SigAo12, SigAo13, SigAo14, SigAo15,
   SigAo16, SigAo17, SigAo18, SigAo19, SigAo20, SigAo21, SigAo22, SigAo23,
   SigAo24, SigAo25, SigAo26, SigAo27, SigAo28, SigAo29, SigAo30, SigAo31,

   /*DI pins*/
   SigDi0,   SigDi1,   SigDi2,   SigDi3,   SigDi4,   SigDi5,   SigDi6,   SigDi7,
   SigDi8,   SigDi9,   SigDi10,  SigDi11,  SigDi12,  SigDi13,  SigDi14,  SigDi15,
   SigDi16,  SigDi17,  SigDi18,  SigDi19,  SigDi20,  SigDi21,  SigDi22,  SigDi23,
   SigDi24,  SigDi25,  SigDi26,  SigDi27,  SigDi28,  SigDi29,  SigDi30,  SigDi31,
   SigDi32,  SigDi33,  SigDi34,  SigDi35,  SigDi36,  SigDi37,  SigDi38,  SigDi39,
   SigDi40,  SigDi41,  SigDi42,  SigDi43,  SigDi44,  SigDi45,  SigDi46,  SigDi47,
   SigDi48,  SigDi49,  SigDi50,  SigDi51,  SigDi52,  SigDi53,  SigDi54,  SigDi55,
   SigDi56,  SigDi57,  SigDi58,  SigDi59,  SigDi60,  SigDi61,  SigDi62,  SigDi63,
   SigDi64,  SigDi65,  SigDi66,  SigDi67,  SigDi68,  SigDi69,  SigDi70,  SigDi71,
   SigDi72,  SigDi73,  SigDi74,  SigDi75,  SigDi76,  SigDi77,  SigDi78,  SigDi79,
   SigDi80,  SigDi81,  SigDi82,  SigDi83,  SigDi84,  SigDi85,  SigDi86,  SigDi87,
   SigDi88,  SigDi89,  SigDi90,  SigDi91,  SigDi92,  SigDi93,  SigDi94,  SigDi95,
   SigDi96,  SigDi97,  SigDi98,  SigDi99,  SigDi100, SigDi101, SigDi102, SigDi103,
   SigDi104, SigDi105, SigDi106, SigDi107, SigDi108, SigDi109, SigDi110, SigDi111,
   SigDi112, SigDi113, SigDi114, SigDi115, SigDi116, SigDi117, SigDi118, SigDi119,
   SigDi120, SigDi121, SigDi122, SigDi123, SigDi124, SigDi125, SigDi126, SigDi127,
   SigDi128, SigDi129, SigDi130, SigDi131, SigDi132, SigDi133, SigDi134, SigDi135,
   SigDi136, SigDi137, SigDi138, SigDi139, SigDi140, SigDi141, SigDi142, SigDi143,
   SigDi144, SigDi145, SigDi146, SigDi147, SigDi148, SigDi149, SigDi150, SigDi151,
   SigDi152, SigDi153, SigDi154, SigDi155, SigDi156, SigDi157, SigDi158, SigDi159,
   SigDi160, SigDi161, SigDi162, SigDi163, SigDi164, SigDi165, SigDi166, SigDi167,
   SigDi168, SigDi169, SigDi170, SigDi171, SigDi172, SigDi173, SigDi174, SigDi175,
   SigDi176, SigDi177, SigDi178, SigDi179, SigDi180, SigDi181, SigDi182, SigDi183,
   SigDi184, SigDi185, SigDi186, SigDi187, SigDi188, SigDi189, SigDi190, SigDi191,
   SigDi192, SigDi193, SigDi194, SigDi195, SigDi196, SigDi197, SigDi198, SigDi199,
   SigDi200, SigDi201, SigDi202, SigDi203, SigDi204, SigDi205, SigDi206, SigDi207,
   SigDi208, SigDi209, SigDi210, SigDi211, SigDi212, SigDi213, SigDi214, SigDi215,
   SigDi216, SigDi217, SigDi218, SigDi219, SigDi220, SigDi221, SigDi222, SigDi223,
   SigDi224, SigDi225, SigDi226, SigDi227, SigDi228, SigDi229, SigDi230, SigDi231,
   SigDi232, SigDi233, SigDi234, SigDi235, SigDi236, SigDi237, SigDi238, SigDi239,
   SigDi240, SigDi241, SigDi242, SigDi243, SigDi244, SigDi245, SigDi246, SigDi247,
   SigDi248, SigDi249, SigDi250, SigDi251, SigDi252, SigDi253, SigDi254, SigDi255,

   /*DIO pins*/
   SigDio0,   SigDio1,   SigDio2,   SigDio3,   SigDio4,   SigDio5,   SigDio6,   SigDio7,
   SigDio8,   SigDio9,   SigDio10,  SigDio11,  SigDio12,  SigDio13,  SigDio14,  SigDio15,
   SigDio16,  SigDio17,  SigDio18,  SigDio19,  SigDio20,  SigDio21,  SigDio22,  SigDio23,
   SigDio24,  SigDio25,  SigDio26,  SigDio27,  SigDio28,  SigDio29,  SigDio30,  SigDio31,
   SigDio32,  SigDio33,  SigDio34,  SigDio35,  SigDio36,  SigDio37,  SigDio38,  SigDio39,
   SigDio40,  SigDio41,  SigDio42,  SigDio43,  SigDio44,  SigDio45,  SigDio46,  SigDio47,
   SigDio48,  SigDio49,  SigDio50,  SigDio51,  SigDio52,  SigDio53,  SigDio54,  SigDio55,
   SigDio56,  SigDio57,  SigDio58,  SigDio59,  SigDio60,  SigDio61,  SigDio62,  SigDio63,
   SigDio64,  SigDio65,  SigDio66,  SigDio67,  SigDio68,  SigDio69,  SigDio70,  SigDio71,
   SigDio72,  SigDio73,  SigDio74,  SigDio75,  SigDio76,  SigDio77,  SigDio78,  SigDio79,
   SigDio80,  SigDio81,  SigDio82,  SigDio83,  SigDio84,  SigDio85,  SigDio86,  SigDio87,
   SigDio88,  SigDio89,  SigDio90,  SigDio91,  SigDio92,  SigDio93,  SigDio94,  SigDio95,
   SigDio96,  SigDio97,  SigDio98,  SigDio99,  SigDio100, SigDio101, SigDio102, SigDio103,
   SigDio104, SigDio105, SigDio106, SigDio107, SigDio108, SigDio109, SigDio110, SigDio111,
   SigDio112, SigDio113, SigDio114, SigDio115, SigDio116, SigDio117, SigDio118, SigDio119,
   SigDio120, SigDio121, SigDio122, SigDio123, SigDio124, SigDio125, SigDio126, SigDio127,
   SigDio128, SigDio129, SigDio130, SigDio131, SigDio132, SigDio133, SigDio134, SigDio135,
   SigDio136, SigDio137, SigDio138, SigDio139, SigDio140, SigDio141, SigDio142, SigDio143,
   SigDio144, SigDio145, SigDio146, SigDio147, SigDio148, SigDio149, SigDio150, SigDio151,
   SigDio152, SigDio153, SigDio154, SigDio155, SigDio156, SigDio157, SigDio158, SigDio159,
   SigDio160, SigDio161, SigDio162, SigDio163, SigDio164, SigDio165, SigDio166, SigDio167,
   SigDio168, SigDio169, SigDio170, SigDio171, SigDio172, SigDio173, SigDio174, SigDio175,
   SigDio176, SigDio177, SigDio178, SigDio179, SigDio180, SigDio181, SigDio182, SigDio183,
   SigDio184, SigDio185, SigDio186, SigDio187, SigDio188, SigDio189, SigDio190, SigDio191,
   SigDio192, SigDio193, SigDio194, SigDio195, SigDio196, SigDio197, SigDio198, SigDio199,
   SigDio200, SigDio201, SigDio202, SigDio203, SigDio204, SigDio205, SigDio206, SigDio207,
   SigDio208, SigDio209, SigDio210, SigDio211, SigDio212, SigDio213, SigDio214, SigDio215,
   SigDio216, SigDio217, SigDio218, SigDio219, SigDio220, SigDio221, SigDio222, SigDio223,
   SigDio224, SigDio225, SigDio226, SigDio227, SigDio228, SigDio229, SigDio230, SigDio231,
   SigDio232, SigDio233, SigDio234, SigDio235, SigDio236, SigDio237, SigDio238, SigDio239,
   SigDio240, SigDio241, SigDio242, SigDio243, SigDio244, SigDio245, SigDio246, SigDio247,
   SigDio248, SigDio249, SigDio250, SigDio251, SigDio252, SigDio253, SigDio254, SigDio255,

   /*Counter clock pins*/
   SigCntClk0, SigCntClk1, SigCntClk2, SigCntClk3, SigCntClk4, SigCntClk5, SigCntClk6, SigCntClk7,

   /*counter gate pins*/
   SigCntGate0, SigCntGate1, SigCntGate2, SigCntGate3, SigCntGate4, SigCntGate5, SigCntGate6, SigCntGate7,

   /*counter out pins*/
   SigCntOut0,  SigCntOut1,  SigCntOut2,  SigCntOut3,  SigCntOut4,  SigCntOut5,  SigCntOut6,  SigCntOut7,

   /*counter frequency out pins*/
   SigCntFout0, SigCntFout1, SigCntFout2, SigCntFout3, SigCntFout4, SigCntFout5, SigCntFout6, SigCntFout7,

   /*AMSI pins*/
   SigAmsiPin0,  SigAmsiPin1,  SigAmsiPin2,  SigAmsiPin3,  SigAmsiPin4,  SigAmsiPin5,  SigAmsiPin6,  SigAmsiPin7,
   SigAmsiPin8,  SigAmsiPin9,  SigAmsiPin10, SigAmsiPin11, SigAmsiPin12, SigAmsiPin13, SigAmsiPin14, SigAmsiPin15,
   SigAmsiPin16, SigAmsiPin17, SigAmsiPin18, SigAmsiPin19,

   /*new clocks*/
   SigInternal2Hz,         /* Device built-in clock, 2Hz */
   SigInternal20Hz,        /* Device built-in clock, 20Hz */
   SigInternal200Hz,       /* Device built-in clock, 200KHz */
   SigInternal2KHz,        /* Device built-in clock, 2KHz */
   SigInternal20KHz,       /* Device built-in clock, 20KHz */
   SigInternal200KHz,      /* Device built-in clock, 200KHz */
   SigInternal2MHz,        /* Device built-in clock, 2MHz */

   /*New Function pin on connector*/
   SigExtAnaTrigger1,       /* external analog trigger pin of connector 1 */

   /*Reference clock*/
   SigExtDigRefClock,          /* digital clock pin of connector */

   SigInternal100MHz,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   SigDropUnknown = 0xffffffff,
} SignalDrop;

/*
* Event Id
*/
typedef enum EventId {
   EvtDeviceRemoved = 0,  /* The device was removed from system */
   EvtDeviceReconnected,  /* The device is reconnected */
   EvtPropertyChanged,    /* Some properties of the device were changed */
   /*-----------------------------------------------------------------
   * AI events
   *-----------------------------------------------------------------*/
   EvtBufferedAiDataReady, 
   EvtBufferedAiOverrun,
   EvtBufferedAiCacheOverflow,
   EvtBufferedAiStopped,

   /*-----------------------------------------------------------------
   * AO event IDs
   *-----------------------------------------------------------------*/
   EvtBufferedAoDataTransmitted,
   EvtBufferedAoUnderrun,
   EvtBufferedAoCacheEmptied,
   EvtBufferedAoTransStopped,
   EvtBufferedAoStopped,

   /*-----------------------------------------------------------------
   * DIO event IDs 
   *-----------------------------------------------------------------*/
   EvtDiInterrupt,     EvtDiintChannel000 = EvtDiInterrupt, 
                       EvtDiintChannel001, EvtDiintChannel002, EvtDiintChannel003,
   EvtDiintChannel004, EvtDiintChannel005, EvtDiintChannel006, EvtDiintChannel007,
   EvtDiintChannel008, EvtDiintChannel009, EvtDiintChannel010, EvtDiintChannel011,
   EvtDiintChannel012, EvtDiintChannel013, EvtDiintChannel014, EvtDiintChannel015,
   EvtDiintChannel016, EvtDiintChannel017, EvtDiintChannel018, EvtDiintChannel019,
   EvtDiintChannel020, EvtDiintChannel021, EvtDiintChannel022, EvtDiintChannel023,
   EvtDiintChannel024, EvtDiintChannel025, EvtDiintChannel026, EvtDiintChannel027,
   EvtDiintChannel028, EvtDiintChannel029, EvtDiintChannel030, EvtDiintChannel031,
   EvtDiintChannel032, EvtDiintChannel033, EvtDiintChannel034, EvtDiintChannel035,
   EvtDiintChannel036, EvtDiintChannel037, EvtDiintChannel038, EvtDiintChannel039,
   EvtDiintChannel040, EvtDiintChannel041, EvtDiintChannel042, EvtDiintChannel043,
   EvtDiintChannel044, EvtDiintChannel045, EvtDiintChannel046, EvtDiintChannel047,
   EvtDiintChannel048, EvtDiintChannel049, EvtDiintChannel050, EvtDiintChannel051,
   EvtDiintChannel052, EvtDiintChannel053, EvtDiintChannel054, EvtDiintChannel055,
   EvtDiintChannel056, EvtDiintChannel057, EvtDiintChannel058, EvtDiintChannel059,
   EvtDiintChannel060, EvtDiintChannel061, EvtDiintChannel062, EvtDiintChannel063,
   EvtDiintChannel064, EvtDiintChannel065, EvtDiintChannel066, EvtDiintChannel067,
   EvtDiintChannel068, EvtDiintChannel069, EvtDiintChannel070, EvtDiintChannel071,
   EvtDiintChannel072, EvtDiintChannel073, EvtDiintChannel074, EvtDiintChannel075,
   EvtDiintChannel076, EvtDiintChannel077, EvtDiintChannel078, EvtDiintChannel079,
   EvtDiintChannel080, EvtDiintChannel081, EvtDiintChannel082, EvtDiintChannel083,
   EvtDiintChannel084, EvtDiintChannel085, EvtDiintChannel086, EvtDiintChannel087,
   EvtDiintChannel088, EvtDiintChannel089, EvtDiintChannel090, EvtDiintChannel091,
   EvtDiintChannel092, EvtDiintChannel093, EvtDiintChannel094, EvtDiintChannel095,
   EvtDiintChannel096, EvtDiintChannel097, EvtDiintChannel098, EvtDiintChannel099,
   EvtDiintChannel100, EvtDiintChannel101, EvtDiintChannel102, EvtDiintChannel103,
   EvtDiintChannel104, EvtDiintChannel105, EvtDiintChannel106, EvtDiintChannel107,
   EvtDiintChannel108, EvtDiintChannel109, EvtDiintChannel110, EvtDiintChannel111,
   EvtDiintChannel112, EvtDiintChannel113, EvtDiintChannel114, EvtDiintChannel115,
   EvtDiintChannel116, EvtDiintChannel117, EvtDiintChannel118, EvtDiintChannel119,
   EvtDiintChannel120, EvtDiintChannel121, EvtDiintChannel122, EvtDiintChannel123,
   EvtDiintChannel124, EvtDiintChannel125, EvtDiintChannel126, EvtDiintChannel127,
   EvtDiintChannel128, EvtDiintChannel129, EvtDiintChannel130, EvtDiintChannel131,
   EvtDiintChannel132, EvtDiintChannel133, EvtDiintChannel134, EvtDiintChannel135,
   EvtDiintChannel136, EvtDiintChannel137, EvtDiintChannel138, EvtDiintChannel139,
   EvtDiintChannel140, EvtDiintChannel141, EvtDiintChannel142, EvtDiintChannel143,
   EvtDiintChannel144, EvtDiintChannel145, EvtDiintChannel146, EvtDiintChannel147,
   EvtDiintChannel148, EvtDiintChannel149, EvtDiintChannel150, EvtDiintChannel151,
   EvtDiintChannel152, EvtDiintChannel153, EvtDiintChannel154, EvtDiintChannel155,
   EvtDiintChannel156, EvtDiintChannel157, EvtDiintChannel158, EvtDiintChannel159,
   EvtDiintChannel160, EvtDiintChannel161, EvtDiintChannel162, EvtDiintChannel163,
   EvtDiintChannel164, EvtDiintChannel165, EvtDiintChannel166, EvtDiintChannel167,
   EvtDiintChannel168, EvtDiintChannel169, EvtDiintChannel170, EvtDiintChannel171,
   EvtDiintChannel172, EvtDiintChannel173, EvtDiintChannel174, EvtDiintChannel175,
   EvtDiintChannel176, EvtDiintChannel177, EvtDiintChannel178, EvtDiintChannel179,
   EvtDiintChannel180, EvtDiintChannel181, EvtDiintChannel182, EvtDiintChannel183,
   EvtDiintChannel184, EvtDiintChannel185, EvtDiintChannel186, EvtDiintChannel187,
   EvtDiintChannel188, EvtDiintChannel189, EvtDiintChannel190, EvtDiintChannel191,
   EvtDiintChannel192, EvtDiintChannel193, EvtDiintChannel194, EvtDiintChannel195,
   EvtDiintChannel196, EvtDiintChannel197, EvtDiintChannel198, EvtDiintChannel199,
   EvtDiintChannel200, EvtDiintChannel201, EvtDiintChannel202, EvtDiintChannel203,
   EvtDiintChannel204, EvtDiintChannel205, EvtDiintChannel206, EvtDiintChannel207,
   EvtDiintChannel208, EvtDiintChannel209, EvtDiintChannel210, EvtDiintChannel211,
   EvtDiintChannel212, EvtDiintChannel213, EvtDiintChannel214, EvtDiintChannel215,
   EvtDiintChannel216, EvtDiintChannel217, EvtDiintChannel218, EvtDiintChannel219,
   EvtDiintChannel220, EvtDiintChannel221, EvtDiintChannel222, EvtDiintChannel223,
   EvtDiintChannel224, EvtDiintChannel225, EvtDiintChannel226, EvtDiintChannel227,
   EvtDiintChannel228, EvtDiintChannel229, EvtDiintChannel230, EvtDiintChannel231,
   EvtDiintChannel232, EvtDiintChannel233, EvtDiintChannel234, EvtDiintChannel235,
   EvtDiintChannel236, EvtDiintChannel237, EvtDiintChannel238, EvtDiintChannel239,
   EvtDiintChannel240, EvtDiintChannel241, EvtDiintChannel242, EvtDiintChannel243,
   EvtDiintChannel244, EvtDiintChannel245, EvtDiintChannel246, EvtDiintChannel247,
   EvtDiintChannel248, EvtDiintChannel249, EvtDiintChannel250, EvtDiintChannel251,
   EvtDiintChannel252, EvtDiintChannel253, EvtDiintChannel254, EvtDiintChannel255,

   EvtDiStatusChange,  EvtDiCosintPort000 = EvtDiStatusChange, 
                       EvtDiCosintPort001, EvtDiCosintPort002, EvtDiCosintPort003,
   EvtDiCosintPort004, EvtDiCosintPort005, EvtDiCosintPort006, EvtDiCosintPort007,
   EvtDiCosintPort008, EvtDiCosintPort009, EvtDiCosintPort010, EvtDiCosintPort011,
   EvtDiCosintPort012, EvtDiCosintPort013, EvtDiCosintPort014, EvtDiCosintPort015,
   EvtDiCosintPort016, EvtDiCosintPort017, EvtDiCosintPort018, EvtDiCosintPort019,
   EvtDiCosintPort020, EvtDiCosintPort021, EvtDiCosintPort022, EvtDiCosintPort023,
   EvtDiCosintPort024, EvtDiCosintPort025, EvtDiCosintPort026, EvtDiCosintPort027,
   EvtDiCosintPort028, EvtDiCosintPort029, EvtDiCosintPort030, EvtDiCosintPort031,

   EvtDiPatternMatch,  EvtDiPmintPort000 = EvtDiPatternMatch,  
                       EvtDiPmintPort001,  EvtDiPmintPort002,  EvtDiPmintPort003,
   EvtDiPmintPort004,  EvtDiPmintPort005,  EvtDiPmintPort006,  EvtDiPmintPort007,
   EvtDiPmintPort008,  EvtDiPmintPort009,  EvtDiPmintPort010,  EvtDiPmintPort011,
   EvtDiPmintPort012,  EvtDiPmintPort013,  EvtDiPmintPort014,  EvtDiPmintPort015,
   EvtDiPmintPort016,  EvtDiPmintPort017,  EvtDiPmintPort018,  EvtDiPmintPort019,
   EvtDiPmintPort020,  EvtDiPmintPort021,  EvtDiPmintPort022,  EvtDiPmintPort023,
   EvtDiPmintPort024,  EvtDiPmintPort025,  EvtDiPmintPort026,  EvtDiPmintPort027,
   EvtDiPmintPort028,  EvtDiPmintPort029,  EvtDiPmintPort030,  EvtDiPmintPort031,

   EvtBufferedDiDataReady,
   EvtBufferedDiOverrun,
   EvtBufferedDiCacheOverflow,
   EvtBufferedDiStopped,

   EvtBufferedDoDataTransmitted,
   EvtBufferedDoUnderrun,
   EvtBufferedDoCacheEmptied,
   EvtBufferedDoTransStopped,
   EvtBufferedDoStopped,

   EvtReflectWdtOccured,
   /*-----------------------------------------------------------------
   * Counter/Timer event IDs
   *-----------------------------------------------------------------*/
   EvtCntTerminalCount0, EvtCntTerminalCount1, EvtCntTerminalCount2, EvtCntTerminalCount3,
   EvtCntTerminalCount4, EvtCntTerminalCount5, EvtCntTerminalCount6, EvtCntTerminalCount7,

   EvtCntOverCompare0,   EvtCntOverCompare1,   EvtCntOverCompare2,   EvtCntOverCompare3,
   EvtCntOverCompare4,   EvtCntOverCompare5,   EvtCntOverCompare6,   EvtCntOverCompare7,

   EvtCntUnderCompare0,  EvtCntUnderCompare1,  EvtCntUnderCompare2,  EvtCntUnderCompare3,
   EvtCntUnderCompare4,  EvtCntUnderCompare5,  EvtCntUnderCompare6,  EvtCntUnderCompare7,

   EvtCntEcOverCompare0, EvtCntEcOverCompare1, EvtCntEcOverCompare2, EvtCntEcOverCompare3,
   EvtCntEcOverCompare4, EvtCntEcOverCompare5, EvtCntEcOverCompare6, EvtCntEcOverCompare7,

   EvtCntEcUnderCompare0, EvtCntEcUnderCompare1, EvtCntEcUnderCompare2, EvtCntEcUnderCompare3,
   EvtCntEcUnderCompare4, EvtCntEcUnderCompare5, EvtCntEcUnderCompare6, EvtCntEcUnderCompare7,

   EvtCntOneShot0,       EvtCntOneShot1,       EvtCntOneShot2,       EvtCntOneShot3,
   EvtCntOneShot4,       EvtCntOneShot5,       EvtCntOneShot6,       EvtCntOneShot7,

   EvtCntTimer0,         EvtCntTimer1,         EvtCntTimer2,         EvtCntTimer3,
   EvtCntTimer4,         EvtCntTimer5,         EvtCntTimer6,         EvtCntTimer7,

   EvtCntPwmInOverflow0, EvtCntPwmInOverflow1, EvtCntPwmInOverflow2, EvtCntPwmInOverflow3,
   EvtCntPwmInOverflow4, EvtCntPwmInOverflow5, EvtCntPwmInOverflow6, EvtCntPwmInOverflow7,

   EvtUdIndex0, EvtUdIndex1, EvtUdIndex2, EvtUdIndex3,
   EvtUdIndex4, EvtUdIndex5, EvtUdIndex6, EvtUdIndex7,

   EvtCntPatternMatch0, EvtCntPatternMatch1, EvtCntPatternMatch2, EvtCntPatternMatch3,
   EvtCntPatternMatch4, EvtCntPatternMatch5, EvtCntPatternMatch6, EvtCntPatternMatch7,

   EvtCntCompareTableEnd0, EvtCntCompareTableEnd1, EvtCntCompareTableEnd2, EvtCntCompareTableEnd3,
   EvtCntCompareTableEnd4, EvtCntCompareTableEnd5, EvtCntCompareTableEnd6, EvtCntCompareTableEnd7,

   /*##xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx*/
   /*v1.1: new event of AI                           */
   /*##xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx*/
   EvtBufferedAiBurnOut,
   EvtBufferedAiTimeStampOverrun,
   EvtBufferedAiTimeStampCacheOverflow,
   EvtBufferedAiMarkOverrun,
   EvtBufferedAiConvStopped,

   /*##xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx*/
   /*v1.2: new event of Buffered Counter             */
   /*##xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx*/
   EvtCiDataReady,  EvtCiDataReady0 = EvtCiDataReady, EvtCiDataReady1, EvtCiDataReady2, 
   EvtCiDataReady3, EvtCiDataReady4, EvtCiDataReady5, EvtCiDataReady6, EvtCiDataReady7,

   EvtCiOverrun,  EvtCiOverrun0 = EvtCiOverrun, EvtCiOverrun1, EvtCiOverrun2,
   EvtCiOverrun3, EvtCiOverrun4, EvtCiOverrun5, EvtCiOverrun6, EvtCiOverrun7,

   EvtCiCacheOverflow,  EvtCiCacheOverflow0 = EvtCiCacheOverflow, EvtCiCacheOverflow1, EvtCiCacheOverflow2, 
   EvtCiCacheOverflow3, EvtCiCacheOverflow4, EvtCiCacheOverflow5, EvtCiCacheOverflow6, EvtCiCacheOverflow7,

   EvtCoDataTransmitted,  EvtCoDataTransmitted0 = EvtCoDataTransmitted, EvtCoDataTransmitted1, EvtCoDataTransmitted2, 
   EvtCoDataTransmitted3, EvtCoDataTransmitted4, EvtCoDataTransmitted5, EvtCoDataTransmitted6, EvtCoDataTransmitted7,

   EvtCoUnderrun,  EvtCoUnderrun0 = EvtCoUnderrun, EvtCoUnderrun1, EvtCoUnderrun2, 
   EvtCoUnderrun3, EvtCoUnderrun4, EvtCoUnderrun5, EvtCoUnderrun6, EvtCoUnderrun7,

   EvtCoCacheEmptied,  EvtCoCacheEmptied0 = EvtCoCacheEmptied, EvtCoCacheEmptied1, EvtCoCacheEmptied2, 
   EvtCoCacheEmptied3, EvtCoCacheEmptied4, EvtCoCacheEmptied5, EvtCoCacheEmptied6, EvtCoCacheEmptied7,

   EvtCoTransStopped,  EvtCoTransStopped0 = EvtCoTransStopped, EvtCoTransStopped1, EvtCoTransStopped2, 
   EvtCoTransStopped3, EvtCoTransStopped4, EvtCoTransStopped5, EvtCoTransStopped6, EvtCoTransStopped7,

   EvtCntrStopped,  EvtCntrStopped0 = EvtCntrStopped, EvtCntrStopped1, EvtCntrStopped2, 
   EvtCntrStopped3, EvtCntrStopped4, EvtCntrStopped5, EvtCntrStopped6, EvtCntrStopped7,

   /*----------------------------------------------------------------------*/
   /*Dummy ID, to ensure the type is compiled as 'int' by various compiler */
   EventUnknown = 0xffffffff,
} EventId ;

#define BioFailed(c)  ((unsigned)(c) >= (unsigned)0xE0000000)   

typedef enum ErrorCode {
   /// <summary>
   /// The operation is completed successfully. 
   /// </summary>
   Success = 0, 

   ///************************************************************************
   /// warning                                                              
   ///************************************************************************
   /// <summary>
   /// The interrupt resource is not available. 
   /// </summary>
   WarningIntrNotAvailable = 0xA0000000,

   /// <summary>
   /// The parameter is out of the range. 
   /// </summary>
   WarningParamOutOfRange = 0xA0000001,

   /// <summary>
   /// The property value is out of range. 
   /// </summary>
   WarningPropValueOutOfRange = 0xA0000002,

   /// <summary>
   /// The property value is not supported. 
   /// </summary>
   WarningPropValueNotSpted = 0xA0000003,

   /// <summary>
   /// The property value conflicts with the current state.
   /// </summary>
   WarningPropValueConflict = 0xA0000004,

   /// <summary>
   /// The value range of all channels in a group should be same, 
   /// such as 4~20mA of PCI-1724.
   /// </summary>
   WarningVrgOfGroupNotSame = 0xA0000005,

   /// <summary>
   /// Some properties of a property set are failed to be written into device.
   /// 
   /// </summary>
   WarningPropPartialFailed = 0xA0000006,

   /// <summary>
   /// The operation had been stopped.
   /// 
   /// </summary>
   WarningFuncStopped = 0xA0000007,

   /// <summary>
   /// The operation is time-out.
   /// 
   /// </summary>
   WarningFuncTimeout = 0xA0000008,

   /// <summary>
   /// The cache is over-run.
   /// 
   /// </summary>
   WarningCacheOverflow = 0xA0000009,

   /// <summary>
   /// The channel is burn-out.
   /// 
   /// </summary>
   WarningBurnout = 0xA000000A,

   /// <summary>
   /// The current data record is end.
   /// 
   /// </summary>
   WarningRecordEnd = 0xA000000B,

   /// <summary>
   /// The specified profile is not valid. 
   /// 
   /// </summary>
   WarningProfileNotValid = 0xA000000C,


   ///***********************************************************************
   /// error                                                                
   ///***********************************************************************
   /// <summary>
   /// The handle is NULL or its type doesn't match the required operation. 
   /// </summary>
   ErrorHandleNotValid = 0xE0000000,

   /// <summary>
   /// The parameter value is out of range.
   /// </summary>
   ErrorParamOutOfRange = 0xE0000001,

   /// <summary>
   /// The parameter value is not supported.
   /// </summary>
   ErrorParamNotSpted = 0xE0000002,

   /// <summary>
   /// The parameter value format is not the expected. 
   /// </summary>
   ErrorParamFmtUnexpted = 0xE0000003,

   /// <summary>
   /// Not enough memory is available to complete the operation. 
   /// </summary>
   ErrorMemoryNotEnough = 0xE0000004,

   /// <summary>
   /// The data buffer is null. 
   /// </summary>
   ErrorBufferIsNull = 0xE0000005,

   /// <summary>
   /// The data buffer is too small for the operation. 
   /// </summary>
   ErrorBufferTooSmall = 0xE0000006,

   /// <summary>
   /// The data length exceeded the limitation. 
   /// </summary>
   ErrorDataLenExceedLimit = 0xE0000007,

   /// <summary>
   /// The required function is not supported. 
   /// </summary>
   ErrorFuncNotSpted = 0xE0000008,

   /// <summary>
   /// The required event is not supported. 
   /// </summary>
   ErrorEventNotSpted = 0xE0000009,

   /// <summary>
   /// The required property is not supported. 
   /// </summary>
   ErrorPropNotSpted = 0xE000000A, 

   /// <summary>
   /// The required property is read-only. 
   /// </summary>
   ErrorPropReadOnly = 0xE000000B,

   /// <summary>
   /// The specified property value conflicts with the current state.
   /// </summary>
   ErrorPropValueConflict = 0xE000000C,

   /// <summary>
   /// The specified property value is out of range.
   /// </summary>
   ErrorPropValueOutOfRange = 0xE000000D,

   /// <summary>
   /// The specified property value is not supported. 
   /// </summary>
   ErrorPropValueNotSpted = 0xE000000E,

   /// <summary>
   /// The handle hasn't own the privilege of the operation the user wanted. 
   /// </summary>
   ErrorPrivilegeNotHeld = 0xE000000F,

   /// <summary>
   /// The required privilege is not available because someone else had own it. 
   /// </summary>
   ErrorPrivilegeNotAvailable = 0xE0000010,

   /// <summary>
   /// The driver of specified device was not found. 
   /// </summary>
   ErrorDriverNotFound = 0xE0000011,

   /// <summary>
   /// The driver version of the specified device mismatched. 
   /// </summary>
   ErrorDriverVerMismatch = 0xE0000012,

   /// <summary>
   /// The loaded driver count exceeded the limitation. 
   /// </summary>
   ErrorDriverCountExceedLimit = 0xE0000013,

   /// <summary>
   /// The device is not opened. 
   /// </summary>
   ErrorDeviceNotOpened = 0xE0000014,      

   /// <summary>
   /// The required device does not exist. 
   /// </summary>
   ErrorDeviceNotExist = 0xE0000015,

   /// <summary>
   /// The required device is unrecognized by driver. 
   /// </summary>
   ErrorDeviceUnrecognized = 0xE0000016,

   /// <summary>
   /// The configuration data of the specified device is lost or unavailable. 
   /// </summary>
   ErrorConfigDataLost = 0xE0000017,

   /// <summary>
   /// The function is not initialized and can't be started. 
   /// </summary>
   ErrorFuncNotInited = 0xE0000018,

   /// <summary>
   /// The function is busy. 
   /// </summary>
   ErrorFuncBusy = 0xE0000019,

   /// <summary>
   /// The interrupt resource is not available. 
   /// </summary>
   ErrorIntrNotAvailable = 0xE000001A,

   /// <summary>
   /// The DMA channel is not available. 
   /// </summary>
   ErrorDmaNotAvailable = 0xE000001B,

   /// <summary>
   /// Time out when reading/writing the device. 
   /// </summary>
   ErrorDeviceIoTimeOut = 0xE000001C,

   /// <summary>
   /// The given signature does not match with the device current one.
   /// </summary>
   ErrorSignatureNotMatch = 0xE000001D,

   /// <summary>
   /// The function cannot be executed while the buffered AI is running.
   /// </summary>
   ErrorFuncConflictWithBfdAi = 0xE000001E,

   /// <summary>
   /// The value range is not available in single-ended mode.
   /// </summary>
   ErrorVrgNotAvailableInSeMode = 0xE000001F,

   /// <summary>
   /// The value range is not available in 50omh input impedance mode..
   /// </summary>
   ErrorVrgNotAvailableIn50ohmMode  = 0xE0000020,

   /// <summary>
   /// The coupling type is not available in 50omh input impedance mode..
   /// </summary>
   ErrorCouplingNotAvailableIn50ohmMode  = 0xE0000021,

   /// <summary>
   /// The coupling type is not available in IEPE mode.
   /// </summary>
   ErrorCouplingNotAvailableInIEPEMode  = 0xE0000022,

   /// <summary>
   /// The Communication is failed when reading/writing the device.
   /// </summary>
   ErrorDeviceCommunicationFailed  = 0xE0000023,

   /// <summary>
   /// The device's 'fix number' conflicted with other device's
   /// </summary>
   ErrorFixNumberConflict = 0xE0000024,

   /// <summary>
   /// The Trigger source conflicted with other trigger configuration
   /// </summary>
   ErrorTrigSrcConflict = 0xE0000025,
	
   /// <summary>
   /// All properties of a property set are failed to be written into device.
   /// </summary>
   ErrorPropAllFailed = 0xE0000026,

   /// <summary>
   /// Undefined error 
   /// </summary>
   ErrorUndefined = 0xE000FFFF,
} ErrorCode;

// Advantech CardType ID 
typedef enum ProductId {
   BD_DEMO   = 0x00,       // demo board
   BD_PCL818 = 0x05,       // PCL-818 board
   BD_PCL818H = 0x11,      // PCL-818H
   BD_PCL818L = 0x21,      // PCL-818L
   BD_PCL818HG = 0x22,     // PCL-818HG
   BD_PCL818HD = 0x2b,     // PCL-818HD
   BD_PCM3718 = 0x37,      // PCM-3718
   BD_PCM3724 = 0x38,      // PCM-3724
   BD_PCM3730 = 0x5a,      // PCM-3730
   BD_PCI1750 = 0x5e,      // PCI-1750
   BD_PCI1751 = 0x5f,      // PCI-1751
   BD_PCI1710 = 0x60,      // PCI-1710
   BD_PCI1712 = 0x61,      // PCI-1712
   BD_PCI1710HG = 0x67,    // PCI-1710HG
   BD_PCI1711 = 0x73,      // PCI-1711
   BD_PCI1711L = 0x75,     // PCI-1711L 
   BD_PCI1713 = 0x68,      // PCI-1713
   BD_PCI1753 = 0x69,      // PCI-1753
   BD_PCI1760 = 0x6a,      // PCI-1760
   BD_PCI1720 = 0x6b,      // PCI-1720
   BD_PCM3718H = 0x6d,     // PCM-3718H
   BD_PCM3718HG = 0x6e,    // PCM-3718HG
   BD_PCI1716 = 0x74,      // PCI-1716
   BD_PCI1731 = 0x75,      // PCI-1731
   BD_PCI1754 = 0x7b,      // PCI-1754
   BD_PCI1752 = 0x7c,      // PCI-1752
   BD_PCI1756 = 0x7d,      // PCI-1756
   BD_PCM3725 = 0x7f,      // PCM-3725
   BD_PCI1762 = 0x80,      // PCI-1762
   BD_PCI1721 = 0x81,      // PCI-1721
   BD_PCI1761 = 0x82,      // PCI-1761
   BD_PCI1723 = 0x83,      // PCI-1723
   BD_PCI1730 = 0x87,      // PCI-1730
   BD_PCI1733 = 0x88,      // PCI-1733
   BD_PCI1734 = 0x89,      // PCI-1734
   BD_PCI1710L = 0x90,     // PCI-1710L
   BD_PCI1710HGL = 0x91,   // PCI-1710HGL
   BD_PCM3712 = 0x93,      // PCM-3712
   BD_PCM3723 = 0x94,      // PCM-3723
   BD_PCI1780 = 0x95,      // PCI-1780
   BD_MIC3756 = 0x96,      // MIC-3756
   BD_PCI1755 = 0x97,      // PCI-1755
   BD_PCI1714 = 0x98,      // PCI-1714
   BD_PCI1757 = 0x99,      // PCI-1757
   BD_MIC3716 = 0x9A,      // MIC-3716
   BD_MIC3761 = 0x9B,      // MIC-3761
   BD_MIC3753 = 0x9C,      // MIC-3753
   BD_MIC3780 = 0x9D,      // MIC-3780
   BD_PCI1724 = 0x9E,      // PCI-1724
   BD_PCI1758UDI = 0xA3,   // PCI-1758UDI
   BD_PCI1758UDO = 0xA4,   // PCI-1758UDO
   BD_PCI1747 = 0xA5,      // PCI-1747
   BD_PCM3780 = 0xA6,      // PCM-3780 
   BD_MIC3747 = 0xA7,      // MIC-3747
   BD_PCI1758UDIO = 0xA8,  // PCI-1758UDIO
   BD_PCI1712L = 0xA9,     // PCI-1712L
   BD_PCI1763UP = 0xAC,    // PCI-1763UP
   BD_PCI1736UP = 0xAD,    // PCI-1736UP
   BD_PCI1714UL = 0xAE,    // PCI-1714UL
   BD_MIC3714 = 0xAF,      // MIC-3714
   BD_PCM3718HO = 0xB1,    // PCM-3718HO
   BD_PCI1741U = 0xB3,     // PCI-1741U
   BD_MIC3723 = 0xB4,      // MIC-3723 
   BD_PCI1718HDU = 0xB5,   // PCI-1718HDU
   BD_MIC3758DIO = 0xB6,   // MIC-3758DIO
   BD_PCI1727U = 0xB7,     // PCI-1727U
   BD_PCI1718HGU = 0xB8,   // PCI-1718HGU
   BD_PCI1715U = 0xB9,     // PCI-1715U
   BD_PCI1716L = 0xBA,     // PCI-1716L
   BD_PCI1735U = 0xBB,     // PCI-1735U
   BD_USB4711 = 0xBC,      // USB4711
   BD_PCI1737U = 0xBD,     // PCI-1737U
   BD_PCI1739U = 0xBE,     // PCI-1739U
   BD_PCI1742U = 0xC0,     // PCI-1742U
   BD_USB4718 = 0xC6,      // USB-4718
   BD_MIC3755 = 0xC7,      // MIC3755
   BD_USB4761 = 0xC8,      // USB4761
   BD_PCI1784 = 0XCC,      // PCI-1784
   BD_USB4716 = 0xCD,      // USB4716
   BD_PCI1752U = 0xCE,     // PCI-1752U
   BD_PCI1752USO = 0xCF,   // PCI-1752USO
   BD_USB4751 = 0xD0,      // USB4751
   BD_USB4751L = 0xD1,     // USB4751L
   BD_USB4750 = 0xD2,      // USB4750
   BD_MIC3713 = 0xD3,      // MIC-3713
   BD_USB4711A = 0xD8,     // USB4711A
   BD_PCM3753P = 0xD9,     // PCM3753P
   BD_PCM3784  = 0xDA,     // PCM3784
   BD_PCM3761I = 0xDB,     // PCM-3761I
   BD_MIC3751  = 0xDC,     // MIC-3751
   BD_PCM3730I = 0xDD,     // PCM-3730I
   BD_PCM3813I = 0xE0,     // PCM-3813I
   BD_PCIE1744   = 0xE1,   // PCIE-1744
   BD_PCI1730U   = 0xE2,   // PCI-1730U
   BD_PCI1760U   = 0xE3,   // PCI-1760U
   BD_MIC3720   = 0xE4,    // MIC-3720
   BD_PCM3810I = 0xE9,     // PCM-3810I
   BD_USB4702  = 0xEA,     // USB4702
   BD_USB4704  = 0xEB,     // USB4704
   BD_PCM3810I_HG = 0xEC,  // PCM-3810I_HG
   BD_PCI1713U = 0xED,     // PCI-1713U 

   // !!!BioDAQ only Product ID starts from here!!!
   BD_PCI1706U   = 0x800,
   BD_PCI1706MSU = 0x801,
   BD_PCI1706UL  = 0x802,
   BD_PCIE1752   = 0x803,
   BD_PCIE1754   = 0x804,
   BD_PCIE1756   = 0x805,
   BD_MIC1911    = 0x806,
   BD_MIC3750    = 0x807,
   BD_MIC3711    = 0x808,
   BD_PCIE1730   = 0x809,
   BD_PCI1710_ECU = 0x80A,
   BD_PCI1720_ECU = 0x80B,
   BD_PCIE1760   = 0x80C,
   BD_PCIE1751   = 0x80D,
   BD_ECUP1706   = 0x80E,
   BD_PCIE1753   = 0x80F,
   BD_PCIE1810   = 0x810,
   BD_ECUP1702L  = 0x811,
   BD_PCIE1816   = 0x812,
   BD_PCM27D24DI = 0x813,
   BD_PCIE1816H  = 0x814,
   BD_PCIE1840   = 0x815,
   BD_PCL725     = 0x816,
   BD_PCI176E    = 0x817,
   BD_PCIE1802   = 0x818,
   BD_AIISE730   = 0x819,
   BD_PCIE1812   = 0x81A,
   BD_MIOe3810   = 0x81B,
   BD_PCIE1802L  = 0x81C,
   BD_PCIE1813   = 0x81D,
   BD_PCIE1840L  = 0x81E,
   BD_PCIE1730H  = 0x81F,
   BD_PCIE1756H  = 0x820,
   BD_PCIERXM01  = 0x821,          // PCIe-RXM01
   BD_MIOE3816   = 0x822,
	BD_USB5830    = 0x823,
	BD_USB5850    = 0x824,
	BD_USB5860    = 0x825,
   BD_VPX1172    = 0x826,
	BD_USB5855		= 0x827,
	BD_USB5856		= 0x828,
	BD_USB5862		= 0x829,


} ProductId;

typedef struct MathInterval {
   int32  Type; 
   double Min;
   double Max;
} MathInterval, * PMathInterval;
typedef MathInterval const *PCMathInterval;

typedef struct MapFuncPiece {
   int32  Size;         /* the size of the structure  */
   int32  Degree;       /* the polynomial degree */
   double UpperLimit;   /* the upper limit for this scaling polynomial. */
   double Coef[2];      /* variable length array for the coefficient of polynomial, in increasing degree. */
} MapFuncPiece, * PMapFuncPiece;
typedef MapFuncPiece const *PCMapFuncPiece;

typedef struct DataMark {
   int64 DataIndex;
   int32 SrcId;
   int32 _reserved_;
} DataMark;

typedef struct CalibrationData {
   int32 Size;
   int32 ShuntResistance;
}CalibrationData;

END_NAMEAPCE_AUTOMATION_BDAQ

#endif /*_BDAQ_TYPES_DEFINED*/

/**********************************************************/
/*Bionic DAQ COM style class library                      */
/**********************************************************/
#if !defined(_BDAQ_TYPES_ONLY) && !defined(_BDAQ_COM_STYLE_CLASS_LIB)
#define _BDAQ_COM_STYLE_CLASS_LIB

#include <stdlib.h>
#if defined(_WIN32) || defined(WIN32)
#   include <windows.h>
#endif

#ifdef _CVI_
#	define __inline static
#endif

BEGIN_NAMEAPCE_AUTOMATION_BDAQ

/**********************************************************/
/*CSCL types definition                                   */
/**********************************************************/
typedef enum ControlState {
   Idle = 0,
   Ready,
   Running,
   Stopped,
   Uninited = -1,
} ControlState;

typedef enum Scenario {
   SceInstantAi    = 1 << 0,
   SceBufferedAi   = 1 << 1,
   SceWaveformAi   = 1 << 2,
   SceInstantAo    = 1 << 3,
   SceBufferedAo   = 1 << 4,
   SceInstantDi    = 1 << 5,
   SceInstantDo    = 1 << 6,
   SceEventCounter = 1 << 7,
   SceFreqMeter    = 1 << 8,
   SceOneShot      = 1 << 9,
   SceTimerPulse   = 1 << 10,
   ScePwMeter      = 1 << 11,
   ScePwModulator  = 1 << 12,
   SceUdCounter    = 1 << 13,
   SceBufferedEventCounter = 1 << 14,
   SceBufferedPwMeter      = 1 << 15,
   SceBufferedPwModulator  = 1 << 16,
   SceBufferedUdCounter    = 1 << 17,
} Scenario;

typedef struct DeviceInformation{
   int32      DeviceNumber;
   AccessMode DeviceMode;
   int32      ModuleIndex;
   wchar_t    Description[MAX_DEVICE_DESC_LEN]; 

#if defined(__cplusplus)
   explicit DeviceInformation(int32 deviceNumber = -1, AccessMode mode = ModeWrite, int32 moduleIndex = 0)
   {
      Init(deviceNumber, NULL, mode, moduleIndex);
   }
   explicit DeviceInformation(wchar_t const *deviceDesc, AccessMode mode = ModeWrite, int32 moduleIndex = 0)
   {
      Init(-1, deviceDesc, mode, moduleIndex);
   }
   void Init(int32 deviceNumber, wchar_t const *deviceDesc, AccessMode mode, int32 moduleIndex)
   {
      DeviceNumber = deviceNumber;
      DeviceMode   = mode;
      ModuleIndex  = moduleIndex;
      if (deviceDesc == NULL) Description[0] = L'\0';
      else {
         for (int i = 0; i < MAX_DEVICE_DESC_LEN && (Description[i] = *deviceDesc++) != 0; ++i){}
         Description[MAX_DEVICE_DESC_LEN - 1] = L'\0';
      }
   }
#endif
} DeviceInformation;

typedef struct DeviceTreeNode {
   int32      DeviceNumber;
   int32      ModulesIndex[8];
   wchar_t    Description[MAX_DEVICE_DESC_LEN];
} DeviceTreeNode;

typedef struct DeviceEventArgs {
   EventId Id;
} DeviceEventArgs; 

typedef struct BfdAiEventArgs {
   EventId Id;
   int32   Offset;    /*offset of the new data*/
   int32   Count;     /*amount of the new data*/
   int32   MarkCount; /*amount of the new marks*/
} BfdAiEventArgs;

typedef struct BfdAoEventArgs {
   EventId Id;
   int32   Offset; /*offset of blank area*/
   int32   Count;  /*amount of blank area*/
} BfdAoEventArgs;

typedef struct DiSnapEventArgs {
   EventId Id;
   int32   SrcNum;
   int32   Length;
   uint8   PortData[MAX_DIO_PORT_COUNT];
} DiSnapEventArgs;

typedef struct CntrEventArgs {
   EventId Id;
   int32   Channel;
} CntrEventArgs;

typedef struct UdCntrEventArgs {
   EventId Id;
   int32   Length;
   int32   Data[MAX_CNTR_CH_COUNT];
} UdCntrEventArgs;

typedef struct BfdCntrEventArgs {
   EventId Id;
   int32   Channel;
   int32   Offset;    /*offset of the new data*/
   int32   Count;     /*amount of the new data*/
} BfdCntrEventArgs;

typedef struct PulseWidth {
   double HiPeriod;
   double LoPeriod;
} PulseWidth;

typedef void (BDAQCALL * DAQ_FN)(void *);
typedef struct DaqNaviLib {
   int32 Version;
   int32 Revision;

   DAQ_FN const * Global;
   DAQ_FN const * Base;
   DAQ_FN const * Ai;
   DAQ_FN const * Ao;
   DAQ_FN const * Dio;
   DAQ_FN const * Cntr;
} DaqNaviLib;

typedef struct IArray IArray;
typedef struct ICounterIndexer ICounterIndexer;

#if !defined(__cplusplus) || defined(_BDAQ_C_INTERFACE)
typedef struct ScanChannel       ScanChannel;
typedef struct ConvertClock      ConvertClock;
typedef struct ScanClock         ScanClock;
typedef struct Trigger           Trigger;
typedef struct Conversion        Conversion;
typedef struct Record            Record;
typedef struct NosFltChannel     NosFltChannel;
typedef struct DeviceCtrl        DeviceCtrl;
typedef struct DaqCtrlBase       DaqCtrlBase;

typedef struct CjcSetting        CjcSetting;
typedef struct AiChannel         AiChannel;
typedef struct AiFeatures        AiFeatures;
typedef struct AiCtrlBase        AiCtrlBase;
typedef struct InstantAiCtrl     InstantAiCtrl;
typedef struct BufferedAiCtrl    BufferedAiCtrl;
typedef struct WaveformAiCtrl    WaveformAiCtrl;

typedef struct AoChannel         AoChannel;
typedef struct AoFeatures        AoFeatures;
typedef struct AoCtrlBase        AoCtrlBase;
typedef struct InstantAoCtrl     InstantAoCtrl;
typedef struct BufferedAoCtrl    BufferedAoCtrl;

typedef struct DioPort           DioPort;
typedef struct DiintChannel      DiintChannel;
typedef struct DiCosintPort      DiCosintPort;
typedef struct DiPmintPort       DiPmintPort;
typedef struct DioFeatures       DioFeatures;
typedef struct DioCtrlBase       DioCtrlBase;
typedef struct InstantDiCtrl     InstantDiCtrl;
typedef struct InstantDoCtrl     InstantDoCtrl;

typedef struct CntrFeatures      CntrFeatures;
typedef struct CntrCtrlBase      CntrCtrlBase;
typedef struct EcChannel         EcChannel;
typedef struct EventCounterCtrl  EventCounterCtrl;
typedef struct FmChannel         FmChannel;
typedef struct FreqMeterCtrl     FreqMeterCtrl;
typedef struct OsChannel         OsChannel;
typedef struct OneShotCtrl       OneShotCtrl;
typedef struct TmrChannel        TmrChannel;
typedef struct TimerPulseCtrl    TimerPulseCtrl;
typedef struct PiChannel         PiChannel;
typedef struct PwMeterCtrl       PwMeterCtrl;
typedef struct PoChannel         PoChannel;
typedef struct PwModulatorCtrl   PwModulatorCtrl;
typedef struct UdChannel         UdChannel;
typedef struct UdCounterCtrl     UdCounterCtrl;
typedef struct BufferedEventCounterCtrl  BufferedEventCounterCtrl;
typedef struct BufferedPwMeterCtrl       BufferedPwMeterCtrl;
typedef struct BufferedUdCounterCtrl     BufferedUdCounterCtrl;
typedef struct BufferedPwModulatorCtrl   BufferedPwModulatorCtrl;

#else

class ScanChannel;
class ConvertClock;
class ScanClock;
class Trigger;
class Conversion;
class Record;
class NosFltChannel;
class DeviceCtrl;
class DaqCtrlBase;
              
class CjcSetting;
class AiChannel;
class AiFeatures;
class AiCtrlBase;
class InstantAiCtrl;
class BufferedAiCtrl;
class WaveformAiCtrl;
                              
class AoChannel;
class AoFeatures;
class AoCtrlBase;
class InstantAoCtrl;
class BufferedAoCtrl;
                              
class DioPort;
class DiintChannel;
class DiCosintPort;
class DiPmintPort;
class DioFeatures;
class DioCtrlBase;
class InstantDiCtrl;
class InstantDoCtrl;
                              
class CntrFeatures;
class CntrCtrlBase;
class EcChannel;
class EventCounterCtrl;
class FmChannel;
class FreqMeterCtrl;
class OsChannel;
class OneShotCtrl;
class TmrChannel;
class TimerPulseCtrl;
class PiChannel;
class PwMeterCtrl;
class PoChannel;
class PwModulatorCtrl;
class UdChannel;
class UdCounterCtrl;
class BufferedEventCounterCtrl;
class BufferedPwMeterCtrl;
class BufferedUdCounterCtrl;
class BufferedPwModulatorCtrl;
#endif

/*Callback function prototype*/
typedef void (BDAQCALL * DeviceEventProc)(void  *sender, DeviceEventArgs *args, void *userParam);
typedef void (BDAQCALL * BfdAiEventProc )(void  *sender, BfdAiEventArgs  *args, void *userParam);
typedef void (BDAQCALL * BfdAoEventProc )(void  *sender, BfdAoEventArgs  *args, void *userParam);
typedef void (BDAQCALL * DiSnapEventProc)(void  *sender, DiSnapEventArgs *args, void *userParam);
typedef void (BDAQCALL * CntrEventProc  )(void  *sender, CntrEventArgs   *args, void *userParam);
typedef void (BDAQCALL * UdCntrEventProc)(void  *sender, UdCntrEventArgs *args, void *userParam);
typedef void (BDAQCALL * BfdCntrEventProc)(void *sender, BfdCntrEventArgs *args, void *userParam);

/**********************************************************/
/*                                                        */
/*              DAQNAVI CSCL ANSI-C APIs                  */
/*                                                        */
/**********************************************************/
#ifndef DAQNAVI_CORE

/* ----------------------------------------------------------*/
/* Helper Macros & Methods                                   */
/* ----------------------------------------------------------*/
   __inline DaqNaviLib const ** DNL_PPtr()
   {
      static DaqNaviLib const * lib = NULL;
      return &lib;
   }
#if defined(_WIN32) || defined(WIN32)
   __inline HMODULE DNL_Instance()
   {
      static HMODULE instance = NULL;
      if (instance == NULL) { 
         instance = LoadLibrary(TEXT("biodaq.dll")); 
      }
      return instance;
   }
   __inline int32 DNL_Initialize()
   {
      if (*DNL_PPtr() == NULL) {
   #ifdef _WIN32_WCE
      FARPROC fn = GetProcAddressA(DNL_Instance(), "AdxDaqNaviLibInitialize");
   #else
      FARPROC fn = GetProcAddress(DNL_Instance(), "AdxDaqNaviLibInitialize");
   #endif
      *DNL_PPtr() = ((DaqNaviLib const *( BDAQCALL *)(void))fn)();
      }
      return *DNL_PPtr() != NULL;
   }
#else /*Non-Win32*/
   #ifdef __cplusplus
   extern "C" {
   #endif
      DaqNaviLib const * AdxDaqNaviLibInitialize();
   #ifdef __cplusplus
   }
   #endif

   __inline int32 DNL_Initialize()
   {
      if (*DNL_PPtr() == NULL) { *DNL_PPtr() = AdxDaqNaviLibInitialize(); }
      return *DNL_PPtr() != NULL;
   }
#endif

#define DNLAPI_GBL                                         (*DNL_PPtr())->Global
#define DNLAPI_BASE                                        (*DNL_PPtr())->Base
#define DNLAPI_AI                                          (*DNL_PPtr())->Ai
#define DNLAPI_AO                                          (*DNL_PPtr())->Ao
#define DNLAPI_DIO                                         (*DNL_PPtr())->Dio
#define DNLAPI_CNTR                                        (*DNL_PPtr())->Cntr

#define DAQFN_VV(Apis,   n)                                ((void      (BDAQCALL *)(void *))Apis[n])
#define DAQFN_VT(Apis,   n, t1)                            ((void      (BDAQCALL *)(void *, t1))Apis[n])
#define DAQFN_VTT(Apis,  n, t1, t2)                        ((void      (BDAQCALL *)(void *, t1, t2))Apis[n])
#define DAQFN_VTTT(Apis, n, t1, t2, t3)                    ((void      (BDAQCALL *)(void *, t1, t2, t3))Apis[n])
#define DAQFN_TV(Apis,   n, rt)                            ((rt        (BDAQCALL *)(void *))Apis[n])
#define DAQFN_TT(Apis,   n, rt, t1)                        ((rt        (BDAQCALL *)(void *, t1))Apis[n])
#define DAQFN_TTT(Apis,  n, rt, t1, t2)                    ((rt        (BDAQCALL *)(void *, t1, t2))Apis[n])
#define DAQFN_EV(Apis,   n)                                ((ErrorCode (BDAQCALL *)(void *))Apis[n])
#define DAQFN_ET(Apis, n, t1)                              ((ErrorCode (BDAQCALL *)(void *, t1))Apis[n])
#define DAQFN_ETT(Apis, n, t1, t2)                         ((ErrorCode (BDAQCALL *)(void *, t1, t2))Apis[n])
#define DAQFN_ETTT(Apis, n, t1, t2, t3)                    ((ErrorCode (BDAQCALL *)(void *, t1, t2, t3))Apis[n])
#define DAQFN_ETTTT(Apis, n, t1, t2, t3, t4)               ((ErrorCode (BDAQCALL *)(void *, t1, t2, t3, t4))Apis[n])
#define DAQFN_ETTTTT(Apis, n, t1, t2, t3, t4, t5)          ((ErrorCode (BDAQCALL *)(void *, t1, t2, t3, t4, t5))Apis[n])
#define DAQFN_GBL_T(Apis, n, rt)                           ((rt        (BDAQCALL *)())Apis[n])
#define DAQFN_GBL_TT(Apis, n, rt, t1)                      ((rt        (BDAQCALL *)(t1))Apis[n])
#define DAQFN_GBL_TTT(Apis, n, rt, t1, t2)                 ((rt        (BDAQCALL *)(t1, t2))Apis[n])
#define DAQFN_GBL_TTTT(Apis, n, rt, t1, t2, t3)            ((rt        (BDAQCALL *)(t1, t2, t3))Apis[n])
#define DAQFN_GBL_ETTT(Apis, n, t1, t2, t3)                ((ErrorCode (BDAQCALL *)(t1, t2, t3))Apis[n])
#define DAQFN_GBL_ETTTT(Apis, n, t1, t2, t3, t4)           ((ErrorCode (BDAQCALL *)(t1, t2, t3, t4))Apis[n])
#define DAQFN_GBL_ETTTTT(Apis, n, t1, t2, t3, t4, t5)      ((ErrorCode (BDAQCALL *)(t1, t2, t3, t4, t5))Apis[n])
#define DAQFN_GBL_ETTTTTT(Apis, n, t1, t2, t3, t4, t5, t6) ((ErrorCode (BDAQCALL *)(t1, t2, t3, t4, t5, t6))Apis[n])

/* ----------------------------------------------------------*/
/*Global APIs                                                */
/* ----------------------------------------------------------*/
__inline ErrorCode AdxGetValueRangeInformation(ValueRange vrg, int32 sizeofDesc, wchar_t *description, MathInterval *range, ValueUnit *unit)
{
   DNL_Initialize(); 
   return DAQFN_GBL_ETTTTT(DNLAPI_GBL, 0, ValueRange, int32, wchar_t *, MathInterval *, ValueUnit *)(vrg, sizeofDesc, description, range, unit);
}
__inline ErrorCode AdxGetSignalConnectionInformation(SignalDrop signalType, int32 sizeofDesc, wchar_t *description, SignalPosition *position)
{
   DNL_Initialize(); 
   return DAQFN_GBL_ETTTT(DNLAPI_GBL, 1, SignalDrop, int32, wchar_t *, SignalPosition *)(signalType, sizeofDesc, description, position);
}
__inline double AdxTranslateTemperatureScale(TemperatureDegree degreeType, double degreeCelsius )
{
   DNL_Initialize(); 
   return DAQFN_GBL_TTT(DNLAPI_GBL, 2, double, TemperatureDegree, double)(degreeType, degreeCelsius);
}
__inline ErrorCode AdxEnumToString(wchar_t const *enumName, int32 enumValue, int32 strLength, wchar_t *strBuffer)
{
   DNL_Initialize(); 
   return DAQFN_GBL_ETTTT(DNLAPI_GBL, 3, wchar_t const *, int32, int32, wchar_t *)(enumName, enumValue, strLength, strBuffer);
}

__inline ErrorCode AdxStringToEnum(wchar_t const *enumName, wchar_t const *enumString, int32 *enumValue)
{
   DNL_Initialize(); 
   return DAQFN_GBL_ETTT(DNLAPI_GBL, 4, wchar_t const *, wchar_t const *, int32 *)(enumName, enumString, enumValue);
}
__inline ErrorCode AdxDeviceGetLinkageInfo(int32 parent, int32 index, int32 *devNumber, wchar_t *description, int32 *subCount)
{
   DNL_Initialize(); 
   return DAQFN_GBL_ETTTTT(DNLAPI_GBL, 5, int32, int32, int32 *, wchar_t *, int32 *)(parent, index, devNumber, description, subCount);
}
__inline ErrorCode AdxEventClearFlag(HANDLE handle, int32 id, int32 flagLParam, int32 flagRParam)
{  
   return DAQFN_GBL_ETTTT(DNLAPI_GBL, 6, HANDLE, int32, int32, int32)(handle, id, flagLParam, flagRParam);
}
/* ----------------------------------------------------------*/
/*common classes : Array                                     */
/* ----------------------------------------------------------*/
__inline void              Array_Dispose(IArray * obj)                                                                                    {        DAQFN_VV(DNLAPI_BASE, 0)(obj);                       }
__inline int32             Array_getLength(IArray * obj)                                                                                  { return DAQFN_TV(DNLAPI_BASE, 1, int32)(obj);                }
__inline void*             Array_getItem(IArray * obj, int32 index)                                                                       { return DAQFN_TT(DNLAPI_BASE, 2, void *, int32)(obj, index); }

/* ----------------------------------------------------------*/
/*common classes : ScanChannel                               */
/* ----------------------------------------------------------*/
__inline int32             ScanChannel_getChannelStart(ScanChannel *obj)                                                                  { return DAQFN_TV(DNLAPI_BASE, 3, int32)(obj);         }
__inline ErrorCode         ScanChannel_setChannelStart(ScanChannel *obj, int32 value)                                                     { return DAQFN_ET(DNLAPI_BASE, 4, int32)(obj, value);  }
__inline int32             ScanChannel_getChannelCount(ScanChannel *obj)                                                                  { return DAQFN_TV(DNLAPI_BASE, 5, int32)(obj);         }
__inline ErrorCode         ScanChannel_setChannelCount(ScanChannel *obj, int32 value)                                                     { return DAQFN_ET(DNLAPI_BASE, 6, int32)(obj, value);  }
__inline int32             ScanChannel_getIntervalCount(ScanChannel *obj)                                                                 { return DAQFN_TV(DNLAPI_BASE, 7, int32)(obj);         }
__inline ErrorCode         ScanChannel_setIntervalCount(ScanChannel *obj, int32 value)                                                    { return DAQFN_ET(DNLAPI_BASE, 8, int32)(obj, value);  }
__inline int32             ScanChannel_getSamples(ScanChannel *obj)                                                                       { return DAQFN_TV(DNLAPI_BASE, 9, int32)(obj);         }
__inline ErrorCode         ScanChannel_setSamples(ScanChannel *obj, int32 value)                                                          { return DAQFN_ET(DNLAPI_BASE, 10, int32)(obj, value); }

/* ----------------------------------------------------------*/
/*common classes : ConvertClock                              */
/* ----------------------------------------------------------*/
__inline SignalDrop        ConvertClock_getSource(ConvertClock *obj)                                                                      { return DAQFN_TV(DNLAPI_BASE, 11, SignalDrop)(obj);       }
__inline ErrorCode         ConvertClock_setSource(ConvertClock *obj, SignalDrop value)                                                    { return DAQFN_ET(DNLAPI_BASE, 12, SignalDrop)(obj, value);}
__inline double            ConvertClock_getRate(ConvertClock *obj)                                                                        { return DAQFN_TV(DNLAPI_BASE, 13, double)(obj);           }
__inline ErrorCode         ConvertClock_setRate(ConvertClock *obj, double value)                                                          { return DAQFN_ET(DNLAPI_BASE, 14, double)(obj, value);    }

/* ----------------------------------------------------------*/                                                                              
/*common classes : ScanClock                                 */                                                                              
/* ----------------------------------------------------------*/                                                                              
__inline SignalDrop        ScanClock_getSource(ScanClock *obj)                                                                            { return DAQFN_TV(DNLAPI_BASE, 15, SignalDrop)(obj);        }
__inline ErrorCode         ScanClock_setSource(ScanClock *obj, SignalDrop value)                                                          { return DAQFN_ET(DNLAPI_BASE, 16, SignalDrop)(obj, value); }
__inline double            ScanClock_getRate(ScanClock *obj)                                                                              { return DAQFN_TV(DNLAPI_BASE, 17, double)(obj);            }
__inline ErrorCode         ScanClock_setRate(ScanClock *obj, double value)                                                                { return DAQFN_ET(DNLAPI_BASE, 18, double)(obj, value);     }
__inline int32             ScanClock_getScanCount(ScanClock *obj)                                                                         { return DAQFN_TV(DNLAPI_BASE, 19, int32)(obj);             }
__inline ErrorCode         ScanClock_setScanCount(ScanClock *obj, int32 value)                                                            { return DAQFN_ET(DNLAPI_BASE, 20, int32)(obj, value);      }

/* ----------------------------------------------------------*/
/*common classes : Trigger                                   */
/* ----------------------------------------------------------*/
__inline SignalDrop        Trigger_getSource(Trigger *obj)                                                                                { return DAQFN_TV(DNLAPI_BASE, 21, SignalDrop)(obj);           }
__inline ErrorCode         Trigger_setSource(Trigger *obj, SignalDrop value)                                                              { return DAQFN_ET(DNLAPI_BASE, 22, SignalDrop)(obj, value);    }
__inline ActiveSignal      Trigger_getEdge(Trigger *obj)                                                                                  { return DAQFN_TV(DNLAPI_BASE, 23, ActiveSignal)(obj);         }
__inline ErrorCode         Trigger_setEdge(Trigger *obj, ActiveSignal value)                                                              { return DAQFN_ET(DNLAPI_BASE, 24, ActiveSignal)(obj, value);  }
__inline double            Trigger_getLevel(Trigger *obj)                                                                                 { return DAQFN_TV(DNLAPI_BASE, 25, double)(obj);               }
__inline ErrorCode         Trigger_setLevel(Trigger *obj, double value)                                                                   { return DAQFN_ET(DNLAPI_BASE, 26, double)(obj, value);        }
__inline TriggerAction     Trigger_getAction(Trigger *obj)                                                                                { return DAQFN_TV(DNLAPI_BASE, 27, TriggerAction)(obj);        }
__inline ErrorCode         Trigger_setAction(Trigger *obj, TriggerAction value)                                                           { return DAQFN_ET(DNLAPI_BASE, 28, TriggerAction)(obj, value); }
__inline int32             Trigger_getDelayCount(Trigger *obj)                                                                            { return DAQFN_TV(DNLAPI_BASE, 29, int32)(obj);                }
__inline ErrorCode         Trigger_setDelayCount(Trigger *obj, int32 value)                                                               { return DAQFN_ET(DNLAPI_BASE, 30, int32)(obj, value);         }
__inline double            Trigger_getHysteresisIndex(Trigger *obj)                                                                       { return DAQFN_TV(DNLAPI_BASE, 31, double)(obj);               }
__inline ErrorCode         Trigger_setHysteresisIndex(Trigger *obj, double value)                                                         { return DAQFN_ET(DNLAPI_BASE, 32, double)(obj, value);        }
__inline FilterType        Trigger_getFilterType(Trigger *obj)                                                                            { return DAQFN_TV(DNLAPI_BASE, 94, FilterType)(obj);           }
__inline ErrorCode         Trigger_setFilterType(Trigger *obj, FilterType value)                                                          { return DAQFN_ET(DNLAPI_BASE, 95, FilterType)(obj, value);    }
__inline double            Trigger_getFilterCutoffFreq(Trigger *obj)                                                                      { return DAQFN_TV(DNLAPI_BASE, 96, double)(obj);               }
__inline ErrorCode         Trigger_setFilterCutoffFreq(Trigger *obj, double value)                                                        { return DAQFN_ET(DNLAPI_BASE, 97, double)(obj, value);        }

/* ----------------------------------------------------------*/
/*common classes : Conversion                                */
/* ----------------------------------------------------------*/
__inline SignalDrop        Conversion_getClockSource(Conversion *obj)                                                                     { return DAQFN_TV(DNLAPI_BASE, 33, SignalDrop)(obj);            }
__inline ErrorCode         Conversion_setClockSource(Conversion *obj, SignalDrop value)                                                   { return DAQFN_ET(DNLAPI_BASE, 34, SignalDrop)(obj, value);     }
__inline double            Conversion_getClockRate(Conversion *obj)                                                                       { return DAQFN_TV(DNLAPI_BASE, 35, double)(obj);                }
__inline ErrorCode         Conversion_setClockRate(Conversion *obj, double value)                                                         { return DAQFN_ET(DNLAPI_BASE, 36, double)(obj, value);         }
__inline int32             Conversion_getChannelStart(Conversion *obj)                                                                    { return DAQFN_TV(DNLAPI_BASE, 37, int32)(obj);                 }
__inline ErrorCode         Conversion_setChannelStart(Conversion *obj, int32 value)                                                       { return DAQFN_ET(DNLAPI_BASE, 38, int32)(obj, value);          }
__inline int32             Conversion_getChannelCount(Conversion *obj)                                                                    { return DAQFN_TV(DNLAPI_BASE, 39, int32)(obj);                 }
__inline ErrorCode         Conversion_setChannelCount(Conversion *obj, int32 value)                                                       { return DAQFN_ET(DNLAPI_BASE, 40, int32)(obj, value);          }
__inline void              Conversion_getChannelMap(Conversion *obj, int32 count, int8 *chMap)                                            {        DAQFN_VTT(DNLAPI_BASE, 41, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         Conversion_setChannelMap(Conversion *obj, int32 count, int8 *chMap)                                            { return DAQFN_ETT(DNLAPI_BASE, 42, int32, int8 *)(obj, count, chMap); }

/* ----------------------------------------------------------*/
/*common classes : Record                                    */
/* ----------------------------------------------------------*/
__inline int32             Record_getSectionLength(Record *obj)                                                                           { return DAQFN_TV(DNLAPI_BASE, 43, int32)(obj);        }
__inline ErrorCode         Record_setSectionLength(Record *obj, int32 value)                                                              { return DAQFN_ET(DNLAPI_BASE, 44, int32)(obj, value); }
__inline int32             Record_getSectionCount(Record *obj)                                                                            { return DAQFN_TV(DNLAPI_BASE, 45, int32)(obj);        }
__inline ErrorCode         Record_setSectionCount(Record *obj, int32 value)                                                               { return DAQFN_ET(DNLAPI_BASE, 46, int32)(obj, value); }
__inline int32             Record_getCycles(Record *obj)                                                                                  { return DAQFN_TV(DNLAPI_BASE, 47, int32)(obj);        }
__inline ErrorCode         Record_setCycles(Record *obj, int32 value)                                                                     { return DAQFN_ET(DNLAPI_BASE, 48, int32)(obj, value); }
                                                                                                                                
/* ----------------------------------------------------------*/
/* common classes : NosFltChannel                            */
/* ----------------------------------------------------------*/
__inline int32             NosFltChannel_getChannel(NosFltChannel *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 49, int32)(obj);      }
__inline int8              NosFltChannel_getEnabled(NosFltChannel *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 50, int8)(obj);       }
__inline ErrorCode         NosFltChannel_setEnabled(NosFltChannel *obj, int8 value)                                                       { return DAQFN_ET(DNLAPI_BASE, 51, int8)(obj, value);}

/*************************************************************/
/* ctrl class : device ctrl class                            */
/* ----------------------------------------------------------*/
/* Event */
__inline void              DeviceCtrl_addEventHandler(DeviceCtrl *obj, EventId id, DeviceEventProc proc, void *userParam)                 { DAQFN_VTTT(DNLAPI_BASE, 52, EventId, DeviceEventProc, void *)(obj, id, proc, userParam);} 
__inline void              DeviceCtrl_removeEventHandler(DeviceCtrl *obj, EventId id, DeviceEventProc proc, void *userParam)              { DAQFN_VTTT(DNLAPI_BASE, 53, EventId, DeviceEventProc, void *)(obj, id, proc, userParam);}
/* Method */                                                                                                                                 
__inline ErrorCode         DeviceCtrl_Refresh(DeviceCtrl *obj)                                                                            { return DAQFN_EV(DNLAPI_BASE, 54)(obj); }
__inline ErrorCode         DeviceCtrl_ReadRegister(DeviceCtrl *obj, int32 space, int32 offset, int32 length, void *data)                  { return DAQFN_ETTTT(DNLAPI_BASE, 55, int32, int32, int32, void *)(obj, space, offset, length, data); }
__inline ErrorCode         DeviceCtrl_WriteRegister(DeviceCtrl *obj, int32 space, int32 offset, int32 length, void *data)                 { return DAQFN_ETTTT(DNLAPI_BASE, 56, int32, int32, int32, void *)(obj, space, offset, length, data); }
__inline ErrorCode         DeviceCtrl_ReadPrivateRegion(DeviceCtrl *obj, int32 signature, int32 length, uint8 *data)                      { return DAQFN_ETTT(DNLAPI_BASE, 57, int32, int32, uint8*)(obj, signature, length, data);    } 
__inline ErrorCode         DeviceCtrl_WritePrivateRegion(DeviceCtrl *obj, int32 signature, int32 length, uint8 *data)                     { return DAQFN_ETTT(DNLAPI_BASE, 58, int32, int32, uint8*)(obj, signature, length, data);    }
/* Property */                                                                                                                                                                                                                             
__inline int32             DeviceCtrl_getDeviceNumber(DeviceCtrl *obj)                                                                    { return DAQFN_TV(DNLAPI_BASE, 59, int32)(obj);                          }                                            
__inline ErrorCode         DeviceCtrl_getDescription(DeviceCtrl *obj, int32 length, wchar_t *desc)                                        { return DAQFN_ETT(DNLAPI_BASE, 60, int32, wchar_t *)(obj, length, desc);}
__inline ErrorCode         DeviceCtrl_setDescription(DeviceCtrl *obj, int32 length, wchar_t *desc)                                        { return DAQFN_ETT(DNLAPI_BASE, 76, int32, wchar_t *)(obj, length, desc);}
__inline AccessMode        DeviceCtrl_getAccessMode(DeviceCtrl *obj)                                                                      { return DAQFN_TV(DNLAPI_BASE, 61, AccessMode)(obj);                     }
__inline ProductId         DeviceCtrl_getProductId(DeviceCtrl *obj)                                                                       { return DAQFN_TV(DNLAPI_BASE, 62, ProductId)(obj);                      }
__inline int32             DeviceCtrl_getBoardId(DeviceCtrl *obj)                                                                         { return DAQFN_TV(DNLAPI_BASE, 63, int32)(obj);                          }
__inline ErrorCode         DeviceCtrl_setBoardId(DeviceCtrl *obj, int32 value)                                                            { return DAQFN_ET(DNLAPI_BASE, 77, int32)(obj, value);                   }
__inline ErrorCode         DeviceCtrl_getBoardVersion(DeviceCtrl *obj, int32 length, wchar_t *ver)                                        { return DAQFN_ETT(DNLAPI_BASE, 64, int32, wchar_t *)(obj, length, ver); }
__inline ErrorCode         DeviceCtrl_getDriverVersion(DeviceCtrl *obj, int32 length, wchar_t *ver)                                       { return DAQFN_ETT(DNLAPI_BASE, 65, int32, wchar_t *)(obj, length, ver); }
__inline ErrorCode         DeviceCtrl_getDllVersion(DeviceCtrl *obj, int32 length, wchar_t *ver)                                          { return DAQFN_ETT(DNLAPI_BASE, 66, int32, wchar_t *)(obj, length, ver); }
__inline ErrorCode         DeviceCtrl_getLocation(DeviceCtrl *obj, int32 length, wchar_t *loc)                                            { return DAQFN_ETT(DNLAPI_BASE, 67, int32, wchar_t *)(obj, length, loc); }
__inline int32             DeviceCtrl_getPrivateRegionLength(DeviceCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 68, int32)(obj);                          }
__inline int32             DeviceCtrl_getHotResetPreventable(DeviceCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 69, int32)(obj);                          }
__inline IArray *          DeviceCtrl_getBaseAddresses(DeviceCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 70, IArray *)(obj);                       }
__inline IArray *          DeviceCtrl_getInterrupts(DeviceCtrl *obj)                                                                      { return DAQFN_TV(DNLAPI_BASE, 71, IArray *)(obj);                       }
__inline IArray *          DeviceCtrl_getSupportedTerminalBoard(DeviceCtrl *obj)                                                          { return DAQFN_TV(DNLAPI_BASE, 72, IArray *)(obj);                       }
__inline IArray *          DeviceCtrl_getSupportedEvents(DeviceCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_BASE, 73, IArray *)(obj);                       } 
__inline TerminalBoard     DeviceCtrl_getTerminalBoard(DeviceCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 74, TerminalBoard)(obj);        }
__inline ErrorCode         DeviceCtrl_setTerminalBoard(DeviceCtrl *obj, TerminalBoard value)                                              { return DAQFN_ET(DNLAPI_BASE, 75, TerminalBoard)(obj, value); }
__inline ErrorCode         DeviceCtrl_setLocateEnabled(DeviceCtrl *obj, int32 value)                                                      { return DAQFN_ET(DNLAPI_BASE, 99, int32)(obj, value);         }
__inline int32             DeviceCtrl_getSupportedScenarios(DeviceCtrl *obj)                                                              { return DAQFN_TV(DNLAPI_BASE, 78, int32)(obj);                }
__inline HANDLE            DeviceCtrl_getDevice(DeviceCtrl *obj)                                                                          { return DAQFN_TV(DNLAPI_BASE, 92, HANDLE)(obj);               }   
__inline void              DeviceCtrl_Dispose(DeviceCtrl *obj)                                                                            {        DAQFN_VV(DNLAPI_BASE, 79)(obj);                       }
/*Creator*/
__inline DeviceCtrl *      DeviceCtrl_Create(int32 devNumber, wchar_t const *desc, AccessMode mode)                                       { DNL_Initialize(); return DAQFN_GBL_TTTT(DNLAPI_BASE, 80, DeviceCtrl *, int32, wchar_t const *, AccessMode)(devNumber, desc, mode); } 
__inline IArray *          DeviceCtrl_getInstalledDevices()                                                                               { DNL_Initialize(); return DAQFN_GBL_T(DNLAPI_BASE, 100, IArray *)(); } 

/*Helpers*/
__inline void              DeviceCtrl_addRemovedHandler(DeviceCtrl *obj, DeviceEventProc proc, void *userParam)                           { DeviceCtrl_addEventHandler   (obj, EvtDeviceRemoved,     proc, userParam); }
__inline void              DeviceCtrl_removeRemovedHandler(DeviceCtrl *obj, DeviceEventProc proc, void *userParam)                        { DeviceCtrl_removeEventHandler(obj, EvtDeviceRemoved,     proc, userParam); }
__inline void              DeviceCtrl_addReconnectedHandler(DeviceCtrl *obj, DeviceEventProc proc, void *userParam)                       { DeviceCtrl_addEventHandler   (obj, EvtDeviceReconnected, proc, userParam); }
__inline void              DeviceCtrl_removeReconnectedHandler(DeviceCtrl *obj, DeviceEventProc proc, void *userParam)                    { DeviceCtrl_removeEventHandler(obj, EvtDeviceReconnected, proc, userParam); }
__inline void              DeviceCtrl_addPropertyChangedHandler(DeviceCtrl *obj, DeviceEventProc proc, void *userParam)                   { DeviceCtrl_addEventHandler   (obj, EvtPropertyChanged,   proc, userParam); }
__inline void              DeviceCtrl_removePropertyChangedHandler(DeviceCtrl *obj, DeviceEventProc proc, void *userParam)                { DeviceCtrl_removeEventHandler(obj, EvtPropertyChanged,   proc, userParam); }

/*************************************************************/
/* ctrl class : daq ctrl base class                            */
/* ----------------------------------------------------------*/
/* Event */
typedef void (BDAQCALL * DaqEventProc)(void *sender, void *args, void *userParam);
__inline void              DaqCtrlBase_addEventHandler(DaqCtrlBase *obj, EventId id, DaqEventProc proc, void *userParam)                  { DAQFN_VTTT(DNLAPI_BASE, 81, EventId, DaqEventProc, void *)(obj, id, proc, userParam); } 
__inline void              DaqCtrlBase_removeEventHandler(DaqCtrlBase *obj, EventId id, DaqEventProc proc, void *userParam)               { DAQFN_VTTT(DNLAPI_BASE, 82, EventId, DaqEventProc, void *)(obj, id, proc, userParam); }
__inline DaqCtrlBase *     DaqCtrlBase_Create(Scenario type)                                                                              { DNL_Initialize(); return DAQFN_GBL_TT(DNLAPI_BASE, 91, DaqCtrlBase *, Scenario)(type);}  

/*************************************************************/
/* ctrl class : AI related                                   */
/* ----------------------------------------------------------*/
/* CjcSetting */
/* ----------------------------------------------------------*/
__inline int32             CjcSetting_getChannel(CjcSetting *obj)                                                                         { return DAQFN_TV(DNLAPI_AI, 0, int32) (obj);       }
__inline ErrorCode         CjcSetting_setChannel(CjcSetting *obj, int32 ch)                                                               { return DAQFN_ET(DNLAPI_AI, 1, int32) (obj, ch);   }
__inline double            CjcSetting_getValue(CjcSetting *obj)                                                                           { return DAQFN_TV(DNLAPI_AI, 2, double)(obj);       }
__inline ErrorCode         CjcSetting_setValue(CjcSetting *obj, double value)                                                             { return DAQFN_ET(DNLAPI_AI, 3, double)(obj, value);}
__inline double            CjcSetting_getUpdateFreq(CjcSetting *obj)                                                                      { return DAQFN_TV(DNLAPI_AI, 4, double)(obj);       }
__inline ErrorCode         CjcSetting_setUpdateFreq(CjcSetting *obj, double value)                                                        { return DAQFN_ET(DNLAPI_AI, 5, double)(obj, value);}

/* ----------------------------------------------------------*/
/* AiChannel                                                 */
/* ----------------------------------------------------------*/
__inline int32             AiChannel_getChannel(AiChannel *obj)                                                                           { return DAQFN_TV(DNLAPI_AI, 6, int32)(obj);                 }
__inline ValueRange        AiChannel_getValueRange(AiChannel *obj)                                                                        { return DAQFN_TV(DNLAPI_AI, 7, ValueRange)(obj);            }
__inline ErrorCode         AiChannel_setValueRange(AiChannel *obj, ValueRange value)                                                      { return DAQFN_ET(DNLAPI_AI, 8, ValueRange)(obj, value);     }
__inline AiSignalType      AiChannel_getSignalType(AiChannel *obj)                                                                        { return DAQFN_TV(DNLAPI_AI, 9,  AiSignalType)(obj);         }
__inline ErrorCode         AiChannel_setSignalType(AiChannel *obj, AiSignalType value)                                                    { return DAQFN_ET(DNLAPI_AI, 10, AiSignalType)(obj, value);  }
__inline BurnoutRetType    AiChannel_getBurnoutRetType(AiChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_AI, 11, BurnoutRetType)(obj);       }
__inline ErrorCode         AiChannel_setBurnoutRetType(AiChannel *obj, BurnoutRetType value)                                              { return DAQFN_ET(DNLAPI_AI, 12, BurnoutRetType)(obj, value);}
__inline double            AiChannel_getBurnoutRetValue(AiChannel *obj)                                                                   { return DAQFN_TV(DNLAPI_AI, 13, double)(obj);               }
__inline ErrorCode         AiChannel_setBurnoutRetValue(AiChannel *obj, double value)                                                     { return DAQFN_ET(DNLAPI_AI, 14, double)(obj, value);        }
__inline FilterType        AiChannel_getFilterType(AiChannel *obj)                                                                        { return DAQFN_TV(DNLAPI_AI, 15, FilterType) (obj);          }
__inline ErrorCode         AiChannel_setFilterType(AiChannel *obj, FilterType value)                                                      { return DAQFN_ET(DNLAPI_AI, 16, FilterType) (obj, value);   }
__inline double            AiChannel_getFilterCutoffFreq(AiChannel *obj)                                                                  { return DAQFN_TV(DNLAPI_AI, 17, double)(obj);               }
__inline ErrorCode         AiChannel_setFilterCutoffFreq(AiChannel *obj, double value)                                                    { return DAQFN_ET(DNLAPI_AI, 18, double)(obj, value);        }
__inline double            AiChannel_getFilterCutoffFreq1(AiChannel *obj)                                                                 { return DAQFN_TV(DNLAPI_AI, 19, double)(obj);               }
__inline ErrorCode         AiChannel_setFilterCutoffFreq1(AiChannel *obj, double value)                                                   { return DAQFN_ET(DNLAPI_AI, 20, double)(obj, value);        }
__inline int32             AiChannel_getLogicalNumber(AiChannel *obj)                                                                     { return DAQFN_TV(DNLAPI_AI, 21, int32)(obj);                }

__inline CouplingType      AiChannel_getCouplingType(AiChannel *obj)                                                                      { return DAQFN_TV(DNLAPI_AI, 95, CouplingType)(obj);         }
__inline ErrorCode         AiChannel_setCouplingType(AiChannel *obj, CouplingType value)                                                  { return DAQFN_ET(DNLAPI_AI, 96, CouplingType)(obj, value);  }
__inline IepeType          AiChannel_getIepeType(AiChannel *obj)                                                                          { return DAQFN_TV(DNLAPI_AI, 97, IepeType)(obj);             }
__inline ErrorCode         AiChannel_setIepeType(AiChannel *obj, IepeType value)                                                          { return DAQFN_ET(DNLAPI_AI, 98, IepeType)(obj, value);      }
__inline ImpedanceType     AiChannel_getImpedanceType(AiChannel *obj)                                                                     { return DAQFN_TV(DNLAPI_AI, 99, ImpedanceType)(obj);        }
__inline ErrorCode         AiChannel_setImpedanceType(AiChannel *obj, ImpedanceType value)                                                { return DAQFN_ET(DNLAPI_AI, 100, ImpedanceType)(obj, value);}

__inline ErrorCode         AiChannel_getSensorDescription(AiChannel *obj, int32 *size, wchar_t desc[])                                     { return DAQFN_ETT(DNLAPI_AI, 109, int32 *, wchar_t *)(obj, size, desc); }
__inline ErrorCode         AiChannel_setSensorDescription(AiChannel *obj, int32 size, wchar_t desc[])                                      { return DAQFN_ETT(DNLAPI_AI, 110, int32  , wchar_t *)(obj, size, desc); }

/* ----------------------------------------------------------*/                                                                                                
/* AI features                                               */                                                                                                                                                      
/* ----------------------------------------------------------*/
/*ADC features*/
__inline int32             AiFeatures_getResolution(AiFeatures *obj)                                                                      { return DAQFN_TV(DNLAPI_AI, 22, int32)(obj); }
__inline int32             AiFeatures_getDataSize(AiFeatures *obj)                                                                        { return DAQFN_TV(DNLAPI_AI, 23, int32)(obj); }
__inline int32             AiFeatures_getDataMask(AiFeatures *obj)                                                                        { return DAQFN_TV(DNLAPI_AI, 24, int32)(obj); }
__inline double            AiFeatures_getTimestampResolution(AiFeatures *obj)                                                             { return DAQFN_TV(DNLAPI_AI, 108, int32)(obj); }

/*channel features*/                                                         
__inline int32             AiFeatures_getChannelCountMax(AiFeatures *obj)                                                                 { return DAQFN_TV(DNLAPI_AI, 25, int32)(obj);         }
__inline AiChannelType     AiFeatures_getChannelType(AiFeatures *obj)                                                                     { return DAQFN_TV(DNLAPI_AI, 26, AiChannelType)(obj); }
__inline int8              AiFeatures_getOverallValueRange(AiFeatures *obj)                                                               { return DAQFN_TV(DNLAPI_AI, 27, int8)(obj);          }
__inline IArray *          AiFeatures_getValueRanges(AiFeatures *obj)                                                                     { return DAQFN_TV(DNLAPI_AI, 28, IArray *)(obj);      }
__inline IArray *          AiFeatures_getBurnoutReturnTypes(AiFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_AI, 29, IArray *)(obj);      }
__inline IArray *          AiFeatures_getConnectionTypes(AiFeatures *obj)                                                                 { return DAQFN_TV(DNLAPI_AI, 106, IArray *)(obj);     }
__inline int8              AiFeatures_getOverallConnection(AiFeatures *obj)                                                               { return DAQFN_TV(DNLAPI_AI, 107, int8)(obj);         }
__inline IArray *          AiFeatures_getCouplingTypes(AiFeatures *obj)                                                                   { return DAQFN_TV(DNLAPI_AI, 101,  IArray *)(obj);    }
__inline IArray *          AiFeatures_getIepeTypes(AiFeatures *obj)                                                                       { return DAQFN_TV(DNLAPI_AI, 102,  IArray *)(obj);    }
__inline IArray *          AiFeatures_getImpedanceTypes(AiFeatures *obj)                                                                  { return DAQFN_TV(DNLAPI_AI, 103,  IArray *)(obj);    }

/*filter*/             
__inline IArray *          AiFeatures_getFilterTypes(AiFeatures *obj)                                                                     { return DAQFN_TV(DNLAPI_AI, 30, IArray *)(obj);         }
__inline void              AiFeatures_getFilterCutoffFreqRange(AiFeatures *obj, MathInterval *x)                                          {        DAQFN_VT(DNLAPI_AI, 31, MathInterval *)(obj, x);}
__inline void              AiFeatures_getFilterCutoffFreq1Range(AiFeatures *obj, MathInterval *x)                                         {        DAQFN_VT(DNLAPI_AI, 32, MathInterval *)(obj, x);}

/*CJC features */                                                                                                                             
__inline int8              AiFeatures_getThermoSupported(AiFeatures *obj)                                                                 { return DAQFN_TV(DNLAPI_AI, 33, int8)(obj);    }    
__inline IArray *          AiFeatures_getCjcChannels(AiFeatures *obj)                                                                     { return DAQFN_TV(DNLAPI_AI, 34, IArray *)(obj);}

/*buffered ai->basic features*/
__inline int8              AiFeatures_getBufferedAiSupported(AiFeatures *obj)                                                             { return DAQFN_TV(DNLAPI_AI, 35, int8)(obj);           }
__inline SamplingMethod    AiFeatures_getSamplingMethod(AiFeatures *obj)                                                                  { return DAQFN_TV(DNLAPI_AI, 36, SamplingMethod)(obj); }
__inline int32             AiFeatures_getChannelStartBase(AiFeatures *obj)                                                                { return DAQFN_TV(DNLAPI_AI, 37, int32)(obj);    }
__inline int32             AiFeatures_getChannelCountBase(AiFeatures *obj)                                                                { return DAQFN_TV(DNLAPI_AI, 38, int32)(obj);    }

/*buffered ai->conversion clock features*/
__inline IArray *          AiFeatures_getConvertClockSources(AiFeatures *obj)                                                             { return DAQFN_TV(DNLAPI_AI, 39, IArray *)(obj);         }
__inline void              AiFeatures_getConvertClockRange(AiFeatures *obj, MathInterval *x)                                              {        DAQFN_VT(DNLAPI_AI, 40, MathInterval *)(obj, x);}

/*buffered ai->burst scan*/
__inline int8              AiFeatures_getBurstScanSupported(AiFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_AI, 41, int8)(obj);        }
__inline IArray *          AiFeatures_getScanClockSources(AiFeatures *obj)                                                                { return DAQFN_TV(DNLAPI_AI, 42, IArray *)(obj);    }
__inline void              AiFeatures_getScanClockRange(AiFeatures *obj, MathInterval *x)                                                 {        DAQFN_VT(DNLAPI_AI, 43, MathInterval *)(obj, x); }
__inline int32             AiFeatures_getScanCountMax(AiFeatures *obj)                                                                    { return DAQFN_TV(DNLAPI_AI, 44, int32)(obj);             }

/*buffered ai->trigger features*/
__inline int8              AiFeatures_getRetriggerable(AiFeatures *obj)                                                                   { return DAQFN_TV(DNLAPI_AI,  45,  int8)(obj);  }
__inline int32             AiFeatures_getTriggerCount(AiFeatures *obj)                                                                    { return DAQFN_TV(DNLAPI_AI,  46,  int32)(obj); }
__inline IArray *          AiFeatures_getTriggerFilterTypes(AiFeatures *obj)                                                              { return DAQFN_TT(DNLAPI_AI,  104, IArray *, int32)(obj, 0);         }
__inline void              AiFeatures_getTriggerFilterCutoffFreq(AiFeatures *obj, MathInterval *x)                                        {        DAQFN_VTT(DNLAPI_AI, 105, int32, MathInterval *)(obj, 0, x);}

/*buffered ai->trigger0 features*/
__inline IArray *          AiFeatures_getTriggerActions(AiFeatures *obj)                                                                  { return DAQFN_TT(DNLAPI_AI,  47,  IArray *, int32)(obj, 0);         }
__inline void              AiFeatures_getTriggerDelayRange(AiFeatures *obj, MathInterval *x)                                              {        DAQFN_VTT(DNLAPI_AI, 48,  int32, MathInterval *)(obj, 0, x);}
__inline IArray *          AiFeatures_getTriggerSources(AiFeatures *obj)                                                                  { return DAQFN_TT(DNLAPI_AI,  49,  IArray *, int32)(obj, 0);         }
__inline ValueRange        AiFeatures_getTriggerSourceVrg(AiFeatures *obj)                                                                { return DAQFN_TT(DNLAPI_AI,  50,  ValueRange, int32)(obj, 0);       }
__inline double            AiFeatures_getTriggerHysteresisIndexMax(AiFeatures *obj)                                                       { return DAQFN_TT(DNLAPI_AI,  51,  double, int32)(obj, 0);           }
__inline int32             AiFeatures_getTriggerHysteresisIndexStep(AiFeatures *obj)                                                      { return DAQFN_TT(DNLAPI_AI,  52,  int32, int32)(obj, 0);            }
/*buffered ai->trigger1 features*/                                                                  
__inline IArray *          AiFeatures_getTrigger1Actions(AiFeatures *obj)                                                                 { return DAQFN_TT(DNLAPI_AI,  47,  IArray *, int32)(obj, 1);         }
__inline void              AiFeatures_getTrigger1DelayRange(AiFeatures *obj, MathInterval *x)                                             {        DAQFN_VTT(DNLAPI_AI, 48,  int32, MathInterval *)(obj, 1, x);}
__inline IArray *          AiFeatures_getTrigger1Sources(AiFeatures *obj)                                                                 { return DAQFN_TT(DNLAPI_AI,  49,  IArray *, int32)(obj, 1);         }
__inline ValueRange        AiFeatures_getTrigger1SourceVrg(AiFeatures *obj)                                                               { return DAQFN_TT(DNLAPI_AI,  50,  ValueRange, int32)(obj, 1);       }
__inline double            AiFeatures_getTrigger1HysteresisIndexMax(AiFeatures *obj)                                                      { return DAQFN_TT(DNLAPI_AI,  51,  double, int32)(obj, 1);           }
__inline int32             AiFeatures_getTrigger1HysteresisIndexStep(AiFeatures *obj)                                                     { return DAQFN_TT(DNLAPI_AI,  52,  int32, int32)(obj, 1);            }
/*buffered ai->trigger2 features*/                                                                  
__inline IArray *          AiFeatures_getTrigger2Actions(AiFeatures *obj)                                                                 { return DAQFN_TT(DNLAPI_AI,  47,  IArray *, int32)(obj, 2);         }
__inline void              AiFeatures_getTrigger2DelayRange(AiFeatures *obj, MathInterval *x)                                             {        DAQFN_VTT(DNLAPI_AI, 48,  int32, MathInterval *)(obj, 2, x);}
__inline IArray *          AiFeatures_getTrigger2Sources(AiFeatures *obj)                                                                 { return DAQFN_TT(DNLAPI_AI,  49,  IArray *, int32)(obj, 2);         }
__inline ValueRange        AiFeatures_getTrigger2SourceVrg(AiFeatures *obj)                                                               { return DAQFN_TT(DNLAPI_AI,  50,  ValueRange, int32)(obj, 2);       }
__inline double            AiFeatures_getTrigger2HysteresisIndexMax(AiFeatures *obj)                                                      { return DAQFN_TT(DNLAPI_AI,  51,  double, int32)(obj, 2);           }
__inline int32             AiFeatures_getTrigger2HysteresisIndexStep(AiFeatures *obj)                                                     { return DAQFN_TT(DNLAPI_AI,  52,  int32, int32)(obj, 2);            }
/*buffered ai->trigger3 features*/                                                                  
__inline IArray *          AiFeatures_getTrigger3Actions(AiFeatures *obj)                                                                 { return DAQFN_TT(DNLAPI_AI,  47,  IArray *, int32)(obj, 3);         }
__inline void              AiFeatures_getTrigger3DelayRange(AiFeatures *obj, MathInterval *x)                                             {        DAQFN_VTT(DNLAPI_AI, 48,  int32, MathInterval *)(obj, 3, x);}
__inline IArray *          AiFeatures_getTrigger3Sources(AiFeatures *obj)                                                                 { return DAQFN_TT(DNLAPI_AI,  49,  IArray *, int32)(obj, 3);         }
__inline ValueRange        AiFeatures_getTrigger3SourceVrg(AiFeatures *obj)                                                               { return DAQFN_TT(DNLAPI_AI,  50,  ValueRange, int32)(obj, 3);       }
__inline double            AiFeatures_getTrigger3HysteresisIndexMax(AiFeatures *obj)                                                      { return DAQFN_TT(DNLAPI_AI,  51,  double, int32)(obj, 3);           }
__inline int32             AiFeatures_getTrigger3HysteresisIndexStep(AiFeatures *obj)                                                     { return DAQFN_TT(DNLAPI_AI,  52,  int32, int32)(obj, 3);            }

/* ----------------------------------------------------------*/
/*InstantAiCtrl                                              */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         InstantAiCtrl_LoadProfile(InstantAiCtrl *obj, wchar_t const *fileName)                                          { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              InstantAiCtrl_Cleanup(InstantAiCtrl *obj)                                                                       {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              InstantAiCtrl_Dispose(InstantAiCtrl *obj)                                                                       {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              InstantAiCtrl_getSelectedDevice(InstantAiCtrl *obj, DeviceInformation *x)                                       {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         InstantAiCtrl_setSelectedDevice(InstantAiCtrl *obj, DeviceInformation const *x)                                 { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      InstantAiCtrl_getState(void *obj)                                                                               { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);  }
__inline DeviceCtrl *      InstantAiCtrl_getDevice(InstantAiCtrl *obj)                                                                     { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);  }
__inline IArray *          InstantAiCtrl_getSupportedDevices(InstantAiCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);      }
__inline IArray *          InstantAiCtrl_getSupportedModes(InstantAiCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);      }
/*base2*/                                                                                                                                    
__inline AiFeatures *      InstantAiCtrl_getFeatures(InstantAiCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_AI, 58, AiFeatures *)(obj); }
__inline IArray *          InstantAiCtrl_getChannels(InstantAiCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_AI, 59, IArray *    )(obj); }
__inline int32             InstantAiCtrl_getChannelCount(InstantAiCtrl *obj)                                                               { return DAQFN_TV(DNLAPI_AI, 60, int32       )(obj); }
__inline ErrorCode         InstantAiCtrl_runSensorCalibration(InstantAiCtrl *obj, int32 ch, CalibrationData const *data)                   { return DAQFN_ETT(DNLAPI_AI, 111, int32, CalibrationData const*)(obj, ch, data);   }
/* InstantAiCtrl Methods & Properties*/                                                                                                      
__inline ErrorCode         InstantAiCtrl_ReadAny(InstantAiCtrl *obj, int32 chStart, int32 chCount, void *dataRaw, double *dataScaled)      { return DAQFN_ETTTT(DNLAPI_AI, 61, int32, int32, void *, double *)(obj, chStart, chCount, dataRaw, dataScaled); }
__inline CjcSetting *      InstantAiCtrl_getCjc(InstantAiCtrl *obj)                                                                        { return DAQFN_TV(DNLAPI_AI, 62, CjcSetting *)(obj);  }
__inline double            InstantAiCtrl_getAutoConvertClockRate(InstantAiCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_AI, 63, double)(obj);        }
__inline ErrorCode         InstantAiCtrl_setAutoConvertClockRate(InstantAiCtrl *obj, double value)                                         { return DAQFN_ET(DNLAPI_AI, 64, double)(obj, value); }
__inline int32             InstantAiCtrl_getAutoConvertChannelStart(InstantAiCtrl *obj)                                                    { return DAQFN_TV(DNLAPI_AI, 65, int32)(obj);         }
__inline ErrorCode         InstantAiCtrl_setAutoConvertChannelStart(InstantAiCtrl *obj, int32 value)                                       { return DAQFN_ET(DNLAPI_AI, 66, int32)(obj, value);  }
__inline int32             InstantAiCtrl_getAutoConvertChannelCount(InstantAiCtrl *obj)                                                    { return DAQFN_TV(DNLAPI_AI, 67, int32)(obj);         }
__inline ErrorCode         InstantAiCtrl_setAutoConvertChannelCount(InstantAiCtrl *obj, int32 value)                                       { return DAQFN_ET(DNLAPI_AI, 68, int32)(obj, value);  }
/*creator*/
__inline InstantAiCtrl *   InstantAiCtrl_Create(void)                                                                                      { return (InstantAiCtrl*)DaqCtrlBase_Create(SceInstantAi); } 

/* ----------------------------------------------------------*/
/*BufferedAiCtrl                                             */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         BufferedAiCtrl_LoadProfile(BufferedAiCtrl *obj, wchar_t const *fileName)                                        { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              BufferedAiCtrl_Cleanup(BufferedAiCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              BufferedAiCtrl_Dispose(BufferedAiCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              BufferedAiCtrl_getSelectedDevice(BufferedAiCtrl *obj, DeviceInformation *x)                                     {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         BufferedAiCtrl_setSelectedDevice(BufferedAiCtrl *obj, DeviceInformation const *x)                               { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      BufferedAiCtrl_getState(void *obj)                                                                              { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);}
__inline DeviceCtrl *      BufferedAiCtrl_getDevice(BufferedAiCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);}
__inline IArray *          BufferedAiCtrl_getSupportedDevices(BufferedAiCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);    }
__inline IArray *          BufferedAiCtrl_getSupportedModes(BufferedAiCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);    }
/*base2*/                
__inline AiFeatures *      BufferedAiCtrl_getFeatures(BufferedAiCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_AI, 58, AiFeatures *)(obj);}
__inline IArray *          BufferedAiCtrl_getChannels(BufferedAiCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_AI, 59, IArray *)(obj);    }
__inline int32             BufferedAiCtrl_getChannelCount(BufferedAiCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_AI, 60, int32 )(obj);      }
__inline ErrorCode         BufferedAiCtrl_runSensorCalibration(BufferedAiCtrl *obj, int32 ch, CalibrationData const *data)                 { return DAQFN_ETT(DNLAPI_AI, 111, int32, CalibrationData const*)(obj, ch, data);  }
/* BufferedAiCtrl Methods & Properties */
__inline ErrorCode         BufferedAiCtrl_Prepare(BufferedAiCtrl *obj)                                                                     { return DAQFN_EV(DNLAPI_AI, 69)(obj);   }
__inline ErrorCode         BufferedAiCtrl_RunOnce(BufferedAiCtrl *obj)                                                                     { return DAQFN_EV(DNLAPI_AI, 70)(obj);   }
__inline ErrorCode         BufferedAiCtrl_Start(BufferedAiCtrl *obj)                                                                       { return DAQFN_EV(DNLAPI_AI, 71)(obj);   }
__inline ErrorCode         BufferedAiCtrl_Stop(BufferedAiCtrl *obj)                                                                        { return DAQFN_EV(DNLAPI_AI, 72)(obj);   }
__inline void              BufferedAiCtrl_Release(BufferedAiCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_AI, 73)(obj);   }
__inline int8              BufferedAiCtrl_GetBufferStatus(BufferedAiCtrl *obj, int32 *count, int32 *offset)                                { return DAQFN_TTT(DNLAPI_AI, 74, int8, int32 *, int32 *)(obj, count, offset);    }  
__inline ErrorCode         BufferedAiCtrl_GetDataI16(BufferedAiCtrl *obj, int32 count, int16 rawData[])                                    { return DAQFN_ETTT(DNLAPI_AI, 75, int32, int32, void*)(obj, sizeof(int16), count, rawData);    }
__inline ErrorCode         BufferedAiCtrl_GetDataI32(BufferedAiCtrl *obj, int32 count, int32 rawData[])                                    { return DAQFN_ETTT(DNLAPI_AI, 75, int32, int32, void*)(obj, sizeof(int32), count, rawData);    }
__inline ErrorCode         BufferedAiCtrl_GetDataF64(BufferedAiCtrl *obj, int32 count, double scaledData[])                                { return DAQFN_ETTT(DNLAPI_AI, 75, int32, int32, void*)(obj, sizeof(double),count, scaledData); }
__inline void*             BufferedAiCtrl_getBuffer(BufferedAiCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_AI, 76, void*)(obj);               }
__inline int32             BufferedAiCtrl_getBufferCapacity(BufferedAiCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_AI, 77, int32)(obj);               }
__inline ScanChannel *     BufferedAiCtrl_getScanChannel(BufferedAiCtrl *obj)                                                              { return DAQFN_TV(DNLAPI_AI, 78, ScanChannel *)(obj);       }
__inline ConvertClock *    BufferedAiCtrl_getConvertClock(BufferedAiCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_AI, 79, ConvertClock *)(obj);      }
__inline ScanClock *       BufferedAiCtrl_getScanClock(BufferedAiCtrl *obj)                                                                { return DAQFN_TV(DNLAPI_AI, 80, ScanClock *)(obj);         }
__inline Trigger *         BufferedAiCtrl_getTrigger(BufferedAiCtrl *obj)                                                                  { return DAQFN_TT(DNLAPI_AI, 81, Trigger *, int32)(obj, 0); }
__inline Trigger *         BufferedAiCtrl_getTrigger1(BufferedAiCtrl *obj)                                                                 { return DAQFN_TT(DNLAPI_AI, 81, Trigger *, int32)(obj, 1); }
__inline Trigger *         BufferedAiCtrl_getTrigger2(BufferedAiCtrl *obj)                                                                 { return DAQFN_TT(DNLAPI_AI, 81, Trigger *, int32)(obj, 2); }
__inline Trigger *         BufferedAiCtrl_getTrigger3(BufferedAiCtrl *obj)                                                                 { return DAQFN_TT(DNLAPI_AI, 81, Trigger *, int32)(obj, 3); }
__inline int8              BufferedAiCtrl_getStreaming(BufferedAiCtrl *obj)                                                                { return DAQFN_TV(DNLAPI_AI, 82, int8)(obj);                }
__inline ErrorCode         BufferedAiCtrl_setStreaming(BufferedAiCtrl *obj, int8 value)                                                    { return DAQFN_ET(DNLAPI_AI, 83, int8)(obj, value);         }
/*creator*/
__inline BufferedAiCtrl *  BufferedAiCtrl_Create(void)                                                                                     { return (BufferedAiCtrl *)DaqCtrlBase_Create(SceBufferedAi); } 

/*Helpers*/
__inline void              BufferedAiCtrl_addDataReadyHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                   { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_removeDataReadyHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_addOverrunHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_removeOverrunHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_addCacheOverflowHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)               { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_removeCacheOverflowHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)            { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_addStoppedHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiStopped,       (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_removeStoppedHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiStopped,       (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_addBurnOutHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiBurnOut,       (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_removeBurnOutHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiBurnOut,       (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_addConvStoppedHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                 { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiConvStopped,   (DaqEventProc)proc, userParam);}
__inline void              BufferedAiCtrl_removeConvStoppedHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)              { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiConvStopped,   (DaqEventProc)proc, userParam);}

/* ----------------------------------------------------------*/
/*WaveformAiCtrl                                             */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         WaveformAiCtrl_LoadProfile(WaveformAiCtrl *obj, wchar_t const *fileName)                                        { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              WaveformAiCtrl_Cleanup(WaveformAiCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              WaveformAiCtrl_Dispose(WaveformAiCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              WaveformAiCtrl_getSelectedDevice(WaveformAiCtrl *obj, DeviceInformation *x)                                     {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         WaveformAiCtrl_setSelectedDevice(WaveformAiCtrl *obj, DeviceInformation const *x)                               { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      WaveformAiCtrl_getState(void *obj)                                                                              { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);}
__inline DeviceCtrl *      WaveformAiCtrl_getDevice(WaveformAiCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);}
__inline IArray *          WaveformAiCtrl_getSupportedDevices(WaveformAiCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);    }
__inline IArray *          WaveformAiCtrl_getSupportedModes(WaveformAiCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);    }
/*base2*/                
__inline AiFeatures *      WaveformAiCtrl_getFeatures(WaveformAiCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_AI, 58, AiFeatures *)(obj);}
__inline IArray *          WaveformAiCtrl_getChannels(WaveformAiCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_AI, 59, IArray *)(obj);    }
__inline int32             WaveformAiCtrl_getChannelCount(WaveformAiCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_AI, 60, int32 )(obj);      }
__inline ErrorCode         WaveformAiCtrl_runSensorCalibration(WaveformAiCtrl *obj, int32 ch, CalibrationData const *data)                 { return DAQFN_ETT(DNLAPI_AI, 111, int32, CalibrationData const*)(obj, ch, data);  }
/* WaveformAiCtrl Methods & Properties */
__inline ErrorCode         WaveformAiCtrl_Prepare(WaveformAiCtrl *obj)                                                                     { return DAQFN_EV(DNLAPI_AI, 84)(obj);   }
__inline ErrorCode         WaveformAiCtrl_Start(WaveformAiCtrl *obj)                                                                       { return DAQFN_EV(DNLAPI_AI, 85)(obj);   }
__inline ErrorCode         WaveformAiCtrl_Stop(WaveformAiCtrl *obj)                                                                        { return DAQFN_EV(DNLAPI_AI, 86)(obj);   }
__inline void              WaveformAiCtrl_Release(WaveformAiCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_AI, 87)(obj);   }
__inline int8              WaveformAiCtrl_GetBufferStatus(WaveformAiCtrl *obj, int32 *count, int32 *offset)                                { return DAQFN_TTT(DNLAPI_AI, 88, int8, int32 *, int32 *)(obj, count, offset); }  

__inline ErrorCode WaveformAiCtrl_GetDataI16(WaveformAiCtrl *obj, int32 count, int16 dataBuf[], 
   int32 timeout, int32 *returned, double * startTime, int32 *markCount, DataMark *markBuf)
{ 
   typedef ErrorCode (BDAQCALL *FNGETDATA)(void *, int32, int32, void *, int32, int32 *, double *, int32 *, DataMark *);
   return ((FNGETDATA)DNLAPI_AI[89])(obj, sizeof(int16), count, dataBuf, timeout, returned, startTime, markCount, markBuf);        
}
__inline ErrorCode WaveformAiCtrl_GetDataI32(WaveformAiCtrl *obj, int32 count, int32 dataBuf[], 
   int32 timeout, int32 *returned, double * startTime, int32 *markCount, DataMark *markBuf)
{ 
   typedef ErrorCode (BDAQCALL *FNGETDATA)(void *, int32, int32, void *, int32, int32 *, double *, int32 *, DataMark *);
   return ((FNGETDATA)DNLAPI_AI[89])(obj, sizeof(int32), count, dataBuf, timeout, returned, startTime, markCount, markBuf);        
}
__inline ErrorCode WaveformAiCtrl_GetDataF64(WaveformAiCtrl *obj, int32 count, double dataBuf[], 
   int32 timeout, int32 *returned, double * startTime, int32 *markCount, DataMark *markBuf) 
{ 
   typedef ErrorCode (BDAQCALL *FNGETDATA)(void *, int32, int32, void *, int32, int32 *, double *, int32 *, DataMark *);
   return ((FNGETDATA)DNLAPI_AI[89])(obj, sizeof(double), count, dataBuf, timeout, returned, startTime, markCount, markBuf);        
}

__inline void*             WaveformAiCtrl_getBuffer(WaveformAiCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_AI, 90, void*)(obj);        }
__inline int32             WaveformAiCtrl_getBufferCapacity(WaveformAiCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_AI, 91, int32)(obj);        }
__inline Conversion *      WaveformAiCtrl_getConversion(WaveformAiCtrl *obj)                                                               { return DAQFN_TV(DNLAPI_AI, 92, Conversion *)(obj); }
__inline Record *          WaveformAiCtrl_getRecord(WaveformAiCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_AI, 93, Record *)(obj);     }
__inline Trigger *         WaveformAiCtrl_getTrigger(WaveformAiCtrl *obj)                                                                  { return DAQFN_TT(DNLAPI_AI, 94, Trigger *, int32)(obj, 0); }
__inline Trigger *         WaveformAiCtrl_getTrigger1(WaveformAiCtrl *obj)                                                                 { return DAQFN_TT(DNLAPI_AI, 94, Trigger *, int32)(obj, 1); }
__inline Trigger *         WaveformAiCtrl_getTrigger2(WaveformAiCtrl *obj)                                                                 { return DAQFN_TT(DNLAPI_AI, 94, Trigger *, int32)(obj, 2); }
__inline Trigger *         WaveformAiCtrl_getTrigger3(WaveformAiCtrl *obj)                                                                 { return DAQFN_TT(DNLAPI_AI, 94, Trigger *, int32)(obj, 3); }
/*creator*/
__inline WaveformAiCtrl *  WaveformAiCtrl_Create(void)                                                                                     { return (WaveformAiCtrl *)DaqCtrlBase_Create(SceWaveformAi); } 

/*Helpers*/
__inline void              WaveformAiCtrl_addDataReadyHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)                   { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeDataReadyHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)                { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_addOverrunHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeOverrunHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_addCacheOverflowHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)               { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeCacheOverflowHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)            { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_addStoppedHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiStopped,       (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeStoppedHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiStopped,       (DaqEventProc)proc, userParam);}

__inline void              WaveformAiCtrl_addBurnOutHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiBurnOut,       (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeBurnOutHandler(WaveformAiCtrl *obj, BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiBurnOut,       (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_addTimeStampOverrunHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)            { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiTimeStampOverrun, (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeTimeStampOverrunHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)         { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiTimeStampOverrun, (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_addTimeStampCacheOverflowHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)      { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiTimeStampCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeTimeStampCacheOverflowHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)   { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiTimeStampCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_addMarkOverrunHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                 { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiMarkOverrun,      (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeMarkOverrunHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)              { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiMarkOverrun,      (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_addConvStoppedHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)                 { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAiConvStopped,   (DaqEventProc)proc, userParam);}
__inline void              WaveformAiCtrl_removeConvStoppedHandler(BufferedAiCtrl *obj, BfdAiEventProc proc, void *userParam)              { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAiConvStopped,   (DaqEventProc)proc, userParam);}

/*************************************************************/
/* ctrl class : AO related                                   */
/* ----------------------------------------------------------*/
/*AoChannel*/
/* ----------------------------------------------------------*/
__inline int32             AoChannel_getChannel(AoChannel *obj)                                                                            { return DAQFN_TV(DNLAPI_AO, 0,  int32)(obj);            }
__inline ValueRange        AoChannel_getValueRange(AoChannel *obj)                                                                         { return DAQFN_TV(DNLAPI_AO, 1,  ValueRange)(obj);       }
__inline ErrorCode         AoChannel_setValueRange(AoChannel *obj, ValueRange value)                                                       { return DAQFN_ET(DNLAPI_AO, 2,  ValueRange)(obj, value);}
__inline double            AoChannel_getExtRefBipolar(AoChannel *obj)                                                                      { return DAQFN_TV(DNLAPI_AO, 47, double)(obj);           }
__inline ErrorCode         AoChannel_setExtRefBipolar(AoChannel *obj, double value)                                                        { return DAQFN_ET(DNLAPI_AO, 48, double)(obj, value);    }
__inline double            AoChannel_getExtRefUnipolar(AoChannel *obj)                                                                     { return DAQFN_TV(DNLAPI_AO, 49, double)(obj);           }
__inline ErrorCode         AoChannel_setExtRefUnipolar(AoChannel *obj, double value)                                                       { return DAQFN_ET(DNLAPI_AO, 50, double)(obj, value);    }

/* ----------------------------------------------------------*/                                          
/*AO features                                                */
/* ----------------------------------------------------------*/                                          
/*DAC features  */
__inline int32             AoFeatures_getResolution(AoFeatures *obj)                                                                       { return DAQFN_TV(DNLAPI_AO, 5, int32)(obj); }
__inline int32             AoFeatures_getDataSize(AoFeatures *obj)                                                                         { return DAQFN_TV(DNLAPI_AO, 6, int32)(obj); }
__inline int32             AoFeatures_getDataMask(AoFeatures *obj)                                                                         { return DAQFN_TV(DNLAPI_AO, 7, int32)(obj); }

/*channel features  */   
__inline int32             AoFeatures_getChannelCountMax(AoFeatures *obj)                                                                  { return DAQFN_TV(DNLAPI_AO, 8, int32)(obj);   }
__inline IArray *          AoFeatures_getValueRanges(AoFeatures *obj)                                                                      { return DAQFN_TV(DNLAPI_AO, 9, IArray *)(obj);}
__inline int8              AoFeatures_getExternalRefAntiPolar(AoFeatures *obj)                                                             { return DAQFN_TV(DNLAPI_AO, 10, int8)(obj);   }
__inline void              AoFeatures_getExternalRefRange(AoFeatures *obj, MathInterval *x)                                                {        DAQFN_VT(DNLAPI_AO, 11, MathInterval *)(obj, x); }

/*buffered ao->basic features */
__inline int8              AoFeatures_getBufferedAoSupported(AoFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_AO, 12, int8)(obj); }
__inline SamplingMethod    AoFeatures_getSamplingMethod(AoFeatures *obj)                                                                   { return DAQFN_TV(DNLAPI_AO, 13, SamplingMethod)(obj); }
__inline int32             AoFeatures_getChannelStartBase(AoFeatures *obj)                                                                 { return DAQFN_TV(DNLAPI_AO, 14, int32)(obj); }
__inline int32             AoFeatures_getChannelCountBase(AoFeatures *obj)                                                                 { return DAQFN_TV(DNLAPI_AO, 15, int32)(obj); }

/*buffered ao->conversion clock features*/
__inline IArray *          AoFeatures_getConvertClockSources(AoFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_AO, 16, IArray *)(obj); }
__inline void              AoFeatures_getConvertClockRange(AoFeatures *obj, MathInterval *x)                                               {        DAQFN_VT(DNLAPI_AO, 17, MathInterval *)(obj, x);}

/*buffered ao->trigger features*/
__inline int32             AoFeatures_getTriggerCount(AoFeatures *obj)                                                                     { return DAQFN_TV(DNLAPI_AO, 18, int32)(obj); }
__inline int8              AoFeatures_getRetriggerable(AoFeatures *obj)                                                                    { return DAQFN_TV(DNLAPI_AO, 51, int8)(obj);  }

/*buffered ao->trigger0 features */
__inline IArray *          AoFeatures_getTriggerSources(AoFeatures *obj)                                                                   { return DAQFN_TT(DNLAPI_AO, 19, IArray *, int32)(obj, 0); }
__inline IArray *          AoFeatures_getTriggerActions(AoFeatures *obj)                                                                   { return DAQFN_TT(DNLAPI_AO, 20, IArray *, int32)(obj, 0); }
__inline void              AoFeatures_getTriggerDelayRange(AoFeatures *obj, MathInterval *x)                                               {        DAQFN_VTT(DNLAPI_AO, 21, int32, MathInterval *)(obj, 0, x); }

/*buffered ao->trigger1 features */
__inline IArray *          AoFeatures_getTrigger1Sources(AoFeatures *obj)                                                                  { return DAQFN_TT(DNLAPI_AO, 19, IArray *, int32)(obj, 1); }
__inline IArray *          AoFeatures_getTrigger1Actions(AoFeatures *obj)                                                                  { return DAQFN_TT(DNLAPI_AO, 20, IArray *, int32)(obj, 1); }
__inline void              AoFeatures_getTrigger1DelayRange(AoFeatures *obj, MathInterval *x)                                              {        DAQFN_VTT(DNLAPI_AO, 21, int32, MathInterval *)(obj, 1, x); }

/* ----------------------------------------------------------*/
/*InstantAoCtrl                                              */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         InstantAoCtrl_LoadProfile(InstantAoCtrl *obj, wchar_t const *fileName)                                          { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              InstantAoCtrl_Cleanup(InstantAoCtrl *obj)                                                                       {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              InstantAoCtrl_Dispose(InstantAoCtrl *obj)                                                                       {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              InstantAoCtrl_getSelectedDevice(InstantAoCtrl *obj, DeviceInformation *x)                                       {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         InstantAoCtrl_setSelectedDevice(InstantAoCtrl *obj, DeviceInformation const *x)                                 { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      InstantAoCtrl_getState(void *obj)                                                                               { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);}
__inline DeviceCtrl *      InstantAoCtrl_getDevice(InstantAoCtrl *obj)                                                                     { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);}
__inline IArray *          InstantAoCtrl_getSupportedDevices(InstantAoCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);    }
__inline IArray *          InstantAoCtrl_getSupportedModes(InstantAoCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);    }
/* base2*/                                                                                                                                    
__inline AoFeatures *      InstantAoCtrl_getFeatures(InstantAoCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_AO, 25, AoFeatures *)(obj); }
__inline IArray *          InstantAoCtrl_getChannels(InstantAoCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_AO, 26, IArray *)(obj);     }
__inline int32             InstantAoCtrl_getChannelCount(InstantAoCtrl *obj)                                                               { return DAQFN_TV(DNLAPI_AO, 27, int32 )(obj);       }
__inline double            InstantAoCtrl_getExtRefValueForUnipolar(InstantAoCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_AO, 28, double)(obj);       }
__inline ErrorCode         InstantAoCtrl_setExtRefValueForUnipolar(InstantAoCtrl *obj, double value)                                       { return DAQFN_ET(DNLAPI_AO, 29, double)(obj, value);}
__inline double            InstantAoCtrl_getExtRefValueForBipolar(InstantAoCtrl *obj)                                                      { return DAQFN_TV(DNLAPI_AO, 30, double)(obj);       }
__inline ErrorCode         InstantAoCtrl_setExtRefValueForBipolar(InstantAoCtrl *obj, double value)                                        { return DAQFN_ET(DNLAPI_AO, 31, double)(obj, value);}
/* InstantAoCtrl methods */                                                                                                                 
__inline ErrorCode         InstantAoCtrl_WriteAny(InstantAoCtrl *obj, int32 chStart, int32 chCount, void *dataRaw, double *dataScaled)     { return DAQFN_ETTTT(DNLAPI_AO, 32, int32, int32, void *, double *)(obj, chStart, chCount, dataRaw, dataScaled); }
/*creator*/
__inline InstantAoCtrl *   InstantAoCtrl_Create(void)                                                                                      { return (InstantAoCtrl *)DaqCtrlBase_Create(SceInstantAo); } 

/* ----------------------------------------------------------*/
/*BufferedAoCtrl                                             */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         BufferedAoCtrl_LoadProfile(BufferedAoCtrl *obj, wchar_t const *fileName)                                        { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              BufferedAoCtrl_Cleanup(BufferedAoCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              BufferedAoCtrl_Dispose(BufferedAoCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              BufferedAoCtrl_getSelectedDevice(BufferedAoCtrl *obj, DeviceInformation *x)                                     {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         BufferedAoCtrl_setSelectedDevice(BufferedAoCtrl *obj, DeviceInformation const *x)                               { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      BufferedAoCtrl_getState(void *obj)                                                                              { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);}
__inline DeviceCtrl *      BufferedAoCtrl_getDevice(BufferedAoCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);}
__inline IArray *          BufferedAoCtrl_getSupportedDevices(BufferedAoCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);    }
__inline IArray *          BufferedAoCtrl_getSupportedModes(BufferedAoCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);    }
/* base2*/                                                                                                                                  
__inline AoFeatures *      BufferedAoCtrl_getFeatures(BufferedAoCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_AO, 25, AoFeatures *)(obj); }
__inline IArray *          BufferedAoCtrl_getChannels(BufferedAoCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_AO, 26, IArray *)(obj);     }
__inline int32             BufferedAoCtrl_getChannelCount(BufferedAoCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_AO, 27, int32 )(obj);       }
__inline double            BufferedAoCtrl_getExtRefValueForUnipolar(BufferedAoCtrl *obj)                                                   { return DAQFN_TV(DNLAPI_AO, 28, double)(obj);       }
__inline ErrorCode         BufferedAoCtrl_setExtRefValueForUnipolar(BufferedAoCtrl *obj, double value)                                     { return DAQFN_ET(DNLAPI_AO, 29, double)(obj, value);}
__inline double            BufferedAoCtrl_getExtRefValueForBipolar(BufferedAoCtrl *obj)                                                    { return DAQFN_TV(DNLAPI_AO, 30, double)(obj);       }
__inline ErrorCode         BufferedAoCtrl_setExtRefValueForBipolar(BufferedAoCtrl *obj, double value)                                      { return DAQFN_ET(DNLAPI_AO, 31, double)(obj, value);}
/* BufferedAoCtrl Methods & Properties */                                                                                                   
__inline ErrorCode         BufferedAoCtrl_Prepare(BufferedAoCtrl *obj)                                                                     { return DAQFN_EV(DNLAPI_AO, 33)(obj);   }
__inline ErrorCode         BufferedAoCtrl_RunOnce(BufferedAoCtrl *obj)                                                                     { return DAQFN_EV(DNLAPI_AO, 34)(obj);   }
__inline ErrorCode         BufferedAoCtrl_Start(BufferedAoCtrl *obj)                                                                       { return DAQFN_EV(DNLAPI_AO, 35)(obj);   }
__inline ErrorCode         BufferedAoCtrl_Stop(BufferedAoCtrl *obj, int32 action)                                                          { return DAQFN_ET(DNLAPI_AO, 36, int32)(obj, action); }
__inline void              BufferedAoCtrl_Release(BufferedAoCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_AO, 37)(obj);                }
__inline int8              BufferedAoCtrl_GetBufferStatus(BufferedAoCtrl *obj, int32 *count, int32 *offset)                                { return DAQFN_TTT(DNLAPI_AO, 38, int8, int32 *, int32 *)(obj, count, offset);    }  
__inline ErrorCode         BufferedAoCtrl_SetDataI16(BufferedAoCtrl *obj, int32 count, int16 rawData[])                                    { return DAQFN_ETTT(DNLAPI_AO, 39, int32, int32, void*)(obj, sizeof(int16), count, rawData);     }
__inline ErrorCode         BufferedAoCtrl_SetDataI32(BufferedAoCtrl *obj, int32 count, int32 rawData[])                                    { return DAQFN_ETTT(DNLAPI_AO, 39, int32, int32, void*)(obj, sizeof(int32), count, rawData);     }
__inline ErrorCode         BufferedAoCtrl_SetDataF64(BufferedAoCtrl *obj, int32 count, double scaledData[])                                { return DAQFN_ETTT(DNLAPI_AO, 39, int32, int32, void*)(obj, sizeof(double), count, scaledData); }
__inline void*             BufferedAoCtrl_getBuffer(BufferedAoCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_AO, 40, void*)(obj);               }
__inline int32             BufferedAoCtrl_getBufferCapacity(BufferedAoCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_AO, 41, int32)(obj);               }
__inline ScanChannel *     BufferedAoCtrl_getScanChannel(BufferedAoCtrl *obj)                                                              { return DAQFN_TV(DNLAPI_AO, 42, ScanChannel *)(obj);       }
__inline ConvertClock *    BufferedAoCtrl_getConvertClock(BufferedAoCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_AO, 43, ConvertClock *)(obj);      }
__inline Trigger *         BufferedAoCtrl_getTrigger(BufferedAoCtrl *obj)                                                                  { return DAQFN_TT(DNLAPI_AO, 44, Trigger *, int32)(obj, 0); }
__inline Trigger *         BufferedAoCtrl_getTrigger1(BufferedAoCtrl *obj)                                                                 { return DAQFN_TT(DNLAPI_AO, 44, Trigger *, int32)(obj, 1); }
__inline int8              BufferedAoCtrl_getStreaming(BufferedAoCtrl *obj)                                                                { return DAQFN_TV(DNLAPI_AO, 45, int8)(obj);                }
__inline ErrorCode         BufferedAoCtrl_setStreaming(BufferedAoCtrl *obj, int8 value)                                                    { return DAQFN_ET(DNLAPI_AO, 46, int8)(obj, value);         }
/*creator*/
__inline BufferedAoCtrl *  BufferedAoCtrl_Create(void)                                                                                     { return (BufferedAoCtrl *)DaqCtrlBase_Create(SceBufferedAo); } 

/*Helpers*/
__inline void              BufferedAoCtrl_addDataTransmittedHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)             { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAoDataTransmitted, (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_removeDataTransmittedHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)          { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAoDataTransmitted, (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_addUnderrunHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)                    { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAoUnderrun,        (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_removeUnderrunHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)                 { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAoUnderrun,        (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_addCacheEmptiedHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAoCacheEmptied,    (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_removeCacheEmptiedHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAoCacheEmptied,    (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_addTransitStoppedHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)              { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAoTransStopped,    (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_removeTransitStoppedHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)           { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAoTransStopped,    (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_addStoppedHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtBufferedAoStopped,         (DaqEventProc)proc, userParam); }
__inline void              BufferedAoCtrl_removeStoppedHandler(BufferedAoCtrl *obj, BfdAoEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtBufferedAoStopped,         (DaqEventProc)proc, userParam); }

/*************************************************************/                                                                              
/*ctrl class : DIO related classes                           */
/* ----------------------------------------------------------*/
/*DioPort*/
/* ----------------------------------------------------------*/
__inline int32             DioPort_getPort(DioPort *obj)                                                                                 { return DAQFN_TV(DNLAPI_DIO, 0, int32)(obj);        }
__inline uint8             DioPort_getDirectionMask(DioPort *obj)                                                                        { return (uint8)DAQFN_TV(DNLAPI_DIO, 1, int32)(obj); }
__inline ErrorCode         DioPort_setDirectionMask(DioPort *obj, uint8 value)                                                           { return DAQFN_ET(DNLAPI_DIO, 2, int32)(obj, value); }
/*DI port property*/     
__inline uint8             DioPort_getDiInversePort(DioPort *obj)                                                                        { return DAQFN_TV(DNLAPI_DIO, 3, uint8)(obj);        }
__inline ErrorCode         DioPort_setDiInversePort(DioPort *obj, uint8 value)                                                           { return DAQFN_ET(DNLAPI_DIO, 4, uint8)(obj, value); }
/*DO port property*/     
__inline uint8             DioPort_getDoPresetValue(DioPort *obj)                                                                        { return DAQFN_TV(DNLAPI_DIO, 64, uint8)(obj);       }
__inline ErrorCode         DioPort_setDoPresetValue(DioPort *obj, uint8 value)                                                           { return DAQFN_ET(DNLAPI_DIO, 65, uint8)(obj, value);}

// For compatible
#define DioPort_getDirection(obj)        (DioPortDir)DioPort_getDirectionMask(obj)
#define DioPort_setDirection(obj, value) DioPort_setDirectionMask(obj, (uint8)value)

/* ----------------------------------------------------------*/                                          
/*DiintChannel*/
/* ----------------------------------------------------------*/                                          
__inline int32             DiintChannel_getChannel(DiintChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_DIO, 7 , int32)(obj);       }
__inline int8              DiintChannel_getEnabled(DiintChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_DIO, 8 , int8)(obj);        }
__inline ErrorCode         DiintChannel_setEnabled(DiintChannel *obj, int8 value)                                                        { return DAQFN_ET(DNLAPI_DIO, 9 , int8)(obj, value); }
__inline int8              DiintChannel_getGated(DiintChannel *obj)                                                                      { return DAQFN_TV(DNLAPI_DIO, 10, int8)(obj);        }
__inline ErrorCode         DiintChannel_setGated(DiintChannel *obj, int8 value)                                                          { return DAQFN_ET(DNLAPI_DIO, 11, int8)(obj, value); }
__inline ActiveSignal      DiintChannel_getTrigEdge(DiintChannel *obj)                                                                   { return DAQFN_TV(DNLAPI_DIO, 12, ActiveSignal)(obj);}
__inline ErrorCode         DiintChannel_setTrigEdge(DiintChannel *obj, ActiveSignal value)                                               { return DAQFN_ET(DNLAPI_DIO, 13, ActiveSignal)(obj, value);}

/* ----------------------------------------------------------*/                                          
/*DiCosintPort                                               */                                          
/* ----------------------------------------------------------*/                                          
__inline int32             DiCosintPort_getPort(DiCosintPort *obj)                                                                       { return DAQFN_TV(DNLAPI_DIO, 14, int32)(obj);       }
__inline uint8             DiCosintPort_getMask(DiCosintPort *obj)                                                                       { return DAQFN_TV(DNLAPI_DIO, 15, uint8)(obj);       }
__inline ErrorCode         DiCosintPort_setMask(DiCosintPort *obj, uint8 value)                                                          { return DAQFN_ET(DNLAPI_DIO, 16, uint8)(obj, value);}

/* ----------------------------------------------------------*/                                          
/*DiPmintPort                                                */
/* ----------------------------------------------------------*/                                          
__inline int32             DiPmintPort_getPort(DiPmintPort *obj)                                                                         { return DAQFN_TV(DNLAPI_DIO, 17, int32)(obj);       }
__inline uint8             DiPmintPort_getMask(DiPmintPort *obj)                                                                         { return DAQFN_TV(DNLAPI_DIO, 18, uint8)(obj);       }
__inline ErrorCode         DiPmintPort_setMask(DiPmintPort *obj, uint8 value)                                                            { return DAQFN_ET(DNLAPI_DIO, 19, uint8)(obj, value);}
__inline uint8             DiPmintPort_getPattern(DiPmintPort *obj)                                                                      { return DAQFN_TV(DNLAPI_DIO, 20, uint8)(obj);       }
__inline ErrorCode         DiPmintPort_setPattern(DiPmintPort *obj, uint8 value)                                                         { return DAQFN_ET(DNLAPI_DIO, 21, uint8)(obj, value);}

/* ----------------------------------------------------------*/                                          
/*DIO features                                               */
/* ----------------------------------------------------------*/
/*Common*/
__inline int8              DioFeatures_getPortProgrammable(DioFeatures *obj)                                                             { return DAQFN_TV(DNLAPI_DIO, 22, int8)(obj);   }
__inline int32             DioFeatures_getChannelCountMax(DioFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_DIO, 23, int32)(obj);  }
__inline int32             DioFeatures_getPortCount(DioFeatures *obj)                                                                    { return DAQFN_TV(DNLAPI_DIO, 24, int32)(obj);  }
__inline IArray *          DioFeatures_getPortsType(DioFeatures *obj)                                                                    { return DAQFN_TV(DNLAPI_DIO, 25, IArray *)(obj); }
__inline int8              DioFeatures_getDiSupported(DioFeatures *obj)                                                                  { return DAQFN_TV(DNLAPI_DIO, 26, int8)(obj);   }
__inline int8              DioFeatures_getDoSupported(DioFeatures *obj)                                                                  { return DAQFN_TV(DNLAPI_DIO, 27, int8)(obj);   }

/*DI Features*/          
__inline IArray *          DioFeatures_getDiDataMask(DioFeatures *obj)                                                                   { return DAQFN_TV(DNLAPI_DIO, 28, IArray *)(obj); }

/*di noise filter features*/
__inline int8              DioFeatures_getDiNoiseFilterSupported(DioFeatures *obj)                                                       { return DAQFN_TV(DNLAPI_DIO, 29, int8)(obj);     }
__inline IArray *          DioFeatures_getDiNoiseFilterOfChannels(DioFeatures *obj)                                                      { return DAQFN_TV(DNLAPI_DIO, 30, IArray *)(obj); }
__inline void              DioFeatures_getDiNoiseFilterBlockTimeRange(DioFeatures *obj, MathInterval *x)                                 {        DAQFN_VT(DNLAPI_DIO, 31, MathInterval *)(obj, x); }

/*di interrupt features*/
__inline int8              DioFeatures_getDiintSupported(DioFeatures *obj)                                                               { return DAQFN_TV(DNLAPI_DIO, 32, int8)(obj);     }
__inline int8              DioFeatures_getDiintGateSupported(DioFeatures *obj)                                                           { return DAQFN_TV(DNLAPI_DIO, 33, int8)(obj);     }
__inline int8              DioFeatures_getDiCosintSupported(DioFeatures *obj)                                                            { return DAQFN_TV(DNLAPI_DIO, 34, int8)(obj);     }
__inline int8              DioFeatures_getDiPmintSupported(DioFeatures *obj)                                                             { return DAQFN_TV(DNLAPI_DIO, 35, int8)(obj);     }
__inline IArray *          DioFeatures_getDiintTriggerEdges(DioFeatures *obj)                                                            { return DAQFN_TV(DNLAPI_DIO, 36, IArray *)(obj); }
__inline IArray *          DioFeatures_getDiintOfChannels(DioFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_DIO, 37, IArray *)(obj); }
__inline IArray *          DioFeatures_getDiintGateOfChannels(DioFeatures *obj)                                                          { return DAQFN_TV(DNLAPI_DIO, 38, IArray *)(obj); }
__inline IArray *          DioFeatures_getDiCosintOfPorts(DioFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_DIO, 39, IArray *)(obj); }
__inline IArray *          DioFeatures_getDiPmintOfPorts(DioFeatures *obj)                                                               { return DAQFN_TV(DNLAPI_DIO, 40, IArray *)(obj); }
__inline IArray *          DioFeatures_getDiSnapEventSources(DioFeatures *obj)                                                           { return DAQFN_TV(DNLAPI_DIO, 41, IArray *)(obj); }

/*DO features */     
__inline IArray *          DioFeatures_getDoDataMask(DioFeatures *obj)                                                                   { return DAQFN_TV(DNLAPI_DIO, 42, IArray *)(obj);         }
__inline IArray *          DioFeatures_getDoFreezeSignalSources(DioFeatures *obj)                                                        { return DAQFN_TV(DNLAPI_DIO, 43, IArray *)(obj);         }
__inline void              DioFeatures_getDoReflectWdtFeedIntervalRange(DioFeatures *obj, MathInterval *x)                               {        DAQFN_VT(DNLAPI_DIO, 44, MathInterval *)(obj, x);}
__inline Depository        DioFeatures_getDoPresetValueDepository(DioFeatures *obj)                                                      { return DAQFN_TV(DNLAPI_DIO, 63, Depository)(obj);       }

/*For compatible*/
__inline IArray *          DioFeatures_getDataMask(DioFeatures *obj)                                                                     { return DAQFN_TV(DNLAPI_DIO, 62, IArray *)(obj); }

/* ----------------------------------------------------------*/
/*InstantDiCtrl                                              */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         InstantDiCtrl_LoadProfile(InstantDiCtrl *obj, wchar_t const *fileName)                                        { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              InstantDiCtrl_Cleanup(InstantDiCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              InstantDiCtrl_Dispose(InstantDiCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              InstantDiCtrl_getSelectedDevice(InstantDiCtrl *obj, DeviceInformation *x)                                     {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         InstantDiCtrl_setSelectedDevice(InstantDiCtrl *obj, DeviceInformation const *x)                               { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      InstantDiCtrl_getState(void *obj)                                                                             { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj); }
__inline DeviceCtrl *      InstantDiCtrl_getDevice(InstantDiCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj); }
__inline IArray *          InstantDiCtrl_getSupportedDevices(InstantDiCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);     }
__inline IArray *          InstantDiCtrl_getSupportedModes(InstantDiCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);     }
/*base2*/                
__inline DioFeatures *     InstantDiCtrl_getFeatures(InstantDiCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_DIO, 45, DioFeatures *)(obj); }
__inline int32             InstantDiCtrl_getPortCount(InstantDiCtrl *obj)                                                                { return DAQFN_TV(DNLAPI_DIO, 46, int32)(obj);         }
__inline IArray *          InstantDiCtrl_getPorts(InstantDiCtrl *obj)                                                                    { return DAQFN_TV(DNLAPI_DIO, 47, IArray *)(obj);      }
/* Instant DI Methods & Properties */
__inline ErrorCode         InstantDiCtrl_ReadAny(InstantDiCtrl *obj, int32 portStart, int32 portCount, uint8 data[])                     { return DAQFN_ETTT(DNLAPI_DIO, 48, int32, int32, uint8 *)(obj, portStart, portCount, data);}
__inline ErrorCode         InstantDiCtrl_ReadBit(InstantDiCtrl *obj, int32 port, int32 bit, uint8* data)                                 { return DAQFN_ETTT(DNLAPI_DIO, 49, int32, int32, uint8 *)(obj, port, bit, data);           }
__inline ErrorCode         InstantDiCtrl_SnapStart(InstantDiCtrl *obj)                                                                   { return DAQFN_EV(DNLAPI_DIO, 50)(obj);                }
__inline ErrorCode         InstantDiCtrl_SnapStop(InstantDiCtrl *obj)                                                                    { return DAQFN_EV(DNLAPI_DIO, 51)(obj);                }
__inline double            InstantDiCtrl_getNoiseFilterBlockTime(InstantDiCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_DIO, 52, double)(obj);        }
__inline ErrorCode         InstantDiCtrl_setNoiseFilterBlockTime(InstantDiCtrl *obj, double value)                                       { return DAQFN_ET(DNLAPI_DIO, 53, double)(obj, value); }
__inline IArray *          InstantDiCtrl_getNoiseFilter(InstantDiCtrl *obj)                                                              { return DAQFN_TV(DNLAPI_DIO, 54, IArray *)(obj);      }
__inline IArray *          InstantDiCtrl_getDiintChannels(InstantDiCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_DIO, 55, IArray *)(obj);      }
__inline IArray *          InstantDiCtrl_getDiCosintPorts(InstantDiCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_DIO, 56, IArray *)(obj);      }
__inline IArray *          InstantDiCtrl_getDiPmintPorts(InstantDiCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_DIO, 57, IArray *)(obj);      }
/*creator*/
__inline InstantDiCtrl *   InstantDiCtrl_Create(void)                                                                                    { return (InstantDiCtrl *)DaqCtrlBase_Create(SceInstantDi); } 

/*Helpers*/
__inline void              InstantDiCtrl_addInterruptHandler(InstantDiCtrl *obj,DiSnapEventProc proc, void *userParam)                   { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtDiInterrupt,    (DaqEventProc)proc, userParam);}
__inline void              InstantDiCtrl_removeInterruptHandler(InstantDiCtrl *obj,DiSnapEventProc proc, void *userParam)                { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtDiInterrupt,    (DaqEventProc)proc, userParam);}
__inline void              InstantDiCtrl_addChangeOfStateHandler(InstantDiCtrl *obj,DiSnapEventProc proc, void *userParam)               { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtDiStatusChange, (DaqEventProc)proc, userParam);}
__inline void              InstantDiCtrl_removeChangeOfStateHandler(InstantDiCtrl *obj,DiSnapEventProc proc, void *userParam)            { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtDiStatusChange, (DaqEventProc)proc, userParam);}
__inline void              InstantDiCtrl_addPatternMatchHandler(InstantDiCtrl *obj,DiSnapEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtDiPatternMatch, (DaqEventProc)proc, userParam);}
__inline void              InstantDiCtrl_removePatternMatchHandler(InstantDiCtrl *obj,DiSnapEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtDiPatternMatch, (DaqEventProc)proc, userParam);}

/* ----------------------------------------------------------*/
/*InstantDoCtrl                                              */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         InstantDoCtrl_LoadProfile(InstantDoCtrl *obj, wchar_t const *fileName)                                        { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              InstantDoCtrl_Cleanup(InstantDoCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              InstantDoCtrl_Dispose(InstantDoCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              InstantDoCtrl_getSelectedDevice(InstantDoCtrl *obj, DeviceInformation *x)                                     {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         InstantDoCtrl_setSelectedDevice(InstantDoCtrl *obj, DeviceInformation const *x)                               { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      InstantDoCtrl_getState(void *obj)                                                                             { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj); }
__inline DeviceCtrl *      InstantDoCtrl_getDevice(InstantDoCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj); }
__inline IArray *          InstantDoCtrl_getSupportedDevices(InstantDoCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);     }
__inline IArray *          InstantDoCtrl_getSupportedModes(InstantDoCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);     }
/*base2*/                
__inline DioFeatures *     InstantDoCtrl_getFeatures(InstantDoCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_DIO, 45, DioFeatures *)(obj); }
__inline int32             InstantDoCtrl_getPortCount(InstantDoCtrl *obj)                                                                { return DAQFN_TV(DNLAPI_DIO, 46, int32)(obj);         }
__inline IArray *          InstantDoCtrl_getPorts(InstantDoCtrl *obj)                                                                    { return DAQFN_TV(DNLAPI_DIO, 47, IArray *)(obj);      }
/* Instant DO methods */ 
__inline ErrorCode         InstantDoCtrl_WriteAny(InstantDoCtrl *obj, int32 portStart, int32 portCount, uint8 data[])                    { return DAQFN_ETTT(DNLAPI_DIO, 58, int32, int32, uint8 *)(obj, portStart, portCount, data); }
__inline ErrorCode         InstantDoCtrl_ReadAny(InstantDoCtrl *obj, int32 portStart, int32 portCount, uint8 data[])                     { return DAQFN_ETTT(DNLAPI_DIO, 59, int32, int32, uint8 *)(obj, portStart, portCount, data); }
__inline ErrorCode         InstantDoCtrl_WriteBit(InstantDoCtrl *obj, int32 port, int32 bit, uint8 data)                                 { return DAQFN_ETTT(DNLAPI_DIO, 60, int32, int32, uint8  )(obj, port, bit, data);            }
__inline ErrorCode         InstantDoCtrl_ReadBit(InstantDoCtrl *obj, int32 port, int32 bit, uint8* data)                                 { return DAQFN_ETTT(DNLAPI_DIO, 61, int32, int32, uint8 *)(obj, port, bit, data);            }
/*creator*/
__inline InstantDoCtrl *   InstantDoCtrl_Create(void)                                                                                    { return (InstantDoCtrl *)DaqCtrlBase_Create(SceInstantDo); } 

/*************************************************************/
/*ctrl class : Counter related                               */
/* ----------------------------------------------------------*/
/*Counter Indexer*/
/* ----------------------------------------------------------*/
__inline void              CounterIndexer_Dispose(ICounterIndexer *obj)                                                                         {        DAQFN_VV(DNLAPI_CNTR, 0)(obj);         }
__inline int32             CounterIndexer_getLength(ICounterIndexer *obj)                                                                       { return DAQFN_TV(DNLAPI_CNTR, 1, int32)(obj);  }
__inline IArray *          CounterIndexer_getItem(ICounterIndexer *obj, int32 channel)                                                          { return DAQFN_TT(DNLAPI_CNTR, 2, IArray *, int32)(obj, channel); }

/* ----------------------------------------------------------*/
/*Counter features                                           */
/* ----------------------------------------------------------*/
/*Channel features*/
__inline int32             CntrFeatures_getChannelCountMax(CntrFeatures *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 3, int32)(obj);        }
__inline int32             CntrFeatures_getResolution(CntrFeatures *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 4, int32)(obj);        }
__inline int32             CntrFeatures_getDataSize(CntrFeatures *obj)                                                                   { return DAQFN_TV(DNLAPI_CNTR, 5, int32)(obj);        }
__inline ICounterIndexer * CntrFeatures_getCapabilities(CntrFeatures *obj)                                                               { return DAQFN_TV(DNLAPI_CNTR, 6, ICounterIndexer *)(obj); }

/*noise filter features*/
__inline int8              CntrFeatures_getNoiseFilterSupported(CntrFeatures *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 7, int8)(obj);     }
__inline IArray *          CntrFeatures_getNoiseFilterOfChannels(CntrFeatures *obj)                                                      { return DAQFN_TV(DNLAPI_CNTR, 8, IArray *)(obj); }
__inline void              CntrFeatures_getNoiseFilterBlockTimeRange(CntrFeatures *obj, MathInterval *x)                                 {        DAQFN_VT(DNLAPI_CNTR, 9, MathInterval *)(obj, x); } 

/*event counting features*/
__inline IArray *          CntrFeatures_getEcClockPolarities(CntrFeatures *obj)                                                          { return DAQFN_TV(DNLAPI_CNTR, 10, IArray *)(obj); }
__inline IArray *          CntrFeatures_getEcGatePolarities(CntrFeatures *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 11, IArray *)(obj); }
__inline IArray *          CntrFeatures_getEcGateControlOfChannels(CntrFeatures *obj)                                                    { return DAQFN_TV(DNLAPI_CNTR, 12, IArray *)(obj); }

/*frequency measurement features*/                                                                      
__inline IArray *          CntrFeatures_getFmMethods(CntrFeatures *obj)                                                                  { return DAQFN_TV(DNLAPI_CNTR, 13, IArray *)(obj); }

/*one-shot features*/                                                                                   
__inline ICounterIndexer * CntrFeatures_getOsClockSources(CntrFeatures *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR, 14, ICounterIndexer *)(obj);}
__inline IArray *          CntrFeatures_getOsClockPolarities(CntrFeatures *obj)                                                          { return DAQFN_TV(DNLAPI_CNTR, 15, IArray *)(obj);      }
__inline ICounterIndexer * CntrFeatures_getOsGateSources(CntrFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_CNTR, 16, ICounterIndexer *)(obj);}
__inline IArray *          CntrFeatures_getOsGatePolarities(CntrFeatures *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 17, IArray *)(obj);      }
__inline IArray *          CntrFeatures_getOsOutSignals(CntrFeatures *obj)                                                               { return DAQFN_TV(DNLAPI_CNTR, 18, IArray *)(obj);      }
__inline void              CntrFeatures_getOsDelayCountRange(CntrFeatures *obj, MathInterval *x)                                         {        DAQFN_VT(DNLAPI_CNTR, 19, MathInterval *)(obj, x); }

/*timer/pulse features */                                                                               
__inline IArray *          CntrFeatures_getTmrGateControlOfChannels(CntrFeatures *obj)                                                   { return DAQFN_TV(DNLAPI_CNTR, 20, IArray *)(obj); }
__inline IArray *          CntrFeatures_getTmrGatePolarities(CntrFeatures *obj)                                                          { return DAQFN_TV(DNLAPI_CNTR, 21, IArray *)(obj); }
__inline IArray *          CntrFeatures_getTmrOutSignals(CntrFeatures *obj)                                                              { return DAQFN_TV(DNLAPI_CNTR, 22, IArray *)(obj); }
__inline void              CntrFeatures_getTmrFrequencyRange(CntrFeatures *obj, MathInterval *x)                                         {        DAQFN_VT(DNLAPI_CNTR, 23, MathInterval *)(obj, x); }

/*pulse width measurement features*/                                                                    
__inline IArray *          CntrFeatures_getPiCascadeGroup(CntrFeatures *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR, 24, IArray *)(obj); }

/*pulse width modulation features*/                                                                     
__inline IArray *          CntrFeatures_getPoGateControlOfChannels(CntrFeatures *obj)                                                    { return DAQFN_TV(DNLAPI_CNTR, 25, IArray *)(obj); }
__inline IArray *          CntrFeatures_getPoGatePolarities(CntrFeatures *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 26, IArray *)(obj); }
__inline IArray *          CntrFeatures_getPoOutSignals(CntrFeatures *obj)                                                               { return DAQFN_TV(DNLAPI_CNTR, 27, IArray *)(obj); }
__inline void              CntrFeatures_getPoHiPeriodRange(CntrFeatures *obj, MathInterval *x)                                           {        DAQFN_VT(DNLAPI_CNTR, 28, MathInterval *)(obj, x); } 
__inline void              CntrFeatures_getPoLoPeriodRange(CntrFeatures *obj, MathInterval *x)                                           {        DAQFN_VT(DNLAPI_CNTR, 29, MathInterval *)(obj, x); } 
__inline void              CntrFeatures_getPoOutCountRange(CntrFeatures *obj, MathInterval *x)                                           {        DAQFN_VT(DNLAPI_CNTR, 30, MathInterval *)(obj, x); }

/*up-down counter features */
__inline IArray *          CntrFeatures_getUdCountingTypes(CntrFeatures *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 31, IArray *)(obj); }
__inline IArray *          CntrFeatures_getUdInitialValues(CntrFeatures *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 32, IArray *)(obj); }
__inline IArray *          CntrFeatures_getUdSnapEventSources(CntrFeatures *obj)                                                         { return DAQFN_TV(DNLAPI_CNTR, 33, IArray *)(obj); }

/*new*/
__inline void              CntrFeatures_getMeasurementTimeoutRange(CntrFeatures *obj, MathInterval *x)                                   {        DAQFN_VT(DNLAPI_CNTR, 129, MathInterval *)(obj, x); }
 
 /*buffered counter*/
__inline int32             CntrFeatures_getTriggerCount(CntrFeatures *obj)                                                               { return DAQFN_TV(DNLAPI_CNTR, 132, int32)(obj);              }
__inline IArray *          CntrFeatures_getTriggerSources(CntrFeatures *obj)                                                             { return DAQFN_TT(DNLAPI_CNTR, 133, IArray *, int32)(obj, 0); }
__inline IArray *          CntrFeatures_getTriggerActions(CntrFeatures *obj)                                                             { return DAQFN_TT(DNLAPI_CNTR, 134, IArray *, int32)(obj, 0); }
__inline ICounterIndexer * CntrFeatures_getEcSampleClockSources(CntrFeatures *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 135, ICounterIndexer *)(obj);  }
__inline ICounterIndexer * CntrFeatures_getPiSampleClockSources(CntrFeatures *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 136, ICounterIndexer *)(obj);  }
__inline ICounterIndexer * CntrFeatures_getPoSampleClockSources(CntrFeatures *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 137, ICounterIndexer *)(obj);  }
__inline ICounterIndexer * CntrFeatures_getUdSampleClockSources(CntrFeatures *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 138, ICounterIndexer *)(obj);  }
/* ----------------------------------------------------------*/
/*EcChannel                                                  */
/* ----------------------------------------------------------*/
__inline int32             EcChannel_getChannel(EcChannel *obj)                                                                          { return DAQFN_TV(DNLAPI_CNTR, 45, int32)(obj);                 }
__inline int8              EcChannel_getNoiseFiltered(EcChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_CNTR, 46, int8)(obj);                  }
__inline ErrorCode         EcChannel_setNoiseFiltered(EcChannel *obj, int8 value)                                                        { return DAQFN_ET(DNLAPI_CNTR, 47, int8)(obj, value);           }
__inline SignalPolarity    EcChannel_getClockPolarity(EcChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_CNTR, 48, SignalPolarity)(obj);        }
__inline ErrorCode         EcChannel_setClockPolarity(EcChannel *obj, SignalPolarity value)                                              { return DAQFN_ET(DNLAPI_CNTR, 49, SignalPolarity)(obj, value); }
__inline SignalPolarity    EcChannel_getGatePolarity(EcChannel *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 50, SignalPolarity)(obj);        }
__inline ErrorCode         EcChannel_setGatePolarity(EcChannel *obj, SignalPolarity value)                                               { return DAQFN_ET(DNLAPI_CNTR, 51, SignalPolarity)(obj, value); }
__inline int8              EcChannel_getGated(EcChannel *obj)                                                                            { return DAQFN_TV(DNLAPI_CNTR, 52, int8)(obj);                  }
__inline ErrorCode         EcChannel_setGated(EcChannel *obj, int8 value)                                                                { return DAQFN_ET(DNLAPI_CNTR, 53, int8)(obj, value);           }

/* ----------------------------------------------------------*/                                          
/*EventCounterCtrl                                           */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         EventCounterCtrl_LoadProfile(EventCounterCtrl *obj, wchar_t const *fileName)                                  { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              EventCounterCtrl_Cleanup(EventCounterCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              EventCounterCtrl_Dispose(EventCounterCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              EventCounterCtrl_getSelectedDevice(EventCounterCtrl *obj, DeviceInformation *x)                               {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         EventCounterCtrl_setSelectedDevice(EventCounterCtrl *obj, DeviceInformation const *x)                         { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      EventCounterCtrl_getState(void *obj)                                                                          { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      EventCounterCtrl_getDevice(EventCounterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          EventCounterCtrl_getSupportedDevices(EventCounterCtrl *obj)                                                   { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          EventCounterCtrl_getSupportedModes(EventCounterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                 
__inline CntrFeatures *    EventCounterCtrl_getFeatures(EventCounterCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             EventCounterCtrl_getChannelStart(EventCounterCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         EventCounterCtrl_setChannelStart(EventCounterCtrl *obj, int32 value)                                          { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             EventCounterCtrl_getChannelCount(EventCounterCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         EventCounterCtrl_setChannelCount(EventCounterCtrl *obj, int32 value)                                          { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              EventCounterCtrl_getEnabled(EventCounterCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         EventCounterCtrl_setEnabled(EventCounterCtrl *obj, int8 enabled)                                              { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              EventCounterCtrl_getRunning(EventCounterCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            EventCounterCtrl_getNoiseFilterBlockTime(EventCounterCtrl *obj)                                               { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         EventCounterCtrl_setNoiseFilterBlockTime(EventCounterCtrl *obj, double value)                                 { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   EventCounterCtrl_getNoiseFilter(EventCounterCtrl *obj)                                                        { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
__inline double            EventCounterCtrl_getMeasurementTimeout(EventCounterCtrl *obj)                                                 { return DAQFN_TV(DNLAPI_CNTR, 130, double)(obj);        }
__inline ErrorCode         EventCounterCtrl_setMeasurementTimeout(EventCounterCtrl *obj, double value)                                   { return DAQFN_ET(DNLAPI_CNTR, 131, double)(obj, value); }
/* Event counter methods */
__inline ErrorCode         EventCounterCtrl_Read(EventCounterCtrl *obj, int32 count, int32 *buffer)                                      { return DAQFN_ETT(DNLAPI_CNTR, 54, int32, int32 *)(obj, count, buffer); }
__inline IArray *          EventCounterCtrl_getChannels(EventCounterCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 55, IArray *)(obj);                       }
/*creator*/
__inline EventCounterCtrl* EventCounterCtrl_Create(void)                                                                                 { return (EventCounterCtrl *)DaqCtrlBase_Create(SceEventCounter); } 

/*BufferedEventCounterCtrl                                   */
/* ----------------------------------------------------------*/
/*base1*/
__inline ErrorCode         BufferedEventCounterCtrl_LoadProfile(BufferedEventCounterCtrl *obj, wchar_t const *fileName)                                  { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              BufferedEventCounterCtrl_Cleanup(BufferedEventCounterCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              BufferedEventCounterCtrl_Dispose(BufferedEventCounterCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              BufferedEventCounterCtrl_getSelectedDevice(BufferedEventCounterCtrl *obj, DeviceInformation *x)                               {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         BufferedEventCounterCtrl_setSelectedDevice(BufferedEventCounterCtrl *obj, DeviceInformation const *x)                         { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      BufferedEventCounterCtrl_getState(void *obj)                                                                                  { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      BufferedEventCounterCtrl_getDevice(BufferedEventCounterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          BufferedEventCounterCtrl_getSupportedDevices(BufferedEventCounterCtrl *obj)                                                   { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          BufferedEventCounterCtrl_getSupportedModes(BufferedEventCounterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/
__inline CntrFeatures    * BufferedEventCounterCtrl_getFeatures(BufferedEventCounterCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             BufferedEventCounterCtrl_getChannelStart(BufferedEventCounterCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         BufferedEventCounterCtrl_setChannelStart(BufferedEventCounterCtrl *obj, int32 value)                                          { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             BufferedEventCounterCtrl_getChannelCount(BufferedEventCounterCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         BufferedEventCounterCtrl_setChannelCount(BufferedEventCounterCtrl *obj, int32 value)                                          { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              BufferedEventCounterCtrl_getEnabled(BufferedEventCounterCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         BufferedEventCounterCtrl_setEnabled(BufferedEventCounterCtrl *obj, int8 enable)                                               { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enable);  }
__inline int8              BufferedEventCounterCtrl_getRunning(BufferedEventCounterCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            BufferedEventCounterCtrl_getNoiseFilterBlockTime(BufferedEventCounterCtrl *obj)                                               { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         BufferedEventCounterCtrl_setNoiseFilterBlockTime(BufferedEventCounterCtrl *obj, double value)                                 { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   BufferedEventCounterCtrl_getNoiseFilter(BufferedEventCounterCtrl *obj)                                                        { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
__inline double            BufferedEventCounterCtrl_getMeasurementTimeout(BufferedEventCounterCtrl *obj)                                                 { return DAQFN_TV(DNLAPI_CNTR, 130, double)(obj);        }
__inline ErrorCode         BufferedEventCounterCtrl_setMeasurementTimeout(BufferedEventCounterCtrl *obj, double value)                                   { return DAQFN_ET(DNLAPI_CNTR, 131, double)(obj, value); }

/* Buffered Event counter methods & properties */
__inline int32             BufferedEventCounterCtrl_getBufferCapacity(BufferedEventCounterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_CNTR,  183,  int32)(obj);              }
__inline Trigger*          BufferedEventCounterCtrl_getTrigger(BufferedEventCounterCtrl *obj)                                                            { return DAQFN_TT(DNLAPI_CNTR,  141, Trigger *, int32)(obj, 0); }
__inline Record*           BufferedEventCounterCtrl_getRecord(BufferedEventCounterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR,  142, Record *)(obj);            }
__inline ErrorCode         BufferedEventCounterCtrl_setSampleClkSrc(BufferedEventCounterCtrl *obj, SignalDrop drop)                                      { return DAQFN_ET(DNLAPI_CNTR,  143, SignalDrop)(obj, drop);    }
__inline SignalDrop        BufferedEventCounterCtrl_getSampleClkSrc(BufferedEventCounterCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR,  144, SignalDrop)(obj);          }
__inline void              BufferedEventCounterCtrl_getChannelMap(BufferedEventCounterCtrl *obj, int32 count, int8 *chMap)                               {        DAQFN_VTT(DNLAPI_CNTR, 139, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         BufferedEventCounterCtrl_setChannelMap(BufferedEventCounterCtrl *obj, int32 count, int8 *chMap)                               { return DAQFN_ETT(DNLAPI_CNTR, 140, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         BufferedEventCounterCtrl_Prepare(BufferedEventCounterCtrl *obj)                                                               { return DAQFN_EV(DNLAPI_CNTR,  146)(obj);                      }
__inline void              BufferedEventCounterCtrl_Release(BufferedEventCounterCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_CNTR,  147)(obj);                      }
__inline IArray *          BufferedEventCounterCtrl_getChannels(BufferedEventCounterCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR,  145, IArray *)(obj);            }
__inline ErrorCode         BufferedEventCounterCtrl_GetDataI32(BufferedEventCounterCtrl *obj, int32 cntr, int32 count, int32 dataBuf[], int32 timeout, int32 *returned, double *startTime)
{
   typedef ErrorCode (BDAQCALL *FNGETDATA)(void *, int32, int32, void*, int32, int32*, double*);
   return ((FNGETDATA)DNLAPI_CNTR[148])(obj, cntr, count, dataBuf, timeout, returned, startTime);
}

/*creator*/
__inline BufferedEventCounterCtrl* BufferedEventCounterCtrl_Create(void)                                                                                 { return (BufferedEventCounterCtrl *)DaqCtrlBase_Create(SceBufferedEventCounter); } 

/*Helpers*/
__inline void              BufferedEventCounterCtrl_addDataReadyHandler(BufferedEventCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                   { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              BufferedEventCounterCtrl_removeDataReadyHandler(BufferedEventCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              BufferedEventCounterCtrl_addOverrunHandler(BufferedEventCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              BufferedEventCounterCtrl_removeOverrunHandler(BufferedEventCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              BufferedEventCounterCtrl_addCacheOverflowHandler(BufferedEventCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              BufferedEventCounterCtrl_removeCacheOverflowHandler(BufferedEventCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)            { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              BufferedEventCounterCtrl_addStoppedHandler(BufferedEventCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCntrStopped,     (DaqEventProc)proc, userParam);}
__inline void              BufferedEventCounterCtrl_removeStoppedHandler(BufferedEventCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCntrStopped,     (DaqEventProc)proc, userParam);}

/* ----------------------------------------------------------*/                                          
/* ----------------------------------------------------------*/                                          
/*FmChannel                                                  */
/* ----------------------------------------------------------*/                                          
__inline int32             FmChannel_getChannel(FmChannel *obj)                                                                          { return DAQFN_TV(DNLAPI_CNTR, 56, int32)(obj);                   }
__inline int8              FmChannel_getNoiseFiltered(FmChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_CNTR, 57, int8)(obj);                    }
__inline ErrorCode         FmChannel_setNoiseFiltered(FmChannel *obj, int8 value)                                                        { return DAQFN_ET(DNLAPI_CNTR, 58, int8)(obj, value);             }
__inline FreqMeasureMethod FmChannel_getFmMethod(FmChannel *obj)                                                                         { return DAQFN_TV(DNLAPI_CNTR, 59, FreqMeasureMethod)(obj);       }
__inline ErrorCode         FmChannel_setFmMethod(FmChannel *obj, FreqMeasureMethod value)                                                { return DAQFN_ET(DNLAPI_CNTR, 60, FreqMeasureMethod)(obj, value);}
__inline double            FmChannel_getCollectionPeriod(FmChannel *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 61, double)(obj);                  }
__inline ErrorCode         FmChannel_setCollectionPeriod(FmChannel *obj, double value)                                                   { return DAQFN_ET(DNLAPI_CNTR, 62, double)(obj, value);           }

/* ----------------------------------------------------------*/                                        
/*FreqMeterCtrl                                              */
/* ----------------------------------------------------------*/   
/*base1*/
__inline ErrorCode         FreqMeterCtrl_LoadProfile(FreqMeterCtrl *obj, wchar_t const *fileName)                                        { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              FreqMeterCtrl_Cleanup(FreqMeterCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              FreqMeterCtrl_Dispose(FreqMeterCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              FreqMeterCtrl_getSelectedDevice(FreqMeterCtrl *obj, DeviceInformation *x)                                     {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         FreqMeterCtrl_setSelectedDevice(FreqMeterCtrl *obj, DeviceInformation const *x)                               { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      FreqMeterCtrl_getState(void *obj)                                                                             { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      FreqMeterCtrl_getDevice(FreqMeterCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          FreqMeterCtrl_getSupportedDevices(FreqMeterCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          FreqMeterCtrl_getSupportedModes(FreqMeterCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                        
__inline CntrFeatures *    FreqMeterCtrl_getFeatures(FreqMeterCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             FreqMeterCtrl_getChannelStart(FreqMeterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         FreqMeterCtrl_setChannelStart(FreqMeterCtrl *obj, int32 value)                                                { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             FreqMeterCtrl_getChannelCount(FreqMeterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         FreqMeterCtrl_setChannelCount(FreqMeterCtrl *obj, int32 value)                                                { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              FreqMeterCtrl_getEnabled(FreqMeterCtrl *obj)                                                                  { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         FreqMeterCtrl_setEnabled(FreqMeterCtrl *obj, int8 enabled)                                                    { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              FreqMeterCtrl_getRunning(FreqMeterCtrl *obj)                                                                  { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            FreqMeterCtrl_getNoiseFilterBlockTime(FreqMeterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         FreqMeterCtrl_setNoiseFilterBlockTime(FreqMeterCtrl *obj, double value)                                       { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   FreqMeterCtrl_getNoiseFilter(FreqMeterCtrl *obj)                                                              { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
__inline double            FreqMeterCtrl_getMeasurementTimeout(EventCounterCtrl *obj)                                                    { return DAQFN_TV(DNLAPI_CNTR, 130, double)(obj);        }
__inline ErrorCode         FreqMeterCtrl_setMeasurementTimeout(EventCounterCtrl *obj, double value)                                      { return DAQFN_ET(DNLAPI_CNTR, 131, double)(obj, value); }
/* Frequency meter methods */                                                                                    
__inline ErrorCode         FreqMeterCtrl_Read(FreqMeterCtrl *obj, int32 count, double *buffer)                                           { return DAQFN_ETT(DNLAPI_CNTR, 63, int32, double *)(obj, count, buffer); }
__inline IArray *          FreqMeterCtrl_getChannels(FreqMeterCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 64, IArray *)(obj);                        }
/*creator*/
__inline FreqMeterCtrl*    FreqMeterCtrl_Create(void)                                                                                    { return (FreqMeterCtrl *)DaqCtrlBase_Create(SceFreqMeter); } 

/* ----------------------------------------------------------*/
/*OsChannel                                                  */
/* ----------------------------------------------------------*/
__inline int32             OsChannel_getChannel(OsChannel *obj)                                                                          { return DAQFN_TV(DNLAPI_CNTR, 65, int32)(obj);                 }
__inline int8              OsChannel_getNoiseFiltered(OsChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_CNTR, 66, int8)(obj);                  }
__inline ErrorCode         OsChannel_setNoiseFiltered(OsChannel *obj, int8 value)                                                        { return DAQFN_ET(DNLAPI_CNTR, 67, int8)(obj, value);           }
__inline int32             OsChannel_getDelayCount(OsChannel *obj)                                                                       { return DAQFN_TV(DNLAPI_CNTR, 68, int32)(obj);                 }
__inline ErrorCode         OsChannel_setDelayCount(OsChannel *obj, int32 value)                                                          { return DAQFN_ET(DNLAPI_CNTR, 69, int32)(obj, value);          }
__inline SignalDrop        OsChannel_getClockSource(OsChannel *obj)                                                                      { return DAQFN_TV(DNLAPI_CNTR, 70, SignalDrop)(obj);            }
__inline ErrorCode         OsChannel_setClockSource(OsChannel *obj, SignalDrop value)                                                    { return DAQFN_ET(DNLAPI_CNTR, 71, SignalDrop)(obj, value);     }
__inline SignalPolarity    OsChannel_getClockPolarity(OsChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_CNTR, 72, SignalPolarity)(obj);        }
__inline ErrorCode         OsChannel_setClockPolarity(OsChannel *obj, SignalPolarity value)                                              { return DAQFN_ET(DNLAPI_CNTR, 73, SignalPolarity)(obj, value); }
__inline SignalDrop        OsChannel_getGateSource(OsChannel *obj)                                                                       { return DAQFN_TV(DNLAPI_CNTR, 74, SignalDrop)(obj);            }
__inline ErrorCode         OsChannel_setGateSource(OsChannel *obj, SignalDrop value)                                                     { return DAQFN_ET(DNLAPI_CNTR, 75, SignalDrop)(obj, value);     }
__inline SignalPolarity    OsChannel_getGatePolarity(OsChannel *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 76, SignalPolarity)(obj);        }
__inline ErrorCode         OsChannel_setGatePolarity(OsChannel *obj, SignalPolarity value)                                               { return DAQFN_ET(DNLAPI_CNTR, 77, SignalPolarity)(obj, value); }
__inline OutSignalType     OsChannel_getOutSignal(OsChannel *obj)                                                                        { return DAQFN_TV(DNLAPI_CNTR, 78, OutSignalType)(obj);         }
__inline ErrorCode         OsChannel_setOutSignal(OsChannel *obj, OutSignalType value)                                                   { return DAQFN_ET(DNLAPI_CNTR, 79, OutSignalType)(obj, value);  }

/* ----------------------------------------------------------*/                                                                          
/*OneShotCtrl                                                */                                                                          
/* ----------------------------------------------------------*/                                                                          
/*base1*/                                                                                                                                
__inline ErrorCode         OneShotCtrl_LoadProfile(OneShotCtrl *obj, wchar_t const *fileName)                                            { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              OneShotCtrl_Cleanup(OneShotCtrl *obj)                                                                         {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              OneShotCtrl_Dispose(OneShotCtrl *obj)                                                                         {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              OneShotCtrl_getSelectedDevice(OneShotCtrl *obj, DeviceInformation *x)                                         {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         OneShotCtrl_setSelectedDevice(OneShotCtrl *obj, DeviceInformation const *x)                                   { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      OneShotCtrl_getState(void *obj)                                                                               { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      OneShotCtrl_getDevice(OneShotCtrl *obj)                                                                       { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          OneShotCtrl_getSupportedDevices(OneShotCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          OneShotCtrl_getSupportedModes(OneShotCtrl *obj)                                                               { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                                                   
__inline CntrFeatures *    OneShotCtrl_getFeatures(OneShotCtrl *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             OneShotCtrl_getChannelStart(OneShotCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         OneShotCtrl_setChannelStart(OneShotCtrl *obj, int32 value)                                                    { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             OneShotCtrl_getChannelCount(OneShotCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         OneShotCtrl_setChannelCount(OneShotCtrl *obj, int32 value)                                                    { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              OneShotCtrl_getEnabled(OneShotCtrl *obj)                                                                      { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         OneShotCtrl_setEnabled(OneShotCtrl *obj, int8 enabled)                                                        { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              OneShotCtrl_getRunning(OneShotCtrl *obj)                                                                      { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            OneShotCtrl_getNoiseFilterBlockTime(OneShotCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         OneShotCtrl_setNoiseFilterBlockTime(OneShotCtrl *obj, double value)                                           { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   OneShotCtrl_getNoiseFilter(OneShotCtrl *obj)                                                                  { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
/* one shot methods */                                                                                                                      
__inline IArray *          OneShotCtrl_getChannels(OneShotCtrl *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 80, IArray *)(obj); }
/*creator*/
__inline OneShotCtrl*      OneShotCtrl_Create(void)                                                                                      { return (OneShotCtrl *)DaqCtrlBase_Create(SceOneShot); } 

/*Helpers*/
__inline void              OneShotCtrl_addOneShotHandler(OneShotCtrl *obj, CntrEventProc proc, void *userParam)                          { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCntOneShot0, (DaqEventProc)proc, userParam);}
__inline void              OneShotCtrl_removeOneShotHandler(OneShotCtrl *obj, CntrEventProc proc, void *userParam)                       { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCntOneShot0, (DaqEventProc)proc, userParam);}

/* ----------------------------------------------------------*/
/*TmrChannel                                                 */
/* ----------------------------------------------------------*/
__inline int32             TmrChannel_getChannel(TmrChannel *obj)                                                                        { return DAQFN_TV(DNLAPI_CNTR, 81, int32)(obj);                 }
__inline int8              TmrChannel_getNoiseFiltered(TmrChannel *obj)                                                                  { return DAQFN_TV(DNLAPI_CNTR, 82, int8)(obj);                  }
__inline ErrorCode         TmrChannel_setNoiseFiltered(TmrChannel *obj, int8 value)                                                      { return DAQFN_ET(DNLAPI_CNTR, 83, int8)(obj, value);           }
__inline double            TmrChannel_getFrequency(TmrChannel *obj)                                                                      { return DAQFN_TV(DNLAPI_CNTR, 84, double)(obj);                }
__inline ErrorCode         TmrChannel_setFrequency(TmrChannel *obj, double value)                                                        { return DAQFN_ET(DNLAPI_CNTR, 85, double)(obj, value);         }
__inline int8              TmrChannel_getGated(TmrChannel *obj)                                                                          { return DAQFN_TV(DNLAPI_CNTR, 86, int8)(obj);                  }
__inline ErrorCode         TmrChannel_setGated(TmrChannel *obj, int8 value)                                                              { return DAQFN_ET(DNLAPI_CNTR, 87, int8)(obj, value);           }
__inline SignalPolarity    TmrChannel_getGatePolarity(TmrChannel *obj)                                                                   { return DAQFN_TV(DNLAPI_CNTR, 88, SignalPolarity)(obj);        }
__inline ErrorCode         TmrChannel_setGatePolarity(TmrChannel *obj, SignalPolarity value)                                             { return DAQFN_ET(DNLAPI_CNTR, 89, SignalPolarity)(obj, value); }
__inline OutSignalType     TmrChannel_getOutSignal(TmrChannel *obj)                                                                      { return DAQFN_TV(DNLAPI_CNTR, 90, OutSignalType)(obj);         }
__inline ErrorCode         TmrChannel_setOutSignal(TmrChannel *obj, OutSignalType value)                                                 { return DAQFN_ET(DNLAPI_CNTR, 91, OutSignalType)(obj, value);  }

/* ----------------------------------------------------------*/                                                                             
/*TimerPulseCtrl                                             */                                                                             
/* ----------------------------------------------------------*/                                                                             
/*base1*/                                                                                                                                      
__inline ErrorCode         TimerPulseCtrl_LoadProfile(TimerPulseCtrl *obj, wchar_t const *fileName)                                      { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              TimerPulseCtrl_Cleanup(TimerPulseCtrl *obj)                                                                   {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              TimerPulseCtrl_Dispose(TimerPulseCtrl *obj)                                                                   {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              TimerPulseCtrl_getSelectedDevice(TimerPulseCtrl *obj, DeviceInformation *x)                                   {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         TimerPulseCtrl_setSelectedDevice(TimerPulseCtrl *obj, DeviceInformation const *x)                             { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      TimerPulseCtrl_getState(void *obj)                                                                            { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      TimerPulseCtrl_getDevice(TimerPulseCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          TimerPulseCtrl_getSupportedDevices(TimerPulseCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          TimerPulseCtrl_getSupportedModes(TimerPulseCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                                                   
__inline CntrFeatures *    TimerPulseCtrl_getFeatures(TimerPulseCtrl *obj)                                                               { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             TimerPulseCtrl_getChannelStart(TimerPulseCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         TimerPulseCtrl_setChannelStart(TimerPulseCtrl *obj, int32 value)                                              { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             TimerPulseCtrl_getChannelCount(TimerPulseCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         TimerPulseCtrl_setChannelCount(TimerPulseCtrl *obj, int32 value)                                              { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              TimerPulseCtrl_getEnabled(TimerPulseCtrl *obj)                                                                { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         TimerPulseCtrl_setEnabled(TimerPulseCtrl *obj, int8 enabled)                                                  { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              TimerPulseCtrl_getRunning(TimerPulseCtrl *obj)                                                                { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            TimerPulseCtrl_getNoiseFilterBlockTime(TimerPulseCtrl *obj)                                                   { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         TimerPulseCtrl_setNoiseFilterBlockTime(TimerPulseCtrl *obj, double value)                                     { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   TimerPulseCtrl_getNoiseFilter(TimerPulseCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
/* Timer/Pulse methods */                                                                                                                   
__inline IArray *          TimerPulseCtrl_getChannels(TimerPulseCtrl *obj)                                                               { return DAQFN_TV(DNLAPI_CNTR, 92, IArray *)(obj);       }
/*creator*/
__inline TimerPulseCtrl*   TimerPulseCtrl_Create(void)                                                                                   { return (TimerPulseCtrl *)DaqCtrlBase_Create(SceTimerPulse); } 

/*Helper*/
__inline void              TimerPulseCtrl_addTimerTickHandler(TimerPulseCtrl *obj, CntrEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCntTimer0, (DaqEventProc)proc, userParam); }
__inline void              TimerPulseCtrl_removeTimerTickHandler(TimerPulseCtrl *obj, CntrEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCntTimer0, (DaqEventProc)proc, userParam); }

/* ----------------------------------------------------------*/                                            
/*PiChannel                                                  */
/* ----------------------------------------------------------*/                                          
__inline int32             PiChannel_getChannel(PiChannel *obj)                                                                          { return DAQFN_TV(DNLAPI_CNTR, 93, int32)(obj);      }
__inline int8              PiChannel_getNoiseFiltered(PiChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_CNTR, 94, int8)(obj);       }
__inline ErrorCode         PiChannel_setNoiseFiltered(PiChannel *obj, int8 value)                                                        { return DAQFN_ET(DNLAPI_CNTR, 95, int8)(obj, value);}

/* ----------------------------------------------------------*/                                                                              
/*PwMeterCtrl                                                */                                                                              
/* ----------------------------------------------------------*/                                                                              
/*base1*/                                                                                                                                      
__inline ErrorCode         PwMeterCtrl_LoadProfile(PwMeterCtrl *obj, wchar_t const *fileName)                                            { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              PwMeterCtrl_Cleanup(PwMeterCtrl *obj)                                                                         {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              PwMeterCtrl_Dispose(PwMeterCtrl *obj)                                                                         {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              PwMeterCtrl_getSelectedDevice(PwMeterCtrl *obj, DeviceInformation *x)                                         {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         PwMeterCtrl_setSelectedDevice(PwMeterCtrl *obj, DeviceInformation const *x)                                   { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      PwMeterCtrl_getState(void *obj)                                                                               { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      PwMeterCtrl_getDevice(PwMeterCtrl *obj)                                                                       { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          PwMeterCtrl_getSupportedDevices(PwMeterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          PwMeterCtrl_getSupportedModes(PwMeterCtrl *obj)                                                               { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                                                     
__inline CntrFeatures *    PwMeterCtrl_getFeatures(PwMeterCtrl *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             PwMeterCtrl_getChannelStart(PwMeterCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         PwMeterCtrl_setChannelStart(PwMeterCtrl *obj, int32 value)                                                    { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             PwMeterCtrl_getChannelCount(PwMeterCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         PwMeterCtrl_setChannelCount(PwMeterCtrl *obj, int32 value)                                                    { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              PwMeterCtrl_getEnabled(PwMeterCtrl *obj)                                                                      { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         PwMeterCtrl_setEnabled(PwMeterCtrl *obj, int8 enabled)                                                        { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              PwMeterCtrl_getRunning(PwMeterCtrl *obj)                                                                      { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            PwMeterCtrl_getNoiseFilterBlockTime(PwMeterCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         PwMeterCtrl_setNoiseFilterBlockTime(PwMeterCtrl *obj, double value)                                           { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   PwMeterCtrl_getNoiseFilter(PwMeterCtrl *obj)                                                                  { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
__inline double            PwMeterCtrl_getMeasurementTimeout(EventCounterCtrl *obj)                                                      { return DAQFN_TV(DNLAPI_CNTR, 130, double)(obj);        }
__inline ErrorCode         PwMeterCtrl_setMeasurementTimeout(EventCounterCtrl *obj, double value)                                        { return DAQFN_ET(DNLAPI_CNTR, 131, double)(obj, value); }
/* PwMeterCtrl Method */                                                                                                                     
__inline IArray *          PwMeterCtrl_getChannels(PwMeterCtrl *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 96, IArray *)(obj);                            }
__inline ErrorCode         PwMeterCtrl_Read(PwMeterCtrl *obj, int32 count, PulseWidth *buffer)                                           { return DAQFN_ETT(DNLAPI_CNTR, 97, int32, PulseWidth *)(obj, count, buffer); }
/*creator*/
__inline PwMeterCtrl*      PwMeterCtrl_Create(void)                                                                                      { return (PwMeterCtrl *)DaqCtrlBase_Create(ScePwMeter); } 

/*Helpers*/
__inline void              PwMeterCtrl_addOverflowHandler(PwMeterCtrl *obj, CntrEventProc proc, void *userParam)                         { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCntPwmInOverflow0, (DaqEventProc)proc, userParam);}
__inline void              PwMeterCtrl_removeOverflowHandler(PwMeterCtrl *obj, CntrEventProc proc, void *userParam)                      { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCntPwmInOverflow0, (DaqEventProc)proc, userParam);}   

/* ----------------------------------------------------------*/                                                                              
/*BufferedPwMeterCtrl                                        */                                                                              
/* ----------------------------------------------------------*/                                                                              
/*base1*/                                                                                                                                      
__inline ErrorCode         BufferedPwMeterCtrl_LoadProfile(BufferedPwMeterCtrl *obj, wchar_t const *fileName)                           { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              BufferedPwMeterCtrl_Cleanup(BufferedPwMeterCtrl *obj)                                                        {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              BufferedPwMeterCtrl_Dispose(BufferedPwMeterCtrl *obj)                                                        {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              BufferedPwMeterCtrl_getSelectedDevice(BufferedPwMeterCtrl *obj, DeviceInformation *x)                        {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         BufferedPwMeterCtrl_setSelectedDevice(BufferedPwMeterCtrl *obj, DeviceInformation const *x)                  { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      BufferedPwMeterCtrl_getState(void *obj)                                                                      { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      BufferedPwMeterCtrl_getDevice(BufferedPwMeterCtrl *obj)                                                      { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          BufferedPwMeterCtrl_getSupportedDevices(BufferedPwMeterCtrl *obj)                                            { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          BufferedPwMeterCtrl_getSupportedModes(BufferedPwMeterCtrl *obj)                                              { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                                                     
__inline CntrFeatures *    BufferedPwMeterCtrl_getFeatures(BufferedPwMeterCtrl *obj)                                                    { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             BufferedPwMeterCtrl_getChannelStart(BufferedPwMeterCtrl *obj)                                                { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         BufferedPwMeterCtrl_setChannelStart(BufferedPwMeterCtrl *obj, int32 value)                                   { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             BufferedPwMeterCtrl_getChannelCount(BufferedPwMeterCtrl *obj)                                                { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         BufferedPwMeterCtrl_setChannelCount(BufferedPwMeterCtrl *obj, int32 value)                                   { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              BufferedPwMeterCtrl_getEnabled(BufferedPwMeterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         BufferedPwMeterCtrl_setEnabled(BufferedPwMeterCtrl *obj, int8 enabled)                                       { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              BufferedPwMeterCtrl_getRunning(BufferedPwMeterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            BufferedPwMeterCtrl_getNoiseFilterBlockTime(BufferedPwMeterCtrl *obj)                                        { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         BufferedPwMeterCtrl_setNoiseFilterBlockTime(BufferedPwMeterCtrl *obj, double value)                          { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   BufferedPwMeterCtrl_getNoiseFilter(BufferedPwMeterCtrl *obj)                                                 { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
__inline double            BufferedPwMeterCtrl_getMeasurementTimeout(EventCounterCtrl *obj)                                             { return DAQFN_TV(DNLAPI_CNTR, 130, double)(obj);        }
__inline ErrorCode         BufferedPwMeterCtrl_setMeasurementTimeout(EventCounterCtrl *obj, double value)                               { return DAQFN_ET(DNLAPI_CNTR, 131, double)(obj, value); }

/* Buffered PwMeterCtrl Method */     
__inline int32             BufferedPwMeterCtrl_getBufferCapacity(BufferedPwMeterCtrl *obj)                                              { return DAQFN_TV(DNLAPI_CNTR,  184,  int32)(obj);              }
__inline Trigger*          BufferedPwMeterCtrl_getTrigger(BufferedPwMeterCtrl *obj)                                                     { return DAQFN_TT(DNLAPI_CNTR,  151, Trigger *, int32)(obj, 0); }
__inline Record*           BufferedPwMeterCtrl_getRecord(BufferedPwMeterCtrl *obj)                                                      { return DAQFN_TV(DNLAPI_CNTR,  152, Record *)(obj);            }
__inline ErrorCode         BufferedPwMeterCtrl_setSampleClkSrc(BufferedPwMeterCtrl *obj, SignalDrop drop)                               { return DAQFN_ET(DNLAPI_CNTR,  153, SignalDrop)(obj, drop);    }
__inline SignalDrop        BufferedPwMeterCtrl_getSampleClkSrc(BufferedPwMeterCtrl *obj)                                                { return DAQFN_TV(DNLAPI_CNTR,  154, SignalDrop)(obj);          }
__inline void              BufferedPwMeterCtrl_getChannelMap(BufferedPwMeterCtrl *obj, int32 count, int8 *chMap)                        {        DAQFN_VTT(DNLAPI_CNTR, 149, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         BufferedPwMeterCtrl_setChannelMap(BufferedPwMeterCtrl *obj, int32 count, int8 *chMap)                        { return DAQFN_ETT(DNLAPI_CNTR, 150, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         BufferedPwMeterCtrl_Prepare(BufferedPwMeterCtrl *obj)                                                        { return DAQFN_EV(DNLAPI_CNTR,  156)(obj);                      }
__inline void              BufferedPwMeterCtrl_Release(BufferedPwMeterCtrl *obj)                                                        {        DAQFN_VV(DNLAPI_CNTR,  157)(obj);                      }
__inline IArray *          BufferedPwMeterCtrl_getChannels(BufferedPwMeterCtrl *obj)                                                    { return DAQFN_TV(DNLAPI_CNTR,  155, IArray *)(obj);            }
__inline ErrorCode         BufferedPwMeterCtrl_GetData(BufferedPwMeterCtrl *obj, int32 cntr, int32 count, PulseWidth *buffer, int32 timeout, int32 *returned, double *startTime)
{
   typedef ErrorCode (BDAQCALL *FNGETDATA)(void *, int32, int32, void*, int32, int32*, double*);
   return ((FNGETDATA)DNLAPI_CNTR[158])(obj, cntr, count, buffer, timeout, returned, startTime);
}

/*creator*/
__inline BufferedPwMeterCtrl*      BufferedPwMeterCtrl_Create(void)                                                                     { return (BufferedPwMeterCtrl *)DaqCtrlBase_Create(SceBufferedPwMeter); } 

/*Helpers*/
__inline void              BufferedPwMeterCtrl_addDataReadyHandler(BufferedPwMeterCtrl *obj, BfdCntrEventProc proc, void *userParam)             { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              BufferedPwMeterCtrl_removeDataReadyHandler(BufferedPwMeterCtrl *obj, BfdCntrEventProc proc, void *userParam)          { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              BufferedPwMeterCtrl_addOverrunHandler(BufferedPwMeterCtrl *obj, BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              BufferedPwMeterCtrl_removeOverrunHandler(BufferedPwMeterCtrl *obj, BfdCntrEventProc proc, void *userParam)            { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              BufferedPwMeterCtrl_addCacheOverflowHandler(BufferedPwMeterCtrl *obj, BfdCntrEventProc proc, void *userParam)         { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              BufferedPwMeterCtrl_removeCacheOverflowHandler(BufferedPwMeterCtrl *obj, BfdCntrEventProc proc, void *userParam)      { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              BufferedPwMeterCtrl_addStoppedHandler(BufferedPwMeterCtrl *obj, BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCntrStopped,     (DaqEventProc)proc, userParam);}
__inline void              BufferedPwMeterCtrl_removeStoppedHandler(BufferedPwMeterCtrl *obj, BfdCntrEventProc proc, void *userParam)            { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCntrStopped,     (DaqEventProc)proc, userParam);}

/* ----------------------------------------------------------*/                                                                              
/*PoChannel                                                  */                                                                              
/* ----------------------------------------------------------*/                                                                       
__inline int32             PoChannel_getChannel(PoChannel *obj)                                                                          { return DAQFN_TV(DNLAPI_CNTR, 98, int32)(obj);                  }
__inline int8              PoChannel_getNoiseFiltered(PoChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_CNTR, 99, int8)(obj);                   }
__inline ErrorCode         PoChannel_setNoiseFiltered(PoChannel *obj, int8 value)                                                        { return DAQFN_ET(DNLAPI_CNTR, 100, int8)(obj, value);           }
__inline void              PoChannel_getPulseWidth(PoChannel *obj, PulseWidth *x)                                                        {        DAQFN_VT(DNLAPI_CNTR, 101, PulseWidth *)(obj, x);       }
__inline ErrorCode         PoChannel_setPulseWidth(PoChannel *obj, PulseWidth const *x)                                                  { return DAQFN_ET(DNLAPI_CNTR, 102, PulseWidth const*)(obj, x);  }
__inline int8              PoChannel_getGated(PoChannel *obj)                                                                            { return DAQFN_TV(DNLAPI_CNTR, 103, int8)(obj);                  }
__inline ErrorCode         PoChannel_setGated(PoChannel *obj, int8 value)                                                                { return DAQFN_ET(DNLAPI_CNTR, 104, int8)(obj, value);           }
__inline SignalPolarity    PoChannel_getGatePolarity(PoChannel *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 105, SignalPolarity)(obj);        }
__inline ErrorCode         PoChannel_setGatePolarity(PoChannel *obj, SignalPolarity value)                                               { return DAQFN_ET(DNLAPI_CNTR, 106, SignalPolarity)(obj, value); }
__inline OutSignalType     PoChannel_getOutSignal(PoChannel *obj)                                                                        { return DAQFN_TV(DNLAPI_CNTR, 107, OutSignalType)(obj);         }
__inline ErrorCode         PoChannel_setOutSignal(PoChannel *obj, OutSignalType value)                                                   { return DAQFN_ET(DNLAPI_CNTR, 108, OutSignalType)(obj, value);  }
__inline int32             PoChannel_getOutCount(PoChannel *obj)                                                                         { return DAQFN_TV(DNLAPI_CNTR, 109, int32)(obj);                 }
__inline ErrorCode         PoChannel_setOutCount(PoChannel *obj, int32 value)                                                            { return DAQFN_ET(DNLAPI_CNTR, 110, int32)(obj, value);          }

/* ----------------------------------------------------------*/                                         
/*PwModulatorCtrl                                            */
/* ----------------------------------------------------------*/  
/*base1*/                                                                                                                                      
__inline ErrorCode         PwModulatorCtrl_LoadProfile(PwModulatorCtrl *obj, wchar_t const *fileName)                                    { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              PwModulatorCtrl_Cleanup(PwModulatorCtrl *obj)                                                                 {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              PwModulatorCtrl_Dispose(PwModulatorCtrl *obj)                                                                 {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              PwModulatorCtrl_getSelectedDevice(PwModulatorCtrl *obj, DeviceInformation *x)                                 {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         PwModulatorCtrl_setSelectedDevice(PwModulatorCtrl *obj, DeviceInformation const *x)                           { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      PwModulatorCtrl_getState(void *obj)                                                                           { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      PwModulatorCtrl_getDevice(PwModulatorCtrl *obj)                                                               { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          PwModulatorCtrl_getSupportedDevices(PwModulatorCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          PwModulatorCtrl_getSupportedModes(PwModulatorCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                                            
__inline CntrFeatures *    PwModulatorCtrl_getFeatures(PwModulatorCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             PwModulatorCtrl_getChannelStart(PwModulatorCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         PwModulatorCtrl_setChannelStart(PwModulatorCtrl *obj, int32 value)                                            { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             PwModulatorCtrl_getChannelCount(PwModulatorCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         PwModulatorCtrl_setChannelCount(PwModulatorCtrl *obj, int32 value)                                            { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              PwModulatorCtrl_getEnabled(PwModulatorCtrl *obj)                                                              { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         PwModulatorCtrl_setEnabled(PwModulatorCtrl *obj, int8 enabled)                                                { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              PwModulatorCtrl_getRunning(PwModulatorCtrl *obj)                                                              { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            PwModulatorCtrl_getNoiseFilterBlockTime(PwModulatorCtrl *obj)                                                 { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         PwModulatorCtrl_setNoiseFilterBlockTime(PwModulatorCtrl *obj, double value)                                   { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   PwModulatorCtrl_getNoiseFilter(PwModulatorCtrl *obj)                                                          { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
/* Pulse width modulator methods */                                                                   
__inline IArray *          PwModulatorCtrl_getChannels(PwModulatorCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR, 111, IArray *)(obj); }
/*creator*/
__inline PwModulatorCtrl*  PwModulatorCtrl_Create(void)                                                                                  { return (PwModulatorCtrl *)DaqCtrlBase_Create(ScePwModulator); } 

/* ----------------------------------------------------------*/                                         
/*BufferedPwModulatorCtrl                                    */
/* ----------------------------------------------------------*/  
/*base1*/                                                                                                                                      
__inline ErrorCode         BufferedPwModulatorCtrl_LoadProfile(BufferedPwModulatorCtrl *obj, wchar_t const *fileName)                                  { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              BufferedPwModulatorCtrl_Cleanup(BufferedPwModulatorCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              BufferedPwModulatorCtrl_Dispose(BufferedPwModulatorCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              BufferedPwModulatorCtrl_getSelectedDevice(BufferedPwModulatorCtrl *obj, DeviceInformation *x)                               {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         BufferedPwModulatorCtrl_setSelectedDevice(BufferedPwModulatorCtrl *obj, DeviceInformation const *x)                         { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      BufferedPwModulatorCtrl_getState(void *obj)                                                                                 { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      BufferedPwModulatorCtrl_getDevice(BufferedPwModulatorCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          BufferedPwModulatorCtrl_getSupportedDevices(BufferedPwModulatorCtrl *obj)                                                   { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          BufferedPwModulatorCtrl_getSupportedModes(BufferedPwModulatorCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                                            
__inline CntrFeatures *    BufferedPwModulatorCtrl_getFeatures(BufferedPwModulatorCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             BufferedPwModulatorCtrl_getChannelStart(BufferedPwModulatorCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         BufferedPwModulatorCtrl_setChannelStart(BufferedPwModulatorCtrl *obj, int32 value)                                          { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             BufferedPwModulatorCtrl_getChannelCount(BufferedPwModulatorCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         BufferedPwModulatorCtrl_setChannelCount(BufferedPwModulatorCtrl *obj, int32 value)                                          { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              BufferedPwModulatorCtrl_getEnabled(BufferedPwModulatorCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         BufferedPwModulatorCtrl_setEnabled(BufferedPwModulatorCtrl *obj, int8 enabled)                                              { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              BufferedPwModulatorCtrl_getRunning(BufferedPwModulatorCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            BufferedPwModulatorCtrl_getNoiseFilterBlockTime(BufferedPwModulatorCtrl *obj)                                               { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         BufferedPwModulatorCtrl_setNoiseFilterBlockTime(BufferedPwModulatorCtrl *obj, double value)                                 { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   BufferedPwModulatorCtrl_getNoiseFilter(BufferedPwModulatorCtrl *obj)                                                        { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}

/* Buffered Pulse width modulator methods */ 
__inline int32             BufferedPwModulatorCtrl_getBufferCapacity(BufferedPwModulatorCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_CNTR,  186,  int32)(obj);              }
__inline Trigger*          BufferedPwModulatorCtrl_getTrigger(BufferedPwModulatorCtrl *obj)                                                            { return DAQFN_TT(DNLAPI_CNTR,  171, Trigger *, int32)(obj, 0); }
__inline int32             BufferedPwModulatorCtrl_getSamples(BufferedPwModulatorCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR,  172, int32)(obj);               } 
__inline ErrorCode         BufferedPwModulatorCtrl_setSamples(BufferedPwModulatorCtrl *obj, int32 value)                                               { return DAQFN_ET(DNLAPI_CNTR,  173, int32)(obj, value);        }
__inline int32             BufferedPwModulatorCtrl_getIntervalCount(BufferedPwModulatorCtrl *obj)                                                      { return DAQFN_TV(DNLAPI_CNTR,  174, int32)(obj);               }
__inline ErrorCode         BufferedPwModulatorCtrl_setIntervalCount(BufferedPwModulatorCtrl *obj, int32 value)                                         { return DAQFN_ET(DNLAPI_CNTR,  175, int32)(obj, value);        }
__inline int8              BufferedPwModulatorCtrl_getStreaming(BufferedPwModulatorCtrl *obj)                                                          { return DAQFN_TV(DNLAPI_CNTR,  187, int8)(obj);                }
__inline ErrorCode         BufferedPwModulatorCtrl_setStreaming(BufferedPwModulatorCtrl *obj, int8 value)                                              { return DAQFN_ET(DNLAPI_CNTR,  176, int8)(obj, value);         }

__inline ErrorCode         BufferedPwModulatorCtrl_setSampleClkSrc(BufferedPwModulatorCtrl *obj, SignalDrop drop)                                      { return DAQFN_ET(DNLAPI_CNTR,  177, SignalDrop)(obj, drop);    }
__inline SignalDrop        BufferedPwModulatorCtrl_getSampleClkSrc(BufferedPwModulatorCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR,  178, SignalDrop)(obj);          }
__inline void              BufferedPwModulatorCtrl_getChannelMap(BufferedPwModulatorCtrl *obj, int32 count, int8 *chMap)                               {        DAQFN_VTT(DNLAPI_CNTR, 169, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         BufferedPwModulatorCtrl_setChannelMap(BufferedPwModulatorCtrl *obj, int32 count, int8 *chMap)                               { return DAQFN_ETT(DNLAPI_CNTR, 170, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         BufferedPwModulatorCtrl_Prepare(BufferedPwModulatorCtrl *obj)                                                               { return DAQFN_EV(DNLAPI_CNTR,  180)(obj);                      }
__inline void              BufferedPwModulatorCtrl_Release(BufferedPwModulatorCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_CNTR,  181)(obj);                      }
__inline IArray *          BufferedPwModulatorCtrl_getChannels(BufferedPwModulatorCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR,  179, IArray *)(obj);            }
__inline ErrorCode         BufferedPwModulatorCtrl_SetData(BufferedPwModulatorCtrl *obj, int32 cntr, int32 count, PulseWidth *buffer)
{
   typedef ErrorCode (BDAQCALL *FNSETDATA)(void *, int32, int32, void*);
   return ((FNSETDATA)DNLAPI_CNTR[182])(obj, cntr, count, buffer);
}

/*creator*/
__inline BufferedPwModulatorCtrl*  BufferedPwModulatorCtrl_Create(void)                                                                                { return (BufferedPwModulatorCtrl *)DaqCtrlBase_Create(SceBufferedPwModulator); } 

/*Helpers*/
/*event*/
__inline void              BufferedPwModulatorCtrl_addDataTransmittedHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)             { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCoDataTransmitted, (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_removeDataTransmittedHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)          { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCoDataTransmitted, (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_addUnderrunHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)                    { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCoUnderrun,        (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_removeUnderrunHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)                 { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCoUnderrun,        (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_addCacheEmptiedHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCoCacheEmptied,    (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_removeCacheEmptiedHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCoCacheEmptied,    (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_addTransitStoppedHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)              { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCoTransStopped,    (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_removeTransitStoppedHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)           { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCoTransStopped,    (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_addStoppedHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCntrStopped,       (DaqEventProc)proc, userParam); }
__inline void              BufferedPwModulatorCtrl_removeStoppedHandler(BufferedPwModulatorCtrl *obj, BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCntrStopped,       (DaqEventProc)proc, userParam); }

/* ----------------------------------------------------------*/
/*UdChannel                                                  */
/* ----------------------------------------------------------*/
__inline int32             UdChannel_getChannel(UdChannel *obj)                                                                          { return DAQFN_TV(DNLAPI_CNTR, 112, int32)(obj);              }
__inline int8              UdChannel_getNoiseFiltered(UdChannel *obj)                                                                    { return DAQFN_TV(DNLAPI_CNTR, 113, int8)(obj);               }
__inline ErrorCode         UdChannel_setNoiseFiltered(UdChannel *obj, int8 value)                                                        { return DAQFN_ET(DNLAPI_CNTR, 114, int8)(obj, value);        }
__inline CountingType      UdChannel_getCountingType(UdChannel *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 115, CountingType)(obj);       }
__inline ErrorCode         UdChannel_setCountingType(UdChannel *obj, CountingType value)                                                 { return DAQFN_ET(DNLAPI_CNTR, 116, CountingType)(obj, value);}
__inline int32             UdChannel_getInitialValue(UdChannel *obj)                                                                     { return DAQFN_TV(DNLAPI_CNTR, 117, int32)(obj);              }
__inline ErrorCode         UdChannel_setInitialValue(UdChannel *obj, int32 value)                                                        { return DAQFN_ET(DNLAPI_CNTR, 118, int32)(obj, value);       }
__inline int32             UdChannel_getResetTimesByIndex(UdChannel *obj)                                                                { return DAQFN_TV(DNLAPI_CNTR, 119, int32)(obj);              }
__inline ErrorCode         UdChannel_setResetTimesByIndex(UdChannel *obj, int32 value)                                                   { return DAQFN_ET(DNLAPI_CNTR, 120, int32)(obj, value);       }

/* ----------------------------------------------------------*/
/*UdCounterCtrl                                              */
/* ----------------------------------------------------------*/
/*base1*/                                                                                                                               
__inline ErrorCode         UdCounterCtrl_LoadProfile(UdCounterCtrl *obj, wchar_t const *fileName)                                        { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              UdCounterCtrl_Cleanup(UdCounterCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              UdCounterCtrl_Dispose(UdCounterCtrl *obj)                                                                     {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              UdCounterCtrl_getSelectedDevice(UdCounterCtrl *obj, DeviceInformation *x)                                     {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         UdCounterCtrl_setSelectedDevice(UdCounterCtrl *obj, DeviceInformation const *x)                               { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      UdCounterCtrl_getState(void *obj)                                                                             { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      UdCounterCtrl_getDevice(UdCounterCtrl *obj)                                                                   { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          UdCounterCtrl_getSupportedDevices(UdCounterCtrl *obj)                                                         { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          UdCounterCtrl_getSupportedModes(UdCounterCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                                              
__inline CntrFeatures *    UdCounterCtrl_getFeatures(UdCounterCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             UdCounterCtrl_getChannelStart(UdCounterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         UdCounterCtrl_setChannelStart(UdCounterCtrl *obj, int32 value)                                                { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             UdCounterCtrl_getChannelCount(UdCounterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         UdCounterCtrl_setChannelCount(UdCounterCtrl *obj, int32 value)                                                { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              UdCounterCtrl_getEnabled(UdCounterCtrl *obj)                                                                  { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         UdCounterCtrl_setEnabled(UdCounterCtrl *obj, int8 enabled)                                                    { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              UdCounterCtrl_getRunning(UdCounterCtrl *obj)                                                                  { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            UdCounterCtrl_getNoiseFilterBlockTime(UdCounterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         UdCounterCtrl_setNoiseFilterBlockTime(UdCounterCtrl *obj, double value)                                       { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   UdCounterCtrl_getNoiseFilter(UdCounterCtrl *obj)                                                              { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
/* UdCounterCtrl methods */
__inline ErrorCode         UdCounterCtrl_SnapStart(UdCounterCtrl *obj, EventId srcId)                                                    { return DAQFN_ET(DNLAPI_CNTR, 121, EventId)(obj, srcId);                                 }
__inline ErrorCode         UdCounterCtrl_SnapStop(UdCounterCtrl *obj, EventId srcId)                                                     { return DAQFN_ET(DNLAPI_CNTR, 122, EventId)(obj, srcId);                                 }
__inline ErrorCode         UdCounterCtrl_CompareSetTable(UdCounterCtrl *obj, int32 channel, int32 count, int32 *table)                   { return DAQFN_ETTT(DNLAPI_CNTR, 123, int32, int32, int32 *)(obj, channel, count, table); }
__inline ErrorCode         UdCounterCtrl_CompareSetInterval(UdCounterCtrl *obj, int32 channel, int32 start, int32 increment,int32 count) { return DAQFN_ETTTT(DNLAPI_CNTR, 124, int32, int32, int32, int32)(obj, channel, start, increment, count); }
__inline ErrorCode         UdCounterCtrl_CompareClear(UdCounterCtrl *obj, int32 channel)                                                 { return DAQFN_ET(DNLAPI_CNTR, 125, int32)(obj, channel);                 }
__inline ErrorCode         UdCounterCtrl_ValueReset(UdCounterCtrl *obj)                                                                  { return DAQFN_EV(DNLAPI_CNTR, 126)(obj);                                 }
__inline ErrorCode         UdCounterCtrl_Read(UdCounterCtrl *obj, int32 count, int32 *buffer)                                            { return DAQFN_ETT(DNLAPI_CNTR, 127, int32, int32 *)(obj, count, buffer); }
__inline IArray *          UdCounterCtrl_getChannels(UdCounterCtrl *obj)                                                                 { return DAQFN_TV(DNLAPI_CNTR, 128, IArray *)(obj);                       }
/*creator*/
__inline UdCounterCtrl*    UdCounterCtrl_Create(void)                                                                                    { return (UdCounterCtrl *)DaqCtrlBase_Create(SceUdCounter); } 

/*Helpers*/
__inline void              UdCounterCtrl_addUdCntrEventHandler(UdCounterCtrl *obj, UdCntrEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtUdIndex0, (DaqEventProc)proc, userParam);}
__inline void              UdCounterCtrl_removeUdCntrEventHandler(UdCounterCtrl *obj, UdCntrEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtUdIndex0, (DaqEventProc)proc, userParam);}

/* ----------------------------------------------------------*/
/*BufferedUdCounterCtrl                                      */
/* ----------------------------------------------------------*/
/*base1*/                                                                                                                               
__inline ErrorCode         BufferedUdCounterCtrl_LoadProfile(BufferedUdCounterCtrl *obj, wchar_t const *fileName)                                  { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(obj, fileName);   }  
__inline void              BufferedUdCounterCtrl_Cleanup(BufferedUdCounterCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_BASE, 83)(obj);                              }
__inline void              BufferedUdCounterCtrl_Dispose(BufferedUdCounterCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_BASE, 84)(obj);                              }
__inline void              BufferedUdCounterCtrl_getSelectedDevice(BufferedUdCounterCtrl *obj, DeviceInformation *x)                               {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(obj, x);       }
__inline ErrorCode         BufferedUdCounterCtrl_setSelectedDevice(BufferedUdCounterCtrl *obj, DeviceInformation const *x)                         { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(obj, x); }
__inline ControlState      BufferedUdCounterCtrl_getState(void *obj)                                                                               { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(obj);   }
__inline DeviceCtrl *      BufferedUdCounterCtrl_getDevice(BufferedUdCounterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);   }
__inline IArray *          BufferedUdCounterCtrl_getSupportedDevices(BufferedUdCounterCtrl *obj)                                                   { return DAQFN_TV(DNLAPI_BASE, 89, IArray *)(obj);       }
__inline IArray *          BufferedUdCounterCtrl_getSupportedModes(BufferedUdCounterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_BASE, 90, IArray *)(obj);       }
/*base2*/                                                                                                                              
__inline CntrFeatures *    BufferedUdCounterCtrl_getFeatures(BufferedUdCounterCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(obj); }
__inline int32             BufferedUdCounterCtrl_getChannelStart(BufferedUdCounterCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);          }
__inline ErrorCode         BufferedUdCounterCtrl_setChannelStart(BufferedUdCounterCtrl *obj, int32 value)                                          { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, value);   }
__inline int32             BufferedUdCounterCtrl_getChannelCount(BufferedUdCounterCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(obj);          }
__inline ErrorCode         BufferedUdCounterCtrl_setChannelCount(BufferedUdCounterCtrl *obj, int32 value)                                          { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, value);   }
__inline int8              BufferedUdCounterCtrl_getEnabled(BufferedUdCounterCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 39, int8)(obj);           }
__inline ErrorCode         BufferedUdCounterCtrl_setEnabled(BufferedUdCounterCtrl *obj, int8 enabled)                                              { return DAQFN_ET(DNLAPI_CNTR, 40, int8)(obj, enabled);  }
__inline int8              BufferedUdCounterCtrl_getRunning(BufferedUdCounterCtrl *obj)                                                            { return DAQFN_TV(DNLAPI_CNTR, 41, int8)(obj);           }
__inline double            BufferedUdCounterCtrl_getNoiseFilterBlockTime(BufferedUdCounterCtrl *obj)                                               { return DAQFN_TV(DNLAPI_CNTR, 42, double)(obj);         }
__inline ErrorCode         BufferedUdCounterCtrl_setNoiseFilterBlockTime(BufferedUdCounterCtrl *obj, double value)                                 { return DAQFN_ET(DNLAPI_CNTR, 43, double)(obj, value);  }
__inline NosFltChannel *   BufferedUdCounterCtrl_getNoiseFilter(BufferedUdCounterCtrl *obj)                                                        { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(obj);}
/* Buffered UdCounterCtrl methods */
__inline int32             BufferedUdCounterCtrl_getBufferCapacity(BufferedUdCounterCtrl *obj)                                                     { return DAQFN_TV(DNLAPI_CNTR,  185,  int32)(obj);              }
__inline Trigger*          BufferedUdCounterCtrl_getTrigger(BufferedUdCounterCtrl *obj)                                                            { return DAQFN_TT(DNLAPI_CNTR,  161, Trigger *, int32)(obj, 0); }
__inline Record*           BufferedUdCounterCtrl_getRecord(BufferedUdCounterCtrl *obj)                                                             { return DAQFN_TV(DNLAPI_CNTR,  162, Record *)(obj);            }
__inline ErrorCode         BufferedUdCounterCtrl_setSampleClkSrc(BufferedUdCounterCtrl *obj, SignalDrop drop)                                      { return DAQFN_ET(DNLAPI_CNTR,  163, SignalDrop)(obj, drop);    }
__inline SignalDrop        BufferedUdCounterCtrl_getSampleClkSrc(BufferedUdCounterCtrl *obj)                                                       { return DAQFN_TV(DNLAPI_CNTR,  164, SignalDrop)(obj);          }
__inline void              BufferedUdCounterCtrl_getChannelMap(BufferedUdCounterCtrl *obj, int32 count, int8 *chMap)                               {        DAQFN_VTT(DNLAPI_CNTR, 159, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         BufferedUdCounterCtrl_setChannelMap(BufferedUdCounterCtrl *obj, int32 count, int8 *chMap)                               { return DAQFN_ETT(DNLAPI_CNTR, 160, int32, int8 *)(obj, count, chMap); }
__inline ErrorCode         BufferedUdCounterCtrl_Prepare(BufferedUdCounterCtrl *obj)                                                               { return DAQFN_EV(DNLAPI_CNTR,  166)(obj);                      }
__inline void              BufferedUdCounterCtrl_Release(BufferedUdCounterCtrl *obj)                                                               {        DAQFN_VV(DNLAPI_CNTR,  167)(obj);                      }
__inline IArray *          BufferedUdCounterCtrl_getChannels(BufferedUdCounterCtrl *obj)                                                           { return DAQFN_TV(DNLAPI_CNTR,  165, IArray *)(obj);            }
__inline ErrorCode         BufferedUdCounterCtrl_GetDataI32(BufferedUdCounterCtrl *obj, int32 cntr, int32 count, int32 dataBuf[], int32 timeout, int32 *returned, double *startTime)
{
   typedef ErrorCode (BDAQCALL *FNGETDATA)(void *, int32, int32, void*, int32, int32*, double*);
   return ((FNGETDATA)DNLAPI_CNTR[168])(obj, cntr, count, dataBuf, timeout, returned, startTime);
}

/*creator*/
__inline BufferedUdCounterCtrl*    BufferedUdCounterCtrl_Create(void)                                                                              { return (BufferedUdCounterCtrl *)DaqCtrlBase_Create(SceBufferedUdCounter); } 

/*Helpers*/
__inline void              BufferedUdCounterCtrl_addDataReadyHandler(BufferedUdCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                   { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              BufferedUdCounterCtrl_removeDataReadyHandler(BufferedUdCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
__inline void              BufferedUdCounterCtrl_addOverrunHandler(BufferedUdCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              BufferedUdCounterCtrl_removeOverrunHandler(BufferedUdCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
__inline void              BufferedUdCounterCtrl_addCacheOverflowHandler(BufferedUdCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              BufferedUdCounterCtrl_removeCacheOverflowHandler(BufferedUdCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)            { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
__inline void              BufferedUdCounterCtrl_addStoppedHandler(BufferedUdCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                     { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, EvtCntrStopped,     (DaqEventProc)proc, userParam);}
__inline void              BufferedUdCounterCtrl_removeStoppedHandler(BufferedUdCounterCtrl *obj, BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, EvtCntrStopped,     (DaqEventProc)proc, userParam);}

/**********************************************************/
/* For compatible                                         */
/**********************************************************/
#if !defined(__cplusplus) || defined(_BDAQ_C_INTERFACE)

   typedef IArray        ICollection;
   typedef ICounterIndexer  CounterCapabilityIndexer;
   typedef AiChannel     AnalogInputChannel;
   typedef AoChannel     AnalogChannel;
   typedef NosFltChannel NoiseFilterChannel;
   typedef DioFeatures   DiFeatures;
   typedef DioFeatures   DoFeatures;
   typedef CntrFeatures  EventCounterFeatures;
   typedef CntrFeatures  FreqMeterFeatures;
   typedef CntrFeatures  OneShotFeatures;
   typedef CntrFeatures  TimerPulseFeatures;
   typedef CntrFeatures  PwMeterFeatures;
   typedef CntrFeatures  PwModulatorFeatures;
   typedef CntrFeatures  UdCounterFeatures;

   typedef struct DeviceEventHandler {
      void (BDAQCALL *DeviceEvent)(void *obj, void *sender, DeviceEventArgs *args);
   } DeviceEventHandler;
   typedef struct DeviceEventListener {
      DeviceEventHandler const *vtbl;
   }DeviceEventListener;

   typedef struct BfdAiEventHandler {
      void (BDAQCALL *BfdAiEvent)(void *obj, void *sender, BfdAiEventArgs *args);
   } BfdAiEventHandler;
   typedef struct BfdAiEventListener {
      BfdAiEventHandler const *vtbl;
   } BfdAiEventListener;

   typedef struct BfdAoEventHandler {
      void (BDAQCALL *BfdAoEvent)(void *obj, void *sender, BfdAoEventArgs *args);
   } BfdAoEventHandler;
   typedef struct BfdAoEventListener {
      BfdAoEventHandler const *vtbl;
   } BfdAoEventListener;

   typedef struct DiSnapEventHandler {
      void (BDAQCALL *DiSnapEvent)(void *obj, void *sender, DiSnapEventArgs *args);
   } DiSnapEventHandler;
   typedef struct DiSnapEventListener {
      DiSnapEventHandler const *vtbl;
   } DiSnapEventListener;

   typedef struct CntrEventHandler {
      void (BDAQCALL *CntrEvent)(void *obj, void *sender, CntrEventArgs *args);
   } CntrEventHandler;
   typedef struct CntrEventListener {
      CntrEventHandler const *vtbl;
   } CntrEventListener;

   typedef struct UdCntrEventHandler {
      void (BDAQCALL *UdCntrEvent)(void *obj, void *sender, UdCntrEventArgs *args);
   } UdCntrEventHandler;
   typedef struct UdCntrEventListener {
      UdCntrEventHandler const *vtbl;
   } UdCntrEventListener;

   __inline void BDAQCALL xxxCtrl_OnDeviceEvent(void *sender, DeviceEventArgs *args, void *userParam)
   {
      ((DeviceEventListener *)userParam)->vtbl->DeviceEvent(userParam, sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnBfdAiEvent(void *sender, BfdAiEventArgs *args, void *userParam)
   {
      ((BfdAiEventListener *)userParam)->vtbl->BfdAiEvent(userParam, sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnBfdAoEvent(void *sender, BfdAoEventArgs *args, void *userParam)
   {
      ((BfdAoEventListener *)userParam)->vtbl->BfdAoEvent(userParam, sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnDiSnapEvent(void *sender, DiSnapEventArgs *args, void *userParam)
   {
      ((DiSnapEventListener *)userParam)->vtbl->DiSnapEvent(userParam, sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnCntrEvent(void *sender, CntrEventArgs *args, void *userParam)
   {
      ((CntrEventListener *)userParam)->vtbl->CntrEvent(userParam, sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnUdCntrEvent(void *sender, UdCntrEventArgs *args, void *userParam)
   {
      ((UdCntrEventListener *)userParam)->vtbl->UdCntrEvent(userParam, sender, args);
   }
#else
   class DeviceEventListener
   {
   public:
      virtual void BDAQCALL DeviceEvent(void * sender, DeviceEventArgs * args) = 0;
   };
   class BfdAiEventListener
   {
   public:
      virtual void BDAQCALL BfdAiEvent(void * sender, BfdAiEventArgs * args) = 0;
   };
   class BfdAoEventListener
   {
   public:
      virtual void BDAQCALL BfdAoEvent(void * sender, BfdAoEventArgs * args) = 0;
   };
   class DiSnapEventListener
   {
   public:
      virtual void BDAQCALL DiSnapEvent(void * sender, DiSnapEventArgs * args) = 0;
   };
   class CntrEventListener
   {
   public:
      virtual void BDAQCALL CntrEvent(void * sender, CntrEventArgs * args) = 0;
   };
   class UdCntrEventListener
   {
   public:
      virtual void BDAQCALL UdCntrEvent(void * sender, UdCntrEventArgs * args) = 0;
   };

   __inline void BDAQCALL xxxCtrl_OnDeviceEvent(void *sender, DeviceEventArgs *args, void *userParam)
   {
      ((DeviceEventListener *)userParam)->DeviceEvent(sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnBfdAiEvent(void *sender, BfdAiEventArgs *args, void *userParam)
   {
      ((BfdAiEventListener *)userParam)->BfdAiEvent(sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnBfdAoEvent(void *sender, BfdAoEventArgs *args, void *userParam)
   {
      ((BfdAoEventListener *)userParam)->BfdAoEvent(sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnDiSnapEvent(void *sender, DiSnapEventArgs *args, void *userParam)
   {
      ((DiSnapEventListener *)userParam)->DiSnapEvent(sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnCntrEvent(void *sender, CntrEventArgs *args, void *userParam)
   {
      ((CntrEventListener *)userParam)->CntrEvent(sender, args);
   }
   __inline void BDAQCALL xxxCtrl_OnUdCntrEvent(void *sender, UdCntrEventArgs *args, void *userParam)
   {
      ((UdCntrEventListener *)userParam)->UdCntrEvent(sender, args);
   }
#endif

/*
For compatible : helper method
*/
__inline void xxxCtrl_addDeviceEventListener   (void *obj, EventId id, DeviceEventListener *listener)       { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnDeviceEvent, listener); }
__inline void xxxCtrl_removeDeviceEventListener(void *obj, EventId id, DeviceEventListener *listener)       { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnDeviceEvent, listener); }
__inline void xxxCtrl_addBfdAiEventListener    (void *obj, EventId id, BfdAiEventListener  *listener)       { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnBfdAiEvent,  listener); }
__inline void xxxCtrl_removeBfdAiEventListener (void *obj, EventId id, BfdAiEventListener  *listener)       { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnBfdAiEvent,  listener); }
__inline void xxxCtrl_addBfdAoEventListener    (void *obj, EventId id, BfdAoEventListener  *listener)       { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnBfdAoEvent,  listener); }
__inline void xxxCtrl_removeBfdAoEventListener (void *obj, EventId id, BfdAoEventListener  *listener)       { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnBfdAoEvent,  listener); }
__inline void xxxCtrl_addDiSnapEventListener   (void *obj, EventId id, DiSnapEventListener *listener)       { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnDiSnapEvent, listener); }
__inline void xxxCtrl_removeDiSnapEventListener(void *obj, EventId id, DiSnapEventListener *listener)       { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnDiSnapEvent, listener); }
__inline void xxxCtrl_addCntrEventListener     (void *obj, EventId id, CntrEventListener   *listener)       { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnCntrEvent,   listener); }
__inline void xxxCtrl_removeCntrEventListener  (void *obj, EventId id, CntrEventListener   *listener)       { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnCntrEvent,   listener); }
__inline void xxxCtrl_addUdCntrEventListener   (void *obj, EventId id, UdCntrEventListener *listener)       { DaqCtrlBase_addEventHandler   ((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnUdCntrEvent, listener); }
__inline void xxxCtrl_removeUdCntrEventListener(void *obj, EventId id, UdCntrEventListener *listener)       { DaqCtrlBase_removeEventHandler((DaqCtrlBase*)obj, id, (DaqEventProc)xxxCtrl_OnUdCntrEvent, listener); }

__inline ErrorCode xxxCtrl_UpdateProperties(void *obj)
{
   DeviceCtrl *dev = DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(obj);
   return dev ? DeviceCtrl_Refresh(dev) : ErrorFuncNotInited;
}

__inline int32 xxxCntrCtrl_getChannel(void *obj) 
{
   return DAQFN_TV(DNLAPI_CNTR, 35, int32)(obj);
}
__inline int32 xxxCntrCtrl_setChannel(void *obj, int32 ch)
{
   DAQFN_ET(DNLAPI_CNTR, 38, int32)(obj, 1); // set channel count to 1
   return DAQFN_ET(DNLAPI_CNTR, 36, int32)(obj, ch); // set channel start to ch
}

__inline int32 EventCounterCtrl_getValue(EventCounterCtrl *obj)
{
   int32 value = 0; 
   EventCounterCtrl_Read(obj, 1, &value); 
   return value;
}

__inline FmChannel * FreqMeterCtrl_getCHPtr(FreqMeterCtrl *obj)
{
   IArray *chs = FreqMeterCtrl_getChannels(obj);
   return chs ? (FmChannel *)Array_getItem(chs, xxxCntrCtrl_getChannel(obj)) : NULL;
}
__inline FreqMeasureMethod FreqMeterCtrl_getMethod(FreqMeterCtrl *obj)  
{
   FmChannel *ch = FreqMeterCtrl_getCHPtr(obj);
   return ch ? FmChannel_getFmMethod(ch) : AutoAdaptive;
}
__inline ErrorCode FreqMeterCtrl_setMethod(FreqMeterCtrl *obj, FreqMeasureMethod value) 
{
   FmChannel *ch = FreqMeterCtrl_getCHPtr(obj);
   return ch ? FmChannel_setFmMethod(ch, value) : ErrorFuncNotInited;
}
__inline double FreqMeterCtrl_getCollectionPeriod(FreqMeterCtrl *obj)
{
   FmChannel *ch = FreqMeterCtrl_getCHPtr(obj);
   return ch ? FmChannel_getCollectionPeriod(ch) : 0;
}
__inline ErrorCode FreqMeterCtrl_setCollectionPeriod(FreqMeterCtrl *obj, double value)  
{
   FmChannel *ch = FreqMeterCtrl_getCHPtr(obj);
   return ch ? FmChannel_setCollectionPeriod(ch, value) : ErrorFuncNotInited;
}
__inline double FreqMeterCtrl_getValue(FreqMeterCtrl *obj)
{
   double value = 0.0;
   FreqMeterCtrl_Read(obj, 1, &value);
   return value;
}

__inline OsChannel * OneShotCtrl_getCHPtr(OneShotCtrl *obj)
{
   IArray *chs = OneShotCtrl_getChannels(obj);
   return chs ? (OsChannel *)Array_getItem(chs, xxxCntrCtrl_getChannel(obj)) : NULL;
}
__inline int32 OneShotCtrl_getDelayCount(OneShotCtrl *obj) 
{
   OsChannel *ch = OneShotCtrl_getCHPtr(obj);
   return ch ? OsChannel_getDelayCount(ch) : 0;
}
__inline ErrorCode OneShotCtrl_setDelayCount(OneShotCtrl *obj, int32 value)
{
   OsChannel *ch = OneShotCtrl_getCHPtr(obj);
   return ch ? OsChannel_setDelayCount(ch, value) : ErrorFuncNotInited;
}

__inline TmrChannel * TimerPulseCtrl_getCHPtr(TimerPulseCtrl *obj)
{
   IArray *chs = TimerPulseCtrl_getChannels(obj);
   return chs ? (TmrChannel *)Array_getItem(chs, xxxCntrCtrl_getChannel(obj)) : NULL;
}
__inline double TimerPulseCtrl_getFrequency(TimerPulseCtrl *obj)
{
   TmrChannel *ch = TimerPulseCtrl_getCHPtr(obj);
   return ch ? TmrChannel_getFrequency(ch) : 0;
}
__inline ErrorCode TimerPulseCtrl_setFrequency(TimerPulseCtrl *obj, double value)
{
   TmrChannel *ch = TimerPulseCtrl_getCHPtr(obj);
   return ch ? TmrChannel_setFrequency(ch, value) : ErrorFuncNotInited;
}

__inline void PwMeterCtrl_getValue(PwMeterCtrl *obj, PulseWidth *width)
{
   PwMeterCtrl_Read(obj, 1, width);
}

__inline PoChannel * PwModulatorCtrl_getCHPtr(PwModulatorCtrl *obj)
{
   IArray *chs = PwModulatorCtrl_getChannels(obj);
   return chs ? (PoChannel *)Array_getItem(chs, xxxCntrCtrl_getChannel(obj)) : NULL;
}
__inline void PwModulatorCtrl_getPulseWidth(PwModulatorCtrl *obj, PulseWidth *width)
{
   PoChannel *ch = PwModulatorCtrl_getCHPtr(obj);
   if (ch) { PoChannel_getPulseWidth(ch, width);      }
   else    { width->HiPeriod = width->LoPeriod = 0.0; }
}
__inline ErrorCode PwModulatorCtrl_setPulseWidth(PwModulatorCtrl *obj, PulseWidth const *width) 
{
   PoChannel *ch = PwModulatorCtrl_getCHPtr(obj);
   return ch ? PoChannel_setPulseWidth(ch, width) : ErrorFuncNotInited;
}

__inline UdChannel * UdCounterCtrl_getCHPtr(UdCounterCtrl *obj)
{
   IArray *chs = UdCounterCtrl_getChannels(obj);
   return chs ? (UdChannel *)Array_getItem(chs, xxxCntrCtrl_getChannel(obj)) : NULL;
}
__inline SignalCountingType  UdCounterCtrl_getCountingType(UdCounterCtrl *obj)
{
   UdChannel *ch = UdCounterCtrl_getCHPtr(obj);
   return ch ? UdChannel_getCountingType(ch) : CountingNone;
}
__inline ErrorCode UdCounterCtrl_setCountingType(UdCounterCtrl *obj, SignalCountingType value) 
{
   UdChannel *ch = UdCounterCtrl_getCHPtr(obj);
   return ch ? UdChannel_setCountingType(ch, value) : ErrorFuncNotInited;
}
__inline int32 UdCounterCtrl_getInitialValue(UdCounterCtrl *obj)
{
   UdChannel *ch = UdCounterCtrl_getCHPtr(obj);
   return ch ? UdChannel_getInitialValue(ch) : 0;
}
__inline ErrorCode UdCounterCtrl_setInitialValue(UdCounterCtrl *obj, int32 value)  
{
   UdChannel *ch = UdCounterCtrl_getCHPtr(obj);
   return ch ? UdChannel_setInitialValue(ch, value) : ErrorFuncNotInited;
}
__inline int32 UdCounterCtrl_getResetTimesByIndex(UdCounterCtrl *obj)
{
   UdChannel *ch = UdCounterCtrl_getCHPtr(obj);
   return ch ? UdChannel_getResetTimesByIndex(ch) : 0;
}
__inline ErrorCode UdCounterCtrl_setResetTimesByIndex(UdCounterCtrl *obj, int32 value)
{
   UdChannel *ch = UdCounterCtrl_getCHPtr(obj);
   return ch ? UdChannel_setResetTimesByIndex(ch, value) : ErrorFuncNotInited;
}
__inline int32 UdCounterCtrl_getValue(UdCounterCtrl *obj)
{
   int32 value = 0;
   UdCounterCtrl_Read(obj, 1, &value);
   return value;
}

/*
For compatible : Global
*/
#define  AdxInstantAiCtrlCreate       InstantAiCtrl_Create
#define  AdxBufferedAiCtrlCreate      BufferedAiCtrl_Create
#define  AdxInstantAoCtrlCreate       InstantAoCtrl_Create     
#define  AdxBufferedAoCtrlCreate      BufferedAoCtrl_Create  
#define  AdxInstantDiCtrlCreate       InstantDiCtrl_Create   
#define  AdxInstantDoCtrlCreate       InstantDoCtrl_Create   
#define  AdxEventCounterCtrlCreate    EventCounterCtrl_Create
#define  AdxFreqMeterCtrlCreate       FreqMeterCtrl_Create   
#define  AdxOneShotCtrlCreate         OneShotCtrl_Create     
#define  AdxPwMeterCtrlCreate         PwMeterCtrl_Create     
#define  AdxPwModulatorCtrlCreate     PwModulatorCtrl_Create 
#define  AdxTimerPulseCtrlCreate      TimerPulseCtrl_Create  
#define  AdxUdCounterCtrlCreate       UdCounterCtrl_Create   

/*
For compatible : ICollection
*/
#define ICollection_Dispose                     Array_Dispose
#define ICollection_getCount                    Array_getLength
#define ICollection_getItem                     Array_getItem

/*
For compatible : NoiseFilterChannel
*/
#define NoiseFilterChannel_getChannel           NosFltChannel_getChannel
#define NoiseFilterChannel_getEnabled           NosFltChannel_getEnabled
#define NoiseFilterChannel_setEnabled           NosFltChannel_setEnabled

/*
For compatible : AnalogInputChannel
*/
#define AnalogInputChannel_getChannel           AiChannel_getChannel
#define AnalogInputChannel_getValueRange        AiChannel_getValueRange
#define AnalogInputChannel_setValueRange        AiChannel_setValueRange
#define AnalogInputChannel_getSignalType        AiChannel_getSignalType
#define AnalogInputChannel_setSignalType        AiChannel_setSignalType
#define AnalogInputChannel_getBurnoutRetType    AiChannel_getBurnoutRetType
#define AnalogInputChannel_setBurnoutRetType    AiChannel_setBurnoutRetType
#define AnalogInputChannel_getBurnoutRetValue   AiChannel_getBurnoutRetValue
#define AnalogInputChannel_setBurnoutRetValue   AiChannel_setBurnoutRetValue

/*
For compatible : AiFeatures
*/
#define AiFeatures_getTriggerSupported(x)       (AiFeatures_getTriggerCount(x) > 0)
#define AiFeatures_getTrigger1Supported(x)      (AiFeatures_getTriggerCount(x) > 1)

/*
For compatible : AnalogChannel
*/
#define AnalogChannel_getChannel                 AoChannel_getChannel
#define AnalogChannel_getValueRange              AoChannel_getValueRange
#define AnalogChannel_setValueRange              AoChannel_setValueRange

/*
For compatible : PortDirection
*/
#define PortDirection_getPort                    DioPort_getPort     
#define PortDirection_getDirection               DioPort_getDirection
#define PortDirection_setDirection               DioPort_setDirection

/*
For compatible : Counter Capability Indexer
*/
#define CounterCapabilityIndexer_Dispose        CounterIndexer_Dispose       
#define CounterCapabilityIndexer_getCount       CounterIndexer_getLength       
#define CounterCapabilityIndexer_getItem        CounterIndexer_getItem

/*
For compatible : 
*/
#define InstantAiCtrl_getInitialized(obj)                                (InstantAiCtrl_getState(obj) != Uninited)
#define InstantAiCtrl_getCanEditProperty(obj)                            InstantAiCtrl_getInitialized(obj)
#define InstantAiCtrl_UpdateProperties(obj)                              xxxCtrl_UpdateProperties(obj)
#define InstantAiCtrl_addRemovedListener(obj, listener)                  xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define InstantAiCtrl_removeRemovedListener(obj, listener)               xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define InstantAiCtrl_addReconnectedListener(obj, listener)              xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define InstantAiCtrl_removeReconnectedListener(obj, listener)           xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define InstantAiCtrl_addPropertyChangedListener(obj, listener)          xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define InstantAiCtrl_removePropertyChangedListener(obj, listener)       xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)

#define BufferedAiCtrl_getInitialized(obj)                               (BufferedAiCtrl_getState(obj) != Uninited)
#define BufferedAiCtrl_getCanEditProperty(obj)                           BufferedAiCtrl_getInitialized(obj)
#define BufferedAiCtrl_UpdateProperties(obj)                             xxxCtrl_UpdateProperties(obj)
#define BufferedAiCtrl_addRemovedListener(obj, listener)                 xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define BufferedAiCtrl_removeRemovedListener(obj, listener)              xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define BufferedAiCtrl_addReconnectedListener(obj, listener)             xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define BufferedAiCtrl_removeReconnectedListener(obj, listener)          xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define BufferedAiCtrl_addPropertyChangedListener(obj, listener)         xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define BufferedAiCtrl_removePropertyChangedListener(obj, listener)      xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define BufferedAiCtrl_addDataReadyListener(obj, listener)               xxxCtrl_addBfdAiEventListener    (obj, EvtBufferedAiDataReady,     listener)
#define BufferedAiCtrl_removeDataReadyListener(obj, listener)            xxxCtrl_removeBfdAiEventListener (obj, EvtBufferedAiDataReady,     listener)
#define BufferedAiCtrl_addOverrunListener(obj, listener)                 xxxCtrl_addBfdAiEventListener    (obj, EvtBufferedAiOverrun,       listener)
#define BufferedAiCtrl_removeOverrunListener(obj, listener)              xxxCtrl_removeBfdAiEventListener (obj, EvtBufferedAiOverrun,       listener)
#define BufferedAiCtrl_addCacheOverflowListener(obj, listener)           xxxCtrl_addBfdAiEventListener    (obj, EvtBufferedAiCacheOverflow, listener)
#define BufferedAiCtrl_removeCacheOverflowListener(obj, listener)        xxxCtrl_removeBfdAiEventListener (obj, EvtBufferedAiCacheOverflow, listener)
#define BufferedAiCtrl_addStoppedListener(obj, listener)                 xxxCtrl_addBfdAiEventListener    (obj, EvtBufferedAiStopped,       listener)
#define BufferedAiCtrl_removeStoppedListener(obj, listener)              xxxCtrl_removeBfdAiEventListener (obj, EvtBufferedAiStopped,       listener)

#define WaveformAiCtrl_getInitialized(obj)                               (WaveformAiCtrl_getState(obj) != Uninited)
#define WaveformAiCtrl_getCanEditProperty(obj)                           WaveformAiCtrl_getInitialized(obj)
#define WaveformAiCtrl_UpdateProperties(obj)                             xxxCtrl_UpdateProperties(obj)

#define InstantAoCtrl_getInitialized(obj)                                (InstantAoCtrl_getState(obj) != Uninited)
#define InstantAoCtrl_getCanEditProperty(obj)                            InstantAoCtrl_getInitialized(obj)
#define InstantAoCtrl_UpdateProperties(obj)                              xxxCtrl_UpdateProperties(obj)
#define InstantAoCtrl_addRemovedListener(obj, listener)                  xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define InstantAoCtrl_removeRemovedListener(obj, listener)               xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define InstantAoCtrl_addReconnectedListener(obj, listener)              xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define InstantAoCtrl_removeReconnectedListener(obj, listener)           xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define InstantAoCtrl_addPropertyChangedListener(obj, listener)          xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define InstantAoCtrl_removePropertyChangedListener(obj, listener)       xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)

#define BufferedAoCtrl_getInitialized(obj)                               (BufferedAoCtrl_getState(obj) != Uninited)
#define BufferedAoCtrl_getCanEditProperty(obj)                           BufferedAoCtrl_getInitialized(obj)
#define BufferedAoCtrl_UpdateProperties(obj)                             xxxCtrl_UpdateProperties(obj)
#define BufferedAoCtrl_addRemovedListener(obj, listener)                 xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define BufferedAoCtrl_removeRemovedListener(obj, listener)              xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define BufferedAoCtrl_addReconnectedListener(obj, listener)             xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define BufferedAoCtrl_removeReconnectedListener(obj, listener)          xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define BufferedAoCtrl_addPropertyChangedListener(obj, listener)         xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define BufferedAoCtrl_removePropertyChangedListener(obj, listener)      xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define BufferedAoCtrl_addDataTransmittedListener(obj, listener)         xxxCtrl_addBfdAoEventListener    (obj, EvtBufferedAoDataTransmitted, listener)
#define BufferedAoCtrl_removeDataTransmittedListener(obj, listener)      xxxCtrl_removeBfdAoEventListener (obj, EvtBufferedAoDataTransmitted, listener)
#define BufferedAoCtrl_addUnderrunListener(obj, listener)                xxxCtrl_addBfdAoEventListener    (obj, EvtBufferedAoUnderrun,        listener)
#define BufferedAoCtrl_removeUnderrunListener(obj, listener)             xxxCtrl_removeBfdAoEventListener (obj, EvtBufferedAoUnderrun,        listener)
#define BufferedAoCtrl_addCacheEmptiedListener(obj, listener)            xxxCtrl_addBfdAoEventListener    (obj, EvtBufferedAoCacheEmptied,    listener)
#define BufferedAoCtrl_removeCacheEmptiedListener(obj, listener)         xxxCtrl_removeBfdAoEventListener (obj, EvtBufferedAoCacheEmptied,    listener)
#define BufferedAoCtrl_addTransitStoppedListener(obj, listener)          xxxCtrl_addBfdAoEventListener    (obj, EvtBufferedAoTransStopped,    listener)
#define BufferedAoCtrl_removeTransitStoppedListener(obj, listener)       xxxCtrl_removeBfdAoEventListener (obj, EvtBufferedAoTransStopped,    listener)
#define BufferedAoCtrl_addStoppedListener(obj, listener)                 xxxCtrl_addBfdAoEventListener    (obj, EvtBufferedAoStopped,         listener)
#define BufferedAoCtrl_removeStoppedListener(obj, listener)              xxxCtrl_removeBfdAoEventListener (obj, EvtBufferedAoStopped,         listener)

#define InstantDiCtrl_getInitialized(obj)                                (InstantDiCtrl_getState(obj) != Uninited)
#define InstantDiCtrl_getCanEditProperty(obj)                            InstantDiCtrl_getInitialized(obj)
#define InstantDiCtrl_UpdateProperties(obj)                              xxxCtrl_UpdateProperties(obj)
#define InstantDiCtrl_getPortDirection(obj)                              InstantDiCtrl_getPorts(obj)
#define InstantDiCtrl_addRemovedListener(obj, listener)                  xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define InstantDiCtrl_removeRemovedListener(obj, listener)               xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define InstantDiCtrl_addReconnectedListener(obj, listener)              xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define InstantDiCtrl_removeReconnectedListener(obj, listener)           xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define InstantDiCtrl_addPropertyChangedListener(obj, listener)          xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define InstantDiCtrl_removePropertyChangedListener(obj, listener)       xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define InstantDiCtrl_addInterruptListener(obj, listener)                xxxCtrl_addDiSnapEventListener   (obj, EvtDiInterrupt,       listener)
#define InstantDiCtrl_removeInterruptListener(obj, listener)             xxxCtrl_removeDiSnapEventListener(obj, EvtDiInterrupt,       listener)
#define InstantDiCtrl_addChangeOfStateListener(obj, listener)            xxxCtrl_addDiSnapEventListener   (obj, EvtDiStatusChange,    listener)
#define InstantDiCtrl_removeChangeOfStateListener(obj, listener)         xxxCtrl_removeDiSnapEventListener(obj, EvtDiStatusChange,    listener)
#define InstantDiCtrl_addPatternMatchListener(obj, listener)             xxxCtrl_addDiSnapEventListener   (obj, EvtDiPatternMatch,    listener)
#define InstantDiCtrl_removePatternMatchListener(obj, listener)          xxxCtrl_removeDiSnapEventListener(obj, EvtDiPatternMatch,    listener)

#define InstantDoCtrl_getInitialized(obj)                                (InstantDoCtrl_getState(obj) != Uninited)
#define InstantDoCtrl_getCanEditProperty(obj)                            InstantDoCtrl_getInitialized(obj)
#define InstantDoCtrl_UpdateProperties(obj)                              xxxCtrl_UpdateProperties(obj)
#define InstantDoCtrl_getPortDirection(obj)                              InstantDoCtrl_getPorts(obj)
#define InstantDoCtrl_addRemovedListener(obj, listener)                  xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define InstantDoCtrl_removeRemovedListener(obj, listener)               xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define InstantDoCtrl_addReconnectedListener(obj, listener)              xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define InstantDoCtrl_removeReconnectedListener(obj, listener)           xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define InstantDoCtrl_addPropertyChangedListener(obj, listener)          xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define InstantDoCtrl_removePropertyChangedListener(obj, listener)       xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)

#define EventCounterCtrl_getInitialized(obj)                             (EventCounterCtrl_getState(obj) != Uninited)
#define EventCounterCtrl_getCanEditProperty(obj)                         EventCounterCtrl_getInitialized(obj)
#define EventCounterCtrl_UpdateProperties(obj)                           xxxCtrl_UpdateProperties(obj)
#define EventCounterCtrl_addRemovedListener(obj, listener)               xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define EventCounterCtrl_removeRemovedListener(obj, listener)            xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define EventCounterCtrl_addReconnectedListener(obj, listener)           xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define EventCounterCtrl_removeReconnectedListener(obj, listener)        xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define EventCounterCtrl_addPropertyChangedListener(obj, listener)       xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define EventCounterCtrl_removePropertyChangedListener(obj, listener)    xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define EventCounterCtrl_getChannel(obj)                                 xxxCntrCtrl_getChannel(obj)
#define EventCounterCtrl_setChannel(obj, ch)                             xxxCntrCtrl_setChannel(obj, ch)

#define FreqMeterCtrl_getInitialized(obj)                                (FreqMeterCtrl_getState(obj) != Uninited)
#define FreqMeterCtrl_getCanEditProperty(obj)                            FreqMeterCtrl_getInitialized(obj)
#define FreqMeterCtrl_UpdateProperties(obj)                              xxxCtrl_UpdateProperties(obj)
#define FreqMeterCtrl_addRemovedListener(obj, listener)                  xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define FreqMeterCtrl_removeRemovedListener(obj, listener)               xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define FreqMeterCtrl_addReconnectedListener(obj, listener)              xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define FreqMeterCtrl_removeReconnectedListener(obj, listener)           xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define FreqMeterCtrl_addPropertyChangedListener(obj, listener)          xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define FreqMeterCtrl_removePropertyChangedListener(obj, listener)       xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define FreqMeterCtrl_getChannel(obj)                                    xxxCntrCtrl_getChannel(obj)
#define FreqMeterCtrl_setChannel(obj, ch)                                xxxCntrCtrl_setChannel(obj, ch)

#define OneShotCtrl_getInitialized(obj)                                  (OneShotCtrl_getState(obj) != Uninited)
#define OneShotCtrl_getCanEditProperty(obj)                              OneShotCtrl_getInitialized(obj)
#define OneShotCtrl_UpdateProperties(obj)                                xxxCtrl_UpdateProperties(obj)
#define OneShotCtrl_addRemovedListener(obj, listener)                    xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define OneShotCtrl_removeRemovedListener(obj, listener)                 xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define OneShotCtrl_addReconnectedListener(obj, listener)                xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define OneShotCtrl_removeReconnectedListener(obj, listener)             xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define OneShotCtrl_addPropertyChangedListener(obj, listener)            xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define OneShotCtrl_removePropertyChangedListener(obj, listener)         xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define OneShotCtrl_addOneShotListener(obj, listener)                    xxxCtrl_addCntrEventListener     (obj, EvtCntOneShot0,        listener)
#define OneShotCtrl_removeOneShotListener(obj, listener)                 xxxCtrl_removeCntrEventListener  (obj, EvtCntOneShot0,        listener)
#define OneShotCtrl_getChannel(obj)                                      xxxCntrCtrl_getChannel(obj)
#define OneShotCtrl_setChannel(obj, ch)                                  xxxCntrCtrl_setChannel(obj, ch)

#define TimerPulseCtrl_getInitialized(obj)                               (TimerPulseCtrl_getState(obj) != Uninited)
#define TimerPulseCtrl_getCanEditProperty(obj)                           TimerPulseCtrl_getInitialized(obj)
#define TimerPulseCtrl_UpdateProperties(obj)                             xxxCtrl_UpdateProperties(obj)
#define TimerPulseCtrl_addRemovedListener(obj, listener)                 xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define TimerPulseCtrl_removeRemovedListener(obj, listener)              xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define TimerPulseCtrl_addReconnectedListener(obj, listener)             xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define TimerPulseCtrl_removeReconnectedListener(obj, listener)          xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define TimerPulseCtrl_addPropertyChangedListener(obj, listener)         xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define TimerPulseCtrl_removePropertyChangedListener(obj, listener)      xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define TimerPulseCtrl_addTimerTickListener(obj, listener)               xxxCtrl_addCntrEventListener     (obj, EvtCntTimer0,         listener)
#define TimerPulseCtrl_removeTimerTickListener(obj, listener)            xxxCtrl_removeCntrEventListener  (obj, EvtCntTimer0,         listener)
#define TimerPulseCtrl_getChannel(obj)                                   xxxCntrCtrl_getChannel(obj)
#define TimerPulseCtrl_setChannel(obj, ch)                               xxxCntrCtrl_setChannel(obj, ch)

#define PwMeterCtrl_getInitialized(obj)                                 (PwMeterCtrl_getState(obj) != Uninited)
#define PwMeterCtrl_getCanEditProperty(obj)                              PwMeterCtrl_getInitialized(obj)
#define PwMeterCtrl_UpdateProperties(obj)                                xxxCtrl_UpdateProperties(obj)
#define PwMeterCtrl_addRemovedListener(obj, listener)                    xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define PwMeterCtrl_removeRemovedListener(obj, listener)                 xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define PwMeterCtrl_addReconnectedListener(obj, listener)                xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define PwMeterCtrl_removeReconnectedListener(obj, listener)             xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define PwMeterCtrl_addPropertyChangedListener(obj, listener)            xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define PwMeterCtrl_removePropertyChangedListener(obj, listener)         xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define PwMeterCtrl_addOverflowListener(obj, listener)                   xxxCtrl_addCntrEventListener     (obj, EvtCntPwmInOverflow0, listener)
#define PwMeterCtrl_removeOverflowListener(obj, listener)                xxxCtrl_removeCntrEventListener  (obj, EvtCntPwmInOverflow0, listener)
#define PwMeterCtrl_getChannel(obj)                                      xxxCntrCtrl_getChannel(obj)
#define PwMeterCtrl_setChannel(obj, ch)                                  xxxCntrCtrl_setChannel(obj, ch)

#define PwModulatorCtrl_getInitialized(obj)                              (PwModulatorCtrl_getState(obj) != Uninited)
#define PwModulatorCtrl_getCanEditProperty(obj)                          PwModulatorCtrl_getInitialized(obj)
#define PwModulatorCtrl_UpdateProperties(obj)                            xxxCtrl_UpdateProperties(obj)
#define PwModulatorCtrl_addRemovedListener(obj, listener)                xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define PwModulatorCtrl_removeRemovedListener(obj, listener)             xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define PwModulatorCtrl_addReconnectedListener(obj, listener)            xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define PwModulatorCtrl_removeReconnectedListener(obj, listener)         xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define PwModulatorCtrl_addPropertyChangedListener(obj, listener)        xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define PwModulatorCtrl_removePropertyChangedListener(obj, listener)     xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define PwModulatorCtrl_getChannel(obj)                                  xxxCntrCtrl_getChannel(obj)
#define PwModulatorCtrl_setChannel(obj, ch)                              xxxCntrCtrl_setChannel(obj, ch)

#define UdCounterCtrl_getInitialized(obj)                               (UdCounterCtrl_getState(obj) != Uninited)
#define UdCounterCtrl_getCanEditProperty(obj)                            UdCounterCtrl_getInitialized(obj)
#define UdCounterCtrl_UpdateProperties(obj)                              xxxCtrl_UpdateProperties(obj)
#define UdCounterCtrl_addRemovedListener(obj, listener)                  xxxCtrl_addDeviceEventListener   (obj, EvtDeviceRemoved,     listener)
#define UdCounterCtrl_removeRemovedListener(obj, listener)               xxxCtrl_removeDeviceEventListener(obj, EvtDeviceRemoved,     listener)
#define UdCounterCtrl_addReconnectedListener(obj, listener)              xxxCtrl_addDeviceEventListener   (obj, EvtDeviceReconnected, listener)
#define UdCounterCtrl_removeReconnectedListener(obj, listener)           xxxCtrl_removeDeviceEventListener(obj, EvtDeviceReconnected, listener)
#define UdCounterCtrl_addPropertyChangedListener(obj, listener)          xxxCtrl_addDeviceEventListener   (obj, EvtPropertyChanged,   listener)
#define UdCounterCtrl_removePropertyChangedListener(obj, listener)       xxxCtrl_removeDeviceEventListener(obj, EvtPropertyChanged,   listener)
#define UdCounterCtrl_addUdCntrEventListener(obj, listener)              xxxCtrl_addUdCntrEventListener   (obj, EvtUdIndex0,          listener)
#define UdCounterCtrl_removeUdCntrEventListener(obj, listener)           xxxCtrl_removeUdCntrEventListener(obj, EvtUdIndex0,          listener)
#define UdCounterCtrl_getChannel(obj)                                    xxxCntrCtrl_getChannel(obj)
#define UdCounterCtrl_setChannel(obj, ch)                                xxxCntrCtrl_setChannel(obj, ch)

/*
For compatible : DI features / DO Features
*/

#define DiFeatures_getPortProgrammable                DioFeatures_getPortProgrammable
#define DiFeatures_getPortCount                       DioFeatures_getPortCount
#define DiFeatures_getPortsType                       DioFeatures_getPortsType
#define DiFeatures_getDiSupported                     DioFeatures_getDiSupported
#define DiFeatures_getDoSupported                     DioFeatures_getDoSupported
#define DiFeatures_getChannelCountMax                 DioFeatures_getChannelCountMax
#define DiFeatures_getDataMask                        DioFeatures_getDiDataMask
#define DiFeatures_getNoiseFilterSupported            DioFeatures_getDiNoiseFilterSupported
#define DiFeatures_getNoiseFilterOfChannels           DioFeatures_getDiNoiseFilterOfChannels
#define DiFeatures_getNoiseFilterBlockTimeRange       DioFeatures_getDiNoiseFilterBlockTimeRange
#define DiFeatures_getDiintSupported                  DioFeatures_getDiintSupported
#define DiFeatures_getDiintGateSupported              DioFeatures_getDiintGateSupported
#define DiFeatures_getDiCosintSupported               DioFeatures_getDiCosintSupported
#define DiFeatures_getDiPmintSupported                DioFeatures_getDiPmintSupported
#define DiFeatures_getDiintTriggerEdges               DioFeatures_getDiintTriggerEdges
#define DiFeatures_getDiintOfChannels                 DioFeatures_getDiintOfChannels
#define DiFeatures_getDiintGateOfChannels             DioFeatures_getDiintGateOfChannels
#define DiFeatures_getDiCosintOfPorts                 DioFeatures_getDiCosintOfPorts
#define DiFeatures_getDiPmintOfPorts                  DioFeatures_getDiPmintOfPorts
#define DiFeatures_getSnapEventSources                DioFeatures_getDiSnapEventSources

#define DoFeatures_getPortProgrammable                DioFeatures_getPortProgrammable
#define DoFeatures_getPortCount                       DioFeatures_getPortCount
#define DoFeatures_getPortsType                       DioFeatures_getPortsType
#define DoFeatures_getDiSupported                     DioFeatures_getDiSupported
#define DoFeatures_getDoSupported                     DioFeatures_getDoSupported
#define DoFeatures_getChannelCountMax                 DioFeatures_getChannelCountMax
#define DoFeatures_getDataMask                        DioFeatures_getDoDataMask
#define DoFeatures_getDoFreezeSignalSources           DioFeatures_getDoFreezeSignalSources
#define DoFeatures_getDoReflectWdtFeedIntervalRange   DioFeatures_getDoReflectWdtFeedIntervalRange

/*
For compatible : Counter Features
*/
#define EventCounterFeatures_getChannelCountMax               CntrFeatures_getChannelCountMax
#define EventCounterFeatures_getResolution                    CntrFeatures_getResolution
#define EventCounterFeatures_getDataSize                      CntrFeatures_getDataSize
#define EventCounterFeatures_getCapabilities                  CntrFeatures_getCapabilities
#define EventCounterFeatures_getNoiseFilterSupported          CntrFeatures_getNoiseFilterSupported
#define EventCounterFeatures_getNoiseFilterOfChannels         CntrFeatures_getNoiseFilterOfChannels
#define EventCounterFeatures_getNoiseFilterBlockTimeRange     CntrFeatures_getNoiseFilterBlockTimeRange

#define FreqMeterFeatures_getChannelCountMax                  CntrFeatures_getChannelCountMax
#define FreqMeterFeatures_getResolution                       CntrFeatures_getResolution
#define FreqMeterFeatures_getDataSize                         CntrFeatures_getDataSize
#define FreqMeterFeatures_getCapabilities                     CntrFeatures_getCapabilities
#define FreqMeterFeatures_getNoiseFilterSupported             CntrFeatures_getNoiseFilterSupported
#define FreqMeterFeatures_getNoiseFilterOfChannels            CntrFeatures_getNoiseFilterOfChannels
#define FreqMeterFeatures_getNoiseFilterBlockTimeRange        CntrFeatures_getNoiseFilterBlockTimeRange
#define FreqMeterFeatures_getFmMethods                        CntrFeatures_getFmMethods

#define OneShotFeatures_getChannelCountMax                    CntrFeatures_getChannelCountMax
#define OneShotFeatures_getResolution                         CntrFeatures_getResolution
#define OneShotFeatures_getDataSize                           CntrFeatures_getDataSize
#define OneShotFeatures_getCapabilities                       CntrFeatures_getCapabilities
#define OneShotFeatures_getNoiseFilterSupported               CntrFeatures_getNoiseFilterSupported
#define OneShotFeatures_getNoiseFilterOfChannels              CntrFeatures_getNoiseFilterOfChannels
#define OneShotFeatures_getNoiseFilterBlockTimeRange          CntrFeatures_getNoiseFilterBlockTimeRange
#define OneShotFeatures_getDelayCountRange                    CntrFeatures_getOsDelayCountRange

#define TimerPulseFeatures_getChannelCountMax                 CntrFeatures_getChannelCountMax
#define TimerPulseFeatures_getResolution                      CntrFeatures_getResolution
#define TimerPulseFeatures_getDataSize                        CntrFeatures_getDataSize
#define TimerPulseFeatures_getCapabilities                    CntrFeatures_getCapabilities
#define TimerPulseFeatures_getNoiseFilterSupported            CntrFeatures_getNoiseFilterSupported
#define TimerPulseFeatures_getNoiseFilterOfChannels           CntrFeatures_getNoiseFilterOfChannels
#define TimerPulseFeatures_getNoiseFilterBlockTimeRange       CntrFeatures_getNoiseFilterBlockTimeRange
#define TimerPulseFeatures_getTimerFrequencyRange             CntrFeatures_getTmrFrequencyRange
#define TimerPulseFeatures_getTimerEventSupported(_thiz_)     0

#define PwMeterFeatures_getChannelCountMax                    CntrFeatures_getChannelCountMax
#define PwMeterFeatures_getResolution                         CntrFeatures_getResolution
#define PwMeterFeatures_getDataSize                           CntrFeatures_getDataSize
#define PwMeterFeatures_getCapabilities                       CntrFeatures_getCapabilities
#define PwMeterFeatures_getNoiseFilterSupported               CntrFeatures_getNoiseFilterSupported
#define PwMeterFeatures_getNoiseFilterOfChannels              CntrFeatures_getNoiseFilterOfChannels
#define PwMeterFeatures_getNoiseFilterBlockTimeRange          CntrFeatures_getNoiseFilterBlockTimeRange
#define PwMeterFeatures_getPwmCascadeGroup                    CntrFeatures_getPiCascadeGroup
#define PwMeterFeatures_getOverflowEventSupported(_thiz_)     0

#define PwModulatorFeatures_getChannelCountMax                CntrFeatures_getChannelCountMax
#define PwModulatorFeatures_getResolution                     CntrFeatures_getResolution
#define PwModulatorFeatures_getDataSize                       CntrFeatures_getDataSize
#define PwModulatorFeatures_getCapabilities                   CntrFeatures_getCapabilities
#define PwModulatorFeatures_getNoiseFilterSupported           CntrFeatures_getNoiseFilterSupported
#define PwModulatorFeatures_getNoiseFilterOfChannels          CntrFeatures_getNoiseFilterOfChannels
#define PwModulatorFeatures_getNoiseFilterBlockTimeRange      CntrFeatures_getNoiseFilterBlockTimeRange
#define PwModulatorFeatures_getHiPeriodRange                  CntrFeatures_getPoHiPeriodRange
#define PwModulatorFeatures_getLoPeriodRange                  CntrFeatures_getPoLoPeriodRange

#define UdCounterFeatures_getChannelCountMax                  CntrFeatures_getChannelCountMax
#define UdCounterFeatures_getResolution                       CntrFeatures_getResolution
#define UdCounterFeatures_getDataSize                         CntrFeatures_getDataSize
#define UdCounterFeatures_getCapabilities                     CntrFeatures_getCapabilities
#define UdCounterFeatures_getNoiseFilterSupported             CntrFeatures_getNoiseFilterSupported
#define UdCounterFeatures_getNoiseFilterOfChannels            CntrFeatures_getNoiseFilterOfChannels
#define UdCounterFeatures_getNoiseFilterBlockTimeRange        CntrFeatures_getNoiseFilterBlockTimeRange
#define UdCounterFeatures_getCountingTypes                    CntrFeatures_getUdCountingTypes
#define UdCounterFeatures_getInitialValues                    CntrFeatures_getUdInitialValues
#define UdCounterFeatures_getSnapEventSources                 CntrFeatures_getUdSnapEventSources

#define BufferedEventCounterFeatures_getChannelCountMax               CntrFeatures_getChannelCountMax
#define BufferedEventCounterFeatures_getResolution                    CntrFeatures_getResolution
#define BufferedEventCounterFeatures_getDataSize                      CntrFeatures_getDataSize
#define BufferedEventCounterFeatures_getCapabilities                  CntrFeatures_getCapabilities
#define BufferedEventCounterFeatures_getNoiseFilterSupported          CntrFeatures_getNoiseFilterSupported
#define BufferedEventCounterFeatures_getNoiseFilterOfChannels         CntrFeatures_getNoiseFilterOfChannels
#define BufferedEventCounterFeatures_getNoiseFilterBlockTimeRange     CntrFeatures_getNoiseFilterBlockTimeRange
#define BufferedEventCounterFeatures_getTriggerSources                CntrFeatures_getTriggerSources
#define BufferedEventCounterFeatures_getTriggerActions                CntrFeatures_getTriggerActions
#define BufferedEventCounterFeatures_getSamplingClockSources          CntrFeatures_getSamplingClockSources

#define BufferedFreqMeterFeatures_getChannelCountMax                  CntrFeatures_getChannelCountMax
#define BufferedFreqMeterFeatures_getResolution                       CntrFeatures_getResolution
#define BufferedFreqMeterFeatures_getDataSize                         CntrFeatures_getDataSize
#define BufferedFreqMeterFeatures_getCapabilities                     CntrFeatures_getCapabilities
#define BufferedFreqMeterFeatures_getNoiseFilterSupported             CntrFeatures_getNoiseFilterSupported
#define BufferedFreqMeterFeatures_getNoiseFilterOfChannels            CntrFeatures_getNoiseFilterOfChannels
#define BufferedFreqMeterFeatures_getNoiseFilterBlockTimeRange        CntrFeatures_getNoiseFilterBlockTimeRange
#define BufferedFreqMeterFeatures_getFmMethods                        CntrFeatures_getFmMethods
#define BufferedFreqMeterFeatures_getTriggerSources                   CntrFeatures_getTriggerSources
#define BufferedFreqMeterFeatures_getTriggerActions                   CntrFeatures_getTriggerActions
#define BufferedFreqMeterFeatures_getSamplingClockSources             CntrFeatures_getSamplingClockSources

#define BufferedUdCounterFeatures_getChannelCountMax                  CntrFeatures_getChannelCountMax
#define BufferedUdCounterFeatures_getResolution                       CntrFeatures_getResolution
#define BufferedUdCounterFeatures_getDataSize                         CntrFeatures_getDataSize
#define BufferedUdCounterFeatures_getCapabilities                     CntrFeatures_getCapabilities
#define BufferedUdCounterFeatures_getNoiseFilterSupported             CntrFeatures_getNoiseFilterSupported
#define BufferedUdCounterFeatures_getNoiseFilterOfChannels            CntrFeatures_getNoiseFilterOfChannels
#define BufferedUdCounterFeatures_getNoiseFilterBlockTimeRange        CntrFeatures_getNoiseFilterBlockTimeRange
#define BufferedUdCounterFeatures_getCountingTypes                    CntrFeatures_getUdCountingTypes
#define BufferedUdCounterFeatures_getTriggerSources                   CntrFeatures_getTriggerSources
#define BufferedUdCounterFeatures_getTriggerActions                   CntrFeatures_getTriggerActions
#define BufferedUdCounterFeatures_getSamplingClockSources             CntrFeatures_getSamplingClockSources

#define BufferedPwModulatorFeatures_getChannelCountMax                CntrFeatures_getChannelCountMax
#define BufferedPwModulatorFeatures_getResolution                     CntrFeatures_getResolution
#define BufferedPwModulatorFeatures_getDataSize                       CntrFeatures_getDataSize
#define BufferedPwModulatorFeatures_getCapabilities                   CntrFeatures_getCapabilities
#define BufferedPwModulatorFeatures_getNoiseFilterSupported           CntrFeatures_getNoiseFilterSupported
#define BufferedPwModulatorFeatures_getNoiseFilterOfChannels          CntrFeatures_getNoiseFilterOfChannels
#define BufferedPwModulatorFeatures_getNoiseFilterBlockTimeRange      CntrFeatures_getNoiseFilterBlockTimeRange
#define BufferedPwModulatorFeatures_getHiPeriodRange                  CntrFeatures_getPoHiPeriodRange
#define BufferedPwModulatorFeatures_getLoPeriodRange                  CntrFeatures_getPoLoPeriodRange
#define BufferedPwModulatorFeatures_getTriggerSources                 CntrFeatures_getTriggerSources
#define BufferedPwModulatorFeatures_getTriggerActions                 CntrFeatures_getTriggerActions
#define BufferedPwModulatorFeatures_getSamplingClockSources           CntrFeatures_getSamplingClockSources
/**********************************************************/
/*                                                        */
/*             DAQNAVI CSCL Classes                       */
/*                                                        */
/**********************************************************/
#if defined(__cplusplus) && !defined(_BDAQ_C_INTERFACE)

#define DAQCLS_DISABLE_CONSTRUCT(Class)\
   Class(const Class &);\
   Class &operator=(const Class &);\
   ~Class();\

/* ----------------------------------------------------------*/
/*common classes                                             */
/* ----------------------------------------------------------*/
/* Interface Array */
template<class T>
class Array
{
public:
   void  Dispose()                                          {        Array_Dispose((IArray *)this);              }   /*destroy the instance*/
   int32 getLength()                                        { return Array_getLength((IArray *)this);            }
   T &   getItem(int32 index)                               { return *(T *) Array_getItem((IArray *)this, index);}

public: /*for compatible*/
   int32 getCount()                                         { return getLength(); } 

private:
   DAQCLS_DISABLE_CONSTRUCT(Array)
};
/*for compatible*/
#define ICollection Array  

/* Interface ScanChannel */
class ScanChannel
{
public:
   int32     getChannelStart()                              { return ScanChannel_getChannelStart(this);         }
   ErrorCode setChannelStart(int32 value)                   { return ScanChannel_setChannelStart(this, value);  }
   int32     getChannelCount()                              { return ScanChannel_getChannelCount(this);         }
   ErrorCode setChannelCount(int32 value)                   { return ScanChannel_setChannelCount(this, value);  }
   int32     getSamples()                                   { return ScanChannel_getSamples(this);              }
   ErrorCode setSamples(int32 value)                        { return ScanChannel_setSamples(this, value);       }
   int32     getIntervalCount()                             { return ScanChannel_getIntervalCount(this);        }
   ErrorCode setIntervalCount(int32 value)                  { return ScanChannel_setIntervalCount(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(ScanChannel)
};

/* Interface ConvertClock */
class ConvertClock
{
public:
   SignalDrop getSource()                                   { return ConvertClock_getSource(this);        }
   ErrorCode  setSource(SignalDrop value)                   { return ConvertClock_setSource(this, value); }
   double     getRate()                                     { return ConvertClock_getRate(this);          }
   ErrorCode  setRate(double value)                         { return ConvertClock_setRate(this, value);   }

private:
   DAQCLS_DISABLE_CONSTRUCT(ConvertClock)
};

/* Interface ScanClock */
class ScanClock
{
public:
   SignalDrop getSource()                                   { return ScanClock_getSource(this);          }
   ErrorCode  setSource(SignalDrop value)                   { return ScanClock_setSource(this, value);   }
   double     getRate()                                     { return ScanClock_getRate(this);            }
   ErrorCode  setRate(double value)                         { return ScanClock_setRate(this, value);     }
   int32      getScanCount()                                { return ScanClock_getScanCount(this);       }
   ErrorCode  setScanCount(int32 value)                     { return ScanClock_setScanCount(this, value);}

private:
   DAQCLS_DISABLE_CONSTRUCT(ScanClock)
};

/* Interface Trigger */
class Trigger
{
public:
   SignalDrop    getSource()                                { return Trigger_getSource(this);                 }
   ErrorCode     setSource(SignalDrop value)                { return Trigger_setSource(this, value);          }
   ActiveSignal  getEdge()                                  { return Trigger_getEdge(this);                   }
   ErrorCode     setEdge(ActiveSignal value)                { return Trigger_setEdge(this, value);            }
   double        getLevel()                                 { return Trigger_getLevel(this);                  }
   ErrorCode     setLevel(double value)                     { return Trigger_setLevel(this, value);           }
   TriggerAction getAction()                                { return Trigger_getAction(this);                 }
   ErrorCode     setAction(TriggerAction value)             { return Trigger_setAction(this, value);          }
   int32         getDelayCount()                            { return Trigger_getDelayCount(this);             }
   ErrorCode     setDelayCount(int32 value)                 { return Trigger_setDelayCount(this, value);      }
   double        getHysteresisIndex()                       { return Trigger_getHysteresisIndex(this);        }
   ErrorCode     setHysteresisIndex(double value)           { return Trigger_setHysteresisIndex(this, value); }

   FilterType    getFilterType()                            { return Trigger_getFilterType(this);             }
   ErrorCode     setFilterType(FilterType value)            { return Trigger_setFilterType(this, value);      }
   double        getFilterCutoffFreq()                      { return Trigger_getFilterCutoffFreq(this);       }
   ErrorCode     setFilterCutoffFreq(double value)          { return Trigger_setFilterCutoffFreq(this, value);}

private:
   DAQCLS_DISABLE_CONSTRUCT(Trigger)
};

/*Interface Conversion*/
class Conversion
{
public:
   SignalDrop getClockSource()                               { return Conversion_getClockSource(this);        }
   ErrorCode  setClockSource(SignalDrop value)               { return Conversion_setClockSource(this, value); }
   double     getClockRate()                                 { return Conversion_getClockRate(this);          }
   ErrorCode  setClockRate(double value)                     { return Conversion_setClockRate(this, value);   }
   int32      getChannelStart()                              { return Conversion_getChannelStart(this);       }
   ErrorCode  setChannelStart(int32 value)                   { return Conversion_setChannelStart(this, value);}
   int32      getChannelCount()                              { return Conversion_getChannelCount(this);       }
   ErrorCode  setChannelCount(int32 value)                   { return Conversion_setChannelCount(this, value);}
   void       getChannelMap(int32 count, int8 *chMap)        {        Conversion_getChannelMap(this, count, chMap);}
   ErrorCode  setChannelMap(int32 count, int8 *chMap)        { return Conversion_setChannelMap(this, count, chMap);}
private:
   DAQCLS_DISABLE_CONSTRUCT(Conversion)
};

/*Interface Record*/
class Record
{
public:
   int32     getSectionLength()                             { return Record_getSectionLength(this);       }
   ErrorCode setSectionLength(int32 value)                  { return Record_setSectionLength(this, value);}
   int32     getSectionCount()                              { return Record_getSectionCount(this);        }
   ErrorCode setSectionCount(int32 value)                   { return Record_setSectionCount(this, value); }
   int32     getCycles()                                    { return Record_getCycles(this);              }
   ErrorCode setCycles(int32 value)                         { return Record_setCycles(this, value);       }

private:
   DAQCLS_DISABLE_CONSTRUCT(Record)
};

/* Interface NosFltChannel */
class NosFltChannel
{
public:
   int32     getChannel()                                   { return NosFltChannel_getChannel(this);        }
   bool      getEnabled()                                   { return !!NosFltChannel_getEnabled(this);      }
   ErrorCode setEnabled(bool value)                         { return NosFltChannel_setEnabled(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(NosFltChannel)
};
typedef NosFltChannel NoiseFilterChannel;   /*for compatible*/


/* ----------------------------------------------------------*/
/*device ctrl class                                          */
/* ----------------------------------------------------------*/
/* Interface DeviceCtrl */   
class DeviceCtrl
{
   /*event*/
   void addEventHandler(EventId id, DeviceEventProc proc, void *userParam)    { DeviceCtrl_addEventHandler(this, id, proc, userParam);   }
   void removeEventHandler(EventId id, DeviceEventProc proc, void *userParam) { DeviceCtrl_removeEventHandler(this, id, proc, userParam);}
public:
   /*method*/
   ErrorCode Refresh()                                                         { return DeviceCtrl_Refresh(this);                                     }
   ErrorCode ReadRegister(int32 space, int32 offset, int32 length, void *data) { return DeviceCtrl_ReadRegister(this, space, offset, length, data);   }
   ErrorCode WriteRegister(int32 space, int32 offset, int32 length, void *data){ return DeviceCtrl_WriteRegister(this, space, offset, length, data);  }
   ErrorCode ReadPrivateRegion(int32 signature, int32 length, uint8 *data)     { return DeviceCtrl_ReadPrivateRegion(this, signature, length, data);  }
   ErrorCode WritePrivateRegion(int32 signature, int32 length, uint8 *data)    { return DeviceCtrl_WritePrivateRegion(this, signature, length, data); }
   
   /*property*/
   int32                 getDeviceNumber()                                    { return DeviceCtrl_getDeviceNumber(this);               }
   ErrorCode             getDescription(int32 length, wchar_t *desc)          { return DeviceCtrl_getDescription(this, length, desc);  }
   ErrorCode             setDescription(int32 length, wchar_t *desc)          { return DeviceCtrl_setDescription(this, length, desc);  }
   AccessMode            getAccessMode()                                      { return DeviceCtrl_getAccessMode(this);                 }
   ProductId             getProductId()                                       { return DeviceCtrl_getProductId(this);                  }
   int32                 getBoardId()                                         { return DeviceCtrl_getBoardId(this);                    }
   ErrorCode             setBoardId(int32 value)                              { return DeviceCtrl_setBoardId(this, value);             }
   ErrorCode             getBoardVersion(int32 length, wchar_t *ver)          { return DeviceCtrl_getBoardVersion(this, length, ver);  }
   ErrorCode             getDriverVersion(int32 length, wchar_t *ver)         { return DeviceCtrl_getDriverVersion(this, length, ver); }
   ErrorCode             getDllVersion(int32 length, wchar_t *ver)            { return DeviceCtrl_getDllVersion(this, length, ver);    }
   ErrorCode             getLocation(int32 length, wchar_t *loc)              { return DeviceCtrl_getLocation(this, length, loc);      }
   int32                 getPrivateRegionLength()                             { return DeviceCtrl_getPrivateRegionLength(this);        }
   int32                 getHotResetPreventable()                             { return DeviceCtrl_getHotResetPreventable(this);        }
   Array<int64>*         getBaseAddresses()                                   { return (Array<int64>*)DeviceCtrl_getBaseAddresses(this);              }
   Array<int32>*         getInterrupts()                                      { return (Array<int32>*)DeviceCtrl_getInterrupts(this);                 }
   Array<TerminalBoard>* getSupportedTerminalBoard()                          { return (Array<TerminalBoard>*)DeviceCtrl_getSupportedTerminalBoard(this); }
   Array<EventId>*       getSupportedEvents()                                 { return (Array<EventId>*)DeviceCtrl_getSupportedEvents(this);            } 
   int32                 getSupportedScenarios()                              { return DeviceCtrl_getSupportedScenarios(this);         }
   HANDLE                getDevice()                                          { return DeviceCtrl_getDevice(this);                     }
   
   TerminalBoard         getTerminalBoard()                                   { return DeviceCtrl_getTerminalBoard(this);              }
   ErrorCode             setTerminalBoard(TerminalBoard board)                { return DeviceCtrl_setTerminalBoard(this, board);       }
   ErrorCode             setLocateEnabled(bool value)                         { return DeviceCtrl_setLocateEnabled(this, value);       }

   static Array<DeviceTreeNode>* getInstalledDevices()                        { return (Array<DeviceTreeNode>*)DeviceCtrl_getInstalledDevices(); }
public:
   /*Create an independent device ctrl if necessary. Note: Use 'dispose' to free the object*/
   static DeviceCtrl * Create(int32 devNumber, wchar_t const *desc, AccessMode mode) { return DeviceCtrl_Create(devNumber, desc, mode); }
   void                Dispose(void)                                                 { DeviceCtrl_Dispose(this);                        }

public: /*helpers*/
   void addRemovedHandler(DeviceEventProc proc, void *userParam)              { DeviceCtrl_addEventHandler   (this, EvtDeviceRemoved, proc, userParam);    }
   void removeRemovedHandler(DeviceEventProc proc, void *userParam)           { DeviceCtrl_removeEventHandler(this, EvtDeviceRemoved, proc, userParam);    }
   void addReconnectedHandler(DeviceEventProc proc, void *userParam)          { DeviceCtrl_addEventHandler   (this, EvtDeviceReconnected, proc, userParam);}
   void removeReconnectedHandler(DeviceEventProc proc, void *userParam)       { DeviceCtrl_removeEventHandler(this, EvtDeviceReconnected, proc, userParam);}
   void addPropertyChangedHandler(DeviceEventProc proc, void *userParam)      { DeviceCtrl_addEventHandler   (this, EvtPropertyChanged, proc, userParam);  }
   void removePropertyChangedHandler(DeviceEventProc proc, void *userParam)   { DeviceCtrl_removeEventHandler(this, EvtPropertyChanged, proc, userParam);  }

private:
   DAQCLS_DISABLE_CONSTRUCT(DeviceCtrl)
};

/* ----------------------------------------------------------*/
/*daq ctrl base class                                        */
/* ----------------------------------------------------------*/
/* Interface DaqCtrlBase */   
class DaqCtrlBase
{
public:
   /*DAQ Control Common Method*/
   ErrorCode    LoadProfile(wchar_t const *fileName)                       { return DAQFN_ET(DNLAPI_BASE, 98, wchar_t const *)(this, fileName);   }  
   void         Cleanup()                                                  { DAQFN_VV(DNLAPI_BASE, 83)(this); } /*release the resources allocated.*/
   void         Dispose()                                                  { DAQFN_VV(DNLAPI_BASE, 84)(this); } /*destroy the instance*/
                                                               
   /*DAQ Control Common Property*/                             
   void         getSelectedDevice(DeviceInformation &x)                    {        DAQFN_VT(DNLAPI_BASE, 85, DeviceInformation*)(this, &x);        }
   ErrorCode    setSelectedDevice(DeviceInformation const &x)              { return DAQFN_ET(DNLAPI_BASE, 86, DeviceInformation const*)(this, &x);  }
   ControlState getState()                                                 { return DAQFN_TV(DNLAPI_BASE, 87, ControlState)(this);   }
   DeviceCtrl*  getDevice()                                                { return DAQFN_TV(DNLAPI_BASE, 88, DeviceCtrl *)(this);   }
   HANDLE       getModule()                                                { return DAQFN_TV(DNLAPI_BASE, 93, HANDLE)(this);          }

   Array<DeviceTreeNode>* getSupportedDevices()                            { return DAQFN_TV(DNLAPI_BASE, 89, Array<DeviceTreeNode>*)(this); }
   Array<AccessMode>*     getSupportedModes()                              { return DAQFN_TV(DNLAPI_BASE, 90, Array<AccessMode>*)    (this); }

public: /*For compatible ONLY!*/
   void addRemovedListener(DeviceEventListener & listener)                 { xxxCtrl_addDeviceEventListener   (this, EvtDeviceRemoved,     &listener); }
   void removeRemovedListener(DeviceEventListener & listener)              { xxxCtrl_removeDeviceEventListener(this, EvtDeviceRemoved,     &listener); }
   void addReconnectedListener(DeviceEventListener & listener)             { xxxCtrl_addDeviceEventListener   (this, EvtDeviceReconnected, &listener); }
   void removeReconnectedListener(DeviceEventListener & listener)          { xxxCtrl_removeDeviceEventListener(this, EvtDeviceReconnected, &listener); }
   void addPropertyChangedListener(DeviceEventListener & listener)         { xxxCtrl_addDeviceEventListener   (this, EvtPropertyChanged,   &listener); }
   void removePropertyChangedListener(DeviceEventListener & listener)      { xxxCtrl_removeDeviceEventListener(this, EvtPropertyChanged,   &listener); }
   bool getInitialized()                                                   { return getState() != Uninited;                                            }
   bool getCanEditProperty()                                               { return getState() != Uninited;                                            }
   ErrorCode UpdateProperties()                                            { return getDevice()->Refresh();                                            }
private:
   DAQCLS_DISABLE_CONSTRUCT(DaqCtrlBase)
};

/* ----------------------------------------------------------*/
/*AI related classes                                         */
/* ----------------------------------------------------------*/
/* Interface CjcSetting */
class CjcSetting
{
public:
   int32     getChannel()                                            { return CjcSetting_getChannel(this);           }
   ErrorCode setChannel(int32 ch)                                    { return CjcSetting_setChannel(this, ch);       }
   double    getValue()                                              { return CjcSetting_getValue(this);             }
   ErrorCode setValue(double value)                                  { return CjcSetting_setValue(this, value);      }
   double    getUpdateFreq()                                         { return CjcSetting_getUpdateFreq(this);        }
   ErrorCode setUpdateFreq(double value)                             { return CjcSetting_setUpdateFreq(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(CjcSetting)
};

/* Interface AiChannel */
class AiChannel
{
public:
   int32          getChannel()                                       { return AiChannel_getChannel(this);                   }
   ValueRange     getValueRange()                                    { return AiChannel_getValueRange(this);                }
   ErrorCode      setValueRange(ValueRange value)                    { return AiChannel_setValueRange(this, value);         }
   AiSignalType   getSignalType()                                    { return AiChannel_getSignalType(this);                }
   ErrorCode      setSignalType(AiSignalType value)                  { return AiChannel_setSignalType(this, value);         }
   BurnoutRetType getBurnoutRetType()                                { return AiChannel_getBurnoutRetType(this);            }
   ErrorCode      setBurnoutRetType(BurnoutRetType value)            { return AiChannel_setBurnoutRetType(this, value);     }
   double         getBurnoutRetValue()                               { return AiChannel_getBurnoutRetValue(this);           }
   ErrorCode      setBurnoutRetValue(double value)                   { return AiChannel_setBurnoutRetValue(this, value);    }

   /**/                                                                
   FilterType     getFilterType()                                    { return AiChannel_getFilterType(this);                }
   ErrorCode      setFilterType(FilterType value)                    { return AiChannel_setFilterType(this, value);         }
   double         getFilterCutoffFreq()                              { return AiChannel_getFilterCutoffFreq(this);          }
   ErrorCode      setFilterCutoffFreq(double value)                  { return AiChannel_setFilterCutoffFreq(this, value);   }
   double         getFilterCutoffFreq1()                             { return AiChannel_getFilterCutoffFreq1(this);         }
   ErrorCode      setFilterCutoffFreq1(double value)                 { return AiChannel_setFilterCutoffFreq1(this, value);  }
   int32          getLogicalNumber()                                 { return AiChannel_getLogicalNumber(this);             }

   CouplingType   getCouplingType()                                  { return AiChannel_getCouplingType(this);              }
   ErrorCode      setCouplingType(CouplingType value)                { return AiChannel_setCouplingType(this, value);       }
   IepeType       getIepeType()                                      { return AiChannel_getIepeType(this);                  }
   ErrorCode      setIepeType(IepeType value)                        { return AiChannel_setIepeType(this, value);           }
   ImpedanceType  getImpedanceType()                                 { return AiChannel_getImpedanceType(this);             }
   ErrorCode      setImpedanceType(ImpedanceType value)              { return AiChannel_setImpedanceType(this, value);      }

   ErrorCode      getSensorDescription(int32 &size, wchar_t desc[])  { return AiChannel_getSensorDescription(this, &size, desc);}
   ErrorCode      setSensorDescription(int32 size, wchar_t desc[])   { return AiChannel_setSensorDescription(this,  size, desc);}

private:
   DAQCLS_DISABLE_CONSTRUCT(AiChannel)
};
typedef Array<AiChannel>  AiChannelCollection; /*for compatible*/
typedef AiChannel         AnalogInputChannel;  /*for compatible*/

/* Interface AiFeatures */ 
class AiFeatures
{
public:
   /*ADC features*/
   int32                  getResolution()                            { return AiFeatures_getResolution(this);                }
   int32                  getDataSize()                              { return AiFeatures_getDataSize(this);                  }
   int32                  getDataMask()                              { return AiFeatures_getDataMask(this);                  }
   double                 getTimestampResolution()                   { return AiFeatures_getTimestampResolution(this);       }                                                                       
   /*channel features*/                                                
   int32                  getChannelCountMax()                       { return AiFeatures_getChannelCountMax(this);           }
   bool                   getOverallValueRange()                     { return !!AiFeatures_getOverallValueRange(this);       }
   Array<ValueRange>*     getValueRanges()                           { return (Array<ValueRange>*    )AiFeatures_getValueRanges(this);       }
   Array<BurnoutRetType>* getBurnoutReturnTypes()                    { return (Array<BurnoutRetType>*)AiFeatures_getBurnoutReturnTypes(this);}
   Array<AiSignalType>*   getConnectionTypes()                       { return (Array<AiSignalType>*)AiFeatures_getConnectionTypes(this);     }                                                                    
   bool                   getOverallConnection()                     { return !!AiFeatures_getOverallConnection(this);                       }
   Array<CouplingType>*   getCouplingTypes()                         { return (Array<CouplingType>*)AiFeatures_getCouplingTypes(this);       }                                                                    
   Array<IepeType>*       getIepeTypes()                             { return (Array<IepeType>*)AiFeatures_getIepeTypes(this);               }                                                                    
   Array<ImpedanceType>*  getImpedanceTypes()                        { return (Array<ImpedanceType>*)AiFeatures_getImpedanceTypes(this);     }                                                                    

   /*filter*/                                                          
   Array<FilterType>*     getFilterTypes()                           { return (Array<FilterType>*)AiFeatures_getFilterTypes(this); }
   void                   getFilterCutoffFreqRange(MathInterval &x)  {        AiFeatures_getFilterCutoffFreqRange(this, &x); }
   void                   getFilterCutoffFreq1Range(MathInterval &x) {        AiFeatures_getFilterCutoffFreq1Range(this, &x);}
                                                                       
   /*CJC features*/                                                    
   bool                   getThermoSupported()                       { return !!AiFeatures_getThermoSupported(this);         }
   Array<int32>*          getCjcChannels()                           { return (Array<int32>*)AiFeatures_getCjcChannels(this);}
                                                                       
   /*buffered ai->basic features*/                                     
   bool                   getBufferedAiSupported()                   { return !!AiFeatures_getBufferedAiSupported(this);     }
   SamplingMethod         getSamplingMethod()                        { return AiFeatures_getSamplingMethod(this);            }
   int32                  getChannelStartBase()                      { return AiFeatures_getChannelStartBase(this);          }
   int32                  getChannelCountBase()                      { return AiFeatures_getChannelCountBase(this);          }
                                                                       
   /*buffered ai->conversion clock features*/                          
   Array<SignalDrop>*     getConvertClockSources()                   { return (Array<SignalDrop>*)AiFeatures_getConvertClockSources(this);}
   void                   getConvertClockRange(MathInterval &x)      {        AiFeatures_getConvertClockRange(this, &x);     }
                                                                       
   /*buffered ai->burst scan*/                                         
   bool                   getBurstScanSupported()                    { return !!AiFeatures_getBurstScanSupported(this);      }
   Array<SignalDrop>*     getScanClockSources()                      { return (Array<SignalDrop>*)AiFeatures_getScanClockSources(this); }
   void                   getScanClockRange(MathInterval &x)         {        AiFeatures_getScanClockRange(this, &x);        }
   int32                  getScanCountMax()                          { return AiFeatures_getScanCountMax(this);              }

   /*buffered ai->trigger features*/
   bool                   getRetriggerable()                         { return !!AiFeatures_getRetriggerable(this);            }
   int32                  getTriggerCount()                          { return AiFeatures_getTriggerCount(this);               }
   Array<FilterType>*     getTriggerFilterTypes()                    { return (Array<FilterType>*)AiFeatures_getTriggerFilterTypes(this);}
   void                   getTriggerFilterCutoffFreq(MathInterval &x){        AiFeatures_getTriggerFilterCutoffFreq(this, &x);           }
   
   /*trigger 0*/                                                       
   Array<TriggerAction>*  getTriggerActions()                        { return (Array<TriggerAction>*)AiFeatures_getTriggerActions(this);        }
   void                   getTriggerDelayRange(MathInterval &x)      {        AiFeatures_getTriggerDelayRange(this, &x);                        }
   Array<SignalDrop>*     getTriggerSources()                        { return (Array<SignalDrop>*)AiFeatures_getTriggerSources(this);           }
   ValueRange             getTriggerSourceVrg()                      { return AiFeatures_getTriggerSourceVrg(this);                             }
   double                 getTriggerHysteresisIndexMax()             { return AiFeatures_getTriggerHysteresisIndexMax(this);                    }
   int32                  getTriggerHysteresisIndexStep()            { return AiFeatures_getTriggerHysteresisIndexStep(this);                   }
   
   /*trigger 1*/                                                       
   Array<TriggerAction>*  getTrigger1Actions()                       { return (Array<TriggerAction>*)AiFeatures_getTrigger1Actions(this);       }
   void                   getTrigger1DelayRange(MathInterval &x)     {        AiFeatures_getTrigger1DelayRange(this, &x);                       }
   Array<SignalDrop>*     getTrigger1Sources()                       { return (Array<SignalDrop>*)AiFeatures_getTrigger1Sources(this);          }
   ValueRange             getTrigger1SourceVrg()                     { return AiFeatures_getTrigger1SourceVrg(this);                            }
   double                 getTrigger1HysteresisIndexMax()            { return AiFeatures_getTrigger1HysteresisIndexMax(this);                   }
   int32                  getTrigger1HysteresisIndexStep()           { return AiFeatures_getTrigger1HysteresisIndexStep(this);                  }

   /*trigger 2*/                                                       
   Array<TriggerAction>*  getTrigger2Actions()                       { return (Array<TriggerAction>*)AiFeatures_getTrigger2Actions(this);       }
   void                   getTrigger2DelayRange(MathInterval &x)     {        AiFeatures_getTrigger2DelayRange(this, &x);                       }
   Array<SignalDrop>*     getTrigger2Sources()                       { return (Array<SignalDrop>*)AiFeatures_getTrigger2Sources(this);          }
   ValueRange             getTrigger2SourceVrg()                     { return AiFeatures_getTrigger2SourceVrg(this);                            }
   double                 getTrigger2HysteresisIndexMax()            { return AiFeatures_getTrigger2HysteresisIndexMax(this);                   }
   int32                  getTrigger2HysteresisIndexStep()           { return AiFeatures_getTrigger2HysteresisIndexStep(this);                  }

   /*trigger 3*/                                                       
   Array<TriggerAction>*  getTrigger3Actions()                       { return (Array<TriggerAction>*)AiFeatures_getTrigger3Actions(this);       }
   void                   getTrigger3DelayRange(MathInterval &x)     {        AiFeatures_getTrigger3DelayRange(this, &x);                       }
   Array<SignalDrop>*     getTrigger3Sources()                       { return (Array<SignalDrop>*)AiFeatures_getTrigger3Sources(this);          }
   ValueRange             getTrigger3SourceVrg()                     { return AiFeatures_getTrigger3SourceVrg(this);                            }
   double                 getTrigger3HysteresisIndexMax()            { return AiFeatures_getTrigger3HysteresisIndexMax(this);                   }
   int32                  getTrigger3HysteresisIndexStep()           { return AiFeatures_getTrigger3HysteresisIndexStep(this);                  }

public: /*for compatible*/
   MathInterval getConvertClockRange() 
   {
      MathInterval x;
      getConvertClockRange(x);
      return x;
   }
   MathInterval getScanClockRange()
   {
      MathInterval x;
      getScanClockRange(x);
      return x;
   }
   MathInterval getTriggerDelayRange()
   {
      MathInterval x;
      getTriggerDelayRange(x);
      return x;
   }
   MathInterval getTrigger1DelayRange()
   {
      MathInterval x;
      getTrigger1DelayRange(x);
      return x;
   }
   bool getTriggerSupported()
   {
      return getTriggerCount() > 0;
   }
   bool getTrigger1Supported()
   {
      return getTriggerCount() > 1;
   }

   AiChannelType getChannelType()
   { 
      return AiFeatures_getChannelType(this);               
   }

private:
   DAQCLS_DISABLE_CONSTRUCT(AiFeatures)
};

/* Interface AiCtrlBase */
class AiCtrlBase : public DaqCtrlBase
{
public:
   /*AI Control Common Method*/

   /*AI Control Common Property*/
   AiFeatures*       getFeatures()                                                        { return DAQFN_TV(DNLAPI_AI, 58, AiFeatures *)(this); }
   Array<AiChannel>* getChannels()                                                        { return DAQFN_TV(DNLAPI_AI, 59, Array<AiChannel>*)(this); }
   int32             getChannelCount()                                                    { return DAQFN_TV(DNLAPI_AI, 60, int32 )(this);        }
   ErrorCode         runSensorCalibration(int32 ch, CalibrationData const *data)          { return DAQFN_ETT(DNLAPI_AI, 111, int32, CalibrationData const*)(this, ch, data);    }

private:
   DAQCLS_DISABLE_CONSTRUCT(AiCtrlBase)
};

/* Interface InstantAiCtrl */
class InstantAiCtrl : public AiCtrlBase
{
public:
   /*Instant AI Method & Property*/
   CjcSetting* getCjc()                                                                   { return InstantAiCtrl_getCjc(this);  }
   ErrorCode   ReadAny(int32 chStart, int32 chCount, void *dataRaw, double *dataScaled)   { return InstantAiCtrl_ReadAny(this, chStart, chCount, dataRaw, dataScaled); }

   double      getAutoConvertClockRate()                                                  { return InstantAiCtrl_getAutoConvertClockRate(this);           }
   ErrorCode   setAutoConvertClockRate(double value)                                      { return InstantAiCtrl_setAutoConvertClockRate(this, value);    }
   int32       getAutoConvertChannelStart()                                               { return InstantAiCtrl_getAutoConvertChannelStart(this);        }
   ErrorCode   setAutoConvertChannelStart(int32 value)                                    { return InstantAiCtrl_setAutoConvertChannelStart(this, value); }
   int32       getAutoConvertChannelCount()                                               { return InstantAiCtrl_getAutoConvertChannelCount(this);        }
   ErrorCode   setAutoConvertChannelCount(int32 value)                                    { return InstantAiCtrl_setAutoConvertChannelCount(this, value); }

public:
   /*Create an InstantAictrl. Note: Use 'dispose' to free the object*/
   static InstantAiCtrl * Create(void)                                                    { return InstantAiCtrl_Create();  }

public: /*helpers*/
   ErrorCode   Read(int32 ch, double &dataScaled)                                         { return ReadAny(ch, 1, NULL, &dataScaled);             }
   ErrorCode   Read(int32 ch, int16 &dataRaw)                                             { return ReadAny(ch, 1, &dataRaw, NULL);                }
   ErrorCode   Read(int32 ch, int32 &dataRaw)                                             { return ReadAny(ch, 1, &dataRaw, NULL);                }
   ErrorCode   Read(int32 chStart, int32 chCount, double dataScaled[])                    { return ReadAny(chStart, chCount, NULL, dataScaled);   }
   ErrorCode   Read(int32 chStart, int32 chCount, int16 dataRaw[], double dataScaled[])   { return ReadAny(chStart, chCount, dataRaw, dataScaled);}
   ErrorCode   Read(int32 chStart, int32 chCount, int32 dataRaw[], double dataScaled[])   { return ReadAny(chStart, chCount, dataRaw, dataScaled);}

private:
   DAQCLS_DISABLE_CONSTRUCT(InstantAiCtrl)
};

/* Interface BufferedAiCtrl */
class BufferedAiCtrl : public AiCtrlBase
{
public:
   /*Buffered AI Method*/
   ErrorCode     Prepare()                                                                { return BufferedAiCtrl_Prepare(this); }
   ErrorCode     RunOnce()                                                                { return BufferedAiCtrl_RunOnce(this); }
   ErrorCode     Start()                                                                  { return BufferedAiCtrl_Start(this);   }
   ErrorCode     Stop()                                                                   { return BufferedAiCtrl_Stop(this);    }
   void          Release()                                                                {        BufferedAiCtrl_Release(this); }
   bool          GetBufferStatus(int32 *count, int32 *offset)                             { return !!BufferedAiCtrl_GetBufferStatus(this, count, offset);}
   ErrorCode     GetData(int32 count, int16 rawData[])                                    { return BufferedAiCtrl_GetDataI16(this, count, rawData);    }
   ErrorCode     GetData(int32 count, int32 rawData[])                                    { return BufferedAiCtrl_GetDataI32(this, count, rawData);    }
   ErrorCode     GetData(int32 count, double scaledData[])                                { return BufferedAiCtrl_GetDataF64(this, count, scaledData); }
   
   /*Buffered AI Property*/                                                                 
   void*         getBuffer()                                                              { return BufferedAiCtrl_getBuffer(this);         }
   int32         getBufferCapacity()                                                      { return BufferedAiCtrl_getBufferCapacity(this); }
   ScanChannel*  getScanChannel()                                                         { return BufferedAiCtrl_getScanChannel(this);    }
   ConvertClock* getConvertClock()                                                        { return BufferedAiCtrl_getConvertClock(this);   }
   ScanClock*    getScanClock()                                                           { return BufferedAiCtrl_getScanClock(this);      }
   Trigger*      getTrigger()                                                             { return BufferedAiCtrl_getTrigger(this);        }
   Trigger*      getTrigger1()                                                            { return BufferedAiCtrl_getTrigger1(this);       }
   Trigger*      getTrigger2()                                                            { return BufferedAiCtrl_getTrigger2(this);       }
   Trigger*      getTrigger3()                                                            { return BufferedAiCtrl_getTrigger3(this);       }
   bool          getStreaming()                                                           { return !!BufferedAiCtrl_getStreaming(this);     }
   ErrorCode     setStreaming(bool value)                                                 { return BufferedAiCtrl_setStreaming(this, value);}

public:
   /*Create an BufferedAiCtrl. Note: Use 'dispose' to free the object*/
   static BufferedAiCtrl * Create(void)                                                   { return BufferedAiCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void          addDataReadyHandler(BfdAiEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiDataReady,     (DaqEventProc)proc, userParam);}
   void          removeDataReadyHandler(BfdAiEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiDataReady,     (DaqEventProc)proc, userParam);}
   void          addOverrunHandler(BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiOverrun,       (DaqEventProc)proc, userParam);}
   void          removeOverrunHandler(BfdAiEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiOverrun,       (DaqEventProc)proc, userParam);}
   void          addCacheOverflowHandler(BfdAiEventProc proc, void *userParam)            { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          removeCacheOverflowHandler(BfdAiEventProc proc, void *userParam)         { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          addStoppedHandler(BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiStopped,       (DaqEventProc)proc, userParam);}
   void          removeStoppedHandler(BfdAiEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiStopped,       (DaqEventProc)proc, userParam);}
   void          addBurnOutHandler(BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiBurnOut,       (DaqEventProc)proc, userParam);}
   void          removeBurnOutHandler(BfdAiEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiBurnOut,       (DaqEventProc)proc, userParam);}
   void          addConvStoppedHandler(BfdAiEventProc proc, void *userParam)              { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiConvStopped,   (DaqEventProc)proc, userParam);}
   void          removeConvStoppedHandler(BfdAiEventProc proc, void *userParam)           { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiConvStopped,   (DaqEventProc)proc, userParam);}

public: /*For compatible ONLY!*/
   void          addDataReadyListener(BfdAiEventListener & listener)                      { BufferedAiCtrl_addDataReadyListener(this, &listener);        }
   void          removeDataReadyListener(BfdAiEventListener & listener)                   { BufferedAiCtrl_removeDataReadyListener(this, &listener);     }
   void          addOverrunListener(BfdAiEventListener & listener)                        { BufferedAiCtrl_addOverrunListener(this, &listener);          }
   void          removeOverrunListener(BfdAiEventListener & listener)                     { BufferedAiCtrl_removeOverrunListener(this, &listener);       }
   void          addCacheOverflowListener(BfdAiEventListener & listener)                  { BufferedAiCtrl_addCacheOverflowListener(this, &listener);    }
   void          removeCacheOverflowListener(BfdAiEventListener & listener)               { BufferedAiCtrl_removeCacheOverflowListener(this, &listener); }
   void          addStoppedListener(BfdAiEventListener & listener)                        { BufferedAiCtrl_addStoppedListener(this, &listener);          }
   void          removeStoppedListener(BfdAiEventListener & listener)                     { BufferedAiCtrl_removeStoppedListener(this, &listener);       }

private:
   DAQCLS_DISABLE_CONSTRUCT(BufferedAiCtrl)
};

/* Interface WaveformAiCtrl */
class WaveformAiCtrl : public AiCtrlBase
{
public:
   /*Waveform AI Method*/
   ErrorCode     Prepare()                                                                { return WaveformAiCtrl_Prepare(this); }
   ErrorCode     Start()                                                                  { return WaveformAiCtrl_Start(this);   }
   ErrorCode     Stop()                                                                   { return WaveformAiCtrl_Stop(this);    }
   void          Release()                                                                {        WaveformAiCtrl_Release(this); }
   bool          GetBufferStatus(int32 *count, int32 *offset)                             { return !!WaveformAiCtrl_GetBufferStatus(this, count, offset);}

   ErrorCode GetData(int32 count, int16 rawData[], 
      int32 timeout = 0, int32 *returned = NULL, double *startTime = NULL, int32 *markCount = NULL, DataMark *markBuf = NULL)
   { 
      return WaveformAiCtrl_GetDataI16(this, count, rawData, timeout, returned, startTime, markCount, markBuf);    
   }
   ErrorCode GetData(int32 count, int32 rawData[], 
      int32 timeout = 0, int32 *returned = NULL, double *startTime = NULL, int32 *markCount = NULL, DataMark *markBuf = NULL)
   { 
      return WaveformAiCtrl_GetDataI32(this, count, rawData, timeout, returned, startTime, markCount, markBuf);    
   }
   ErrorCode GetData(int32 count, double scaledData[], 
      int32 timeout = 0, int32 *returned = NULL, double *startTime = NULL, int32 *markCount = NULL, DataMark *markBuf = NULL)
   { 
      return WaveformAiCtrl_GetDataF64(this, count, scaledData, timeout, returned, startTime, markCount, markBuf); 
   }

   /*Waveform AI Property*/                                                                 
   void*         getBuffer()                                                              { return WaveformAiCtrl_getBuffer(this);         }
   int32         getBufferCapacity()                                                      { return WaveformAiCtrl_getBufferCapacity(this); }
   Conversion*   getConversion()                                                          { return WaveformAiCtrl_getConversion(this);     }
   Record*       getRecord()                                                              { return WaveformAiCtrl_getRecord(this);         }
   Trigger*      getTrigger()                                                             { return WaveformAiCtrl_getTrigger(this);        }
   Trigger*      getTrigger1()                                                            { return WaveformAiCtrl_getTrigger1(this);       }
   Trigger*      getTrigger2()                                                            { return WaveformAiCtrl_getTrigger2(this);       }
   Trigger*      getTrigger3()                                                            { return WaveformAiCtrl_getTrigger3(this);       }

public:
   /*Create an WaveformAiCtrl. Note: Use 'dispose' to free the object*/
   static WaveformAiCtrl * Create(void)                                                   { return WaveformAiCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void          addDataReadyHandler(BfdAiEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiDataReady,     (DaqEventProc)proc, userParam);}
   void          removeDataReadyHandler(BfdAiEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiDataReady,     (DaqEventProc)proc, userParam);}
   void          addOverrunHandler(BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiOverrun,       (DaqEventProc)proc, userParam);}
   void          removeOverrunHandler(BfdAiEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiOverrun,       (DaqEventProc)proc, userParam);}
   void          addCacheOverflowHandler(BfdAiEventProc proc, void *userParam)            { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          removeCacheOverflowHandler(BfdAiEventProc proc, void *userParam)         { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          addStoppedHandler(BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiStopped,       (DaqEventProc)proc, userParam);}
   void          removeStoppedHandler(BfdAiEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiStopped,       (DaqEventProc)proc, userParam);}

   void          addBurnOutHandler(BfdAiEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiBurnOut,       (DaqEventProc)proc, userParam);}
   void          removeBurnOutHandler(BfdAiEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiBurnOut,       (DaqEventProc)proc, userParam);}
   void          addTimeStampOverrunHandler(BfdAiEventProc proc, void *userParam)         { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiTimeStampOverrun, (DaqEventProc)proc, userParam);}
   void          removeTimeStampOverrunHandler(BfdAiEventProc proc, void *userParam)      { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiTimeStampOverrun, (DaqEventProc)proc, userParam);}
   void          addTimeStampCacheOverflowHandler(BfdAiEventProc proc, void *userParam)   { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiTimeStampCacheOverflow, (DaqEventProc)proc, userParam);}
   void          removeTimeStampCacheOverflowHandler(BfdAiEventProc proc, void *userParam){ DaqCtrlBase_removeEventHandler(this, EvtBufferedAiTimeStampCacheOverflow, (DaqEventProc)proc, userParam);}
   void          addMarkOverrunHandler(BfdAiEventProc proc, void *userParam)              { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiMarkOverrun, (DaqEventProc)proc, userParam);}
   void          removeMarkOverrunHandler(BfdAiEventProc proc, void *userParam)           { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiMarkOverrun, (DaqEventProc)proc, userParam);}
   void          addConvStoppedHandler(BfdAiEventProc proc, void *userParam)              { DaqCtrlBase_addEventHandler   (this, EvtBufferedAiConvStopped,   (DaqEventProc)proc, userParam);}
   void          removeConvStoppedHandler(BfdAiEventProc proc, void *userParam)           { DaqCtrlBase_removeEventHandler(this, EvtBufferedAiConvStopped,   (DaqEventProc)proc, userParam);}

private:
   DAQCLS_DISABLE_CONSTRUCT(WaveformAiCtrl)
};

/* ----------------------------------------------------------*/
/*AO related classes                                         */
/* ----------------------------------------------------------*/
/* Interface AoChannel */
class AoChannel
{
public:
   int32      getChannel()                                           { return AoChannel_getChannel(this);              }
   ValueRange getValueRange()                                        { return AoChannel_getValueRange(this);           }
   ErrorCode  setValueRange(ValueRange value)                        { return AoChannel_setValueRange(this, value);    }

   double     getExtRefBipolar()                                     { return AoChannel_getExtRefBipolar(this);        } 
   ErrorCode  setExtRefBipolar(double value)                         { return AoChannel_setExtRefBipolar(this, value); } 
   double     getExtRefUnipolar()                                    { return AoChannel_getExtRefUnipolar(this);       } 
   ErrorCode  setExtRefUnipolar(double value)                        { return AoChannel_setExtRefUnipolar(this, value);} 

private:
   DAQCLS_DISABLE_CONSTRUCT(AoChannel)
};
typedef Array<AoChannel> AoChannelCollection; /*for compatible*/
typedef AoChannel        AnalogChannel;       /*for compatible*/

/* Interface AoFeatures */
class AoFeatures
{
public:
   /*DAC features*/
   int32                 getResolution()                             { return AoFeatures_getResolution(this);             }
   int32                 getDataSize()                               { return AoFeatures_getDataSize(this);               }
   int32                 getDataMask()                               { return AoFeatures_getDataMask(this);               }

   /*channel features*/                                                
   int32                 getChannelCountMax()                        { return AoFeatures_getChannelCountMax(this);        }
   Array<ValueRange>*    getValueRanges()                            { return (Array<ValueRange>*)AoFeatures_getValueRanges(this); }
   bool                  getExternalRefAntiPolar()                   { return !!AoFeatures_getExternalRefAntiPolar(this); }
   void                  getExternalRefRange(MathInterval &x)        {        AoFeatures_getExternalRefRange(this, &x);   }

   /*buffered ao->basic features*/                                     
   bool                  getBufferedAoSupported()                    { return !!AoFeatures_getBufferedAoSupported(this);  }
   SamplingMethod        getSamplingMethod()                         { return AoFeatures_getSamplingMethod(this);         }
   int32                 getChannelStartBase()                       { return AoFeatures_getChannelStartBase(this);       }
   int32                 getChannelCountBase()                       { return AoFeatures_getChannelCountBase(this);       }

   /*buffered ao->conversion clock features*/                          
   Array<SignalDrop>*    getConvertClockSources()                    { return (Array<SignalDrop>*)AoFeatures_getConvertClockSources(this);}
   void                  getConvertClockRange(MathInterval &x)       {        AoFeatures_getConvertClockRange(this, &x);  }

   /*buffered ao->trigger features*/                                   
   int32                 getTriggerCount()                           { return AoFeatures_getTriggerCount(this);           }
   bool                  getRetriggerable()                          { return !!AoFeatures_getRetriggerable(this);        }
   /*trigger 0*/                                                       
   Array<SignalDrop>*    getTriggerSources()                         { return (Array<SignalDrop>*)AoFeatures_getTriggerSources(this);   }
   Array<TriggerAction>* getTriggerActions()                         { return (Array<TriggerAction>*)AoFeatures_getTriggerActions(this);}
   void                  getTriggerDelayRange(MathInterval &x)       {        AoFeatures_getTriggerDelayRange(this, &x);  }
   /*trigger 1*/                                                       
   Array<SignalDrop>*    getTrigger1Sources()                        { return (Array<SignalDrop>*)AoFeatures_getTrigger1Sources(this);  }
   Array<TriggerAction>* getTrigger1Actions()                        { return (Array<TriggerAction>*)AoFeatures_getTrigger1Actions(this);}
   void                  getTrigger1DelayRange(MathInterval &x)      {        AoFeatures_getTrigger1DelayRange(this, &x); }

public: /*for compatible*/
   MathInterval getExternalRefRange()
   {
      MathInterval x;
      getExternalRefRange(x);
      return x;
   }
   MathInterval getConvertClockRange() 
   {
      MathInterval x;
      getConvertClockRange(x);
      return x;
   }
   MathInterval getTriggerDelayRange()
   {
      MathInterval x;
      getTriggerDelayRange(x);
      return x;
   }
   MathInterval getTrigger1DelayRange()
   {
      MathInterval x;
      getTrigger1DelayRange(x);
      return x;
   }
   bool getTriggerSupported()
   {
      return getTriggerCount() > 0;
   }
   bool getTrigger1Supported()
   {
      return getTriggerCount() > 1;
   }

private:
   DAQCLS_DISABLE_CONSTRUCT(AoFeatures)
};

/* Interface AoCtrlBase */
class AoCtrlBase : public DaqCtrlBase
{
public:
   /*AI Control Common Method*/

   /*AO Control Common Property*/
   AoFeatures*       getFeatures()                                                        { return DAQFN_TV(DNLAPI_AO, 25, AoFeatures *)(this);    }
   Array<AoChannel>* getChannels()                                                        { return DAQFN_TV(DNLAPI_AO, 26, Array<AoChannel>*)(this);}
   int32             getChannelCount()                                                    { return DAQFN_TV(DNLAPI_AO, 27, int32 )(this);        }

public: /*For compatible ONLY!*/
   /*Please use AoChannel's properties instead. */
   double            getExtRefValueForUnipolar()                                          { return DAQFN_TV(DNLAPI_AO, 28, double)(this);        }
   ErrorCode         setExtRefValueForUnipolar(double value)                              { return DAQFN_ET(DNLAPI_AO, 29, double)(this, value); }
   double            getExtRefValueForBipolar()                                           { return DAQFN_TV(DNLAPI_AO, 30, double)(this);        }
   ErrorCode         setExtRefValueForBipolar(double value)                               { return DAQFN_ET(DNLAPI_AO, 31, double)(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(AoCtrlBase)
};

/* Interface InstantAoCtrl */
class InstantAoCtrl : public AoCtrlBase
{
public:
   /*Instant AO Method*/
   ErrorCode WriteAny(int32 chStart, int32 chCount, void *dataRaw, double *dataScaled)    { return InstantAoCtrl_WriteAny(this, chStart, chCount, dataRaw, dataScaled); }

public:
   /*Create an InstantAoctrl. Note: Use 'dispose' to free the object*/
   static InstantAoCtrl * Create(void)                                                    { return InstantAoCtrl_Create();  }

public: /*helpers*/
   ErrorCode Write(int32 ch, double dataScaled)                                           { return WriteAny(ch, 1, NULL, &dataScaled);          }
   ErrorCode Write(int32 ch, int16 dataRaw)                                               { return WriteAny(ch, 1, &dataRaw, NULL);             }
   ErrorCode Write(int32 ch, int32 dataRaw)                                               { return WriteAny(ch, 1, &dataRaw, NULL);             }
   ErrorCode Write(int32 chStart, int32 chCount, double dataScaled[])                     { return WriteAny(chStart, chCount, NULL, dataScaled);}
   ErrorCode Write(int32 chStart, int32 chCount, int16 dataRaw[])                         { return WriteAny(chStart, chCount, dataRaw, NULL);   }
   ErrorCode Write(int32 chStart, int32 chCount, int32 dataRaw[])                         { return WriteAny(chStart, chCount, dataRaw, NULL);   }

private:
   DAQCLS_DISABLE_CONSTRUCT(InstantAoCtrl)
};

/* Interface BufferedAoCtrl */   
class BufferedAoCtrl : public AoCtrlBase
{
public:
   /*Buffered AO Method*/
   ErrorCode     Prepare()                                                                { return BufferedAoCtrl_Prepare(this);  }
   ErrorCode     RunOnce()                                                                { return BufferedAoCtrl_RunOnce(this);  }
   ErrorCode     Start()                                                                  { return BufferedAoCtrl_Start(this);    }
   ErrorCode     Stop(int32 action)                                                       { return BufferedAoCtrl_Stop(this, action);}
   void          Release()                                                                {        BufferedAoCtrl_Release(this);  }
   bool          GetBufferStatus(int32 *count, int32 *offset)                             { return !!BufferedAoCtrl_GetBufferStatus(this, count, offset);}
   ErrorCode     SetData(int32 count, int16 rawData[])                                    { return BufferedAoCtrl_SetDataI16(this, count, rawData);   }
   ErrorCode     SetData(int32 count, int32 rawData[])                                    { return BufferedAoCtrl_SetDataI32(this, count, rawData);   }
   ErrorCode     SetData(int32 count, double scaledData[])                                { return BufferedAoCtrl_SetDataF64(this, count, scaledData);}
                                                                                            
   /*Buffered AO Property*/                                                                 
   void*         getBuffer()                                                              { return BufferedAoCtrl_getBuffer(this);           }
   int32         getBufferCapacity()                                                      { return BufferedAoCtrl_getBufferCapacity(this);   }
   ScanChannel*  getScanChannel()                                                         { return BufferedAoCtrl_getScanChannel(this);      }
   ConvertClock* getConvertClock()                                                        { return BufferedAoCtrl_getConvertClock(this);     }
   Trigger*      getTrigger()                                                             { return BufferedAoCtrl_getTrigger(this);          }
   Trigger*      getTrigger1()                                                            { return BufferedAoCtrl_getTrigger1(this);         }
   bool          getStreaming()                                                           { return !!BufferedAoCtrl_getStreaming(this);      }
   ErrorCode     setStreaming(bool value)                                                 { return BufferedAoCtrl_setStreaming(this, value); }

public:
   /*Create an BufferedAoCtrl. Note: Use 'dispose' to free the object*/
   static BufferedAoCtrl * Create(void)                                                   { return BufferedAoCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void          addDataTransmittedHandler(BfdAoEventProc proc, void *userParam)          { DaqCtrlBase_addEventHandler   (this, EvtBufferedAoDataTransmitted, (DaqEventProc)proc, userParam); }
   void          removeDataTransmittedHandler(BfdAoEventProc proc, void *userParam)       { DaqCtrlBase_removeEventHandler(this, EvtBufferedAoDataTransmitted, (DaqEventProc)proc, userParam); }
   void          addUnderrunHandler(BfdAoEventProc proc, void *userParam)                 { DaqCtrlBase_addEventHandler   (this, EvtBufferedAoUnderrun,        (DaqEventProc)proc, userParam); }
   void          removeUnderrunHandler(BfdAoEventProc proc, void *userParam)              { DaqCtrlBase_removeEventHandler(this, EvtBufferedAoUnderrun,        (DaqEventProc)proc, userParam); }
   void          addCacheEmptiedHandler(BfdAoEventProc proc, void *userParam)             { DaqCtrlBase_addEventHandler   (this, EvtBufferedAoCacheEmptied,    (DaqEventProc)proc, userParam); }
   void          removeCacheEmptiedHandler(BfdAoEventProc proc, void *userParam)          { DaqCtrlBase_removeEventHandler(this, EvtBufferedAoCacheEmptied,    (DaqEventProc)proc, userParam); }
   void          addTransitStoppedHandler(BfdAoEventProc proc, void *userParam)           { DaqCtrlBase_addEventHandler   (this, EvtBufferedAoTransStopped,    (DaqEventProc)proc, userParam); }
   void          removeTransitStoppedHandler(BfdAoEventProc proc, void *userParam)        { DaqCtrlBase_removeEventHandler(this, EvtBufferedAoTransStopped,    (DaqEventProc)proc, userParam); }
   void          addStoppedHandler(BfdAoEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtBufferedAoStopped,         (DaqEventProc)proc, userParam); }
   void          removeStoppedHandler(BfdAoEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtBufferedAoStopped,         (DaqEventProc)proc, userParam); }

public: /*For compatible ONLY!*/
   void          addDataTransmittedListener(BfdAoEventListener & listener)                { BufferedAoCtrl_addDataTransmittedListener(this, &listener);    }
   void          removeDataTransmittedListener(BfdAoEventListener & listener)             { BufferedAoCtrl_removeDataTransmittedListener(this, &listener); }
   void          addUnderrunListener(BfdAoEventListener & listener)                       { BufferedAoCtrl_addUnderrunListener(this, &listener);           }
   void          removeUnderrunListener(BfdAoEventListener & listener)                    { BufferedAoCtrl_removeUnderrunListener(this, &listener);        }
   void          addCacheEmptiedListener(BfdAoEventListener & listener)                   { BufferedAoCtrl_addCacheEmptiedListener(this, &listener);       }
   void          removeCacheEmptiedListener(BfdAoEventListener & listener)                { BufferedAoCtrl_removeCacheEmptiedListener(this, &listener);    }
   void          addTransitStoppedListener(BfdAoEventListener & listener)                 { BufferedAoCtrl_addTransitStoppedListener(this, &listener);     }
   void          removeTransitStoppedListener(BfdAoEventListener & listener)              { BufferedAoCtrl_removeTransitStoppedListener(this, &listener);  }
   void          addStoppedListener(BfdAoEventListener & listener)                        { BufferedAoCtrl_addStoppedListener(this, &listener);            }
   void          removeStoppedListener(BfdAoEventListener & listener)                     { BufferedAoCtrl_removeStoppedListener(this, &listener);         }

private:
   DAQCLS_DISABLE_CONSTRUCT(BufferedAoCtrl)
};

/* ----------------------------------------------------------*/
/*DIO related classes                                        */
/* ----------------------------------------------------------*/
/* Interface DioPort */
class DioPort
{
public:
   int32      getPort()                                                    { return DioPort_getPort(this);                 }
   uint8      getDirectionMask()                                           { return DioPort_getDirectionMask(this);        }
   ErrorCode  setDirectionMask(uint8 value)                                { return DioPort_setDirectionMask(this, value); }

   /*DI port property*/                                                      
   uint8      getDiInversePort()                                           { return DioPort_getDiInversePort(this);        }
   ErrorCode  setDiInversePort(uint8 value)                                { return DioPort_setDiInversePort(this, value); }

   /*DO port property*/                                                      
   uint8      getPresetValue()                                             { return DioPort_getDoPresetValue(this);        }
   ErrorCode  setPresetValue(uint8 value)                                  { return DioPort_setDoPresetValue(this, value); }

public: /*For compatible ONLY!*/
   DioPortDir getDirection()                                               { return (DioPortDir)getDirectionMask();        }
   ErrorCode  setDirection(DioPortDir value)                               { return setDirectionMask((uint8)value);        }

private:
   DAQCLS_DISABLE_CONSTRUCT(DioPort)
};
typedef DioPort PortDirection;       /*for compatible*/

/* Interface DiintChannel */
class DiintChannel
{
public:
   int32        getChannel()                                               { return DiintChannel_getChannel(this);         }
   bool         getEnabled()                                               { return !!DiintChannel_getEnabled(this);       }
   ErrorCode    setEnabled(bool value)                                     { return DiintChannel_setEnabled(this, value);  }
   bool         getGated()                                                 { return !!DiintChannel_getGated(this);         }
   ErrorCode    setGated(bool value)                                       { return DiintChannel_setGated(this, value);    }
   ActiveSignal getTrigEdge()                                              { return DiintChannel_getTrigEdge(this);        }
   ErrorCode    setTrigEdge(ActiveSignal value)                            { return DiintChannel_setTrigEdge(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(DiintChannel)
};

/* Interface DiCosintPort */
class DiCosintPort
{
public:
   int32     getPort()                                                     { return DiCosintPort_getPort(this);        }
   uint8     getMask()                                                     { return DiCosintPort_getMask(this);        }
   ErrorCode setMask(uint8 value)                                          { return DiCosintPort_setMask(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(DiCosintPort)
};

/* Interface DiPmintPort */
class DiPmintPort
{
public:
   int32     getPort()                                                     { return DiPmintPort_getPort(this);          }
   uint8     getMask()                                                     { return DiPmintPort_getMask(this);          }
   ErrorCode setMask(uint8 value)                                          { return DiPmintPort_setMask(this, value);   }
   uint8     getPattern()                                                  { return DiPmintPort_getPattern(this);       }
   ErrorCode setPattern(uint8 value)                                       { return DiPmintPort_setPattern(this, value);}

private:
   DAQCLS_DISABLE_CONSTRUCT(DiPmintPort)
};

/* Interface DioFeatures */ 
class DioFeatures
{
public:
   /*DIO common*/
   bool                 getPortProgrammable()                              { return !!DioFeatures_getPortProgrammable(this);               }
   int32                getChannelCountMax()                               { return DioFeatures_getChannelCountMax(this);                  }
   int32                getPortCount()                                     { return DioFeatures_getPortCount(this);                        }
   Array<uint8>*        getPortsType()                                     { return (Array<uint8>*)DioFeatures_getPortsType(this);         }
   bool                 getDiSupported()                                   { return !!DioFeatures_getDiSupported(this);                    }
   bool                 getDoSupported()                                   { return !!DioFeatures_getDoSupported(this);                    }
                                                                             
   /*DI features*/                                                           
   Array<uint8>*        getDiDataMask()                                    { return (Array<uint8>*)DioFeatures_getDiDataMask(this);        }
                                                                             
   /*DI noise filter features*/                                              
   bool                 getDiNoiseFilterSupported()                        { return !!DioFeatures_getDiNoiseFilterSupported(this);              }
   Array<uint8>*        getDiNoiseFilterOfChannels()                       { return (Array<uint8>*)DioFeatures_getDiNoiseFilterOfChannels(this);}
   void                 getDiNoiseFilterBlockTimeRange(MathInterval &x)    {        DioFeatures_getDiNoiseFilterBlockTimeRange(this, &x);  }
                                                                             
   /*DI interrupt features*/                                                 
   bool                 getDiintSupported()                                { return !!DioFeatures_getDiintSupported(this);                 }
   bool                 getDiintGateSupported()                            { return !!DioFeatures_getDiintGateSupported(this);             }
   bool                 getDiCosintSupported()                             { return !!DioFeatures_getDiCosintSupported(this);              }
   bool                 getDiPmintSupported()                              { return !!DioFeatures_getDiPmintSupported(this);               }
   Array<ActiveSignal>* getDiintTriggerEdges()                             { return (Array<ActiveSignal>*)DioFeatures_getDiintTriggerEdges(this);}
   Array<uint8>*        getDiintOfChannels()                               { return (Array<uint8>*)DioFeatures_getDiintOfChannels(this);      }
   Array<uint8>*        getDiintGateOfChannels()                           { return (Array<uint8>*)DioFeatures_getDiintGateOfChannels(this);  }
   Array<uint8>*        getDiCosintOfPorts()                               { return (Array<uint8>*)DioFeatures_getDiCosintOfPorts(this);      }
   Array<uint8>*        getDiPmintOfPorts()                                { return (Array<uint8>*)DioFeatures_getDiPmintOfPorts(this);       }
   Array<EventId>*      getDiSnapEventSources()                            { return (Array<EventId>*)DioFeatures_getDiSnapEventSources(this); }
                                                                             
   /*DO features*/                                                           
   Array<uint8>*        getDoDataMask()                                    { return (Array<uint8>*)DioFeatures_getDoDataMask(this);         }
   Array<SignalDrop>*   getDoFreezeSignalSources()                         { return (Array<SignalDrop>*)DioFeatures_getDoFreezeSignalSources(this); }                        
   void                 getDoReflectWdtFeedIntervalRange(MathInterval &x)  {        DioFeatures_getDoReflectWdtFeedIntervalRange(this, &x); } 
   Depository           getDoPresetValueDepository()                       { return DioFeatures_getDoPresetValueDepository(this);           }                        

public: /*for compatible*/
   MathInterval getDiNoiseFilterBlockTimeRange()
   {
      MathInterval x;
      getDiNoiseFilterBlockTimeRange(x);
      return x;
   }
   MathInterval getDoReflectWdtFeedIntervalRange() 
   {
      MathInterval x;
      getDoReflectWdtFeedIntervalRange(x);
      return x;
   }
   Array<uint8>* getDataMask()                  { return (Array<uint8>*)DioFeatures_getDataMask(this); }
   bool          getNoiseFilterSupported()      { return getDiNoiseFilterSupported();                  }
   Array<uint8>* getNoiseFilterOfChannels()     { return getDiNoiseFilterOfChannels();                 }
   MathInterval  getNoiseFilterBlockTimeRange() { return getDiNoiseFilterBlockTimeRange();             }
   Array<int32>* getSnapEventSources()          { return (Array<int32>*)getDiSnapEventSources();       }

private:
   DAQCLS_DISABLE_CONSTRUCT(DioFeatures)
};
typedef DioFeatures DiFeatures; /*for compatible*/
typedef DioFeatures DoFeatures; /*for compatible*/


/* Interface DioCtrlBase */ 
class DioCtrlBase : public DaqCtrlBase
{
public:
   /*DIO Control Common Property*/
   DioFeatures*    getFeatures()                { return DAQFN_TV(DNLAPI_DIO, 45, DioFeatures *)(this); }
   int32           getPortCount()               { return DAQFN_TV(DNLAPI_DIO, 46, int32)(this);          }
   Array<DioPort>* getPorts()                   { return DAQFN_TV(DNLAPI_DIO, 47, Array<DioPort>*)(this);}

public: /*for compatible*/
   Array<DioPort>* getPortDirection()           { return getPorts(); }

private:
   DAQCLS_DISABLE_CONSTRUCT(DioCtrlBase)
};

/* Interface InstantDiCtrl */ 
class InstantDiCtrl : public DioCtrlBase
{
public:
   /*Instant DI Method*/
   ErrorCode ReadAny(int32 portStart, int32 portCount, uint8 data[])                { return InstantDiCtrl_ReadAny(this, portStart, portCount, data); }
   ErrorCode ReadBit(int32 port, int32 bit, uint8* data)                            { return InstantDiCtrl_ReadBit(this, port, bit, data);            }
   ErrorCode SnapStart()                                                            { return InstantDiCtrl_SnapStart(this);  }
   ErrorCode SnapStop()                                                             { return InstantDiCtrl_SnapStop(this);   }

   /*Instant DI Property*/
   double                getNoiseFilterBlockTime()                                  { return InstantDiCtrl_getNoiseFilterBlockTime(this);        }
   ErrorCode             setNoiseFilterBlockTime(double value)                      { return InstantDiCtrl_setNoiseFilterBlockTime(this, value); }
   Array<NosFltChannel>* getNoiseFilter()                                           { return (Array<NosFltChannel>*)InstantDiCtrl_getNoiseFilter(this); }
                                                                                              
   Array<DiintChannel>*  getDiintChannels()                                         { return (Array<DiintChannel>*)InstantDiCtrl_getDiintChannels(this);}
   Array<DiCosintPort>*  getDiCosintPorts()                                         { return (Array<DiCosintPort>*)InstantDiCtrl_getDiCosintPorts(this);}
   Array<DiPmintPort>*   getDiPmintPorts()                                          { return (Array<DiPmintPort>*)InstantDiCtrl_getDiPmintPorts(this);  }

public:
   /*Create an InstantDiCtrl. Note: Use 'dispose' to free the object*/
   static InstantDiCtrl * Create(void)                                              { return InstantDiCtrl_Create();  }

public: /*helpers*/
   ErrorCode Read(int32 port, uint8 & data)                                         { return ReadAny(port, 1, &data);   }
   ErrorCode Read(int32 portStart, int32 portCount, uint8 data[])                   { return ReadAny(portStart, portCount, data);   }

   /*event*/
   void      addInterruptHandler(DiSnapEventProc proc, void *userParam)             { DaqCtrlBase_addEventHandler   (this, EvtDiInterrupt,    (DaqEventProc)proc, userParam);}
   void      removeInterruptHandler(DiSnapEventProc proc, void *userParam)          { DaqCtrlBase_removeEventHandler(this, EvtDiInterrupt,    (DaqEventProc)proc, userParam);}
   void      addChangeOfStateHandler(DiSnapEventProc proc, void *userParam)         { DaqCtrlBase_addEventHandler   (this, EvtDiStatusChange, (DaqEventProc)proc, userParam);}
   void      removeChangeOfStateHandler(DiSnapEventProc proc, void *userParam)      { DaqCtrlBase_removeEventHandler(this, EvtDiStatusChange, (DaqEventProc)proc, userParam);}
   void      addPatternMatchHandler(DiSnapEventProc proc, void *userParam)          { DaqCtrlBase_addEventHandler   (this, EvtDiPatternMatch, (DaqEventProc)proc, userParam);}
   void      removePatternMatchHandler(DiSnapEventProc proc, void *userParam)       { DaqCtrlBase_removeEventHandler(this, EvtDiPatternMatch, (DaqEventProc)proc, userParam);}

public: /*For compatible ONLY!*/
   void      addInterruptListener(DiSnapEventListener & listener)                   { InstantDiCtrl_addInterruptListener(this, &listener);        }
   void      removeInterruptListener(DiSnapEventListener & listener)                { InstantDiCtrl_removeInterruptListener(this, &listener);     }
   void      addChangeOfStateListener(DiSnapEventListener & listener)               { InstantDiCtrl_addChangeOfStateListener(this, &listener);    }
   void      removeChangeOfStateListener(DiSnapEventListener & listener)            { InstantDiCtrl_removeChangeOfStateListener(this, &listener); }
   void      addPatternMatchListener(DiSnapEventListener & listener)                { InstantDiCtrl_addPatternMatchListener(this, &listener);     }
   void      removePatternMatchListener(DiSnapEventListener & listener)             { InstantDiCtrl_removePatternMatchListener(this, &listener);  }

private:
   DAQCLS_DISABLE_CONSTRUCT(InstantDiCtrl)
};

/* Interface InstantDoCtrl */ 
class InstantDoCtrl : public DioCtrlBase
{
public:
   /*Instant DO Method*/
   ErrorCode WriteAny(int32 portStart, int32 portCount, uint8 data[])               { return InstantDoCtrl_WriteAny(this, portStart, portCount, data); }
   ErrorCode ReadAny(int32 portStart, int32 portCount, uint8 data[])                { return InstantDoCtrl_ReadAny(this, portStart, portCount, data);  }
   ErrorCode WriteBit(int32 port, int32 bit, uint8 data)                            { return InstantDoCtrl_WriteBit(this, port, bit, data);   }
   ErrorCode ReadBit(int32 port, int32 bit, uint8* data)                            { return InstantDoCtrl_ReadBit(this, port, bit, data);    }

public:
   /*Create an InstantDoCtrl. Note: Use 'dispose' to free the object*/
   static InstantDoCtrl * Create(void)                                              { return InstantDoCtrl_Create();  }

public: /*helpers*/
   ErrorCode Write(int32 port, uint8 data)                                          { return WriteAny(port, 1, &data);   }
   ErrorCode Write(int32 portStart, int32 portCount, uint8 data[])                  { return WriteAny(portStart, portCount, data);   }
   ErrorCode Read(int32 port, uint8 &data)                                          { return ReadAny(port, 1, &data);   }
   ErrorCode Read(int32 portStart, int32 portCount, uint8 data[])                   { return ReadAny(portStart, portCount, data);   }

private:
   DAQCLS_DISABLE_CONSTRUCT(InstantDoCtrl)
};

/* ----------------------------------------------------------*/
/*Counter related classes                                    */
/* ----------------------------------------------------------*/
template<class T>
class CounterIndexer
{
public:
   void      Dispose()                                               {        CounterIndexer_Dispose((ICounterIndexer*)this);   }   /*destroy the instance*/
   int32     getLength()                                             { return CounterIndexer_getLength((ICounterIndexer*)this);  }
   Array<T>* getItem(int32 channel)                                  { return (Array<T>*)CounterIndexer_getItem((ICounterIndexer*)this, channel); }

public: /*for compatible*/
   int32     getCount()                                              { return getLength(); } 

private:
   DAQCLS_DISABLE_CONSTRUCT(CounterIndexer)
};

typedef CounterIndexer<CounterCapability> CounterCapabilityIndexer;
typedef CounterIndexer<SignalDrop>        CounterClockSourceIndexer;
typedef CounterIndexer<SignalDrop>        CounterGateSourceIndexer;

class CntrFeatures 
{
public:
   /*channel features*/
   int32 getChannelCountMax()                                        { return CntrFeatures_getChannelCountMax(this);              }
   int32 getResolution()                                             { return CntrFeatures_getResolution(this);                   }
   int32 getDataSize()                                               { return CntrFeatures_getDataSize(this);                     }
   CounterCapabilityIndexer* getCapabilities()                       { return (CounterCapabilityIndexer*)CntrFeatures_getCapabilities(this);}
                                                                       
   /*noise filter features*/                                           
   bool          getNoiseFilterSupported()                           { return !!CntrFeatures_getNoiseFilterSupported(this);              }
   Array<uint8>* getNoiseFilterOfChannels()                          { return (Array<uint8>*)CntrFeatures_getNoiseFilterOfChannels(this);}
   void          getNoiseFilterBlockTimeRange(MathInterval &x)       {        CntrFeatures_getNoiseFilterBlockTimeRange(this, &x);       }
                                                                       
   /*event counting features*/                                         
   Array<SignalPolarity>* getEcClockPolarities()                     { return (Array<SignalPolarity>*)CntrFeatures_getEcClockPolarities(this);}
   Array<SignalPolarity>* getEcGatePolarities()                      { return (Array<SignalPolarity>*)CntrFeatures_getEcGatePolarities(this); }
   Array<int32>*          getEcGateControlOfChannels()               { return (Array<int32>*)CntrFeatures_getEcGateControlOfChannels(this);   }
                                                                       
   /*frequency measurement features*/                                  
   Array<FreqMeasureMethod>* getFmMethods()                          { return (Array<FreqMeasureMethod>*)CntrFeatures_getFmMethods(this);     } 
                                                                       
   /*one-shot features*/                                               
   CounterClockSourceIndexer* getOsClockSources()                    { return (CounterClockSourceIndexer*)CntrFeatures_getOsClockSources(this);}
   Array<SignalPolarity>*     getOsClockPolarities()                 { return (Array<SignalPolarity>*)CntrFeatures_getOsClockPolarities(this); }
   CounterGateSourceIndexer*  getOsGateSources()                     { return (CounterGateSourceIndexer*)CntrFeatures_getOsGateSources(this);  }
   Array<SignalPolarity>*     getOsGatePolarities()                  { return (Array<SignalPolarity>*)CntrFeatures_getOsGatePolarities(this);  }
   Array<OutSignalType>*      getOsOutSignals()                      { return (Array<OutSignalType>*)CntrFeatures_getOsOutSignals(this);       }
   void                       getOsDelayCountRange(MathInterval &x)  {        CntrFeatures_getOsDelayCountRange(this, &x);                     }
                                                                       
   /*timer/pulse features*/                                            
   Array<int32>*          getTmrGateControlOfChannels()              { return (Array<int32>*)CntrFeatures_getTmrGateControlOfChannels(this);  }
   Array<SignalPolarity>* getTmrGatePolarities()                     { return (Array<SignalPolarity>*)CntrFeatures_getTmrGatePolarities(this);}
   Array<OutSignalType>*  getTmrOutSignals()                         { return (Array<OutSignalType>*)CntrFeatures_getTmrOutSignals(this);     }
   void                   getTmrFrequencyRange(MathInterval &x)      {        CntrFeatures_getTmrFrequencyRange(this, &x);                    }
                                                                       
   /*pulse width measurement features*/                                
   Array<CounterCascadeGroup>* getPiCascadeGroup()                   { return (Array<CounterCascadeGroup>*)CntrFeatures_getPiCascadeGroup(this);}
                                                                       
   /*pulse width modulation features*/                                 
   Array<int32>*          getPoGateControlOfChannels()               { return (Array<int32>*)CntrFeatures_getPoGateControlOfChannels(this);  }
   Array<SignalPolarity>* getPoGatePolarities()                      { return (Array<SignalPolarity>*)CntrFeatures_getPoGatePolarities(this);}
   Array<OutSignalType>*  getPoOutSignals()                          { return (Array<OutSignalType>*)CntrFeatures_getPoOutSignals(this);     }
   void                   getPoHiPeriodRange(MathInterval &x)        {        CntrFeatures_getPoHiPeriodRange(this, &x);          }
   void                   getPoLoPeriodRange(MathInterval &x)        {        CntrFeatures_getPoLoPeriodRange(this, &x);          }
   void                   getPoOutCountRange(MathInterval &x)        {        CntrFeatures_getPoOutCountRange(this, &x);          }
                                                                       
   /*up-down counter features*/                                        
   Array<CountingType>*   getUdCountingTypes()                       { return (Array<CountingType>*)CntrFeatures_getUdCountingTypes(this);} 
   Array<int32>*          getUdInitialValues()                       { return (Array<int32>*)CntrFeatures_getUdInitialValues(this);       }
   Array<EventId>*        getUdSnapEventSources()                    { return (Array<EventId>*)CntrFeatures_getUdSnapEventSources(this);  }

   /*new*/
   void                   getMeasurementTimeoutRange(MathInterval &x){        CntrFeatures_getMeasurementTimeoutRange(this, &x);          }

   /*new buffered counter*/
   int32                  getTriggerCount()                          { return CntrFeatures_getTriggerCount(this);                               }
   Array<SignalDrop>*     getTriggerSources()                        { return (Array<SignalDrop>*)CntrFeatures_getTriggerSources(this);         }
   Array<TriggerAction>*  getTriggerActions()                        { return (Array<TriggerAction>*)CntrFeatures_getTriggerActions(this);      }

   CounterClockSourceIndexer* getEcSampleClockSources()              { return (CounterClockSourceIndexer*)CntrFeatures_getEcSampleClockSources(this);  }
   CounterClockSourceIndexer* getPiSampleClockSources()              { return (CounterClockSourceIndexer*)CntrFeatures_getPiSampleClockSources(this);  }      
   CounterClockSourceIndexer* getPoSampleClockSources()              { return (CounterClockSourceIndexer*)CntrFeatures_getPoSampleClockSources(this);  }
   CounterClockSourceIndexer* getUdSampleClockSources()              { return (CounterClockSourceIndexer*)CntrFeatures_getUdSampleClockSources(this);  }

public: /*for compatible*/
   MathInterval getNoiseFilterBlockTimeRange()
   {
      MathInterval x;
      getNoiseFilterBlockTimeRange(x);
      return x;
   }
   MathInterval getDelayCountRange()
   {
      MathInterval x;
      getOsDelayCountRange(x);
      return x;
   }
   MathInterval getTimerFrequencyRange()
   {
      MathInterval x;
      getTmrFrequencyRange(x);
      return x;
   }
   Array<CounterCascadeGroup>* getPwmCascadeGroup()
   {
      return getPiCascadeGroup();
   }
   MathInterval getHiPeriodRange()
   {
      MathInterval x;
      getPoHiPeriodRange(x);
      return x;
   }
   MathInterval getLoPeriodRange()
   {
      MathInterval x;
      getPoLoPeriodRange(x);
      return x;
   }
   Array<CountingType>* getCountingTypes()   
   {
      return getUdCountingTypes();
   }
   Array<int32>* getInitialValues()   
   {
      return getUdInitialValues();
   }
   Array<int32>* getSnapEventSources()
   {
      return (Array<int32>*)getUdSnapEventSources();
   }
   bool getOneShotEventSupported()
   {
      return false;
   }
   bool getTimerEventSupported()
   {
      return false;
   }
   bool getOverflowEventSupported()
   {
      return false;
   }

private:
   DAQCLS_DISABLE_CONSTRUCT(CntrFeatures)
};

/*for compatible*/
typedef CntrFeatures    EventCounterFeatures ;     
typedef CntrFeatures    FreqMeterFeatures;  
typedef CntrFeatures    OneShotFeatures;  
typedef CntrFeatures    TimerPulseFeatures; 
typedef CntrFeatures    PwMeterFeatures;    
typedef CntrFeatures    PwModulatorFeatures;
typedef CntrFeatures    UdCounterFeatures;  

/* Interface CntrCtrlBase */
class CntrCtrlBase : public DaqCtrlBase
{
public:
   /*Counter Control Common Property*/
   CntrFeatures*  getFeatures()                         { return DAQFN_TV(DNLAPI_CNTR, 34, CntrFeatures *)(this);  }
   int32          getChannelStart()                     { return DAQFN_TV(DNLAPI_CNTR, 35, int32)(this);           }
   ErrorCode      setChannelStart(int32 value)          { return DAQFN_ET(DNLAPI_CNTR, 36, int32)(this, value);    }
   int32          getChannelCount()                     { return DAQFN_TV(DNLAPI_CNTR, 37, int32)(this);           }
   ErrorCode      setChannelCount(int32 value)          { return DAQFN_ET(DNLAPI_CNTR, 38, int32)(this, value);    }
   bool           getEnabled()                          { return DAQFN_TV(DNLAPI_CNTR, 39, bool)(this);            }
   ErrorCode      setEnabled(bool enabled)              { return DAQFN_ET(DNLAPI_CNTR, 40, bool)(this, enabled);   }
   bool           getRunning()                          { return DAQFN_TV(DNLAPI_CNTR, 41, bool)(this);            }

   //
   double         getNoiseFilterBlockTime()             { return DAQFN_TV(DNLAPI_CNTR, 42, double)(this);          }
   ErrorCode      setNoiseFilterBlockTime(double value) { return DAQFN_ET(DNLAPI_CNTR, 43, double)(this, value);   }

   //
   double         getMeasurementTimeout()               { return DAQFN_TV(DNLAPI_CNTR, 130, double)(this);         }
   ErrorCode      setMeasurementTimeout(double value)   { return DAQFN_ET(DNLAPI_CNTR, 131, double)(this, value);  }

public: /*for compatible*/
   int32          getChannel()                          { return getChannelStart(); }
   ErrorCode      setChannel(int32 ch)                  { setChannelCount(1); return setChannelStart(ch);   }
   NosFltChannel* getNoiseFilter()                      { return DAQFN_TV(DNLAPI_CNTR, 44, NosFltChannel *)(this); }

private:
   DAQCLS_DISABLE_CONSTRUCT(CntrCtrlBase)
};

/* Interface EcChannel */
class EcChannel
{
public:
   /*Common Property*/
   int32          getChannel()                           { return EcChannel_getChannel(this);              }
   bool           getNoiseFiltered()                     { return !!EcChannel_getNoiseFiltered(this);      }
   ErrorCode      setNoiseFiltered(bool value)           { return EcChannel_setNoiseFiltered(this, value); }
                                                           
   /*Event counting Property*/                             
   SignalPolarity getClockPolarity()                     { return EcChannel_getClockPolarity(this);        }
   ErrorCode      setClockPolarity(SignalPolarity value) { return EcChannel_setClockPolarity(this, value); }
   SignalPolarity getGatePolarity()                      { return EcChannel_getGatePolarity(this);         }
   ErrorCode      setGatePolarity(SignalPolarity value)  { return EcChannel_setGatePolarity(this, value);  }
   bool           getGated()                             { return !!EcChannel_getGated(this);              }
   ErrorCode      setGated(bool value)                   { return EcChannel_setGated(this, value);         }

private:
   DAQCLS_DISABLE_CONSTRUCT(EcChannel)
};

/* Interface EventCounterCtrl */
class EventCounterCtrl : public CntrCtrlBase
{
public:
   /*Event Counter Method*/
   ErrorCode Read(int32 &data)                { return EventCounterCtrl_Read(this, 1,    &data); }
   ErrorCode Read(int32 count, int32 *data)   { return EventCounterCtrl_Read(this, count, data); }

   /*Event Counter Property*/
   Array<EcChannel>* getChannels()            { return (Array<EcChannel>*)EventCounterCtrl_getChannels(this); }

   /*Create an EventCounterCtrl. Note: Use 'dispose' to free the object*/
   static EventCounterCtrl * Create(void)     { return EventCounterCtrl_Create();  }

public: /*for compatible*/
   int32 getValue()                           { return EventCounterCtrl_getValue(this); }

private:
   DAQCLS_DISABLE_CONSTRUCT(EventCounterCtrl)
};

/* Interface BufferedEventCounterCtrl */
class BufferedEventCounterCtrl : public CntrCtrlBase
{
public:
   /*Buffered Event Counter Method*/
   ErrorCode GetData(int32 cntr, int32 count, int32 data[], int32 timeout = 0, int32 *returned = NULL, double *startTime = NULL)
   { 
      return BufferedEventCounterCtrl_GetDataI32(this, cntr, count, data, timeout, returned, startTime);    
   }

   int32          getBufferCapacity()                      { return BufferedEventCounterCtrl_getBufferCapacity(this);       }
   Trigger*       getTrigger()                             { return BufferedEventCounterCtrl_getTrigger(this);              }
   Record*        getRecord()                              { return BufferedEventCounterCtrl_getRecord(this);               }
   ErrorCode      setSampleClkSrc(SignalDrop drop)         { return BufferedEventCounterCtrl_setSampleClkSrc(this, drop);   }
   SignalDrop     getSampleClkSrc()                        { return BufferedEventCounterCtrl_getSampleClkSrc(this);         }
   void           getChannelMap(int32 count, int8 *chMap)  {        BufferedEventCounterCtrl_getChannelMap(this, count, chMap);}
   ErrorCode      setChannelMap(int32 count, int8 *chMap)  { return BufferedEventCounterCtrl_setChannelMap(this, count, chMap);}

   ErrorCode      Prepare()                                { return BufferedEventCounterCtrl_Prepare(this); }
   void           Release()                                {        BufferedEventCounterCtrl_Release(this); }

   /*Event Counter Property*/
   Array<EcChannel>* getChannels()                         { return (Array<EcChannel>*)BufferedEventCounterCtrl_getChannels(this); }

   /*Create an BufferedEventCounterCtrl. Note: Use 'dispose' to free the object*/
   static BufferedEventCounterCtrl * Create(void)          { return BufferedEventCounterCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void          addDataReadyHandler(BfdCntrEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   (this, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
   void          removeDataReadyHandler(BfdCntrEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler(this, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
   void          addOverrunHandler(BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
   void          removeOverrunHandler(BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
   void          addCacheOverflowHandler(BfdCntrEventProc proc, void *userParam)            { DaqCtrlBase_addEventHandler   (this, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          removeCacheOverflowHandler(BfdCntrEventProc proc, void *userParam)         { DaqCtrlBase_removeEventHandler(this, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          addStoppedHandler(BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtCntrStopped,     (DaqEventProc)proc, userParam);}
   void          removeStoppedHandler(BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtCntrStopped,     (DaqEventProc)proc, userParam);}

private:
   DAQCLS_DISABLE_CONSTRUCT(BufferedEventCounterCtrl)
};

/* Interface FmChannel */
class FmChannel
{
public:
   /*Common Property*/
   int32             getChannel()                         { return FmChannel_getChannel(this);                 }
   bool              getNoiseFiltered()                   { return !!FmChannel_getNoiseFiltered(this);         }
   ErrorCode         setNoiseFiltered(bool value)         { return FmChannel_setNoiseFiltered(this, value);    }
                                                            
   /*Frequency measurement Property*/                       
   FreqMeasureMethod getFmMethod()                        { return FmChannel_getFmMethod(this);                }
   ErrorCode         setFmMethod(FreqMeasureMethod value) { return FmChannel_setFmMethod(this, value);         }
   double            getCollectionPeriod()                { return FmChannel_getCollectionPeriod(this);        }
   ErrorCode         setCollectionPeriod(double value)    { return FmChannel_setCollectionPeriod(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(FmChannel)
};

/* Interface FreqMeterCtrl */
class FreqMeterCtrl : public CntrCtrlBase
{
public:
   /*Frequency Meter Method*/
   ErrorCode Read(double &data)               { return FreqMeterCtrl_Read(this, 1,    &data); }
   ErrorCode Read(int32 count, double *data)  { return FreqMeterCtrl_Read(this, count, data); }

   /*Frequency Meter Property*/
   Array<FmChannel>* getChannels()            { return (Array<FmChannel>*)FreqMeterCtrl_getChannels(this); }

   /*Create an FreqMeterCtrl. Note: Use 'dispose' to free the object*/
   static FreqMeterCtrl * Create(void)        { return FreqMeterCtrl_Create();  }

public: /*for compatible*/
   double            getValue()                         { return FreqMeterCtrl_getValue(this);  }
   FreqMeasureMethod getMethod()                        { return FreqMeterCtrl_getMethod(this); }
   ErrorCode         setMethod(FreqMeasureMethod value) { return FreqMeterCtrl_setMethod(this, value);    }
   double            getCollectionPeriod()              { return FreqMeterCtrl_getCollectionPeriod(this); }
   ErrorCode         setCollectionPeriod(double value)  { return FreqMeterCtrl_setCollectionPeriod(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(FreqMeterCtrl)
};

/* Interface OsChannel */
class OsChannel
{
public:
   /*Common*/
   int32          getChannel()                             { return OsChannel_getChannel(this);              }
   bool           getNoiseFiltered()                       { return !!OsChannel_getNoiseFiltered(this);      }
   ErrorCode      setNoiseFiltered(bool value)             { return OsChannel_setNoiseFiltered(this, value); }

   /*One-shot Property*/                                     
   int32          getDelayCount()                          { return OsChannel_getDelayCount(this);           }
   ErrorCode      setDelayCount(int32 value)               { return OsChannel_setDelayCount(this, value);    }
   SignalDrop     getClockSource()                         { return OsChannel_getClockSource(this);          }
   ErrorCode      setClockSource(SignalDrop value)         { return OsChannel_setClockSource(this, value);   }
   SignalPolarity getClockPolarity()                       { return OsChannel_getClockPolarity(this);        }
   ErrorCode      setClockPolarity(SignalPolarity value)   { return OsChannel_setClockPolarity(this, value); }
   SignalDrop     getGateSource()                          { return OsChannel_getGateSource(this);           }
   ErrorCode      setGateSource(SignalDrop value)          { return OsChannel_setGateSource(this, value);    }
   SignalPolarity getGatePolarity()                        { return OsChannel_getGatePolarity(this);         }
   ErrorCode      setGatePolarity(SignalPolarity value)    { return OsChannel_setGatePolarity(this, value);  }
   OutSignalType  getOutSignal()                           { return OsChannel_getOutSignal(this);            }
   ErrorCode      setOutSignal(OutSignalType value)        { return OsChannel_setOutSignal(this, value);     }

private:
   DAQCLS_DISABLE_CONSTRUCT(OsChannel)
};

/* Interface OneShotCtrl */
class OneShotCtrl : public CntrCtrlBase 
{
public:
   /*One-shot Property*/
   Array<OsChannel>* getChannels()                                { return (Array<OsChannel>*)OneShotCtrl_getChannels(this);  }

   /*Create an OneShotCtrl. Note: Use 'dispose' to free the object*/
   static OneShotCtrl * Create(void)                              { return OneShotCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void addOneShotHandler(CntrEventProc proc, void *userParam)    { DaqCtrlBase_addEventHandler   (this, EvtCntOneShot0, (DaqEventProc)proc, userParam);}
   void removeOneShotHandler(CntrEventProc proc, void *userParam) { DaqCtrlBase_removeEventHandler(this, EvtCntOneShot0, (DaqEventProc)proc, userParam);}

   /*for compatible*/
   int32 getDelayCount()                                          { return OneShotCtrl_getDelayCount(this);         }
   ErrorCode setDelayCount(int32 value)                           { return OneShotCtrl_setDelayCount(this, value);  }

public: /*For compatible ONLY!*/
   void addOneShotListener(CntrEventListener & listener)          { OneShotCtrl_addOneShotListener(this, &listener);    }
   void removeOneShotListener(CntrEventListener & listener)       { OneShotCtrl_removeOneShotListener(this, &listener); }
   
private:
   DAQCLS_DISABLE_CONSTRUCT(OneShotCtrl)
};

/* Interface TmrChannel */
class TmrChannel
{
public:
   /*Common*/
   int32          getChannel()                           { return TmrChannel_getChannel(this);              }
   bool           getNoiseFiltered()                     { return !!TmrChannel_getNoiseFiltered(this);      }
   ErrorCode      setNoiseFiltered(bool value)           { return TmrChannel_setNoiseFiltered(this, value); }

   /*Timer/Pulse Property*/                                
   double         getFrequency()                         { return TmrChannel_getFrequency(this);            }
   ErrorCode      setFrequency(double value)             { return TmrChannel_setFrequency(this, value);     }
   bool           getGated()                             { return !!TmrChannel_getGated(this);              }
   ErrorCode      setGated(bool value)                   { return TmrChannel_setGated(this, value);         }
   SignalPolarity getGatePolarity()                      { return TmrChannel_getGatePolarity(this);         }
   ErrorCode      setGatePolarity(SignalPolarity value)  { return TmrChannel_setGatePolarity(this, value);  }
   OutSignalType  getOutSignal()                         { return TmrChannel_getOutSignal(this);            }
   ErrorCode      setOutSignal(OutSignalType value)      { return TmrChannel_setOutSignal(this, value);     }

private:
   DAQCLS_DISABLE_CONSTRUCT(TmrChannel)
};

/* Interface TimerPulseCtrl */
class TimerPulseCtrl : public CntrCtrlBase
{
public:
   /*Timer/Pulse Property*/
   Array<TmrChannel>* getChannels()                                   { return (Array<TmrChannel>*)TimerPulseCtrl_getChannels(this);  }

   /*Create an TimerPulseCtrl. Note: Use 'dispose' to free the object*/
   static TimerPulseCtrl * Create(void)                               { return TimerPulseCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void addTimerTickHandler(CntrEventProc proc, void *userParam)      { DaqCtrlBase_addEventHandler   (this, EvtCntTimer0, (DaqEventProc)proc, userParam); }
   void removeTimerTickHandler(CntrEventProc proc, void *userParam)   { DaqCtrlBase_removeEventHandler(this, EvtCntTimer0, (DaqEventProc)proc, userParam); }

   /*for compatible*/
   double getFrequency()                                              { return TimerPulseCtrl_getFrequency(this);         }
   ErrorCode setFrequency(double value)                               { return TimerPulseCtrl_setFrequency(this, value);  }

public: /*For compatible ONLY!*/
   void addTimerTickListener(CntrEventListener & listener)            { TimerPulseCtrl_addTimerTickListener(this, &listener);    } 
   void removeTimerTickListener(CntrEventListener & listener)         { TimerPulseCtrl_removeTimerTickListener(this, &listener); } 

private:
   DAQCLS_DISABLE_CONSTRUCT(TimerPulseCtrl)
};

/* Interface PiChannel */
class PiChannel
{
public:
   /*Common*/
   int32     getChannel()                        { return PiChannel_getChannel(this);              }
   bool      getNoiseFiltered()                  { return !!PiChannel_getNoiseFiltered(this);      }
   ErrorCode setNoiseFiltered(bool value)        { return PiChannel_setNoiseFiltered(this, value); }

private:
   DAQCLS_DISABLE_CONSTRUCT(PiChannel)
};

/* Interface PwMeterCtrl */
class PwMeterCtrl : public CntrCtrlBase
{
public:
   /*Pulse Width Measurement Method*/
   ErrorCode Read(PulseWidth &data)                                { return PwMeterCtrl_Read(this, 1,    &data); }
   ErrorCode Read(int32 count, PulseWidth *data)                   { return PwMeterCtrl_Read(this, count, data); }

   /*Pulse Width Measurement Property*/
   Array<PiChannel>* getChannels()                                 { return (Array<PiChannel>*)PwMeterCtrl_getChannels(this);  }

   /*Create an PwMeterCtrl. Note: Use 'dispose' to free the object*/
   static PwMeterCtrl * Create(void)                               { return PwMeterCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void addOverflowHandler(CntrEventProc proc, void *userParam)    { DaqCtrlBase_addEventHandler   (this, EvtCntPwmInOverflow0, (DaqEventProc)proc, userParam);}
   void removeOverflowHandler(CntrEventProc proc, void *userParam) { DaqCtrlBase_removeEventHandler(this, EvtCntPwmInOverflow0, (DaqEventProc)proc, userParam);}   

   /*for compatible*/
   void getValue(PulseWidth &width)                                {  Read(1, &width); }

public: /*For compatible ONLY!*/
   void addOverflowListener(CntrEventListener & listener)          { PwMeterCtrl_addOverflowListener(this, &listener);    }  
   void removeOverflowListener(CntrEventListener & listener)       { PwMeterCtrl_removeOverflowListener(this, &listener); }  

private:
   DAQCLS_DISABLE_CONSTRUCT(PwMeterCtrl)
};

/* Interface BufferedPwMeterCtrl */
class BufferedPwMeterCtrl : public CntrCtrlBase
{
public:
   /*Buffered Pulse Width Measurement Method*/
   ErrorCode GetData(int32 cntr, int32 count, PulseWidth *data, int32 timeout = 0, int32 *returned = NULL, double *startTime = NULL)
   {
      return BufferedPwMeterCtrl_GetData(this, cntr, count, data, timeout, returned, startTime);    
   }

   int32          getBufferCapacity()                      { return BufferedPwMeterCtrl_getBufferCapacity(this);       }
   Trigger*       getTrigger()                             { return BufferedPwMeterCtrl_getTrigger(this);              }
   Record*        getRecord()                              { return BufferedPwMeterCtrl_getRecord(this);               }
   ErrorCode      setSampleClkSrc(SignalDrop drop)         { return BufferedPwMeterCtrl_setSampleClkSrc(this, drop);   }
   SignalDrop     getSampleClkSrc()                        { return BufferedPwMeterCtrl_getSampleClkSrc(this);         }
   void           getChannelMap(int32 count, int8 *chMap)  {        BufferedPwMeterCtrl_getChannelMap(this, count, chMap);}
   ErrorCode      setChannelMap(int32 count, int8 *chMap)  { return BufferedPwMeterCtrl_setChannelMap(this, count, chMap);}

   ErrorCode      Prepare()                                { return BufferedPwMeterCtrl_Prepare(this); }
   void           Release()                                {        BufferedPwMeterCtrl_Release(this); }

   /*Pulse Width Measurement Property*/
   Array<PiChannel>* getChannels()                         { return (Array<PiChannel>*)BufferedPwMeterCtrl_getChannels(this);  }

   /*Create an BufferedPwMeterCtrl. Note: Use 'dispose' to free the object*/
   static BufferedPwMeterCtrl * Create(void)                               { return BufferedPwMeterCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void          addDataReadyHandler(BfdCntrEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   (this, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
   void          removeDataReadyHandler(BfdCntrEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler(this, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
   void          addOverrunHandler(BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
   void          removeOverrunHandler(BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
   void          addCacheOverflowHandler(BfdCntrEventProc proc, void *userParam)            { DaqCtrlBase_addEventHandler   (this, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          removeCacheOverflowHandler(BfdCntrEventProc proc, void *userParam)         { DaqCtrlBase_removeEventHandler(this, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          addStoppedHandler(BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtCntrStopped,     (DaqEventProc)proc, userParam);}
   void          removeStoppedHandler(BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtCntrStopped,     (DaqEventProc)proc, userParam);}

private:
   DAQCLS_DISABLE_CONSTRUCT(BufferedPwMeterCtrl)
};

/* Interface PoChannel */
class PoChannel
{
public:
   /*Common*/
   int32          getChannel()                           { return PoChannel_getChannel(this);              }
   bool           getNoiseFiltered()                     { return !!PoChannel_getNoiseFiltered(this);      }
   ErrorCode      setNoiseFiltered(bool value)           { return PoChannel_setNoiseFiltered(this, value); }
                                                           
   /*Pulse Width Modulation Property*/                     
   void           getPulseWidth(PulseWidth &width)       {        PoChannel_getPulseWidth(this, &width);  }
   ErrorCode      setPulseWidth(PulseWidth const &width) { return PoChannel_setPulseWidth(this, &width);  }
   bool           getGated()                             { return !!PoChannel_getGated(this);             }
   ErrorCode      setGated(bool value)                   { return PoChannel_setGated(this, value);        }
   SignalPolarity getGatePolarity()                      { return PoChannel_getGatePolarity(this);        }
   ErrorCode      setGatePolarity(SignalPolarity value)  { return PoChannel_setGatePolarity(this, value); }
   OutSignalType  getOutSignal()                         { return PoChannel_getOutSignal(this);           }
   ErrorCode      setOutSignal(OutSignalType value)      { return PoChannel_setOutSignal(this, value);    }
   int32          getOutCount()                          { return PoChannel_getOutCount(this);            }
   ErrorCode      setOutCount(int32 count)               { return PoChannel_setOutCount(this, count);     }

private:
   DAQCLS_DISABLE_CONSTRUCT(PoChannel)
};

/* Interface PwModulatorCtrl */
class PwModulatorCtrl : public CntrCtrlBase
{
public: 
   /*Pulse Width Modulation Property*/
   Array<PoChannel>* getChannels()                  { return (Array<PoChannel>*)PwModulatorCtrl_getChannels(this);  }

   /*Create an PwModulatorCtrl. Note: Use 'dispose' to free the object*/
   static PwModulatorCtrl * Create(void)            { return PwModulatorCtrl_Create();  }

public:
   /*for compatible*/
   void getPulseWidth(PulseWidth &width)            {        PwModulatorCtrl_getPulseWidth(this, &width); }
   ErrorCode setPulseWidth(PulseWidth const &width) { return PwModulatorCtrl_setPulseWidth(this, &width); }

private:
   DAQCLS_DISABLE_CONSTRUCT(PwModulatorCtrl)
};

/* Interface BufferedPwModulatorCtrl */
class BufferedPwModulatorCtrl : public CntrCtrlBase
{
public: 
   /*Buffered Event Counter Method*/
   ErrorCode SetData(int32 cntr, int32 count, PulseWidth *data)
   { 
      return BufferedPwModulatorCtrl_SetData(this, cntr, count, data);    
   }

   int32          getBufferCapacity()                      { return BufferedPwModulatorCtrl_getBufferCapacity(this);       }
   Trigger*       getTrigger()                             { return BufferedPwModulatorCtrl_getTrigger(this);              }
   int32          getSamples()                             { return BufferedPwModulatorCtrl_getSamples(this);              }       
   ErrorCode      setSamples(int32 value)                  { return BufferedPwModulatorCtrl_setSamples(this, value);       }
   int32          getIntervalCount()                       { return BufferedPwModulatorCtrl_getIntervalCount(this);        }
   ErrorCode      setIntervalCount(int32 value)            { return BufferedPwModulatorCtrl_setIntervalCount(this, value); }
   bool           getStreaming()                           { return !!BufferedPwModulatorCtrl_getStreaming(this);          }
   ErrorCode      setStreaming(bool value)                 { return BufferedPwModulatorCtrl_setStreaming(this, value);     }
   ErrorCode      setSampleClkSrc(SignalDrop drop)         { return BufferedPwModulatorCtrl_setSampleClkSrc(this, drop);   }
   SignalDrop     getSampleClkSrc()                        { return BufferedPwModulatorCtrl_getSampleClkSrc(this);         }
   void           getChannelMap(int32 count, int8 *chMap)  {        BufferedPwModulatorCtrl_getChannelMap(this, count, chMap);}
   ErrorCode      setChannelMap(int32 count, int8 *chMap)  { return BufferedPwModulatorCtrl_setChannelMap(this, count, chMap);}

   ErrorCode      Prepare()                                { return BufferedPwModulatorCtrl_Prepare(this); }
   void           Release()                                {        BufferedPwModulatorCtrl_Release(this); }

   /*Pulse Width Modulation Property*/
   Array<PoChannel>* getChannels()                         { return (Array<PoChannel>*)BufferedPwModulatorCtrl_getChannels(this);  }

   /*Create an BufferedPwModulatorCtrl. Note: Use 'dispose' to free the object*/
   static BufferedPwModulatorCtrl * Create(void)           { return BufferedPwModulatorCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void          addDataTransmittedHandler(BfdCntrEventProc proc, void *userParam)          { DaqCtrlBase_addEventHandler   (this, EvtCoDataTransmitted, (DaqEventProc)proc, userParam); }
   void          removeDataTransmittedHandler(BfdCntrEventProc proc, void *userParam)       { DaqCtrlBase_removeEventHandler(this, EvtCoDataTransmitted, (DaqEventProc)proc, userParam); }
   void          addUnderrunHandler(BfdCntrEventProc proc, void *userParam)                 { DaqCtrlBase_addEventHandler   (this, EvtCoUnderrun,        (DaqEventProc)proc, userParam); }
   void          removeUnderrunHandler(BfdCntrEventProc proc, void *userParam)              { DaqCtrlBase_removeEventHandler(this, EvtCoUnderrun,        (DaqEventProc)proc, userParam); }
   void          addCacheEmptiedHandler(BfdCntrEventProc proc, void *userParam)             { DaqCtrlBase_addEventHandler   (this, EvtCoCacheEmptied,    (DaqEventProc)proc, userParam); }
   void          removeCacheEmptiedHandler(BfdCntrEventProc proc, void *userParam)          { DaqCtrlBase_removeEventHandler(this, EvtCoCacheEmptied,    (DaqEventProc)proc, userParam); }
   void          addTransitStoppedHandler(BfdCntrEventProc proc, void *userParam)           { DaqCtrlBase_addEventHandler   (this, EvtCoTransStopped,    (DaqEventProc)proc, userParam); }
   void          removeTransitStoppedHandler(BfdCntrEventProc proc, void *userParam)        { DaqCtrlBase_removeEventHandler(this, EvtCoTransStopped,    (DaqEventProc)proc, userParam); }
   void          addStoppedHandler(BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtCntrStopped,       (DaqEventProc)proc, userParam); }
   void          removeStoppedHandler(BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtCntrStopped,       (DaqEventProc)proc, userParam); }

private:
   DAQCLS_DISABLE_CONSTRUCT(BufferedPwModulatorCtrl)
};

/* Interface UdChannel */
class UdChannel
{
public:
   /*Common*/
   int32        getChannel()                        { return UdChannel_getChannel(this);                   }
   bool         getNoiseFiltered()                  { return !!UdChannel_getNoiseFiltered(this);           }
   ErrorCode    setNoiseFiltered(bool value)        { return UdChannel_setNoiseFiltered(this, value);      }

   /*Up-down Counter Property*/                       
   CountingType getCountingType()                   { return UdChannel_getCountingType(this);              }
   ErrorCode    setCountingType(CountingType value) { return UdChannel_setCountingType(this, value);       }
   int32        getInitialValue()                   { return UdChannel_getInitialValue(this);              }
   ErrorCode    setInitialValue(int32 value)        { return UdChannel_setInitialValue(this, value);       }
   int32        getResetTimesByIndex()              { return UdChannel_getResetTimesByIndex(this);         }
   ErrorCode    setResetTimesByIndex(int32 value)   { return UdChannel_setResetTimesByIndex(this, value);  }

private:
   DAQCLS_DISABLE_CONSTRUCT(UdChannel)
};

/* Interface UdCounterCtrl */
class UdCounterCtrl : public CntrCtrlBase
{
public:
   /*Up-down Counter Method*/
   ErrorCode SnapStart(EventId srcId)                                                     { return UdCounterCtrl_SnapStart(this, srcId); }
   ErrorCode SnapStop(EventId srcId)                                                      { return UdCounterCtrl_SnapStop(this, srcId);  }
   ErrorCode CompareSetTable(int32 channel, int32 count, int32 *table)                    { return UdCounterCtrl_CompareSetTable(this, channel, count, table);  }
   ErrorCode CompareSetInterval(int32 channel, int32 start, int32 increment,int32 count)  { return UdCounterCtrl_CompareSetInterval(this, channel, start, increment, count);  }
   ErrorCode CompareClear(int32 channel)                                                  { return UdCounterCtrl_CompareClear(this, channel);  } 
   ErrorCode ValueReset()                                                                 { return UdCounterCtrl_ValueReset(this);             }
   
   ErrorCode Read(int32 &data)                                                            { return UdCounterCtrl_Read(this, 1,    &data);      }
   ErrorCode Read(int32 count, int32 *data)                                               { return UdCounterCtrl_Read(this, count, data);      }

   /*Up-down Counter Property*/
   Array<UdChannel>* getChannels()                                                        { return (Array<UdChannel>*)UdCounterCtrl_getChannels(this);  }

   /*Create an UdCounterCtrl. Note: Use 'dispose' to free the object*/
   static UdCounterCtrl * Create(void)                                                    { return UdCounterCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void addUdCntrEventHandler(UdCntrEventProc proc, void *userParam)                      { DaqCtrlBase_addEventHandler   (this, EvtUdIndex0, (DaqEventProc)proc, userParam);}
   void removeUdCntrEventHandler(UdCntrEventProc proc, void *userParam)                   { DaqCtrlBase_removeEventHandler(this, EvtUdIndex0, (DaqEventProc)proc, userParam);}

   /*for compatible*/
   int32        getValue()                                                                { return UdCounterCtrl_getValue(this);            }
   CountingType getCountingType()                                                         { return UdCounterCtrl_getCountingType(this);            }
   ErrorCode    setCountingType(CountingType value)                                       { return UdCounterCtrl_setCountingType(this, value);     }
   int32        getInitialValue()                                                         { return UdCounterCtrl_getInitialValue(this);            }
   ErrorCode    setInitialValue(int32 value)                                              { return UdCounterCtrl_setInitialValue(this, value);     }
   int32        getResetTimesByIndex()                                                    { return UdCounterCtrl_getResetTimesByIndex(this);       }
   ErrorCode    setResetTimesByIndex(int32 value)                                         { return UdCounterCtrl_setResetTimesByIndex(this, value);}

   ErrorCode CompareSetTable(int32 count, int32 *table)                                   { return CompareSetTable(getChannel(), count, table);               }
   ErrorCode CompareSetInterval(int32 start, int32 increment,int32 count)                 { return CompareSetInterval(getChannel(), start, increment, count); }
   ErrorCode CompareClear()                                                               { return CompareClear(getChannel());                                }

public: /*For compatible ONLY!*/
   void addUdCntrEventListener(UdCntrEventListener &listener)                             { UdCounterCtrl_addUdCntrEventListener(this, &listener);    }
   void removeUdCntrEventListener(UdCntrEventListener &listener)                          { UdCounterCtrl_removeUdCntrEventListener(this, &listener); }
   ErrorCode SnapStart(int32 srcId)                                                       { return UdCounterCtrl_SnapStart(this, (EventId)srcId);     }
   ErrorCode SnapStop(int32 srcId)                                                        { return UdCounterCtrl_SnapStop(this, (EventId)srcId);      }

private:
   DAQCLS_DISABLE_CONSTRUCT(UdCounterCtrl)
};

/* Interface BufferedUdCounterCtrl */
class BufferedUdCounterCtrl : public CntrCtrlBase
{
public:
   /*Buffered Up-down Counter Method*/
   ErrorCode GetData(int32 cntr, int32 count, int32 data[], int32 timeout = 0, int32 *returned = NULL, double *startTime = NULL)
   { 
      return BufferedUdCounterCtrl_GetDataI32(this, cntr, count, data, timeout, returned, startTime);    
   }

   int32          getBufferCapacity()                      { return BufferedUdCounterCtrl_getBufferCapacity(this);       }
   Trigger*       getTrigger()                             { return BufferedUdCounterCtrl_getTrigger(this);              }
   Record*        getRecord()                              { return BufferedUdCounterCtrl_getRecord(this);               }
   ErrorCode      setSampleClkSrc(SignalDrop drop)         { return BufferedUdCounterCtrl_setSampleClkSrc(this, drop);   }
   SignalDrop     getSampleClkSrc()                        { return BufferedUdCounterCtrl_getSampleClkSrc(this);         }
   void           getChannelMap(int32 count, int8 *chMap)  {        BufferedUdCounterCtrl_getChannelMap(this, count, chMap);}
   ErrorCode      setChannelMap(int32 count, int8 *chMap)  { return BufferedUdCounterCtrl_setChannelMap(this, count, chMap);}

   ErrorCode      Prepare()                                { return BufferedUdCounterCtrl_Prepare(this); }
   void           Release()                                {        BufferedUdCounterCtrl_Release(this); }

   /*Up-down Counter Property*/
   Array<UdChannel>* getChannels()                         { return (Array<UdChannel>*)BufferedUdCounterCtrl_getChannels(this);  }

   /*Create an UdCounterCtrl. Note: Use 'dispose' to free the object*/
   static BufferedUdCounterCtrl * Create(void)             { return BufferedUdCounterCtrl_Create();  }

public: /*helpers*/
   /*event*/
   void          addDataReadyHandler(BfdCntrEventProc proc, void *userParam)                { DaqCtrlBase_addEventHandler   (this, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
   void          removeDataReadyHandler(BfdCntrEventProc proc, void *userParam)             { DaqCtrlBase_removeEventHandler(this, EvtCiDataReady,     (DaqEventProc)proc, userParam);}
   void          addOverrunHandler(BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
   void          removeOverrunHandler(BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtCiOverrun,       (DaqEventProc)proc, userParam);}
   void          addCacheOverflowHandler(BfdCntrEventProc proc, void *userParam)            { DaqCtrlBase_addEventHandler   (this, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          removeCacheOverflowHandler(BfdCntrEventProc proc, void *userParam)         { DaqCtrlBase_removeEventHandler(this, EvtCiCacheOverflow, (DaqEventProc)proc, userParam);}
   void          addStoppedHandler(BfdCntrEventProc proc, void *userParam)                  { DaqCtrlBase_addEventHandler   (this, EvtCntrStopped,     (DaqEventProc)proc, userParam);}
   void          removeStoppedHandler(BfdCntrEventProc proc, void *userParam)               { DaqCtrlBase_removeEventHandler(this, EvtCntrStopped,     (DaqEventProc)proc, userParam);}

private:
   DAQCLS_DISABLE_CONSTRUCT(BufferedUdCounterCtrl)
};

#endif //defined(__cplusplus) && !defined(_BDAQ_C_INTERFACE)

#endif // DAQNAVI_CORE

END_NAMEAPCE_AUTOMATION_BDAQ

#endif /*_BDAQ_COM_LIKE_CLASS_LIB*/

