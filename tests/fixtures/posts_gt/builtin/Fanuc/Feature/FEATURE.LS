/PROG  FEATURE
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "RoboDK sequence";
PROG_SIZE	= 0;
CREATE		= DATE 26-10-26  TIME 01:01:16;
MODIFIED	= DATE 26-10-26  TIME 01:01:16;
FILE_NAME	= ;
VERSION	= 0;
LINE_COUNT	= 78;
MEMORY_SIZE	= 0;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
TIME_SLICE	= 0,
BUSY_LAMP_OFF	= 0,
ABORT_REQUEST	= 0,
PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:  UTOOL_NUM=1;
   2:  UFRAME_NUM=1;
   3:  PR[10,1]=598.963    ;
   4:  PR[10,2]=-0.000    ;
   5:  PR[10,3]=688.822    ;
   6:  PR[10,4]=0.000    ;
   7:  PR[10,5]=0.000    ;
   8:  PR[10,6]=30.000    ;
   9:  UFRAME[1]=PR[10] ;
  10:  PR[10,1]=0.000    ;
  11:  PR[10,2]=0.000    ;
  12:  PR[10,3]=120.000    ;
  13:  PR[10,4]=0.000    ;
  14:  PR[10,5]=15.000    ;
  15:  PR[10,6]=0.000    ;
  16:  UTOOL[1]=PR[10] ;
  17:J P[1] 50% FINE ;
  18:  R[10]=250 ;
  19:J P[2] 50% CNT5 ;
  20:L P[3] R[10]mm/sec CNT5 ;
  21:L P[4] R[10]mm/sec CNT5 ;
  22:C P[5]   
    :  P[6] R[10]mm/sec CNT5 ;
  23:  R[10]=120 ;
  24:L P[7] R[10]mm/sec FINE ;
  25:L P[8] R[10]mm/sec FINE ;
  26:J P[9] 50% FINE ;
  27:  R[10]=300 ;
  28:  DO[5]=ON ;
  29:  DO[3]=OFF ;
  30:  WAIT DI[2]=ON ;
  31:  MESSAGE[Use custom post processor to s] ;
  32:  MESSAGE[upport timeout] ;
  33:  WAIT DI[4]=OFF ;
  34:  WAIT  1.50(sec) ;
  35:  PAUSE ;
  36:  MESSAGE[A message] ;
  37:  CALL SUBPROG ;
  38:  CALL SPACEDNAME ;
  39:  CALL EXTERNALPR ;
  40:  CALL LONG_PROGR ;
  41:  CALL ANOTHERLO ;
  42:  CALL SETRPM700 ;
  43:  CALL AB_C ;
  44:  custom_code(1); 
  45:  RUN THREADPROG ;
  46:  DO[2]=3.5 ;
  47:  PR[10,1]=638.963    ;
  48:  PR[10,2]=-30.000    ;
  49:  PR[10,3]=708.822    ;
  50:  PR[10,4]=0.000    ;
  51:  PR[10,5]=0.000    ;
  52:  PR[10,6]=0.000    ;
  53:  UFRAME[1]=PR[10] ;
  54:  PR[10,1]=10.000    ;
  55:  PR[10,2]=0.000    ;
  56:  PR[10,3]=150.000    ;
  57:  PR[10,4]=0.000    ;
  58:  PR[10,5]=0.000    ;
  59:  PR[10,6]=0.000    ;
  60:  UTOOL[1]=PR[10] ;
  61:J P[10] 50% FINE ;
  62:J P[11] 50% FINE ;
  63:L P[12] R[10]mm/sec FINE ;
  64:  PR[10,1]=0.000    ;
  65:  PR[10,2]=0.000    ;
  66:  PR[10,3]=120.000    ;
  67:  PR[10,4]=0.000    ;
  68:  PR[10,5]=15.000    ;
  69:  PR[10,6]=0.000    ;
  70:  UTOOL[1]=PR[10] ;
  71:  PR[10,1]=598.963    ;
  72:  PR[10,2]=-0.000    ;
  73:  PR[10,3]=688.822    ;
  74:  PR[10,4]=0.000    ;
  75:  PR[10,5]=0.000    ;
  76:  PR[10,6]=30.000    ;
  77:  UFRAME[1]=PR[10] ;
  78:J P[13] 50% FINE ;
/POS
P[1]{
   GP1:
	UF : 1, UT : 1,	
	J1=    0.000 deg,	J2=    0.000 deg,	J3=    10.000 deg,
	J4=    0.000 deg,	J5=    35.000 deg,	J6=    0.000 deg
};
P[2]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =     0.000  mm,	Y =     0.000  mm,	Z =     0.000  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =   150.000 deg
};
P[3]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =    40.000  mm,	Y =     0.000  mm,	Z =     0.000  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =   150.000 deg
};
P[4]{
   GP1:
	UF : 1, UT : 1,	
	J1=    -0.865 deg,	J2=    3.778 deg,	J3=    10.839 deg,
	J4=    1.089 deg,	J5=    34.169 deg,	J6=    -1.513 deg
};
P[5]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =    40.000  mm,	Y =    40.000  mm,	Z =     0.000  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =   150.000 deg
};
P[6]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =     0.000  mm,	Y =    40.000  mm,	Z =    10.000  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =   150.000 deg
};
P[7]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =     0.000  mm,	Y =     0.000  mm,	Z =     0.000  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =   150.000 deg
};
P[8]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =     0.000  mm,	Y =     0.000  mm,	Z =     0.000  mm,
	W =   -11.170 deg,	P =   -28.024 deg,	R =   172.796 deg
};
P[9]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =     0.000  mm,	Y =     0.000  mm,	Z =     0.000  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =   150.000 deg
};
P[10]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =   -40.000  mm,	Y =    30.000  mm,	Z =   -40.000  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =  -180.000 deg
};
P[11]{
   GP1:
	UF : 1, UT : 1,	
	J1=    0.000 deg,	J2=    0.000 deg,	J3=    10.000 deg,
	J4=    0.000 deg,	J5=    35.000 deg,	J6=    0.000 deg
};
P[12]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =   -50.000  mm,	Y =    30.000  mm,	Z =   -57.321  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =  -180.000 deg
};
P[13]{
   GP1:
	UF : 1, UT : 1,	
	J1=    0.000 deg,	J2=    0.000 deg,	J3=    10.000 deg,
	J4=    0.000 deg,	J5=    35.000 deg,	J6=    0.000 deg
};
/END
