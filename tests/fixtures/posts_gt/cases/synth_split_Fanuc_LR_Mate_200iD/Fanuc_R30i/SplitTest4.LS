/PROG  SplitTest4
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "RoboDK sequence";
PROG_SIZE	= 0;
CREATE		= DATE 31-12-14  TIME 12:00:00;
MODIFIED	= DATE 31-12-14  TIME 12:00:00;
FILE_NAME	= SplitTest4;
VERSION		= 0;
LINE_COUNT	= 3;
MEMORY_SIZE	= 0;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/MN
   1:L P[1] 200mm/sec FINE ;
   2:L P[2] 200mm/sec FINE ;
   3:L P[3] 200mm/sec FINE ;
/POS
P[1]{
   GP1:
    UF : 2, UT : 1,        CONFIG : 'F U T, 0, 0, 0',
	X =   286.258  mm,	Y =  -372.547  mm,	Z =   823.386  mm,
	W =   -52.463 deg,	P =     8.848 deg,	R =  -172.388 deg
};
P[2]{
   GP1:
    UF : 2, UT : 1,        CONFIG : 'F U T, 0, 0, 0',
	X =   332.153  mm,	Y =  -359.974  mm,	Z =   826.762  mm,
	W =   -51.647 deg,	P =     9.441 deg,	R =  -168.258 deg
};
P[3]{
   GP1:
    UF : 2, UT : 1,        CONFIG : 'F U T, 0, 0, 0',
	X =   286.258  mm,	Y =  -372.547  mm,	Z =   823.386  mm,
	W =   -52.463 deg,	P =     8.848 deg,	R =  -172.388 deg
};
/END
