/PROG  SUBPROG
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "RoboDK sequence";
PROG_SIZE	= 0;
CREATE		= DATE 26-10-26  TIME 01:01:16;
MODIFIED	= DATE 26-10-26  TIME 01:01:16;
FILE_NAME	= ;
VERSION	= 0;
LINE_COUNT	= 17;
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
  17:L P[1] R[10]mm/sec FINE ;
/POS
P[1]{
   GP1:
	UF : 1, UT : 1,		CONFIG : 'F U T, 0, 0, 0',
	X =     0.000  mm,	Y =     0.000  mm,	Z =     0.000  mm,
	W =    -0.000 deg,	P =   -30.000 deg,	R =   150.000 deg
};
/END
