%%%
  VERSION:1
  LANGUAGE:ENGLISH
%%%
MODULE MOD_SplitTest

    ! -------------------------------
    ! Define customized variables here
    ! ...

	! Tool variables: 
	PERS tooldata Tool1 := [TRUE,[[10.000,0.000,120.000],[0.99144486,0.00000000,0.13052619,0.00000001]],[1,[0,0,20],[1,0,0,0],0,0,0.005]];

	! Reference variables:
	PERS wobjdata Frame2 := [FALSE, TRUE, "", [[0,0,0],[1,0,0,0]],[[100.000,-50.000,20.000],[0.96592583,0.00000000,0.00000000,0.25881905]]];


	LOCAL PERS string modulepath1 := "";
	LOCAL PERS string modulepath2 := "";
	LOCAL VAR loadsession load1;
	LOCAL VAR loadsession load2;
	LOCAL VAR string module_unload := "";
	
	PROC Main()
		! This program has automatically been split into smaller subprograms
		TPErase;

		TPWrite "Starting Subprogram 1/3";
		! Load the first program (may take some time)
		modulepath1 := "/hd0a/Enter-Serial-Number/HOME/RoboDK/SplitTest_0.mod";
		Load \Dynamic, modulepath1;
		modulepath2 := "/hd0a/Enter-Serial-Number/HOME/RoboDK/SplitTest_1.mod";
		StartLoad modulepath2, load2;
		%"MOD_Tmp1:Prg_Tmp1"%;
		Unload modulepath1;
		modulepath1 := "";


		TPWrite "Starting Subprogram 2/3";
		! Wait for the program to be ready (fast)
		WaitLoad load2;
		modulepath1 := "/hd0a/Enter-Serial-Number/HOME/RoboDK/SplitTest_2.mod";
		StartLoad modulepath1, load1;
		%"MOD_Tmp2:Prg_Tmp2"%;
		Unload modulepath2;
		modulepath2 := "";


		TPWrite "Starting Subprogram 3/3";
		! Wait for the program to be ready (fast)
		WaitLoad load1;
		%"MOD_Tmp3:Prg_Tmp3"%;
		Unload modulepath1;
		modulepath1 := "";


		TPWrite "Program completed";

	ENDPROC

ENDMODULE
