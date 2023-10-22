% calling in the robots
waitMsg = msgbox("Workspace loading, please wait.");
r1 = Pulse75
r2 = Pulse75(transl(0,-0.5,0))
delete(waitMsg);