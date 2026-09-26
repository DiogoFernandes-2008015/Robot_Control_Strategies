
syms th1 th2 'real'

tdh_n = [1 0 0 th1;1 0 0 th2];
info = [1;1];
qout = out.logsout{1}.Values.Data;
tout = out.tout;
xd = out.logsout{2}.Values.Data;
yd = out.logsout{3}.Values.Data;

ref = [xd yd];

dinanima_ref(tdh_n,info,qout,tout,1,ref')