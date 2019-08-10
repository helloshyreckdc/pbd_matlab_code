% http://blog.sina.com.cn/s/blog_13e2751cb0102v4pm.html
% need connect in and out in simulink
SYS = linmod('mass_spring');
TSS = ss(SYS.a, SYS.b, SYS.c, SYS.d);
[n,d]=tfdata(TSS,'v');
T=tf(n,d)