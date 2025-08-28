%definimos dH/dt = f
% x1=H  x1'=H'=f


syms la lb L Qi S u g x1

optionss = bodeoptions;
optionss.PhaseMatching = 'on';
optionss.PhaseMatchingFreq = 1;
optionss.PhaseMatchingValue = -180;
optionss.Grid = 'on';

f= (Qi - S*u*sqrt(2*g*x1))/(lb + (la - lb)*x1/L);
x=x1;
y=x1;
% salida es u

% definimos valores conocidos

Qi=8/(1000*60);  %metros^3/segundo
d_tuberia= 10.65e-3; % metros
lb = 10e-2;  % metros
la= 40e-2; % metros
S= pi*((d_tuberia/2)^2); % metros^2
L=0.9; % metros
g=9.81; % metros/seg^2

f= (Qi - S*u*sqrt(2*g*x1))/(lb + (la - lb)*x1/L);

% valores equilibrio x1e=0.45;
x1e=0.45;
ue=solve(Qi - S*u*sqrt(2*g*x1e)==0,u);
ue=double(ue);


%%
% valores de equilibrio
x1e=0.45;
ue=0.5037;
ye=0.45;

A=jacobian(f,x);
B=jacobian(f,u);
C=jacobian(y,x);
D=jacobian(y,u);

A=subs(A,{'x1','u','y'},{x1e,ue,ye});
B=subs(B,{'x1','u','y'},{x1e,ue,ye});
C=subs(C,{'x1','u','y'},{x1e,ue,ye});
D=subs(D,{'x1','u','y'},{x1e  ,ue,ye});

Ass=double(A);
Bss=double(B);
Css=double(C);
Dss=double(D);

G= zpk(ss(Ass,Bss,Css,Dss));
G

%%
clc;
Qi=8/(1000*60);  %metros^3/segundo

% he=0.1;
G1= linealizar_Fun(0.1,valores_Equilibrio(0.1,Qi),0.1,Qi);

%he = 0.2
G2= linealizar_Fun(0.2,valores_Equilibrio(0.2,Qi),0.2,Qi);

% he= 0.3
G3= linealizar_Fun(0.3,valores_Equilibrio(0.3,Qi),0.3,Qi);

% he= 0.4
G4= linealizar_Fun(0.4,valores_Equilibrio(0.4,Qi),0.4,Qi);

% he= 0.5
G5= linealizar_Fun(0.5,valores_Equilibrio(0.5,Qi),0.5,Qi);

% he= 0.6
G6= linealizar_Fun(0.6,valores_Equilibrio(0.6,Qi),0.6,Qi);

% he= 0.7
G7= linealizar_Fun(0.7,valores_Equilibrio(0.7,Qi),0.7,Qi);

% he= 0.8
G8= linealizar_Fun(0.8,valores_Equilibrio(0.8,Qi),0.8,Qi);

% figure ()
% hold on 
% bode(G1);
% bode(G2);
% bode(G3);
% bode(G4);
% bode(G5);
% bode(G6);
% bode(G7);
% bode(G8);
% legend();
% hold off
% T1=1 - 1/(1+G1);
% T2=1 - 1/(1+G2);
% T3=1 - 1/(1+G3);
% T4=1 - 1/(1+G4);
% T5=1 - 1/(1+G5);
% T6=1 - 1/(1+G6);
% T7=1 - 1/(1+G7);
% T8=1 - 1/(1+G8);
% figure ()
% hold on 
% bode(T1);
% bode(T2);
% bode(T3);
% bode(T4);
% bode(T5);
% bode(T6);
% bode(T7);
% bode(T8);
% legend();
% hold off

%% Controlador
k=db2mag(-3.65);
%Se agrega un polo en -1 para tener un MF de 60°
% C=(1/G)*zpk([],[0 ],k); %Se utilizó un PI. 
bode(G*C,optionss);
L=C*G; %Lazo abierto;
H=L/(1+L); %Lazo cerrado;
% step(H);
S=1/(L+1);

T=1-S;
% bode(T);

function ue = valores_Equilibrio(h1e,Qi)
    syms u
    d_tuberia= 10.65e-3; % metros
    lb = 10e-2;  % metros
    la= 40e-2; % metros
    S= pi*((d_tuberia/2)^2); % metros^2
    L=0.9; % metros
    g=9.81; % metros/seg^2

    ue=solve(Qi - S*u*sqrt(2*g*h1e)==0,u);
    ue=double(ue);
end

function G=linealizar_Fun(x1e,ue,ye,Qi)
    syms x1 u
    d_tuberia= 10.65e-3; % metros
    lb = 10e-2;  % metros
    la= 40e-2; % metros
    S= pi*((d_tuberia/2)^2); % metros^2
    L=0.9; % metros
    g=9.81; % metros/seg^2
    f= (Qi - S*u*sqrt(2*g*x1))/(lb + (la - lb)*x1/L);

    x=x1;
    y=x1;
    A=jacobian(f,x);
    B=jacobian(f,u);
    C=jacobian(y,x);
    D=jacobian(y,u);

    A=subs(A,{'x1','u','y'},{x1e,ue,ye});
    B=subs(B,{'x1','u','y'},{x1e,ue,ye});
    C=subs(C,{'x1','u','y'},{x1e,ue,ye});
    D=subs(D,{'x1','u','y'},{x1e,ue,ye});

    Ass=double(A);
    Bss=double(B);
    Css=double(C);
    Dss=double(D);

    G= zpk(ss(Ass,Bss,Css,Dss));
end



