function [a0 a1 a2 a3 qk_der] = rob_sic_fun_q(qi,qk,qf,qi_der,T,k,N)
    a0=qi;
    a1=qi_der;

    vk=(qk-qi)/T;
    vk1=(qf-qk)/T;
    if(sign(vk)~=sign(vk1) | k==N+1)
        qk_der=zeros(length(qi),1);
    else
        qk_der=(vk+vk1)/2;
    end

    a3=(qk_der*T-2*qk+a1*T+2*a0)/(T^3);
    a2=(qk-a3*T^3-a1*T-a0)/(T^2);
end

