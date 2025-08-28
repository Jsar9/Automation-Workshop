function d = derivada_n(y, Ts, orden)
    N = length(y);
    d = zeros(N,1);

    if orden == 1
        for k = 2:N-1
            d(k) = (y(k+1) - y(k-1)) / (2*Ts);
        end
    elseif orden == 2
        for k = 2:N-1
            d(k) = (y(k+1) - 2*y(k) + y(k-1)) / (Ts^2);
        end
    elseif orden == 3
        for k = 3:N-2
            d(k) = (y(k-2) - 2*y(k-1) + 2*y(k+1) - y(k+2)) / (2*Ts^3);
        end
    elseif orden == 4
        for k = 3:N-2
            d(k) = (-y(k+2) + 4*y(k+1) - 6*y(k) + 4*y(k-1) - y(k-2)) / (Ts^4);
        end
    end
end
