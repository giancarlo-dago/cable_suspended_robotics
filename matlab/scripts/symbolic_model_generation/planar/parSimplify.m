function mat = parSimplify(mat)
    [n,m] = size(mat);
    disp('PARSYMP----riordino la matrice');
    if n > m
        for i = 1:n
            mat_{i} = mat(i,:);
        end
        k=n;
    else
        for i = 1:m
            mat_{i} = mat(:,i);
        end
        k=m;
    end

    parfor i = 1:k
            disp(strcat('PARSYMP----Semplifico la colonna o la riga: ', int2str(i), ' di ' , int2str(k)));
            mat_{i} = simplify(mat_{i});
    end

    if n > m
        for i = 1:n
           mat(i,:) = mat_{i};
        end
    else
        for i = 1:m
           mat(:,i) = mat_{i};
        end
    end
end


