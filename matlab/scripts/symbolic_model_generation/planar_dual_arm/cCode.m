function mat = cCode(mat, fileName, symp, parallel)

    disp('CCODE----Creo il file C++');
    if parallel
        for i = 1:size(mat,1)
            mat_{i} = mat(i,:);
        end
        n = length(mat_);
        parfor i=1:n
            if symp 
                disp(strcat('CCODE----Semplifica l elemento: ', int2str(i), ' di: ' , int2str(n)));
                mat_{i} = simplify(mat_{i});
            end
            disp(strcat('CCODE----Creo il file c dell elemento: ', int2str(i), ' di: ' , int2str(n)));
            fileName_ = strcat(fileName,int2str(i));
            ccode(mat_{i},'file',strcat(fileName_,'.txt'));
        end
    else
        if symp 
            disp('MFUNCTION----Semplifica la matrice');
            mat = parSimplify(mat);
        end
        ccode(mat,'file',strcat(fileName,'.txt'));
    end
end
